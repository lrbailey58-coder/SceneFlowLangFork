import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry  # NEW: We need to import the Odometry message type
import torch
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights
from PIL import Image as PILImage
import torchvision.transforms as T
import cv2
import numpy as np
import math
import threading
import networkx as nx
from pass_on_left_ego_only import all_human_properties
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker

class SceneNode:
    def __init__(self, node_id, name, base_class):
        self.id = node_id
        self.name = name
        self.base_class = base_class
        
    def get_id(self): return self.id
    def is_phantom(self): return False
    def __hash__(self): return hash(self.id)
    def __eq__(self, other): return getattr(other, 'id', None) == self.id

class AsyncTrackingNode(Node):
    def __init__(self):
        super().__init__('async_tracking_node')

        self.lock = threading.Lock()

        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.80
        self.categories = weights.meta["categories"]
        self.camera_hfov = math.radians(73.0)
        self.latest_lidar_msg = None

        # NEW: Now our memory tracks GLOBAL Map Coordinates, not Local Robot Coordinates!
        self.human_last_global_x = None
        self.human_last_global_y = None
        self.last_seen_time = 0.0
        
        # NEW: Store the Robot's current position and rotation in the world
        self.robot_global_x = 0.0
        self.robot_global_y = 0.0
        self.robot_global_yaw = 0.0
        self.human_vx = 0.0
        self.human_vy = 0.0
        self.max_jump_distance = 0.32 
        self.memory_timeout = 3.5     

        self.lidar_cb_group = MutuallyExclusiveCallbackGroup()
        self.camera_cb_group = MutuallyExclusiveCallbackGroup()
        self.odom_cb_group = MutuallyExclusiveCallbackGroup()

        self.active_properties = all_human_properties
        self.active_trackers = None
        self.frame_counter = 0
        self.get_logger().info('Object Permanence Tracker with Global Odometry is Operational!')
        
        # Subscriptions
        self.lidar_sub = self.create_subscription(
            PoseArray, '/detected_objects', self.lidar_callback, 10, callback_group=self.lidar_cb_group)
        self.camera_sub = self.create_subscription(
            CompressedImage, '/oak/rgb/image_raw/compressed', self.camera_callback, 1, callback_group=self.camera_cb_group)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10, callback_group=self.odom_cb_group)
        # NEW: Debug Publisher to broadcast where the robot thinks the human is
        self.debug_pub = self.create_publisher(Marker, '/debug/human_tracked_position', 10)

    # NEW: Helper function to convert Robot Quaternion data into simple Yaw (rotation in radians)
    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    # NEW: Keep track of exactly where the robot is in the world at all times
    def odom_callback(self, msg):
        with self.lock:
            self.robot_global_x = msg.pose.pose.position.x
            self.robot_global_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.robot_global_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def camera_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None: return

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_tensor = self.transform(PILImage.fromarray(img_rgb))
        img_w = img_bgr.shape[1]

        with torch.no_grad():
            prediction = self.model([img_tensor])

        for box, score, label in zip(prediction[0]['boxes'], prediction[0]['scores'], prediction[0]['labels']):
            if score > self.threshold:
                if self.categories[label.item()] == 'person':
                    x1, y1, x2, y2 = box.numpy().astype(int)
                    box_center_x = (x1 + x2) / 2.0
                    normalized_x = (box_center_x / img_w) - 0.5
                    camera_angle = -normalized_x * self.camera_hfov
                    self.set_ground_truth_from_camera(camera_angle)

    def set_ground_truth_from_camera(self, camera_angle):
        with self.lock:
            if self.latest_lidar_msg is None: return
            poses = self.latest_lidar_msg.poses
            rob_x = self.robot_global_x
            rob_y = self.robot_global_y
            rob_yaw = self.robot_global_yaw
            
            # NEW: Pull the current human memory so the camera can check its math
            hg_x = self.human_last_global_x
            hg_y = self.human_last_global_y

        best_match = None
        min_physical_distance = 999.0 

        for pose in poses:
            x_local = -pose.position.x
            y_local = -pose.position.y
            lidar_angle = math.atan2(y_local, x_local)

            angle_diff = abs(camera_angle - lidar_angle)
            if angle_diff < math.radians(15.0):
                distance_to_clump = math.hypot(x_local, y_local)
                if distance_to_clump < min_physical_distance:
                    min_physical_distance = distance_to_clump
                    
                    global_x = rob_x + (x_local * math.cos(rob_yaw)) - (y_local * math.sin(rob_yaw))
                    global_y = rob_y + (x_local * math.sin(rob_yaw)) + (y_local * math.cos(rob_yaw))
                    best_match = (global_x, global_y)
                
        if best_match is not None:
            # --- NEW: DIAGNOSTIC AND JUMP LIMIT FOR THE CAMERA ---
            if hg_x is not None and hg_y is not None:
                jump_dist = math.hypot(best_match[0] - hg_x, best_match[1] - hg_y)
                
                # If the camera tries to teleport the human further than your limit, block it!
                if jump_dist > self.max_jump_distance:
                    self.get_logger().warn(f"CAMERA FALSE POSITIVE BLOCKED: Tried to jump {jump_dist:.2f}m")
                    return # Exit the function immediately without updating memory
            
            # If it passes the test (or if we have no current memory), update the coordinates
            with self.lock:
                self.human_last_global_x, self.human_last_global_y = best_match
                self.last_seen_time = self.get_clock().now().nanoseconds / 1e9
            # -----------------------------------------------------

    def lidar_callback(self, msg):
        with self.lock:
            self.latest_lidar_msg = msg
            hg_x, hg_y = self.human_last_global_x, self.human_last_global_y
            rob_x, rob_y, rob_yaw = self.robot_global_x, self.robot_global_y, self.robot_global_yaw
            last_time = self.last_seen_time

        if hg_x is None: return
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - last_time # Calculate time passed since last scan

        # NEW: Predict where the human is based on their current walking speed
        if dt > 0 and dt < 1.0:
            pred_x = hg_x + (self.human_vx * dt)
            pred_y = hg_y + (self.human_vy * dt)
        else:
            # If it's been too long, reset momentum
            pred_x = hg_x
            pred_y = hg_y
            self.human_vx = 0.0
            self.human_vy = 0.0
        if current_time - last_time > self.memory_timeout:
            self.get_logger().warn("Track lost! Human out of range. Resetting graph.")
            self.frame_counter = 0
            with self.lock:
                self.human_last_global_x = None
                self.human_last_global_y = None
                self.active_trackers = None
            return

        self.frame_counter += 1
        closest_distance = 999.0
        best_global_x = None
        best_global_y = None
        best_local_x = None
        best_local_y = None
        
        for pose in msg.poses:
            x_local = -pose.position.x
            y_local = -pose.position.y

            # NEW: Convert EVERY Lidar clump into a Global coordinate before measuring distance
            clump_global_x = rob_x + (x_local * math.cos(rob_yaw)) - (y_local * math.sin(rob_yaw))
            clump_global_y = rob_y + (x_local * math.sin(rob_yaw)) + (y_local * math.cos(rob_yaw))

            # NEW: The jump distance is now based purely on Global map coordinates, immune to robot movement!
            jump_dist = math.hypot(clump_global_x - pred_x, clump_global_y - pred_y)
            
            if jump_dist < closest_distance:
                closest_distance = jump_dist
                best_global_x = clump_global_x
                best_global_y = clump_global_y
                
                # We still save the local coordinates because the DFA rulebook (FRONT/LEFT/RIGHT) needs Ego-centric values!
                best_local_x = x_local
                best_local_y = y_local

        if closest_distance <= self.max_jump_distance and best_global_x is not None:
            with self.lock:
                # NEW: Calculate how fast the human is moving to update their momentum
                if dt > 0:
                    inst_vx = (best_global_x - hg_x) / dt
                    inst_vy = (best_global_y - hg_y) / dt
                    
                    # Smooth the velocity (50% old, 50% new) so it doesn't get jerky
                    self.human_vx = (0.5 * inst_vx) + (0.5 * self.human_vx)
                    self.human_vy = (0.5 * inst_vy) + (0.5 * self.human_vy)

                # Update memory with the new Global position
                self.human_last_global_x = best_global_x
                self.human_last_global_y = best_global_y
                self.last_seen_time = current_time
            
            #debug publisher:
            debug_msg = Marker()
            
            # 1. Header info
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'odom' 
            
            # 2. Marker Configuration
            debug_msg.ns = "human_tracker"
            debug_msg.id = 0
            debug_msg.type = Marker.CYLINDER # Draw a cylinder
            debug_msg.action = Marker.ADD
            
            # 3. Position (Using your global math)
            debug_msg.pose.position.x = best_global_x
            debug_msg.pose.position.y = best_global_y
            debug_msg.pose.position.z = 0.5 # Lift it half a meter off the floor
            
            # 4. Size (Human sized: 0.4m wide, 1.0m tall)
            debug_msg.scale.x = 0.4
            debug_msg.scale.y = 0.4
            debug_msg.scale.z = 1.0
            
            # 5. Color (Bright Green, fully opaque)
            debug_msg.color.r = 0.0
            debug_msg.color.g = 1.0
            debug_msg.color.b = 0.0
            debug_msg.color.a = 1.0 
            
            self.debug_pub.publish(debug_msg)
            
            # The NetworkX graph still gets the local coordinates, so FRONT/LEFT/RIGHT is always relative to the camera
            current_sg = self.build_networkx_graph(best_local_x, best_local_y)
            
            if self.active_trackers is None:
                self.active_trackers = []
                for prop in self.active_properties:
                    self.active_trackers.extend(prop.make_concrete(current_sg))
            
            if self.active_trackers is not None:
                for tracker in self.active_trackers:
                    tracker.step(current_sg)
                    if tracker.is_trap() and not tracker.is_accepting():
                        self.get_logger().error("Trap State: Maneuver Violated!")
                    elif tracker.is_trap():
                        self.get_logger().info("Robot Successfully Passed on Left!  !]")
                    else:
                        self.get_logger().info("Accepting State: Human is at ("+ str(round(self.human_last_global_x*100)/100)+", "+str(round(self.human_last_global_y*100)/100)+")")
    
    def build_networkx_graph(self, human_x, human_y):
        sg = nx.DiGraph()
        ego_node = SceneNode(node_id=0, name="ego", base_class="vehicle")
        human_node = SceneNode(node_id=1, name="person_1", base_class="person")
        sg.add_node(ego_node)
        sg.add_node(human_node)
        
        angle_rad = math.atan2(human_y, human_x)
        angle_deg = math.degrees(angle_rad)

        if -45.0 <= angle_deg <= 45.0: direction = "FRONT"
        elif 45.0 < angle_deg <= 135.0: direction = "LEFT"
        elif -135.0 <= angle_deg < -45.0: direction = "RIGHT"
        else: direction = "BACK"
            
        sg.add_edge(ego_node, human_node, label=direction, type=direction, relation=direction)
        sg.graph['frame'] = self.frame_counter
        sg.graph['cache'] = {}
        return sg

def main(args=None):
    rclpy.init(args=args)
    node = AsyncTrackingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
