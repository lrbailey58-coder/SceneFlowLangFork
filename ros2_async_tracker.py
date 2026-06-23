import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray
import torch
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights
from PIL import Image as PILImage
import torchvision.transforms as T
import cv2
import numpy as np
import math
import threading # Needed to handle safe multi-threaded memory access
import networkx as nx
from pass_on_left_ego_only import all_human_properties

class SceneNode:
    """A custom node object that perfectly matches the SceneFlowLang expectations."""
    def __init__(self, node_id, name, base_class):
        self.id = node_id
        self.name = name
        self.base_class = base_class
        
    def get_id(self):
        return self.id
        
    def is_phantom(self):
        # The repo uses this to ignore certain nodes; we want our human to be real!
        return False
        
    def __hash__(self):
        # NetworkX requires custom objects to be hashable so it can use them as dictionary keys
        return hash(self.id)
        
    def __eq__(self, other):
        return getattr(other, 'id', None) == self.id
class AsyncTrackingNode(Node):
    def __init__(self):
        super().__init__('async_tracking_node')

        # --- 1. THREAD SAFETY LOCK ---
        self.lock = threading.Lock()

        # --- 2. MODEL SETUP ---
        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.80
        self.categories = weights.meta["categories"]
        self.camera_hfov = math.radians(73.0)
        self.latest_lidar_msg = None

        self.human_last_x = None
        self.human_last_y = None
        self.last_seen_time = 0.0
        self.human_angle = 0.0

        self.max_jump_distance = 0.32
        self.memory_timeout = 3.5

        # --- 4. ROS2 MULTI-THREADING SETUP ---
        # Create two separate lane assignments
        self.lidar_cb_group = MutuallyExclusiveCallbackGroup()
        self.camera_cb_group = MutuallyExclusiveCallbackGroup()

        # Assign the subscriptions to their respective lanes
        self.lidar_sub = self.create_subscription(
            PoseArray, '/detected_objects', self.lidar_callback, 10,
            callback_group=self.lidar_cb_group)

        self.camera_sub = self.create_subscription(
            CompressedImage, '/oak/rgb/image_raw/compressed', self.camera_callback, 1,
                  callback_group=self.camera_cb_group)

        self.active_properties = all_human_properties
        self.active_trackers = None
        self.frame_counter =0
        self.get_logger().info('Asynchronous Object Permanence Tracker Operational!')
        

    def camera_callback(self, msg):
        # Heavy computing happens here in Thread B
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

        best_match = None
        min_angle_diff = 100.0

        for pose in poses:
            x_robot = -pose.position.x
            y_robot = -pose.position.y
            lidar_angle = math.atan2(y_robot, x_robot)

            angle_diff = abs(camera_angle - lidar_angle)
            if angle_diff < math.radians(20.0) and angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                best_match = (x_robot, y_robot)
        if best_match is not None:
            # Safely lock the variables while updating memory map values
            with self.lock:
                self.human_last_x, self.human_last_y = best_match
                self.last_seen_time = self.get_clock().now().nanoseconds / 1e9
    def lidar_callback(self, msg):
        # Fast computing happens here in Thread A
        with self.lock:
            self.latest_lidar_msg = msg
            hx, hy = self.human_last_x, self.human_last_y
            last_time = self.last_seen_time

        if hx is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        if current_time - last_time > self.memory_timeout:
            self.get_logger().warn("Track lost! Human out of range. Resetting graph.")
            self.frame_counter = 0
            with self.lock:
                self.human_last_x = None
                self.human_last_y = None
                self.active_trackers = None
            return

        self.frame_counter += 1
        closest_distance = 999.0
        best_x = None
        best_y = None
        for pose in msg.poses:
            x_robot = -pose.position.x
            y_robot = -pose.position.y

            jump_dist = math.hypot(x_robot - hx, y_robot - hy)
            if jump_dist < closest_distance:
                closest_distance = jump_dist
                best_x = x_robot
                best_y = y_robot

        if closest_distance <= self.max_jump_distance and best_x is not None:
            with self.lock:
                self.human_last_x = best_x
                self.human_last_y = best_y
                self.last_seen_time = current_time
            self.human_angle = math.degrees(math.atan2(best_y, best_x))
            self.get_logger().info("Human Angle: "+ str(self.human_angle))
            #self.get_logger().info("Human is at:\nx="+str(best_x)+"\ny="+str(best_y)+"\nDistance: "+str(closest_distan>
            # Generate the NetworkX graph for this exact microsecond in time
            current_sg = self.build_networkx_graph(best_x, best_y)
            if self.active_trackers is None:
                self.active_trackers = []
                for prop in self.active_properties:
                    self.active_trackers.extend(prop.make_concrete(current_sg))
            if self.active_trackers is not None:
                for tracker in self.active_trackers:
                    tracker.step(current_sg)
                    if tracker.is_trap() and not tracker.is_accepting():
                        self.get_logger().error("Robot Made Error (went Right)")
                    elif tracker.is_trap():
                        self.get_logger().info("Robot successfully passe on Left!")
                    else:  
                        self.get_logger().info("Robot has not yet made error")
            self.get_logger().debug("NetworkX Graph successfully built for this frame!")
    
    def build_networkx_graph(self, human_x, human_y):
        """Constructs a fresh NetworkX DiGraph for the current physical frame."""
        # 1. Initialize an empty Directed Graph
        sg = nx.DiGraph()
        
        # 2. Create the Ego (Robot) Node and Human Node
        ego_node = SceneNode(node_id=0, name="ego", base_class="vehicle")
        human_node = SceneNode(node_id=1, name="person_1", base_class="person")
        
        # 3. Add the nodes to the graph
        sg.add_node(ego_node)
        sg.add_node(human_node)
        
        # 4. Calculate the spatial relationship
        angle_rad = math.atan2(human_y, human_x)
        angle_deg = math.degrees(angle_rad)

        if -45.0 <= angle_deg <= 45.0:
            direction = "FRONT"
        elif 45.0 < angle_deg <= 135.0:
            direction = "LEFT"
        elif -135.0 <= angle_deg < -45.0:
            direction = "RIGHT"
        else:
            direction = "BACK"
            
        # 5. SAFE FALLBACK: Add the directed edge with multiple attribute keys
        sg.add_edge(ego_node, human_node, label=direction, type=direction, relation=direction)

        sg.graph['frame'] = self.frame_counter  # Track an incremental counter or timestamp
        sg.graph['cache'] = {}                  # The engine uses this to prevent double-calculations
        return sg
def main(args=None):
    rclpy.init(args=args)
    node = AsyncTrackingNode()

    # NEW FOR PHASE 4.5: Explicitly use a MultiThreadedExecutor instead of the default single thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin() # Spin handles tracking across available threads asynchronously
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
