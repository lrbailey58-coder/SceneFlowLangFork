import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray
import torch
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights
from PIL import Image as PILImage
import torchvision.transforms as T
import cv2
import numpy as np
import math

class SceneGraph:
    def __init__(self):
        self.connection = None 

    def update_connection(self, x, y):
        angle_rad = math.atan2(y, x)
        angle_deg = math.degrees(angle_rad)

        if -45.0 <= angle_deg <= 45.0:
            new_connection = "FRONT"
        elif 45.0 < angle_deg <= 135.0:
            new_connection = "LEFT"
        elif -135.0 <= angle_deg < -45.0:
            new_connection = "RIGHT"
        else:
            new_connection = "BACK"

        if self.connection != new_connection:
            self.connection = new_connection
            return True
        return False

    def get_graph_string(self):
        if not self.connection:
            return "[ROSbot_XL] --- (No Person Detected)"
        return f"[Person] ---> IS IN {self.connection} OF ---> [ROSbot_XL]"


class TrackingGraphNode(Node):
    def __init__(self):
        super().__init__('tracking_graph_node')
        
        # --- 1. MODEL SETUP ---
        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.80
        self.categories = weights.meta["categories"]
        
        # --- 2. SCENE GRAPH & TRACKER STATE ---
        self.graph = SceneGraph()
        self.camera_hfov = math.radians(73.0) 
        
        # Memory variables for Nearest Neighbor Tracking
        self.human_last_x = None
        self.human_last_y = None
        self.last_seen_time = 0.0
        
        # Rules for tracking
        self.max_jump_distance = 0.8  # Max meters the human can move between Lidar frames
        self.memory_timeout = 3.2     # Seconds to remember the human if the Lidar loses them
        
        # --- 3. ROS2 SETUP ---
        self.lidar_sub = self.create_subscription(PoseArray, '/detected_objects', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(CompressedImage, '/oak/rgb/image_raw/compressed', self.camera_callback, 1)
            
        self.get_logger().info('Object Permanence Tracker Started! Waiting for camera confirmation...')

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
                    
                    # We pass this to a special function just for setting the initial memory
                    self.set_ground_truth_from_camera(camera_angle)

    def set_ground_truth_from_camera(self, camera_angle):
        # We need the lidar poses right now to match the camera
        # If we don't have them yet, we skip
        if not hasattr(self, 'latest_lidar_msg'): return
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
            # The camera confirmed the human. Overwrite our memory with this absolute truth.
            self.human_last_x, self.human_last_y = best_match
            self.last_seen_time = self.get_clock().now().nanoseconds / 1e9

    def lidar_callback(self, msg):
        # Save the raw message so the camera can use it for Ground Truth
        self.latest_lidar_msg = msg
        
        # If we have no memory of a human, the Lidar shouldn't guess. 
        # Wait for the camera to find them first.
        if self.human_last_x is None:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # If it's been too long since we tracked the clump, wipe the memory
        if current_time - self.last_seen_time > self.memory_timeout:
            self.get_logger().warn("Track lost! Human disappeared for too long. Resetting graph.")
            self.human_last_x = None
            self.human_last_y = None
            if self.graph.connection is not None:
                self.graph.connection = None
                self.get_logger().info(f"GRAPH UPDATE: {self.graph.get_graph_string()}")
            return

        # --- NEAREST NEIGHBOR TRACKING ---
        # Look through all current clumps for the one closest to our memory
        closest_distance = 99999.0
        best_x = None
        best_y = None
        
        for pose in msg.poses:
            x_robot = -pose.position.x
            y_robot = -pose.position.y
            
            # How far is this Lidar clump from where the human was last frame?
            jump_dist = math.hypot(x_robot - self.human_last_x, y_robot - self.human_last_y)
            
            if jump_dist < closest_distance:
                closest_distance = jump_dist
                best_x = x_robot
                best_y = y_robot
                
        # If the closest clump is within our allowed physical movement limit, it's the human!
        if closest_distance <= self.max_jump_distance and best_x is not None:
            self.human_last_x = best_x
            self.human_last_y = best_y
            self.last_seen_time = current_time
            self.get_logger().info("Human is at:\nx="+str(best_x)+"\ny="+str(best_y)+"\nDistance: "+str(closest_distance))
            # Update the scene graph with the new tracked coordinates
            if self.graph.update_connection(best_x, best_y):
                self.get_logger().info(f"GRAPH UPDATE: {self.graph.get_graph_string()}")

def main(args=None):
    rclpy.init(args=args)
    node = TrackingGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
