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
    """A simple graph structure to hold nodes and their spatial relationships."""
    def __init__(self):
        self.nodes = ["ROSbot_XL", "Person"]
        self.connection = None  # Holds the current edge (e.g., "FRONT", "LEFT")

    def update_connection(self, x, y):
        # Calculate the angle in degrees relative to the robot
        # atan2 returns radians between -pi and pi
        angle_rad = math.atan2(y, x)
        angle_deg = math.degrees(angle_rad)

        # Determine the sector based on 90-degree slices
        if -45.0 <= angle_deg <= 45.0:
            new_connection = "FRONT"
        elif 45.0 < angle_deg <= 135.0:
            new_connection = "LEFT"
        elif -135.0 <= angle_deg < -45.0:
            new_connection = "RIGHT"
        else:
            new_connection = "BACK"

        # Only return True if the state actually changed, so we don't spam the terminal
        if self.connection != new_connection:
            self.connection = new_connection
            return True
        return False

    def get_graph_string(self):
        if not self.connection:
            return "[ROSbot_XL] --- (No Person Detected)"
        return f"[Person] ---> IS IN {self.connection} OF ---> [ROSbot_XL]"


class SceneGraphNode(Node):
    def __init__(self):
        super().__init__('scene_graph_node')
        
        # --- 1. MODEL SETUP ---
        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.80
        self.categories = weights.meta["categories"]
        
        # --- 2. SCENE GRAPH STATE ---
        self.graph = SceneGraph()
        self.latest_lidar_poses = []
        self.camera_hfov = math.radians(73.0) 
        
        # --- 3. ROS2 SETUP ---
        self.lidar_sub = self.create_subscription(PoseArray, '/detected_objects', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(CompressedImage, '/oak/rgb/image_raw/compressed', self.camera_callback, 1)
            
        self.get_logger().info('Scene Graph Generator Started! Mapping environment...')

    def lidar_callback(self, msg):
        self.latest_lidar_poses = msg.poses

    def camera_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None: return
        
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_tensor = self.transform(PILImage.fromarray(img_rgb))
        img_w = img_bgr.shape[1]
        
        with torch.no_grad():
            prediction = self.model([img_tensor])
            
        person_spotted_this_frame = False
            
        for box, score, label in zip(prediction[0]['boxes'], prediction[0]['scores'], prediction[0]['labels']):
            if score > self.threshold:
                if self.categories[label.item()] == 'person':
                    person_spotted_this_frame = True
                    x1, y1, x2, y2 = box.numpy().astype(int)
                    box_center_x = (x1 + x2) / 2.0
                    
                    normalized_x = (box_center_x / img_w) - 0.5
                    camera_angle = -normalized_x * self.camera_hfov
                    
                    self.update_scene_graph(camera_angle)
                    
        # If the person is completely out of frame, clear the connection
        if not person_spotted_this_frame and self.graph.connection is not None:
            self.graph.connection = None
            self.get_logger().info(f"GRAPH UPDATE: {self.graph.get_graph_string()}")

    def update_scene_graph(self, camera_angle):
        if not self.latest_lidar_poses: return
            
        best_match = None
        min_angle_diff = 100.0 
        
        for pose in self.latest_lidar_poses:
            x_robot = -pose.position.x
            y_robot = -pose.position.y
            
            lidar_angle = math.atan2(y_robot, x_robot)
            
            angle_diff = abs(camera_angle - lidar_angle)
            if angle_diff < math.radians(20.0) and angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                best_match = (x_robot, y_robot)
                
        if best_match is not None:
            x, y = best_match
            # Update the graph. If it returns True, the relationship changed!
            if self.graph.update_connection(x, y):
                # Print the new structural relationship to the terminal
                self.get_logger().info(f"GRAPH UPDATE: {self.graph.get_graph_string()}")

def main(args=None):
    rclpy.init(args=args)
    node = SceneGraphNode()
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
