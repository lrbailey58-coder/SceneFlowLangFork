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

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # --- 1. MODEL SETUP ---
        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.80 # Dropped slightly to catch the person more reliably
        self.categories = weights.meta["categories"]
        
        # --- 2. FUSION STATE ---
        self.latest_lidar_poses = []
        self.camera_hfov = math.radians(73.0) 
        
        # --- 3. ROS2 SETUP ---
        self.lidar_sub = self.create_subscription(
            PoseArray, '/detected_objects', self.lidar_callback, 10)
            
        self.camera_sub = self.create_subscription(
            CompressedImage, '/oak/rgb/image_raw/compressed', self.camera_callback, 1)
            
        self.get_logger().info('Sensor Fusion Node Started with Inverted X/Y Correction!')

    def lidar_callback(self, msg):
        self.latest_lidar_poses = msg.poses

    def camera_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None: return
        
        img_h, img_w = img_bgr.shape[:2]
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_tensor = self.transform(PILImage.fromarray(img_rgb))
        
        with torch.no_grad():
            prediction = self.model([img_tensor])
            
        for box, score, label in zip(prediction[0]['boxes'], prediction[0]['scores'], prediction[0]['labels']):
            if score > self.threshold:
                class_name = self.categories[label.item()]
                
                if class_name == 'person':
                    x1, y1, x2, y2 = box.numpy().astype(int)
                    box_center_x = (x1 + x2) / 2.0
                    
                    # Normalize center: Left is positive angle, Right is negative angle
                    normalized_x = (box_center_x / img_w) - 0.5
                    camera_angle = -normalized_x * self.camera_hfov
                    
                    self.match_person_to_lidar(camera_angle)

    def match_person_to_lidar(self, camera_angle):
        if not self.latest_lidar_poses:
            return
            
        best_match = None
        min_angle_diff = 100.0 
        angle_tolerance = math.radians(20.0) # 20-degree window for better consistency
        
        for pose in self.latest_lidar_poses:
            # FIX: Invert both axes because the Lidar sensor is physically turned 180 degrees
            x_robot = -pose.position.x
            y_robot = -pose.position.y
            
            # Calculate metrics using the corrected robot-forward frame
            lidar_angle = math.atan2(y_robot, x_robot)
            distance = math.hypot(x_robot, y_robot)
            
            angle_diff = abs(camera_angle - lidar_angle)
            
            # Un-comment the line below if you need to debug raw angles in your terminal:
            # self.get_logger().info(f"Cam: {math.degrees(camera_angle):.1f}° | Lidar: {math.degrees(lidar_angle):.1f}°")
            
            if angle_diff < angle_tolerance and angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                best_match = (distance, x_robot, y_robot)
                
        if best_match:
            dist, rx, ry = best_match
            self.get_logger().info(
                f"FUSION LOCK: Person detected at {dist:.2f}m ahead (Corrected X: {rx:.2f}, Y: {ry:.2f})"
            )
        else:
            self.get_logger().warn("Person seen by camera, but no matching Lidar clump found nearby.")

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
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
