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

class IntentTrackingNode(Node):
    def __init__(self):
        super().__init__('intent_tracking_node')
        
        # --- 1. MODEL SETUP ---
        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.80
        self.categories = weights.meta["categories"]
        
        # --- 2. FUSION & TRACKING STATE ---
        self.latest_lidar_poses = []
        self.camera_hfov = math.radians(73.0) 
        
        # NEW PHASE 3 VARIABLES:
        self.prev_distance = None       # Tracks the last known distance of the person
        self.approach_zone = 2.5        # Distance in meters where we start caring about an approach
        self.passing_triggered = False  # Flag to ensure we don't spam the terminal continuously
        
        # --- 3. ROS2 SETUP ---
        self.lidar_sub = self.create_subscription(
            PoseArray, '/detected_objects', self.lidar_callback, 10)
            
        self.camera_sub = self.create_subscription(
            CompressedImage, '/oak/rgb/image_raw/compressed', self.camera_callback, 1)
            
        self.get_logger().info('Phase 3: Intent Tracking Node Started!')

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
            
        person_spotted_this_frame = False
            
        for box, score, label in zip(prediction[0]['boxes'], prediction[0]['scores'], prediction[0]['labels']):
            if score > self.threshold:
                class_name = self.categories[label.item()]
                
                if class_name == 'person':
                    person_spotted_this_frame = True
                    x1, y1, x2, y2 = box.numpy().astype(int)
                    box_center_x = (x1 + x2) / 2.0
                    
                    normalized_x = (box_center_x / img_w) - 0.5
                    camera_angle = -normalized_x * self.camera_hfov
                    
                    self.track_person_intent(camera_angle)
                    
        # Reset tracking state if the person completely leaves the frame
        if not person_spotted_this_frame:
            if self.prev_distance is not None:
                self.get_logger().info("Target lost. Resetting tracking data.")
            self.prev_distance = None
            self.passing_triggered = False

    def track_person_intent(self, camera_angle):
        if not self.latest_lidar_poses:
            return
            
        best_match = None
        min_angle_diff = 100.0 
        angle_tolerance = math.radians(20.0) 
        
        for pose in self.latest_lidar_poses:
            x_robot = -pose.position.x
            y_robot = -pose.position.y
            
            lidar_angle = math.atan2(y_robot, x_robot)
            distance = math.hypot(x_robot, y_robot)
            
            angle_diff = abs(camera_angle - lidar_angle)
            
            if angle_diff < angle_tolerance and angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                best_match = distance
                
        if best_match is not None:
            current_distance = best_match
            
            # NEW PHASE 3 LOGIC: Check if the robot is approaching the person
            if self.prev_distance is not None:
                # Positive delta means the distance is decreasing
                distance_delta = self.prev_distance - current_distance
                
                # If shrinking AND inside the safety zone
                if current_distance <= self.approach_zone and distance_delta > 0.02:
                    if not self.passing_triggered:
                        self.get_logger().warn(
                            f"[APPROACH DETECTED] Moving towards person! Distance: {current_distance:.2f}m. "
                            f"Initiating USA passing rule simulation: must pass on the left (person on right)!"
                        )
                        self.passing_triggered = True
                
                # Reset the trigger flag if the robot backs away out of the zone
                elif current_distance > self.approach_zone:
                    self.passing_triggered = False
            
            # Update our history for the next frame calculation
            self.prev_distance = current_distance


def main(args=None):
    rclpy.init(args=args)
    node = IntentTrackingNode()
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
