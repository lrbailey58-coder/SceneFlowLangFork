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

class PassingEvaluatorNode(Node):
    def __init__(self):
        super().__init__('passing_evaluator_node')
        
        # --- 1. MODEL SETUP ---
        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.80
        self.categories = weights.meta["categories"]
        
        # --- 2. FUSION STATE ---
        self.latest_lidar_poses = []
        self.camera_hfov = math.radians(73.0) 
        
        # --- 3. PHASE 4: EVALUATION STATE VARIABLES ---
        self.in_encounter = False        # Are we currently interacting with a person?
        self.min_distance = 999.0        # Tracks the closest the robot ever gets to the person
        self.y_at_min_distance = 0.0     # Tracks the Left/Right position at the closest point
        self.encounter_threshold = 2.5   # Start grading when person is within 2.5 meters
        
        # --- 4. ROS2 SETUP ---
        self.lidar_sub = self.create_subscription(PoseArray, '/detected_objects', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(CompressedImage, '/oak/rgb/image_raw/compressed', self.camera_callback, 1)
            
        self.get_logger().info('Phase 4: Passing Evaluator Node Started! Awaiting manual driving test...')

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
                if self.categories[label.item()] == 'person':
                    person_spotted_this_frame = True
                    x1, y1, x2, y2 = box.numpy().astype(int)
                    box_center_x = (x1 + x2) / 2.0
                    
                    normalized_x = (box_center_x / img_w) - 0.5
                    camera_angle = -normalized_x * self.camera_hfov
                    
                    self.evaluate_pass(camera_angle)
                    
        # If the person leaves the frame, it usually means the robot just successfully drove past them.
        # We need to finalize the evaluation grade here too!
        if not person_spotted_this_frame and self.in_encounter:
            self.finalize_grade()

    def evaluate_pass(self, camera_angle):
        if not self.latest_lidar_poses: return
            
        best_match = None
        min_angle_diff = 100.0 
        
        for pose in self.latest_lidar_poses:
            x_robot = -pose.position.x
            y_robot = -pose.position.y
            
            lidar_angle = math.atan2(y_robot, x_robot)
            distance = math.hypot(x_robot, y_robot)
            
            angle_diff = abs(camera_angle - lidar_angle)
            if angle_diff < math.radians(20.0) and angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                best_match = (distance, y_robot)
                
        if best_match is not None:
            current_distance, current_y = best_match
            
            # 1. Start the encounter if we get close enough
            if current_distance <= self.encounter_threshold and not self.in_encounter:
                self.in_encounter = True
                self.get_logger().info("Encounter started! Tracking passing maneuver...")
                
            if self.in_encounter:
                # 2. Track the absolute minimum distance and the Y coordinate at that exact moment
                if current_distance < self.min_distance:
                    self.min_distance = current_distance
                    self.y_at_min_distance = current_y
                    
                # 3. If the distance starts increasing significantly, the robot is driving away. Grade the pass!
                elif current_distance > self.min_distance + 0.5:
                    self.finalize_grade()

    def finalize_grade(self):
        # We only grade if the robot actually got somewhat close to the person (e.g., within 2 meters)
        if self.min_distance < 2.0:
            self.get_logger().info("--- MANEUVER COMPLETE ---")
            
            # Positive Y means the person was on the left. This means the robot passed on the Right!
            if self.y_at_min_distance > 0:
                self.get_logger().error(
                    f"VIOLATION: Robot passed on the RIGHT! (Person was {self.y_at_min_distance:.2f}m to the Left)"
                )
            # Negative Y means the person was on the right. This means the robot passed on the Left!
            else:
                self.get_logger().info(
                    f"SUCCESS: Robot followed rule and passed on the LEFT! (Person was {abs(self.y_at_min_distance):.2f}m to the Right)"
                )
        
        # Reset everything for the next test
        self.in_encounter = False
        self.min_distance = 999.0
        self.y_at_min_distance = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PassingEvaluatorNode()
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
