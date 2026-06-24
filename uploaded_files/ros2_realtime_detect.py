import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import torch
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights
from PIL import Image as PILImage
import torchvision.transforms as T
import cv2
import numpy as np
import time

class RealTimeDetector(Node):
    def __init__(self):
        super().__init__('realtime_detector')
        
        # --- 1. MODEL SETUP ---
        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.8
        
        # NEW: Extract the human-readable class names from the model's metadata
        self.categories = weights.meta["categories"]
        
        # --- 2. ROS2 TOPICS SETUP ---
        self.camera_topic = '/oak/rgb/image_raw/compressed'
        self.output_topic = '/oak/bounding_box/compressed'
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            1) 
            
        self.publisher = self.create_publisher(CompressedImage, self.output_topic, 1)
        self.get_logger().info(f'Listening on {self.camera_topic} and publishing to {self.output_topic}...')

    def image_callback(self, msg):
        start_time = time.time() 
        
        # --- 3. DECODE ---
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None: return

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(img_rgb)
        img_tensor = self.transform(pil_img)
        
        # --- 4. INFERENCE ---
        with torch.no_grad():
            prediction = self.model([img_tensor])
            
        # --- 5. PROCESS CLASSES AND DRAW BOXES ---
        # We loop through the boxes, scores, and the integer labels
        for box, score, label in zip(prediction[0]['boxes'], prediction[0]['scores'], prediction[0]['labels']):
            if score > self.threshold:
                x1, y1, x2, y2 = box.numpy().astype(int)
                
                # NEW: Convert the integer label to a string (e.g., 1 -> 'person')
                # We use .item() to pull the integer out of the PyTorch tensor
                class_name = self.categories[label.item()]
                
                # Draw the bounding box
                cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # NEW: Draw the class name and score above the bounding box
                # Syntax: cv2.putText(image, text, coordinates, font, scale, color, thickness)
                label_text = f"{class_name} {score:.2f}"
                cv2.putText(img_bgr, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Print the finding to your Ubuntu terminal
                self.get_logger().info(f"Spotted: {class_name} (Confidence: {score:.2f})")
                
        # --- 6. ENCODE AND PUBLISH ---
        success, encoded_img = cv2.imencode('.jpg', img_bgr)
        if success:
            out_msg = CompressedImage()
            out_msg.header = msg.header 
            out_msg.format = "jpeg"
            out_msg.data = encoded_img.tobytes()
            self.publisher.publish(out_msg)
            
        fps = 1.0 / (time.time() - start_time)
        self.get_logger().debug(f"FPS: {fps:.2f}") # Changed to debug so it doesn't drown out the object spotting logs

def main(args=None):
    rclpy.init(args=args)
    node = RealTimeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down detector node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
