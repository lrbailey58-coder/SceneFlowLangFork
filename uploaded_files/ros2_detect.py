import sys
import rclpy
from rclpy.node import Node
# We swap out the standard Image message for the CompressedImage message type
from sensor_msgs.msg import CompressedImage
import torch
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights
from PIL import Image as PILImage
import torchvision.transforms as T
import cv2
import numpy as np

class SingleFrameDetector(Node):
    def __init__(self):
        super().__init__('single_frame_detector')
        
        # --- 1. MODEL SETUP ---
        self.get_logger().info('Loading PyTorch model...')
        weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.transform = T.Compose([T.ToTensor()])
        self.threshold = 0.8
        
        # --- 2. ROS2 SETUP ---
        # Set to the compressed topic you discovered
        self.camera_topic = '/oak/rgb/image_raw/compressed' 
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            1) 
        self.get_logger().info(f'Listening for a compressed frame on {self.camera_topic}...')

    def image_callback(self, msg):
        self.get_logger().info('Compressed frame received! Unpacking and processing...')
        
        # --- 3. DECODE COMPRESSED IMAGE ---
        # Convert raw byte string from the ROS2 message into a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        
        # cv2.imdecode decodes the JPEG/PNG compressed bytes back into standard BGR image array
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if img_bgr is None:
            self.get_logger().error("Failed to decode compressed image!")
            return

        # PyTorch needs RGB, OpenCV decodes to BGR
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(img_rgb)
        img_tensor = self.transform(pil_img)
        
        # --- 4. PERFORM INFERENCE ---
        with torch.no_grad():
            prediction = self.model([img_tensor])
            
        # --- 5. DRAW BOXES ---
        for box, score, label in zip(prediction[0]['boxes'], prediction[0]['scores'], prediction[0]['labels']):
            if score > self.threshold:
                x1, y1, x2, y2 = box.numpy().astype(int)
                # We draw directly on the img_bgr array since OpenCV likes BGR
                cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.get_logger().info(f"Detected object with confidence {score:.2f}")

        # --- 6. SAVE AND EXIT ---
        cv2.imwrite('camera_output.jpg', img_bgr)
        self.get_logger().info("Saved bounding box image to 'camera_output.jpg'. Shutting down.")
        
        raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    node = SingleFrameDetector()
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
