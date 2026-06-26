import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data # NEW: The secret to zero-lag camera streams!
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage
import torch
import torchvision.transforms as T
from PIL import Image as PILImage
import cv2
import numpy as np
import sys
import time # NEW: For latency tracking
from collections import OrderedDict

try:
    from DAVE2pytorch import *
except ImportError:
    print("ERROR: DAVE2pytorch.py not found!")
    sys.exit(1)

class MLDriverNode(Node):
    def __init__(self):
        super().__init__('ml_driver_node')

        self.get_logger().info('Loading pre-trained DAVE2v3 model...')
        self.model_path = '/home/husarion/SceneFlowLangFork/M28-DAVE2v3-30K/model-DAVE2v3-2560x720-lr0.0001-100epoch-64batch-lossMSE-30Ksamples.pt'
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.model = DAVE2v3(input_shape=(2560, 720))
        checkpoint = torch.load(self.model_path, map_location=self.device, weights_only=False)
        
        if isinstance(checkpoint, dict) or isinstance(checkpoint, OrderedDict):
            self.model.load_state_dict(checkpoint)
        else:
            self.model = checkpoint
            
        self.model.to(self.device)
        self.model.eval() 
        self.transform = T.Compose([T.ToTensor()])
        
        # We need a callback group to allow multithreading
        self.camera_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.get_logger().info(f'Model loaded successfully on {self.device}! Ready to drive.')

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # FIX: Added qos_profile_sensor_data to prevent image queueing/lag
        self.camera_sub = self.create_subscription(
            CompressedImage, 
            '/oak/rgb/image_raw/compressed', 
            self.camera_callback, 
            qos_profile_sensor_data, 
            callback_group=self.camera_cb_group)

    def camera_callback(self, msg):
        # --- DIAGNOSTIC: Calculate Camera Transport Lag ---
        # msg.header.stamp is the exact microsecond the OAK-D lens captured the light
        image_time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        current_time = self.get_clock().now().nanoseconds / 1e9
        transport_lag = current_time - image_time
        
        # If the image is older than 0.15 seconds, THROW IT AWAY! Do not drive blindly.
        if transport_lag > 0.15:
            self.get_logger().warn(f"Image too old ({transport_lag:.2f}s lag). Dropping frame to catch up!")
            return

        inference_start = time.time()

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None: return

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        
        # Crop the image to a widescreen ratio before resizing
        # (Assuming a 1080p image: keep the middle, cut top/bottom)
        height, width, _ = img_rgb.shape
        crop_top = int(height * 0.25)
        crop_bottom = int(height * 0.75)
        img_cropped = img_rgb[crop_top:crop_bottom, :] 
        
        # Now resize the correctly-proportioned widescreen image
        img_resized = cv2.resize(img_cropped, (2560, 720))
        img_tensor = self.transform(PILImage.fromarray(img_resized))
        input_tensor = img_tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            prediction = self.model(input_tensor)
        
        predicted_angular_z = -float(prediction[0].item())
        predicted_linear_x = 0.2

        drive_msg = TwistStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.twist.linear.x = predicted_linear_x
        drive_msg.twist.angular.z = predicted_angular_z
        
        self.cmd_pub.publish(drive_msg)

        # --- DIAGNOSTIC: Calculate AI Math Time ---
        inference_time = time.time() - inference_start
        self.get_logger().info(f"Transport Lag: {transport_lag*1000:.0f}ms | AI Math Time: {inference_time*1000:.0f}ms | Steer: {predicted_angular_z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MLDriverNode()
    
    # FIX: Upgraded to MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        node.cmd_pub.publish(stop_msg)
        
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
