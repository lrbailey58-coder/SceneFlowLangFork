import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped # NEW: Correct import for Jazzy

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')
        
        self.create_subscription(
            String,
            '/safety_status',
            self.status_callback,
            10
        )
        
        # NEW: Publish as TwistStamped
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.current_decision = "STOP"
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info("Safety Controller active. WAITING FOR COMMANDS...")

    def status_callback(self, msg):
        self.current_decision = msg.data

    def publish_velocity(self):
        msg = TwistStamped()
        
        # THE FIX: Inject the live, real-time clock stamp into the header!
        # Without this line, the ROSbot XL firmware will block the movement.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" # Tell the robot the command is relative to its own center
        
        if self.current_decision == "GO":
            # Note: Stamped messages nest the linear/angular data inside a '.twist' attribute
            msg.twist.linear.x = 0.15
            msg.twist.angular.z = 0.0
        else: 
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
