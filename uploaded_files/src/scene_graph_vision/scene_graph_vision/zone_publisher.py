import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32

class ZonePublisher(Node):
    def __init__(self):
        super().__init__('zone_publisher')
        
        # Publish the polygon so we can visualize the "Stop Zone" in RViz2
        self.publisher_ = self.create_publisher(PolygonStamped, '/crosswalk_zone', 10)
        
        # Create a timer to publish the zone constantly at 1.0 Hz
        self.timer = self.create_timer(1.0, self.publish_zone)

    def publish_zone(self):
        msg = PolygonStamped()
        
        # We use the same coordinate frame as the LIDAR scan so they align perfectly.
        # 'laser' is common, but it might be 'rplidar_link' depending on your config.
        msg.header.frame_id = 'laser' 
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Define the 4 corners of our rectangular Stop Zone
        # Coordinates are in meters: (x is forward/back, y is left/right)
        p1 = Point32(x=-0.1, y=-0.5, z=0.0)  # Bottom Right
        p2 = Point32(x=-2.0, y=-0.5, z=0.0)  # Top Right
        p3 = Point32(x=-2.0, y=0.5, z=0.0)   # Top Left
        p4 = Point32(x=-0.1, y=0.5, z=0.0)   # Bottom Left
        
        msg.polygon.points = [p1, p2, p3, p4]
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZonePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
