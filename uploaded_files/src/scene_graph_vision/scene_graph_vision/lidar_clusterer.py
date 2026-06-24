import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import math

class LidarClusterer(Node):
    def __init__(self):
        super().__init__('lidar_clusterer')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.publisher_ = self.create_publisher(PoseArray, '/detected_objects', 10)
        
        # Max distance between points to be considered the same object (1.75 centimeter)
        self.jump_distance = 0.2
        
        # NEW: Physical size thresholds for our objects (in meters)
        self.min_size = 0.05  # roughly a table leg (10 cm)
        self.max_size = 1   # roughly a cabinet (35 cm)

    def scan_callback(self, msg):
        points = []
        
        # 1. Convert polar to Cartesian
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))

        if not points:
            return

        clusters = []
        current_cluster = [points[0]]

        # 2. Group points into clusters based on proximity
        for i in range(1, len(points)):
            x1, y1 = current_cluster[-1]
            x2, y2 = points[i]
            dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            if dist < self.jump_distance:
                current_cluster.append(points[i])
            else:
                clusters.append(current_cluster)
                current_cluster = [points[i]]
        clusters.append(current_cluster)

        # 3. Size Filtering, Centroids, and Publishing
        pose_array = PoseArray()
        pose_array.header = msg.header 

        for c in clusters:
            # First, keep the noise filter (must have at least 5 laser hits)
            if len(c) >= 5:
                # NEW: Calculate the physical width of the object
                # Since points sweep sequentially, the width is roughly the 
                # distance between the first point c[0] and the last point c[-1]
                p_start = c[0]
                p_end = c[-1]
                cluster_width = math.sqrt((p_end[0] - p_start[0])**2 + (p_end[1] - p_start[1])**2)
                
                # NEW: Only proceed if the object is within our size boundaries
                if self.min_size <= cluster_width <= self.max_size:
                    
                    cx = sum(p[0] for p in c) / len(c)
                    cy = sum(p[1] for p in c) / len(c)
                    
                    pose = Pose()
                    pose.position.x = cx
                    pose.position.y = cy
                    pose_array.poses.append(pose)

        self.publisher_.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
