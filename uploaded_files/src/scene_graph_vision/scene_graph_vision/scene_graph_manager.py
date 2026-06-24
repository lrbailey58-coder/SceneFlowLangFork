import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String # NEW: Import standard string messages for our decision
import json

class SceneGraphManager(Node):
    def __init__(self):
        super().__init__('scene_graph_manager')
        
        self.nodes = {}
        self.edges = set()
        
        # Permanent Static Nodes
        self.add_node("rosbot", {"type": "robot", "state": "driving"})
        self.add_node("crosswalk_zone", {"type": "zone", "state": "clear"})
        
        # Subscribe to the LIDAR clusters
        self.create_subscription(
            PoseArray, 
            '/detected_objects', 
            self.objects_callback, 
            10
        )
        self.cam_sub = self.create_subscription(
            String,
            '/oak/bounding_box/data',
            self.camera_data_callback,
            10
        )
        self.camera_objects = []

        # NEW: Create a publisher that outputs our final driving decision ("GO" or "STOP")
        self.safety_pub = self.create_publisher(String, '/safety_status', 10)
        
        self.timer = self.create_timer(2.0, self.log_graph_state)
        self.get_logger().info("Rule Evaluation Engine is active and monitoring...")


    def objects_callback(self, msg):
        # Wipe old obstacles and edges
        keys_to_remove = [node_id for node_id in self.nodes if str(node_id).startswith("obstacle_")]
        for key in keys_to_remove:
            del self.nodes[key]
            
        edges_to_remove = [edge for edge in self.edges if str(edge[0]).startswith("obstacle_")]
        for edge in edges_to_remove:
            self.edges.remove(edge)
            
        # Reset crosswalk state to clear by default
        self.nodes["crosswalk_zone"]["state"] = "clear"
            
        # Parse incoming clusters
        for i, pose in enumerate(msg.poses):
            node_id = f"obstacle_{i}"
            x = round(pose.position.x, 2)
            y = round(pose.position.y, 2)
            
            self.add_node(node_id, {"type": "obstacle", "x": x, "y": y})
            
            # Spatial query boundary check
            if -2.0 <= x <= -0.1 and -0.5 <= y <= 0.5:
                self.add_edge(node_id, "crosswalk_zone", "is_in")
                self.nodes["crosswalk_zone"]["state"] = "occupied"

        # NEW: Trigger the Rule Evaluation Engine immediately after updating the graph!
        self.evaluate_rules()

    def evaluate_rules(self):
        """THE RULE EVALUATION ENGINE
        This function queries the current graph state and decides the behavior rule.
        """
        decision_msg = String()
        
        # Query the graph: Check if the crosswalk zone is blocked
        if self.nodes["crosswalk_zone"]["state"] == "occupied":
            # Update the robot's internal node state attribute
            self.nodes["rosbot"]["state"] = "stop_required"
            # Set our decision text to STOP
            decision_msg.data = "STOP"
        else:
            # The crosswalk is clear, the robot can proceed
            self.nodes["rosbot"]["state"] = "driving"
            decision_msg.data = "GO"
            
        # Broadcast the decision to the rest of the ROS 2 network
        self.safety_pub.publish(decision_msg)

    def add_node(self, node_id, attributes):
        self.nodes[node_id] = attributes

    def add_edge(self, source, target, relation_type):
        self.edges.add((source, target, relation_type))

    def log_graph_state(self):
        self.get_logger().info("========= LIVE SCENE GRAPH STATUS =========")
        self.get_logger().info(f"Nodes Count: {len(self.nodes)}")
        for node_id, attrs in self.nodes.items():
            self.get_logger().info(f"  -> Node ID: [{node_id}] | Attributes: {attrs}")
            
        self.get_logger().info(f"Edges Count: {len(self.edges)}")
        if not self.edges:
            self.get_logger().info("  -> (No relationships established yet)")
        for edge in self.edges:
            self.get_logger().info(f"  -> [{edge[0]}] ---({edge[2]})---> [{edge[1]}]")
        self.get_logger().info("===========================================")
    def camera_data_callback(self, msg):
        # Convert the JSON string back into a Python list of dictionaries
        self.camera_objects = json.loads(msg.data)

        for obj in self.camera_objects:
            self.get_logger().info(f"Camera confirms a {obj['label']} is at pixel coordinates {obj['box']}")

def main(args=None):
    rclpy.init(args=args)
    node = SceneGraphManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
