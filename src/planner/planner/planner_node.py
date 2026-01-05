import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from planner.matrix_utils import cell_to_world
from planner.representation_env import matrix, grid_height, grid_size, auto_load_default_sdf
from planner.dijkstra import dijkstra
from planner.a_star import a_star
from planner.q_learning import q_learning_plan


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")

        # Paramètre pour choisir l'algo: dijkstra | astar | qlearning
        self.declare_parameter("algo", "dijkstra")
        self.algo = self.get_parameter("algo").get_parameter_value().string_value
        
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/planner_path", qos)

        # Start/goal in grid coordinates (i, j, k)
        # For a ground robot: k=0
        self.start = (1, 1, 0)
        self.goal = (grid_size - 2, grid_size - 2, 0)

        # Publish once at startup
        self.timer = self.create_timer(0.5, self.publish_once)
        self.published = False

        auto_load_default_sdf()
        self.get_logger().info(f"PlannerNode started with algo={self.algo}")

    def path_to_ros_path(self, path_cells):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (i, j, k) in path_cells:
            x, y, z = cell_to_world(i, j, k)
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = "odom"
            path_msg.poses.append(pose)

        return path_msg

    def publish_once(self):
        if self.published:
            return

        if self.algo == "dijkstra":
            path_cells = dijkstra(self.start, self.goal, matrix)
        elif self.algo == "astar":
            path_cells = a_star(self.start, self.goal, matrix)
        elif self.algo == "qlearning":
            path_cells = q_learning_plan(self.start, self.goal, matrix)
        else:
            self.get_logger().error("Unknown algo. Use dijkstra|astar|qlearning")
            path_cells = []

        if not path_cells:
            self.get_logger().warning("No path found.")
            self.published = True
            return

        path_msg = self.path_to_ros_path(path_cells)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_cells)} waypoints.")
        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
