import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from planner.dijkstra import dijkstra
from planner.matrix_utils import cell_to_world

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner')

        self.path_pub = self.create_publisher(Path, 'planned_path', 10)

        start = (1, 1)
        goal = (6, 6)

        path_cells = dijkstra(start, goal)
        self.publish_path(path_cells)

    def publish_path(self, path_cells):
        path_msg = Path()
        path_msg.header.frame_id = "map"

        for cell in path_cells:
            i, j = cell
            x, y = cell_to_world(i, j)

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Chemin publié")

def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()
 
