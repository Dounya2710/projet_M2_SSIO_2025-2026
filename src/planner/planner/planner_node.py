#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from planner.matrix_utils import cell_to_world
from planner.representation_env import matrix

from planner.dijkstra import dijkstra
from planner.a_star import a_star
from planner.q_learning import q_learning_plan


def in_bounds(i: int, j: int, grid) -> bool:
    return 0 <= i < len(grid) and 0 <= j < len(grid[0])


def is_free(i: int, j: int, grid) -> bool:
    return grid[i][j] == 0


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")

        # -------- Params --------
        self.declare_parameter("algo", "dijkstra")  # dijkstra|astar|qlearning
        self.declare_parameter("start_i", 1)
        self.declare_parameter("start_j", 1)
        self.declare_parameter("goal_i", 6)
        self.declare_parameter("goal_j", 6)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("publish_period_s", 0.5)
        self.declare_parameter("publish_once", True)

        self.algo = self.get_parameter("algo").value
        self.start = (
            int(self.get_parameter("start_i").value),
            int(self.get_parameter("start_j").value),
        )
        self.goal = (
            int(self.get_parameter("goal_i").value),
            int(self.get_parameter("goal_j").value),
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_period_s = float(self.get_parameter("publish_period_s").value)
        self.publish_once = bool(self.get_parameter("publish_once").value)

        # -------- Publisher --------
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        # -------- Timer --------
        self.published = False
        self.timer = self.create_timer(self.publish_period_s, self._tick)

        # -------- Startup logs --------
        h = len(matrix)
        w = len(matrix[0]) if h > 0 else 0
        self.get_logger().info(f"PlannerNode started | algo={self.algo}")
        self.get_logger().info(f"Grid size: {h}x{w}")
        self.get_logger().info(f"Start={self.start} Goal={self.goal} frame_id={self.frame_id}")

        # Validation rapide
        self._validate_inputs()

    def _validate_inputs(self):
        if not matrix or not matrix[0]:
            self.get_logger().error("Matrix is empty. Check planner/representation_env.py")
            return

        for name, (i, j) in [("start", self.start), ("goal", self.goal)]:
            if not in_bounds(i, j, matrix):
                self.get_logger().error(f"{name} out of bounds: {(i,j)}")
            else:
                self.get_logger().info(f"{name} cell value={matrix[i][j]} (0=free assumed)")

        # Alerte si start/goal non libres selon convention 0 libre
        si, sj = self.start
        gi, gj = self.goal
        if in_bounds(si, sj, matrix) and not is_free(si, sj, matrix):
            self.get_logger().warn("Start is NOT free (according to 0=free). Maybe your convention is inverted?")
        if in_bounds(gi, gj, matrix) and not is_free(gi, gj, matrix):
            self.get_logger().warn("Goal is NOT free (according to 0=free). Maybe your convention is inverted?")

    def _tick(self):
        if self.publish_once and self.published:
            return

        path_cells = self._compute_path()
        if not path_cells:
            self.get_logger().warn("No path found (empty path).")
            self.published = True
            return

        msg = self._cells_to_path_msg(path_cells)
        self.path_pub.publish(msg)
        self.get_logger().info(f"Published path: {len(path_cells)} cells.")
        self.published = True

    def _compute_path(self):
        try:
            if self.algo == "dijkstra":
                return dijkstra(self.start, self.goal, matrix)
            elif self.algo == "astar":
                return a_star(self.start, self.goal, matrix)
            elif self.algo in ("qlearning", "q_learning", "q"):
                return q_learning_plan(self.start, self.goal, matrix)
            else:
                self.get_logger().error("Unknown algo. Use dijkstra|astar|qlearning")
                return []
        except Exception as e:
            self.get_logger().error(f"Exception while computing path: {e}")
            return []

    def _cells_to_path_msg(self, path_cells):
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (i, j) in path_cells:
            # Sécurité
            if not in_bounds(i, j, matrix):
                self.get_logger().warn(f"Skipping out-of-bounds cell in path: {(i,j)}")
                continue

            x, y = cell_to_world(i, j)
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
