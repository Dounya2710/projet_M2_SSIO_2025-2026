import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from planner.dijkstra import dijkstra
from planner._test_helpers import sample_grid, validate_path


def flatten_grid(grid):
    """
    Convertit une grille dont les cellules peuvent être [0]/[1] en grille d'entiers 0/1.
    Ex:
      [[ [0], [1] ], ...]  ->  [[0, 1], ...]
    """
    flat = []
    for row in grid:
        new_row = []
        for cell in row:
            if isinstance(cell, (list, tuple)):
                # ex: [0] ou (0,)
                new_row.append(cell[0])
            else:
                new_row.append(cell)
        flat.append(new_row)
    return flat


class DijkstraTurtlebotNode(Node):
    def __init__(self):
        super().__init__('dijkstra_turtlebot_node')
        self.get_logger().info("Node Dijkstra pour TurtleBot4 lancé")

        # Topic cohérent avec RViz (tes captures montraient /planner_path)
        self.pub = self.create_publisher(Path, '/planner_path', 10)

        # --- Grille de test ---
        raw_grid = sample_grid()
        grid = flatten_grid(raw_grid)

        if not grid or not grid[0]:
            self.get_logger().error("Grille invalide (vide).")
            return

        rows = len(grid)
        cols = len(grid[0])
        self.get_logger().info(
            f"grid dims: {rows}x{cols} | grid[0][0]={grid[0][0]} (type={type(grid[0][0])})"
        )

        start = (0, 0)
        goal = (6, 7)

        def in_bounds(p):
            return 0 <= p[0] < rows and 0 <= p[1] < cols

        if not in_bounds(start) or not in_bounds(goal):
            self.get_logger().error(f"Start/goal hors limites : start={start}, goal={goal}, dims={rows}x{cols}")
            return

        self.get_logger().info(f"start={start}, goal={goal}")
        self.get_logger().info(f"start cell={grid[start[0]][start[1]]}, goal cell={grid[goal[0]][goal[1]]}")

        # --- Calcul du chemin ---
        path = dijkstra(start, goal, grid)

        if not path:
            self.get_logger().error(
                "Chemin vide : goal inatteignable OU convention obstacle/libre incohérente."
            )
            return

        # dijkstra renvoie visiblement (i, j, k) -> on convertit en (i, j) pour validate_path et RViz
        path2d = [(c[0], c[1]) for c in path]

        # Validation (helper attend un chemin 2D)
        validate_path(path2d, start, goal, grid)

        self.get_logger().info(f"Chemin calculé | longueur = {len(path2d)}")
        self.get_logger().info(f"Premières cellules : {path2d[:5]}")

        # --- Publication pour RViz ---
        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = now

        for (i, j) in path2d:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = float(j)  # x = colonne
            pose.pose.position.y = float(i)  # y = ligne
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.pub.publish(path_msg)
        self.get_logger().info("Chemin publié sur /planner_path pour RViz")


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraTurtlebotNode()
    # Laisse le temps à RViz de recevoir la publication
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

