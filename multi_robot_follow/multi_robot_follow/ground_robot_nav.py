import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GroundRobotNav(Node):
    def __init__(self):
        super().__init__('ground_robot_nav')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/obstacle_alert',
            self.obstacle_callback,
            10)

    def obstacle_callback(self, msg: PoseStamped):
        self.get_logger().warn("Obstacle detected from drone – adjusting path!")
        # Integrate with Nav2 here (e.g. update costmap or trigger re-planning)

def main(args=None):
    rclpy.init(args=args)
    node = GroundRobotNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
