import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped

class DroneFollower(Node):
    def __init__(self):
        super().__init__('drone_follower')
        # Subscribe to the ground robot’s pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ground_robot/pose',
            self.ground_pose_callback,
            10)
        # Publisher for drone velocity commands
        self.publisher = self.create_publisher(TwistStamped, '/drone/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.ground_pose = None

    def ground_pose_callback(self, msg: PoseStamped):
        self.ground_pose = msg

    def control_loop(self):
        if self.ground_pose is None:
            return

        cmd = TwistStamped()
        # A simple proportional controller to follow the ground robot (tune as needed)
        cmd.twist.linear.x = 0.5 * (self.ground_pose.pose.position.x)
        cmd.twist.linear.y = 0.5 * (self.ground_pose.pose.position.y)
        cmd.twist.linear.z = 0.5 * (self.ground_pose.pose.position.z) + 1.0

        self.publisher.publish(cmd)
        self.get_logger().info(
            f"Publishing cmd_vel: x={cmd.twist.linear.x:.2f}, y={cmd.twist.linear.y:.2f}, z={cmd.twist.linear.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DroneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
