import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class ObstacleCommNode(Node):
    def __init__(self):
        super().__init__('obstacle_comm_node')
        self.subscription = self.create_subscription(
            String,
            '/object_detections',
            self.detection_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, '/obstacle_alert', 10)

    def detection_callback(self, msg: String):
        self.get_logger().info(f"Received detection: {msg.data}")
        alert = PoseStamped()
        alert.header.stamp = self.get_clock().now().to_msg()
        alert.header.frame_id = 'drone_base'
        alert.pose.position.x = 1.0  # Example value (compute based on detection)
        alert.pose.position.y = 0.0
        alert.pose.position.z = 0.0
        self.publisher.publish(alert)
        self.get_logger().info("Published obstacle alert")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
