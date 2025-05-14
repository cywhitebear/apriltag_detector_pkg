import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class DummyPosePublisher(Node):
    def __init__(self):
        super().__init__('dummy_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, 'apriltag_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.t = 0  # simulation time

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # or "world" or "camera_link"

        # Spiral path: position (x, y, z)
        msg.pose.position.x = 0.5 * math.cos(self.t)
        msg.pose.position.y = 0.5 * math.sin(self.t)
        msg.pose.position.z = 0.1 * self.t

        # Dummy constant orientation (identity quaternion)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)
        self.get_logger().info(f"Published dummy pose at t={self.t:.1f}")
        self.t += 0.2

def main(args=None):
    rclpy.init(args=args)
    node = DummyPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()