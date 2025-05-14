import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA

class AprilTagVisualizer(Node):
    def __init__(self):
        super().__init__('apriltag_visualizer')
        self.publisher = self.create_publisher(Marker, 'apriltag_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'apriltag'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position of the AprilTag
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5

        # Rotation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.707
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.707

        # Size: flat square like AprilTag
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.01  # Flat

        # Color: black with white outline-style color
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # Black

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
