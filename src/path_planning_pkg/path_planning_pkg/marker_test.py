import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MarkerTest(Node):
    def __init__(self):
        super().__init__('marker_test')

        # marker topic名称
        self.declare_parameter('marker_topic', '/visualization_marker')
        self.marker_topic = self.get_parameter('marker_topic').value

        self.publisher_ = self.create_publisher(Marker, self.marker_topic, 10)
        
        self.timer = self.create_timer(1.0, self.publish_marker) # 每秒发布一次

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'custom_markers'
        marker.id = 0
        marker.type = 2 # CUBE=1, SPHERE=2, CYLINDER=3
        marker.action = 0   # ADD=0, MODIFY=0, DELETE=2
        
        # Pose
        marker.pose.position.x = 0.34305
        marker.pose.position.y = -0.04185
        marker.pose.position.z = 0.44799
        marker.pose.orientation.x = 0.98064
        marker.pose.orientation.y = -0.01348
        marker.pose.orientation.z = 0.19523
        marker.pose.orientation.w = 0.00661
        
        # Scale
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Lifetime
        lifetime_sec = 0    # 0表示一直有效
        marker.lifetime.sec = lifetime_sec
        
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
