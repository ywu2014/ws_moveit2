import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import yaml

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')

        package_share_directory = get_package_share_directory('path_planning_pkg')
        self.config_path = f"{package_share_directory}/config/markers_config.yaml"

        # 加载 YAML 配置
        self.markers_config = self.load_config(self.config_path)

        # marker topic名称
        self.declare_parameter('marker_topic', '/visualization_marker')
        self.marker_topic = self.get_parameter('marker_topic').value

        self.publisher_ = self.create_publisher(Marker, self.marker_topic, 10)
        # 创建 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.publish_markers) # 每秒发布一次

    def load_config(self, file_path):
        """加载 YAML 配置文件"""
        self.get_logger().info(f"Loading config from: {file_path}")
        try:
            with open(file_path, 'r') as f:
                config = yaml.safe_load(f)
                return config.get('markers', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            return []

    def publish_markers(self):
        """遍历配置列表, 发布所有 Marker"""
        if not self.markers_config:
            return

        current_time = self.get_clock().now().to_msg()

        for marker_config in self.markers_config:
            # 1. 发布 Marker
            self.publish_single_marker(marker_config, current_time)
            
            # 2. 如果配置了发布 TF，则发布
            if marker_config.get('publish_tf', False):
                self.publish_single_tf(marker_config, current_time)

    def publish_single_marker(self, config, stamp):
        """根据单个配置发布 Marker"""
        marker = Marker()
        marker.header.frame_id = config['frame_id']
        marker.header.stamp = stamp
        marker.ns = config['ns']
        marker.id = config['id']
        marker.type = config['type']
        marker.action = config['action']
        
        # Pose
        marker.pose.position.x = config['pose']['position']['x']
        marker.pose.position.y = config['pose']['position']['y']
        marker.pose.position.z = config['pose']['position']['z']
        marker.pose.orientation.x = config['pose']['orientation']['x']
        marker.pose.orientation.y = config['pose']['orientation']['y']
        marker.pose.orientation.z = config['pose']['orientation']['z']
        marker.pose.orientation.w = config['pose']['orientation']['w']
        
        # Scale
        marker.scale.x = config['scale']['x']
        marker.scale.y = config['scale']['y']
        marker.scale.z = config['scale']['z']
        
        # Color
        marker.color.r = config['color']['r']
        marker.color.g = config['color']['g']
        marker.color.b = config['color']['b']
        marker.color.a = config['color']['a']
        
        # Lifetime
        lifetime_sec = config.get('lifetime_sec', 0)
        marker.lifetime.sec = lifetime_sec
        
        self.publisher_.publish(marker)

    def publish_single_tf(self, config, stamp):
        """根据单个配置发布 TF"""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = config['frame_id']
        t.child_frame_id = config['tf_frame_name']
        
        # Translation
        t.transform.translation.x = config['pose']['position']['x']
        t.transform.translation.y = config['pose']['position']['y']
        t.transform.translation.z = config['pose']['position']['z']
        
        # Rotation
        t.transform.rotation.x = config['pose']['orientation']['x']
        t.transform.rotation.y = config['pose']['orientation']['y']
        t.transform.rotation.z = config['pose']['orientation']['z']
        t.transform.rotation.w = config['pose']['orientation']['w']
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
