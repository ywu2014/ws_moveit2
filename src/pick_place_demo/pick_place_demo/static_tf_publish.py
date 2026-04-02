import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import yaml

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')

        package_share_directory = get_package_share_directory('pick_place_demo')
        self.tf_config_path = f"{package_share_directory}/config/static_tf.yaml"
        
        self.publish_transform()
        self.get_logger().info(f"static_tf_publisher node started")

    def load_config(self, file_path):
        """加载 YAML 配置文件"""
        try:
            with open(file_path, 'r') as f:
                config = yaml.safe_load(f)
                return config.get('tfs', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            raise e
        
    def publish_transform(self):
        """
        读取配置文件, 批量广播静态tf转换
        """
        tfs = self.load_config(self.tf_config_path)
        for tf in tfs:
            frame_id = tf['frame_id']
            child_frame_id = tf['child_frame_id']
            self.get_logger().info(f"publish static tf, frame_id: {frame_id}, child_frame_id: {child_frame_id}, translation: {tf['translation']}, rotation: {tf['rotation']}")
            broadcaster = StaticTransformBroadcaster(self)
            self.publish_single_transform(broadcaster, tf['translation'], tf['rotation'], frame_id, child_frame_id)

    def publish_single_transform(self, broadcaster:StaticTransformBroadcaster, translation, rotation, frame_id, child_frame_id):
        """
        广播单个静态tf转换
        """
        t = TransformStamped()
        
        # 设置时间戳和坐标系名称
        t.header.stamp = Time(sec=0, nanosec=0)
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        
        # 设置平移
        t.transform.translation.x = float(translation['x'])
        t.transform.translation.y = float(translation['y'])
        t.transform.translation.z = float(translation['z'])
        
        # 设置旋转
        t.transform.rotation.x = float(rotation['x'])
        t.transform.rotation.y = float(rotation['y'])
        t.transform.rotation.z = float(rotation['z'])
        t.transform.rotation.w = float(rotation['w'])

        broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
