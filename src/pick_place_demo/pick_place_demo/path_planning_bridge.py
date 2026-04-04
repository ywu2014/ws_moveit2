import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
import json
from typing import List, Union
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PathPlanningTestNode(Node):
    def __init__(self):
        super().__init__("path_planning_bridge")

        self.subscription_cmd = self.create_subscription(
            String,
            f"/path_planning_bridge_cmd",
            self.path_planning_cmd_callback,
            10,
        )

        # 创建tf buffer和listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_planning_cmd_publisher = self.create_publisher(String, '/path_planning_cmd', 10)

        self.get_logger().info(f'path_planning_bridge node started')

    def path_planning_cmd_callback(self, msg: String):
        cmd_str = msg.data
        self.get_logger().info(f'receive bridge cmd: {cmd_str}')
        cmd = json.loads(cmd_str)
        bridge_type = cmd['bridge_type']
        if bridge_type == 'path_planning':
            # 路径规划桥接
            params = cmd['params']
            self.path_planning_bridge(params)

    def path_planning_bridge(self, params:dict):
        """
        路径规划桥接, 当target_state为pose时, 将target_frame名称表示的pose转换为具体Pose,
        然后再调用路径规划功能
        """
        target_state_conf = params['target_state']
        pose_conf:dict = target_state_conf.get('pose', None)
        if pose_conf:
            base_frame = pose_conf['base_frame']
            target_frame = pose_conf['target_frame']
            transform = self.get_transform(target_frame, base_frame)
            pose:Pose = self.convert_transform_to_pose(transform)

            pose_conf.pop('target_frame')
            pose_conf["position"] = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            }
            pose_conf["orientation"] = {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w
            }

        self.publish_cmd(params, self.path_planning_cmd_publisher)

    def publish_cmd(self, cmd, publisher):
        cmd_str = json.dumps(cmd)
        self.get_logger().info(f'publish cmd: {cmd_str}')
        msg = String()
        msg.data = cmd_str
        publisher.publish(msg)

    def get_transform(self, source_frame, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
            return transform
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {target_frame} to {source_frame}: {ex}')
            return None
        
    def convert_transform_to_pose(self, transform):
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation.x = transform.transform.rotation.x
        pose.orientation.y = transform.transform.rotation.y
        pose.orientation.z = transform.transform.rotation.z
        pose.orientation.w = transform.transform.rotation.w
        return pose

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathPlanningTestNode()
        rclpy.spin(node)
    except Exception as e:
        raise e
    finally:
        if rclpy.ok(): # 检查上下文是否还存活
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()