import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import glfw
from .mujoco_viewer import BaseViewer

class PickPlaceSim(Node, BaseViewer):
    """
    环境仿真
    """
    def __init__(self):
        Node.__init__(self, "simple_moveit2_demo")

        # 定义回调组
        self.parallel_group = ReentrantCallbackGroup()

        self.package_share_directory = get_package_share_directory('pick_place_demo')

        self.declare_parameter('mjcf_name', 'scene.xml')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.mjcf_name = self.get_parameter('mjcf_name').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value

        # 初始化mujoco环境
        model_path = f'{self.package_share_directory}/model/{self.mjcf_name}'
        self.get_logger().info(f"Load model: {model_path}")
        BaseViewer.__init__(self, model_path)

        self.subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            10,
            callback_group=self.parallel_group
        )

        # 定义关节名称列表
        self.arm_joint_names = [f"joint{i}" for i in range(1, 8)]
        self.gripper_joint_names = ["finger_joint1", "finger_joint2"]
        self.all_joint_names = self.arm_joint_names + self.gripper_joint_names
        self.joint_ids = {name: self.get_joint_id(name) for name in self.all_joint_names}
        
        # 初始化目标位置队列 (用于平滑控制)
        self.target_qpos = np.zeros(9)
        # 初始化目标为当前位置
        self.target_qpos[:] = self.data.qpos[:9]
        self.start()

        self.obj_to_broadcast_list = []
        self.tf_broadcaster = TransformBroadcaster(self)
        self._tf_timer = self.create_timer(0.01, self.publish_transform)

        self.add_obj_to_broadcast('track_link')


    def step_callback(self):
        # for i, v in enumerate(self.target_qpos):
        #     self.data.qpos[i] = v
        self.data.ctrl[:7] = self.target_qpos[:7]
        self.data.ctrl[7] = self.target_qpos[7]

    def joint_state_callback(self, msg: JointState):
        """
        接收 ROS 消息并更新 self.target_qpos
        """
        # self.get_logger().info(f"joint state msg: {msg}")
        # 遍历消息中的每个关节
        for ros_name, ros_pos in zip(msg.name, msg.position):
            # 检查是否是我们关心的关节
            joint_name = ros_name.replace('panda_', '')
            if joint_name in self.all_joint_names:
                if joint_name == 'finger_joint1':
                    # 夹爪
                    torque = self.gripper_joint_to_torque(ros_pos)
                    self.update_target_qpos('finger_joint1', torque)
                    self.update_target_qpos('finger_joint2', torque)
                else:
                    # 手臂
                    self.update_target_qpos(joint_name, ros_pos)

    def gripper_joint_to_torque(self, joint_value):
        torque = None
        if joint_value > 0.03:
            # for open
            torque = 100
        else:
            # for close
            torque = 0
        return torque

    def update_target_qpos(self, joint_name, value):
        # 获取该关节在 MuJoCo 模型中的 ID
        # mj_id = self.get_joint_id(joint_name)
        mj_id = self.joint_ids[joint_name]
        
        if mj_id != -1:
            # 更新目标位置
            self.target_qpos[mj_id] = value


    def timer_callback(self):
        # 进行动力学模拟
        self.step()

    def add_obj_to_broadcast(self, obj_name):
        self.obj_to_broadcast_list.append(obj_name)

    def remove_obj_from_broadcast(self, obj_name):
        self.obj_to_broadcast_list.remove(obj_name)

    def publish_transform(self):
        for obj_name in self.obj_to_broadcast_list:
            self._publish_transform(obj_name, child_frame_id=obj_name)

    def _publish_transform(self, obj_name, frame_id='world', child_frame_id=None):
        position, quat = self.get_body_pose(obj_name)

        t = TransformStamped()
        
        # 设置时间戳和坐标系名称
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id if child_frame_id else obj_name

        # 设置平移
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        
        # 设置旋转
        t.transform.rotation.x = float(quat[1])
        t.transform.rotation.y = float(quat[2])
        t.transform.rotation.z = float(quat[3])
        t.transform.rotation.w = float(quat[0])

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceSim()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    # 清理资源
    glfw.terminate()