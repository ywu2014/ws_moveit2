import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import glfw
from .mujoco_viewer import BaseViewer

class SimpleMoveit2Demo(Node, BaseViewer):
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
        # 注意：self.data.qpos 的长度是 nq (自由度)
        # 创建一个与 qpos 长度相同的数组来存储目标
        self.target_qpos = np.zeros(9)
        # 初始化目标为当前位置
        self.target_qpos[:] = self.data.qpos[:9]
        self.start()


    def step_callback(self):
        for i, v in enumerate(self.target_qpos):
            self.data.qpos[i] = v

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
                # 获取该关节在 MuJoCo 模型中的 ID
                # mj_id = self.get_joint_id(joint_name)
                mj_id = self.joint_ids[joint_name]
                
                if mj_id != -1:
                    # 更新目标位置
                    self.target_qpos[mj_id] = ros_pos

    def timer_callback(self):
        # 进行动力学模拟
        self.step()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveit2Demo()

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