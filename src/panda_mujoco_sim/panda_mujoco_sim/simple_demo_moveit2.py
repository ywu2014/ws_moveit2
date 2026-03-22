import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
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

        self.package_share_directory = get_package_share_directory('panda_mujoco_sim')

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

        self.positions = None

        # 设置定时器，每30ms触发一次回调函数（约33FPS）
        self.timer = self.create_timer(0.03, self.timer_callback)

        # self.start()
        # 2. 定义关节名称列表
        self.arm_joint_names = [f"joint{i}" for i in range(1, 8)]
        self.gripper_joint_names = ["finger_joint1", "finger_joint2"]
        self.all_joint_names = self.arm_joint_names + self.gripper_joint_names
        
        # 3. 初始化目标位置队列 (用于平滑控制)
        # 注意：self.data.qpos 的长度是 nq (自由度)，可能比 nu (执行器数) 大
        # 我们创建一个与 qpos 长度相同的数组来存储目标
        self.target_qpos = np.zeros(self.model.nq)
        # 初始化目标为当前位置
        self.target_qpos[:] = self.data.qpos[:]

        # PID 控制器参数 (P 增益)
        # 手臂需要较大的力矩，夹爪通常较小
        self.kp_arm = 150.0
        self.kp_gripper = 50.0

    # def step_callback(self):
    #     # 获取当前手臂关节的位置索引
    #     arm_ids = [self.get_joint_id(name) for name in self.arm_joint_names]
        
    #     # 简单的 P 控制器: Torque = Kp * (Target - Current)
    #     for i, joint_id in enumerate(arm_ids):
    #         error = self.target_qpos[joint_id] - self.data.qpos[joint_id]
    #         # 假设 actuator i 对应 joint i
    #         if i < self.model.nu: 
    #             self.data.ctrl[i] = self.kp_arm * error

    #     # 2. 计算夹爪误差并设置 ctrl
    #     gripper_ids = [self.get_joint_id(name) for name in self.gripper_joint_names]
        
    #     # 夹爪通常在 actuator 列表的后面 (例如索引 7)
    #     # 这里做一个简单的映射，假设夹爪执行器紧随手臂执行器之后
    #     # 实际项目中建议建立 actuator_name 到 joint_id 的映射字典
    #     gripper_actuator_start_idx = 7 
        
    #     for i, joint_id in enumerate(gripper_ids):
    #         error = self.target_qpos[joint_id] - self.data.qpos[joint_id]
    #         actuator_idx = gripper_actuator_start_idx + i
    #         if actuator_idx < self.model.nu:
    #             self.data.ctrl[actuator_idx] = self.kp_gripper * error
    def step_callback(self):
        for i, v in enumerate(self.target_qpos):
            self.data.qpos[i] = v

    # def joint_state_callback(self, msg: JointState):
    #     """
    #     接收 ROS 消息并更新 self.target_qpos
    #     """
    #     # self.get_logger().info(f"joint state msg: {msg}")
    #     # 遍历消息中的每个关节
    #     for ros_name, ros_pos in zip(msg.name, msg.position):
    #         # 检查是否是我们关心的关节
    #         joint_name = ros_name.replace('panda_', '')
    #         if joint_name in self.all_joint_names:
    #             # 获取该关节在 MuJoCo 模型中的 ID
    #             mj_id = self.get_joint_id(joint_name)
                
    #             if mj_id != -1:
    #                 # 更新目标位置
    #                 self.target_qpos[mj_id] = ros_pos
    #                 # self.get_logger().info(f"Updated {ros_name} (ID: {mj_id}) to {ros_pos}")
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
                mj_id = self.get_joint_id(joint_name)
                
                if mj_id != -1:
                    # 更新目标位置
                    self.target_qpos[mj_id] = ros_pos

    def timer_callback(self):
        self.step()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveit2Demo()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
        # spin_thread = threading.Thread(target=executor.spin, daemon=True)
        # spin_thread.start()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    # 清理资源
    glfw.terminate()