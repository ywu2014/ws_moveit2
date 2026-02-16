import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint, RobotTrajectory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
from moveit_msgs.action._move_group import MoveGroup_Result
from moveit_msgs.action._move_group import MoveGroup_Feedback
from trajectory_msgs.msg import JointTrajectory
from shape_msgs.msg import SolidPrimitive

class PathPlanningActionNode(Node):
    def __init__(self):
        super().__init__("path_planning_action_node")

        # 最大规划尝试次数
        self.declare_parameter('num_planning_attempts', 5)
        self.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        # 速度、加速度伸缩因子
        self.declare_parameter('max_velocity_scaling_factor', 1.0)
        self.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor').value
        self.declare_parameter('max_acceleration_scaling_factor', 1.0)
        self.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling_factor').value
        # 最大规划时间
        self.declare_parameter('allowed_planning_time', 10.0)
        self.allowed_planning_time = self.get_parameter('allowed_planning_time').value
        # 位置偏差(米)
        self.declare_parameter('position_tolerance', 0.01)
        self.position_tolerance = self.get_parameter('position_tolerance').value
        # 旋转偏差(弧度)
        self.declare_parameter('orientation_tolerance', 0.08)
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        # 关节偏差(弧度)
        self.declare_parameter('joint_tolerance', 0.08)
        self.joint_tolerance = self.get_parameter('joint_tolerance').value

        # 创建move_action的客户端
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        self.subscription_cmd = self.create_subscription(
            String,
            "/path_planning_cmd",
            self.path_planning_cmd_callback,
            10,
        )

        self.get_logger().info(f'path planning action node already start')


    def move_to_pose(self, group_name:str, pose_frame_id:str, link_name:str, pose: Pose):
        """
        设置目标姿态规划
        
        :param group_name: 规划组名称
        :param pose_frame_id: pose相对的坐标系名称
        :param link_name: 到达pose位姿的连杆
        :param pose: 连杆到达的位姿
        """
        request = MotionPlanRequest()

        constraints = Constraints()

        constraints.position_constraints = []

        # 位置约束
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = pose_frame_id
        position_constraint.link_name = link_name
        position_constraint.constraint_region.primitive_poses = [
            pose
        ]
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        ## 设置位置偏差, 数值越小精度越高
        sphere.dimensions = [self.position_tolerance]
        position_constraint.constraint_region.primitives = [sphere]
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        # 旋转约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = pose_frame_id
        orientation_constraint.link_name = link_name
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.weight = 1.0
        ## 设置旋转偏差
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        constraints.orientation_constraints.append(orientation_constraint)

        request.num_planning_attempts = self.num_planning_attempts
        request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor
        # is_diff=True表示start_state中没有包含所有关节的具体角度值,规划器会去获取机器人当前的实际状态,这意味着机器人会从“它现在所在的地方”开始规划路径移动到目标点
        request.start_state.is_diff = True
        request.allowed_planning_time = self.allowed_planning_time
        request.group_name = group_name
        request.goal_constraints = [
            constraints
        ]

        self.do_planning(request)

    def move_to_joint_state(self, group_name, joint_state: JointState):
        """
        设置目标关节状态规划
        
        :param group_name: 规划组名称
        :param joint_state: 关节状态
        """
        request = MotionPlanRequest()

        constraints = Constraints()

        constraints.joint_constraints = []

        # 设置每个关节的目标值
        for i, name in enumerate(joint_state.name):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = joint_state.position[i]
            # 设置偏差
            joint_constraint.tolerance_above = self.joint_tolerance
            joint_constraint.tolerance_below = self.joint_tolerance
            constraints.joint_constraints.append(joint_constraint)

        request.group_name = group_name
        request.num_planning_attempts = self.num_planning_attempts
        request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor
        request.start_state.is_diff = False
        request.allowed_planning_time = self.allowed_planning_time
        request.goal_constraints = [
            constraints
        ]

        self.do_planning(request)

    def do_planning(self, request: MotionPlanRequest):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request

        self.get_logger().info(f'Goal: {goal_msg}')

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """action结果(任务规划结果)回调"""
        result: MoveGroup_Result = future.result().result
        self.get_logger().info(f'planning result, error code: {result.error_code}, planning time: {result.planning_time}')
        
        trajectory: RobotTrajectory = result.planned_trajectory
        joint_trajectory: JointTrajectory = trajectory.joint_trajectory
        
        # 获取关节名称
        joint_names = joint_trajectory.joint_names
        # 获取轨迹点
        joint_trajectory_points = joint_trajectory.points
        self.get_logger().info(f'planning result, joint_names: {joint_names}, trajectory_point len: {len(joint_trajectory_points)}')
        for i, jtp in enumerate(joint_trajectory_points):
            self.get_logger().info(f'jtp {i}, position: {jtp}')

    def feedback_callback(self, feedback_msg):
        """实时反馈回调"""
        feedback:MoveGroup_Feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))

    def path_planning_cmd_callback(self, msg: String):
        cmd_str = msg.data
        self.get_logger().info(f'receive plan cmd: {cmd_str}')
        cmd = json.loads(cmd_str)
        cmd_type = cmd['planning_type']

        if cmd_type == 'move_to_joint_state':    # 目标关节状态
            group_name = cmd['group_name']
            joint_name = cmd['joint_name']
            joint_position = cmd['joint_position']

            joint_state = JointState()
            joint_state.name = joint_name
            joint_state.position = joint_position
            self.move_to_joint_state(group_name, joint_state)
        elif cmd_type == 'move_to_pose':   # 目标位姿
            group_name = cmd['group_name']
            base_frame = cmd['base_frame']
            end_frame = cmd['end_frame']
            end_pose = cmd['pose']

            pose = Pose()
            pose.position.x = end_pose['x']
            pose.position.y = end_pose['y']
            pose.position.z = end_pose['z']
            pose.orientation.x = end_pose['ox']
            pose.orientation.y = end_pose['oy']
            pose.orientation.z = end_pose['oz']
            pose.orientation.w = end_pose['ow']
            self.move_to_pose(group_name, base_frame, end_frame, pose)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathPlanningActionNode()
        rclpy.spin(node)
    except Exception as e:
        raise e
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()