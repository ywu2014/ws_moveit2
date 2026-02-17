import time
import rclpy
from rclpy.node import Node

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters, PlanningComponent
from moveit.core.robot_trajectory import RobotTrajectory
from trajectory_msgs.msg._joint_trajectory import JointTrajectory as JointTrajectoryMsg
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from moveit.core.controller_manager import ExecutionStatus
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
import json

class PathPlanningMoveitPy(Node):
    """
    使用muveit python api实现路径规划
    """
    def __init__(self):
        super().__init__("path_planning")

        # 机器人配置
        package_share_directory = get_package_share_directory('path_planning_pkg')
        moveit_cpp_file_path = f"{package_share_directory}/config/moveit_cpp.yaml"
        self.get_logger().info(f'moveit_cpp.yaml path: {moveit_cpp_file_path}')

        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="panda", package_name="panda_moveit_config"
            ).moveit_cpp(moveit_cpp_file_path)
            .to_moveit_configs()
        )

        params = moveit_config.to_dict()
        self.robot = MoveItPy(node_name="moveit_py", config_dict=params)

        # 获取规划组
        self.arm_group: PlanningComponent = self.robot.get_planning_component('arm')
        self.hand_group: PlanningComponent = self.robot.get_planning_component('hand')

        # 规划轨迹发布
        self.robot_trajectory_states_publisher = self.create_publisher(RobotTrajectoryMsg, '/robot_trajectory_states', 10)
        
        # 规划命令
        self.subscription_cmd = self.create_subscription(
            String,
            "/path_planning_cmd",
            self.path_planning_cmd_callback,
            10
        )

        self.get_logger().info("-------------Planning Node Finish Init-------------")

    def move_to_pose(
        self, 
        frame_id:str, end_link:str, goal_pose: Pose,
        start_state:RobotState=None, max_try_times=5, publish_trajectory=True, execute=True
    ):
        """
        规划到目标位姿
        
        :param frame_id: pose相对的坐标系
        :param end_link: 移动到目标位姿的link
        :param goal_pose: link移动到的目标位姿
        :param start_state: 开始位置
        :param max_try_times: 重试次数
        :param publish_trajectory: 是否发布轨迹
        :param execute: 是否执行轨迹
        """
        goal_pose_stamped = None
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header.frame_id = frame_id # 表示末端位姿是在哪个参考坐标系下定义的, 通常是机器人底座或世界坐标系
        goal_pose_stamped.pose = goal_pose

        return self._motion_plan(
            self.arm_group, arm_group_name='arm',
            goal_pose=goal_pose_stamped, arm_end_link=end_link,
            start_state=start_state, max_try_times=max_try_times, publish_trajectory=publish_trajectory, execute=execute
        )
    
    def move_to_joint_state(
        self, 
        goal_state:RobotState=None,
        start_state:RobotState=None, max_try_times=5, publish_trajectory=True, execute=True
    ):
        """
        规划到目标关节状态
        
        :param goal_state: 目标关节状态
        :param start_state: 开始状态
        :param max_try_times: 重试次数
        :param publish_trajectory: 是否发布轨迹
        :param execute: 是否执行轨迹
        """
        return self._motion_plan(
            self.arm_group, arm_group_name='arm',
            goal_state=goal_state,
            start_state=start_state, max_try_times=max_try_times, publish_trajectory=publish_trajectory, execute=execute
        )

    def _motion_plan(
            self, 
            arm_group: PlanningComponent, arm_group_name: str = None,
            goal_state:RobotState=None, goal_pose: PoseStamped=None, arm_end_link:str=None,
            start_state:RobotState=None, max_try_times=5, publish_trajectory=True, execute=True
    ):
        """
        运动规划
        
        :param arm_group: 规划组
        :param arm_group_name: 规划组名称
        :param goal_pose: 目标位姿, 需要和arm_end_link一起使用
        :param goal_state: 目标关节状态
        :param arm_end_link: end link
        :param start_state: 开始关节状态
        :param max_try_times: 重试次数
        :param publish_trajectory: 规划成功后是否发布轨迹数据
        :param execute: 规划成功后是否执行轨迹
        """
        self.get_logger().info(f'arm_group type: {type(arm_group)}, arm_group: {arm_group}')

        # 设置起点位置
        if start_state is None:
            arm_group.set_start_state_to_current_state()
            self.get_logger().info(f'{arm_group_name} start joint motion plan from current state')
        else:
            arm_group.set_start_state(robot_state=start_state)
            self.get_logger().info(f'{arm_group_name} start joint motion plan from start state: {start_state}')

        # 设置目标位置
        if goal_pose is not None:
            if arm_end_link is None:
                raise ValueError("arm_end_link cannot be None when goal_pose is not None")
            # pose_link表示希望机器人的哪一部分到达pose_stamped_msg表示的位姿
            arm_group.set_goal_state(pose_stamped_msg=goal_pose, pose_link=arm_end_link) # 表示希望最终到达目标位置姿态
            self.get_logger().info(f'{arm_group_name} start joint motion plan to goal pose: {goal_pose}, end link: {arm_end_link}')
        elif goal_state is not None:
            arm_group.set_goal_state(robot_state=goal_state)
            self.get_logger().info(f'{arm_group_name} start joint motion plan to goal state: {goal_state}')
        else:
            raise ValueError("goal_pose and goal_state cannot be None at the same time")

        trajectory_msg = None

        # initialise multi-pipeline plan request parameters
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
                self.robot, ["ompl_rrtc", "pilz_lin", "chomp", "ompl_rrt_star"]
        )

        plan_max_try_times = max_try_times
        while not trajectory_msg and plan_max_try_times > 0:
            self.get_logger().info(f"---------{arm_group_name} planning, current retry times {plan_max_try_times}---------")
            plan_result = self.do_plan(arm_group, multi_plan_parameters=multi_pipeline_plan_request_params)
            trajectory_msg = self.parse_plan_result(plan_result)
            plan_max_try_times -= 1
        
        if trajectory_msg:
            success = True

            if execute:
                execute_success = False
                execute_max_try_times = max_try_times
                while not execute_success and execute_max_try_times > 0:
                    self.get_logger().info(f"---------{arm_group_name} executing trajectory data..., current retry times {execute_max_try_times}---------")
                    execute_success = self.execute(plan_result)
                    success = success and execute_success
                    execute_max_try_times -= 1

            if success and publish_trajectory:
                self.get_logger().info(f"---------{arm_group_name} publishing trajectory data...---------")
                self.publish_trajectory(trajectory_msg, arm_group_name)

            return success, trajectory_msg
        return False, trajectory_msg

    def do_plan(self, planning_component:PlanningComponent, single_plan_parameters=None, multi_plan_parameters:MultiPipelinePlanRequestParameters=None):
        """
        路径规划
        """
        self.get_logger().info("Planning trajectory")
        if multi_plan_parameters:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()
        
        return plan_result
    
    def execute(self, plan_result):
        """轨迹执行"""
        if plan_result and plan_result.trajectory:
            self.get_logger().info("start executing trajectory...")
            robot_trajectory = plan_result.trajectory
            execution_status: ExecutionStatus = self.robot.execute(robot_trajectory, controllers=[])
            # 根据执行状态判断执行是否成功
            self.get_logger().info(f"execute trajectory status: {execution_status.status}")
            if execution_status.status == 'SUCCEEDED':
                self.get_logger().info("execute trajectory success")
                return True
        
        self.get_logger().error("execute trajectory failed")
        return False

    def plan_and_execute(self, planning_component, single_plan_parameters=None, multi_plan_parameters=None):
        """规划并执行"""
        plan_result = self.do_plan(planning_component, single_plan_parameters, multi_plan_parameters)
        
        result = self.execute(plan_result)
            
        return result
    
    def parse_plan_result(self, plan_result):
        """
        解析路径规划结果
        """
        if plan_result:
            self.get_logger().info("-------------Planning successful-------------")
            # 获取规划好的轨迹
            trajectory = plan_result.trajectory

            # 检查轨迹类型
            if isinstance(trajectory, RobotTrajectory):
                trajectory_msg: RobotTrajectoryMsg = trajectory.get_robot_trajectory_msg()
                # 获取关节轨迹
                joint_trajectory = trajectory_msg.joint_trajectory
                # 获取路径点数量
                num_waypoints = len(joint_trajectory.points)
                self.get_logger().info(f"## Number of waypoints: {num_waypoints}")
                self.get_logger().info(f"## joint_names: {joint_trajectory.joint_names}")

                # 遍历所有路径点
                for i in range(num_waypoints):
                    # 获取第i个路径点
                    waypoint = joint_trajectory.points[i]

                    # 获取关节位置
                    joint_positions = waypoint.positions
                    
                    # # 获取末端执行器位姿
                    # end_effector_pose = waypoint.effort
                    
                    # 获取时间信息
                    time_from_start = waypoint.time_from_start
                    self.get_logger().info(f"## Step {i}, Joint positions: {joint_positions}, Time from start: {time_from_start}")
                return trajectory_msg
            else:
                self.get_logger().warning(f"invalid trajectory type, actual type is {type(trajectory)}")
        else:
            self.get_logger().info("-------------Planning failed-------------")
        return None
    
    def publish_trajectory(self, trajectory_msg: RobotTrajectoryMsg, arm):
        """发布轨迹数据"""
        self.get_logger().info(f'######## 发布 {arm} 轨迹数据 ########')
        self.robot_trajectory_states_publisher.publish(trajectory_msg)

    def path_planning_cmd_callback(self, msg: String):
        cmd_str = msg.data
        self.get_logger().info(f'receive plan cmd: {cmd_str}')
        cmd = json.loads(cmd_str)
        cmd_type = cmd['planning_type']

        if cmd_type == 'move_to_joint_state':    # 关节空间规划
            group_name = cmd['group_name']
            joint_name = cmd['joint_name']
            joint_position = cmd['joint_position']

            joint_state = JointState()
            joint_state.name = joint_name
            joint_state.position = joint_position
            
            robot_model = self.robot.get_robot_model()

            goal_state = RobotState(robot_model)
            goal_state.set_joint_group_positions('arm', joint_position)

            return self.move_to_joint_state(goal_state=goal_state, publish_trajectory=False)
        elif cmd_type == 'move_to_pose':   # 笛卡尔空间规划
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
            self.move_to_pose(frame_id=base_frame, end_link=end_frame, goal_pose=pose, publish_trajectory=False)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningMoveitPy()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()