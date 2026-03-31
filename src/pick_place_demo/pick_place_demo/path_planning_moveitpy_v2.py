import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters, PlanningComponent
from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.action import ExecuteTrajectory
from trajectory_msgs.msg._joint_trajectory import JointTrajectory as JointTrajectoryMsg
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from moveit.core.controller_manager import ExecutionStatus
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import PoseStamped, Pose
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
import json
import yaml
from typing import List
from moveit_msgs.action._execute_trajectory import ExecuteTrajectory_Feedback

class PathPlanning(Node):
    """
    使用moveit python api实现路径规划
    """
    def __init__(self):
        super().__init__("path_planning_moveitpy")

        # 机器人配置文件名称
        self.declare_parameter('robot_config_file_name', 'robot_config.yaml')
        self.robot_config_file_name = self.get_parameter('robot_config_file_name').value
        
        package_share_directory = get_package_share_directory('pick_place_demo')
        self.mesh_root_path = f"{package_share_directory}/meshes"

        self.declare_parameter('max_velocity_scaling_factor', 0.1)
        self.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor').value
        self.declare_parameter('max_acceleration_scaling_factor', 0.1)
        self.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling_factor').value

        # 机器人配置
        self.robot_config_file_path = f"{package_share_directory}/config/{self.robot_config_file_name}"
        self.robot_conf = self.load_config(self.robot_config_file_path)

        moveit_cpp_file_path = f"{package_share_directory}/config/moveit_cpp.yaml"
        self.get_logger().info(f'moveit_cpp.yaml path: {moveit_cpp_file_path}')

        # 初始化机器人模型
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="panda", package_name="panda_moveit_config"
            ).moveit_cpp(moveit_cpp_file_path)
            .to_moveit_configs()
        )

        params = moveit_config.to_dict()
        self.robot = MoveItPy(node_name="moveit_py", config_dict=params)

        # 初始化预定义状态
        self.init_states()
        # 初始化规划组
        self.init_planning_components()

        # 笛卡尔路径规划(直线运动)
        self.cartesian_path_client = self.create_client(
            srv_type=GetCartesianPath,
            srv_name=f"/compute_cartesian_path",
        )
        # 轨迹执行
        self._execute_trajectory_action_client = ActionClient(
            self,
            action_type=ExecuteTrajectory,
            action_name=f"/execute_trajectory",
        )

        # 规划轨迹发布
        self.robot_trajectory_publisher = self.create_publisher(RobotTrajectoryMsg, '/robot_trajectory', 10)
        
        # 规划命令
        self.subscription_cmd = self.create_subscription(
            String,
            f"path_planning_cmd",
            self.path_planning_cmd_callback,
            10
        )

        self.get_logger().info("-------------path planning(moveitpy) node finish init-------------")

    def load_config(self, file_path):
        """加载 YAML 配置文件"""
        self.get_logger().info(f"Loading config from: {file_path}")
        try:
            with open(file_path, 'r') as f:
                config = yaml.safe_load(f)
                return config
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            raise e
    
    def init_states(self):
        self.states = {}
        self.joints = {}
        for planning_group in self.robot_conf['planning_groups']:
            pds_list = planning_group.get('states', None)
            if pds_list:
                self.states[planning_group['name']] = {}
                for pds in pds_list:
                    self.states[planning_group['name']][pds['name']] = pds.get('value', None)

            joints = planning_group.get('joints', None)
            if joints:
                self.joints[planning_group['name']] = joints

        self.get_logger().info(f'self.states: {self.states}')
        self.get_logger().info(f'self.joints: {self.joints}')

    def get_state_value(self, planning_group:str, state_name:str):
        planning_group_states = self.states.get(planning_group, None)
        if planning_group_states:
            return planning_group_states.get(state_name, None)
        return None
        
    def init_planning_components(self):
        """初始化规划组对象"""
        self.planning_components = { 
            planning_group['name']: self.robot.get_planning_component(planning_group['name']) 
            for planning_group in self.robot_conf['planning_groups']
        }

    def get_planning_component(self, group_name:str) -> PlanningComponent:
        """获取规划组对象
        :param group_name: 规划组名称
        """
        return self.planning_components[group_name]

    def motion_plan(self, request:dict):
        """运动规划"""
        cartesian_conf = request.get('cartesian', None)
        if cartesian_conf:
            self.plan_cartesian_path(request)
        else:
            self.move_to_configuration(request)

    def plan_cartesian_path(self, request:dict):
        """笛卡尔路径规划"""
        cartesian_path_request = self.make_cartesian_path_request(request)
        cartesian_path_future = self.cartesian_path_client.call_async(cartesian_path_request)

        def callback(future):
            try:
                response = future.result()
                if response is not None:
                    # trajectory_msg: RobotTrajectoryMsg
                    trajectory_msg, fraction = self.parse_cartesian_path_plan_result(response)
                    if trajectory_msg:
                        self.get_logger().info(f"---------cartesian path planning success, current fraction: {fraction}---------")
                        execute_trajectory = request.get('execute_trajectory', False)
                        if execute_trajectory:
                            self.execute_cartesian(trajectory_msg)
                        self.publish_trajectory(trajectory_msg)
                else:
                    self.get_logger().error('服务调用返回None')
            except Exception as e:
                self.get_logger().error(f'服务调用异常: {str(e)}')

        cartesian_path_future.add_done_callback(callback)

    def parse_cartesian_path_plan_result(self, response: GetCartesianPath.Response):
        """
        解析cartesian路径规划结果
        """
        fraction = response.fraction
        self.get_logger().info(f'路径计算完成度: {fraction:.2%}')

        if response.solution:
            # moveit_msgs/RobotTrajectory
            robot_trajectory: RobotTrajectoryMsg = response.solution

            joint_trajectory_points: List[JointTrajectoryPoint] = robot_trajectory.joint_trajectory.points
            self.get_logger().info(f'轨迹点个数: {len(joint_trajectory_points)}')
            for joint_trajectory_point in joint_trajectory_points:
                self.get_logger().info(f'cartesian path joint_trajectory position: {joint_trajectory_point.positions}')
        
            return robot_trajectory, fraction
        
        return response.solution, fraction

    def make_cartesian_path_request(self, request:dict) -> GetCartesianPath.Request:
        planning_group_name = request['planning_group']
        start_state_conf = request.get('start_state', None)
        target_state = request['target_state']
        cartesian_conf = request['cartesian']

        cartesian_path_request = GetCartesianPath.Request()

        # 起始位置
        if start_state_conf:
            state_name = start_state_conf.get('state_name', None)
            if state_name:
                start_joint_states = self.get_state_value(planning_group_name, state_name)
            else:
                start_joint_states = start_state_conf.get('joint_states')
            cartesian_path_request.start_state.is_diff = False
            cartesian_path_request.start_state.joint_state.name = self.joints[planning_group_name]
            cartesian_path_request.start_state.joint_state.position = start_joint_states
        else:
            cartesian_path_request.start_state.is_diff = True  # 使用当前状态作为起点
        
        # 目标位置
        pose = target_state['pose']
        cartesian_path_request.link_name = pose['end_link']
        cartesian_path_request.header.frame_id = pose['base_frame']
        target_pose = Pose()
        target_pose.position.x = pose['position']['x']
        target_pose.position.y = pose['position']['y']
        target_pose.position.z = pose['position']['z']
        target_pose.orientation.x = pose['orientation']['x']
        target_pose.orientation.y = pose['orientation']['y']
        target_pose.orientation.z = pose['orientation']['z']
        target_pose.orientation.w = pose['orientation']['w']
        cartesian_path_request.waypoints = [target_pose]

        cartesian_path_request.group_name = planning_group_name
        cartesian_path_request.max_step = cartesian_conf.get('max_step', 0.01)
        cartesian_path_request.jump_threshold = cartesian_conf.get('jump_threshold', 0.0)
        cartesian_path_request.avoid_collisions = cartesian_conf.get('avoid_collisions', True)
        cartesian_path_request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        cartesian_path_request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor

        return cartesian_path_request

    def _move_to_configuration(self, planning_component: PlanningComponent, execute_trajectory=False, end_link=None, publish_trajectory=True, max_try_times=5):
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
                self.robot, ["ompl_rrtc", "pilz_lin", "chomp", "ompl_rrt_star"]
        )

        trajectory_msg = None

        # initialise multi-pipeline plan request parameters
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
                self.robot, ["ompl_rrtc", "pilz_lin", "chomp", "ompl_rrt_star"]
        )

        plan_max_try_times = max_try_times
        while not trajectory_msg and plan_max_try_times > 0:
            plan_result:MotionPlanResponse = self.plan(planning_component, multi_plan_parameters=multi_pipeline_plan_request_params)
            trajectory_msg:RobotTrajectoryMsg = self.parse_plan_result(plan_result)
            plan_max_try_times -= 1
        
        if trajectory_msg:
            success = True

            if execute_trajectory:
                execute_success = False
                execute_max_try_times = max_try_times
                while not execute_success and execute_max_try_times > 0:
                    execute_success = self.execute(plan_result)
                    success = success and execute_success
                    execute_max_try_times -= 1

            if success and publish_trajectory:
                self.publish_trajectory(trajectory_msg)

            return success
        return False
    
    def move_to_configuration(self, request:dict):
        planning_component: PlanningComponent = self.get_planning_component(request['planning_group'])

        # start state
        self.set_start(planning_component, request)

        # target
        self.set_target(planning_component, request)

        execute_trajectory = request.get('execute_trajectory', False)
        success = self._move_to_configuration(planning_component, 
            max_try_times=5, publish_trajectory=self._time_source, execute_trajectory=execute_trajectory
        )

        return success
    
    def set_start(self, planning_component: PlanningComponent, request:dict):
        """设置开始状态"""
        start_state_conf = request.get('start_state', None)
        if start_state_conf:
            state_name = start_state_conf.get('state_name', None)
            if state_name:
                planning_component.set_start_state(configuration_name=state_name)
                return

            joint_states = start_state_conf.get('joint_states', None)
            if joint_states:
                start_state = RobotState(self.robot.get_robot_model())
                start_state.set_joint_group_positions(request['planning_group'], joint_states)
                planning_component.set_start_state(robot_state=start_state)
                return
        else:
            planning_component.set_start_state_to_current_state()
    
    def set_target(self, planning_component: PlanningComponent, request:dict):
        """设置目标"""
        target_state_conf = request.get('target_state', None)

        state_name = target_state_conf.get('state_name', None)
        if state_name:
            planning_component.set_goal_state(configuration_name=state_name)
            return

        joint_states = target_state_conf.get('joint_states', None)
        if joint_states:
            target_state = RobotState(self.robot.get_robot_model())
            target_state.set_joint_group_positions(request['planning_group'], joint_states)
            planning_component.set_goal_state(robot_state=target_state)
            return
        
        pose = target_state_conf.get('pose', None)
        if pose:
            base_frame = pose['base_frame']
            end_link = pose['end_link']

            target_pose = PoseStamped()
            target_pose.header.frame_id = base_frame
            target_pose.pose.position.x = pose['position']['x']
            target_pose.pose.position.y = pose['position']['y']
            target_pose.pose.position.z = pose['position']['z']
            target_pose.pose.orientation.x = pose['orientation']['x']
            target_pose.pose.orientation.y = pose['orientation']['y']
            target_pose.pose.orientation.z = pose['orientation']['z']
            target_pose.pose.orientation.w = pose['orientation']['w']
            planning_component.set_goal_state(pose_stamped_msg=target_pose, pose_link=end_link)

    def publish_trajectory(self, trajectory_msg: RobotTrajectoryMsg):
        """发布轨迹数据"""
        self.robot_trajectory_publisher.publish(trajectory_msg)

    def plan(self, planning_component:PlanningComponent, single_plan_parameters=None, multi_plan_parameters=None) -> MotionPlanResponse:
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
    
    def execute(self, plan_result: MotionPlanResponse):
        """轨迹执行"""
        if plan_result and plan_result.trajectory:
            self.get_logger().info("start executing trajectory...")
            robot_trajectory:RobotTrajectory = plan_result.trajectory
            execution_status: ExecutionStatus = self.robot.execute(robot_trajectory, controllers=[])
            # 根据执行状态判断执行是否成功
            self.get_logger().info(f"execute trajectory status: {execution_status.status}")
            if execution_status.status == 'SUCCEEDED':
                self.get_logger().info("execute trajectory success")
                return True
        
        self.get_logger().error("execute trajectory failed")
        return False
    
    def execute_cartesian(self, robot_trajectory:RobotTrajectoryMsg):
        """
        执行cartesian path轨迹
        """
        execute_trajectory_goal = ExecuteTrajectory.Goal()

        joint_trajectory: JointTrajectoryMsg = robot_trajectory.joint_trajectory
        execute_trajectory_goal.trajectory.joint_trajectory = joint_trajectory

        self._send_goal_future = self._execute_trajectory_action_client.send_goal_async(execute_trajectory_goal, feedback_callback=self.execute_trajectory_feedback_callback)
        self._send_goal_future.add_done_callback(self.execute_trajectory_goal_response_callback)

    def execute_trajectory_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.execute_trajectory_get_result_callback)

    def execute_trajectory_get_result_callback(self, future):
        """action结果(任务规划结果)回调"""
        result = future.result().result
        self.get_logger().info(f'execute result, error code: {result.error_code}')

    def execute_trajectory_feedback_callback(self, feedback_msg):
        feedback:ExecuteTrajectory_Feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))

    def plan_and_execute(self, planning_component, single_plan_parameters=None, multi_plan_parameters=None):
        """规划并执行"""
        plan_result = self.plan(planning_component, single_plan_parameters, multi_plan_parameters)
        
        result = self.execute(plan_result)
            
        time.sleep(1)

        return result
    
    def parse_plan_result(self, plan_result:MotionPlanResponse) -> RobotTrajectoryMsg:
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
                    # self.get_logger().info(f"## Step {i}, Joint positions: {joint_positions}, Time from start: {time_from_start}")
                # return joint_trajectory.points
                return trajectory_msg
            else:
                self.get_logger().warning(f"invalid trajectory type, actual type is {type(trajectory)}")
        else:
            self.get_logger().info("-------------Planning failed-------------")
        return None

    def path_planning_cmd_callback(self, msg: String):
        cmd_str = msg.data
        self.get_logger().info(f'receive plan cmd: {cmd_str}')
        request = json.loads(cmd_str)

        self.motion_plan(request)
        

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanning()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()