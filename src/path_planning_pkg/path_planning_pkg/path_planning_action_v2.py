import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.action._move_group import MoveGroup_Result
from moveit_msgs.action._move_group import MoveGroup_Feedback
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint, RobotTrajectory, CollisionObject, PlanningOptions, RobotState, AttachedCollisionObject
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from typing import List
import pyassimp
import yaml

class PathPlanningActionNode(Node):
    def __init__(self):
        super().__init__("path_planning_action")

        # 机器人配置文件名称
        self.declare_parameter('robot_config_file_name', 'robot_config.yaml')
        self.robot_config_file_name = self.get_parameter('robot_config_file_name').value

        package_share_directory = get_package_share_directory('path_planning_pkg')

        # 机器人配置
        self.robot_config_file_path = f"{package_share_directory}/config/{self.robot_config_file_name}"
        self.robot_conf = self.load_config(self.robot_config_file_path)

        # 最大规划尝试次数
        self.declare_parameter('num_planning_attempts', 5)
        self.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        self.declare_parameter('max_velocity_scaling_factor', 0.1)
        self.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor').value
        self.declare_parameter('max_acceleration_scaling_factor', 0.1)
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

        # arm 规划
        self._move_group_action_client = ActionClient(self, MoveGroup, f'/move_action')
        # cartesian_path规划
        self.cartesian_path_client = self.create_client(
            srv_type=GetCartesianPath,
            srv_name=f"/compute_cartesian_path",
        )

        # 初始化预定义状态
        self.init_states()

        # 轨迹执行
        self._execute_trajectory_action_client = ActionClient(
            self,
            action_type=ExecuteTrajectory,
            action_name=f"/execute_trajectory",
        )

        self.subscription_cmd = self.create_subscription(
            String,
            f"/path_planning_cmd",
            self.path_planning_cmd_callback,
            10,
        )

        # 规划轨迹发布
        self.robot_trajectory_publisher = self.create_publisher(RobotTrajectory, f'/robot_trajectory', 10)

        self.get_logger().info(f'path_planning_action node started')

    def path_planning_cmd_callback(self, msg: String):
        cmd_str = msg.data
        self.get_logger().info(f'receive plan cmd: {cmd_str}')
        request = json.loads(cmd_str)

        self.motion_plan(request)

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

    def motion_plan(self, request:dict):
        """运动规划"""
        cartesian_conf = request.get('cartesian', None)
        if cartesian_conf:
            self.plan_cartesian_path(request)
        else:
            self.move_to_configuration(request)

    def create_start_state(self, group_name, start_state_conf:dict):
        start_state = RobotState()
        if start_state_conf:
            # 开始位置
            state_name = start_state_conf.get('state_name', None)
            start_joint_states = start_state_conf.get('joint_states', None)
            if state_name:
                start_state.is_diff = False
                start_state.joint_state.name = self.joints[group_name]
                start_state.joint_state.position = self.get_state_value(group_name, state_name)
            elif start_joint_states:
                start_state.is_diff = False
                start_state.joint_state.name = self.joints[group_name]
                start_state.joint_state.position = start_joint_states
            else:
                start_state.is_diff = True

        return start_state

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
                            self.execute(trajectory_msg.joint_trajectory)
                        self.publish_robot_trajectory(trajectory_msg)
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
            robot_trajectory: RobotTrajectory = response.solution

            joint_trajectory_points: List[JointTrajectoryPoint] = robot_trajectory.joint_trajectory.points
            self.get_logger().info(f'轨迹点个数: {len(joint_trajectory_points)}')
            for joint_trajectory_point in joint_trajectory_points:
                self.get_logger().info(f'cartesian path joint_trajectory position: {joint_trajectory_point.positions}')
        
            return robot_trajectory, fraction
        
        return response.solution, fraction

    def make_cartesian_path_request(self, request:dict):
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
        # cartesian_path_request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        # cartesian_path_request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor

        return cartesian_path_request

    def move_to_configuration(self, request:dict):
        """关节空间规划"""
        group_name = request['planning_group']
        execute_trajectory = request.get('execute_trajectory', False)
        plan_request = MotionPlanRequest()

        # 开始状态
        start_state: RobotState = self.create_start_state(group_name, request)
        plan_request.start_state = start_state

        # 目标约束
        constraints = self.create_target_constraints(request)
        plan_request.goal_constraints = [
            constraints
        ]

        plan_request.group_name = group_name
        plan_request.num_planning_attempts = self.num_planning_attempts
        plan_request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        plan_request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor
        plan_request.allowed_planning_time = self.allowed_planning_time

        options: PlanningOptions = PlanningOptions()
        options.plan_only = not execute_trajectory

        self.do_planning(plan_request, options)

    def create_target_constraints(self, request:dict):
        """创建目标约束"""
        constraints = Constraints()
        constraints.joint_constraints = []

        group_name = request['planning_group']
        target_state_conf = request.get('target_state', None)

        state_name = target_state_conf.get('state_name', None)
        if state_name:
            joint_names = self.joints[group_name]
            joint_values = self.get_state_value(group_name, state_name)
            # 设置每个关节的目标值
            for i, name in enumerate(joint_names):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = name
                joint_constraint.position = joint_values[i]
                constraints.joint_constraints.append(joint_constraint)
            return constraints

        joint_states = target_state_conf.get('joint_states', None)
        if joint_states:
            joint_names = self.joints[group_name]
            # 设置每个关节的目标值
            for i, name in enumerate(joint_names):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = name
                joint_constraint.position = joint_states[i]
                constraints.joint_constraints.append(joint_constraint)
            return constraints
        
        pose_conf = target_state_conf.get('pose', None)
        pose = self.get_pose(pose_conf)
        if pose_conf:
            base_frame = pose_conf['base_frame']
            end_link = pose_conf['end_link']

            # 位置约束
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = base_frame
            position_constraint.link_name = end_link
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
            orientation_constraint.header.frame_id = base_frame
            orientation_constraint.link_name = end_link
            orientation_constraint.orientation = pose.orientation
            orientation_constraint.weight = 1.0
            ## 设置旋转偏差
            orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
            orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
            orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
            constraints.orientation_constraints.append(orientation_constraint)

            return constraints
        return constraints
    
    def get_pose(self, obj_pose:dict) -> Pose:
        if obj_pose:
            pose = Pose()
            pose.position.x = obj_pose['position']['x']
            pose.position.y = obj_pose['position']['y']
            pose.position.z = obj_pose['position']['z']
            pose.orientation.x = obj_pose['orientation']['x']
            pose.orientation.y = obj_pose['orientation']['y']
            pose.orientation.z = obj_pose['orientation']['z']
            pose.orientation.w = obj_pose['orientation']['w']

            return pose
        return None

    def do_planning(self, request: MotionPlanRequest, options: PlanningOptions):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options = options

        self.get_logger().info(f'Goal: {goal_msg}')

        self._move_group_action_client.wait_for_server()
        self._send_goal_future = self._move_group_action_client.send_goal_async(goal_msg, feedback_callback=self.move_group_feedback_callback)
        self._send_goal_future.add_done_callback(self.move_group_goal_response_callback)

    def move_group_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.move_group_get_result_callback)

    def move_group_get_result_callback(self, future):
        """action结果(任务规划结果)回调"""
        result: MoveGroup_Result = future.result().result
        self.get_logger().info(f'planning result, error code: {result.error_code}, planning time: {result.planning_time}')
        trajectory: RobotTrajectory = result.planned_trajectory
        joint_trajectory: JointTrajectory = trajectory.joint_trajectory
        joint_names = joint_trajectory.joint_names
        joint_trajectory_points = joint_trajectory.points
        self.get_logger().info(f'planning result, joint_names: {joint_names}, trajectory_point len: {len(joint_trajectory_points)}')
        for i, jtp in enumerate(joint_trajectory_points):
            self.get_logger().info(f'jtp {i}, position: {jtp}')

        self.publish_robot_trajectory(trajectory)

    def move_group_feedback_callback(self, feedback_msg):
        feedback:MoveGroup_Feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))

    def gripper_action(self, group_name, action, execute_trajectory=False):
        joint_state: JointState = JointState()
        self.get_logger().info(f'group_name: {group_name}, action: {action}, gripper_joint_state_cfg: {self.gripper_joint_state_cfg}')
        joint_state.name = self.joint_name_cfg[group_name]
        joint_state.position = self.gripper_joint_state_cfg[action]
        self.get_logger().info(f'joint_state: {joint_state}')
        self.move_to_configuration(group_name, joint_state, execute_trajectory=execute_trajectory)

    def execute(self, joint_trajectory: JointTrajectory):
        """
        Execute joint_trajectory by communicating directly with the controller.
        """
        execute_trajectory_goal = ExecuteTrajectory.Goal()

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
        feedback:MoveGroup_Feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))

    def publish_robot_trajectory(self, trajectory: RobotTrajectory):
        self.robot_trajectory_publisher.publish(trajectory)

    def create_mesh_msg(self, mesh_path, scale=(1.0, 1.0, 1.0)):
        try:
            # 使用 with 语句加载模型 (pyassimp 5.x+ 推荐方式)
            with pyassimp.load(mesh_path, processing=pyassimp.postprocess.aiProcess_Triangulate) as load_scene:
                # 创建 ROS Mesh 消息
                mesh_msg = Mesh()
                
                # 遍历 assimp 场景中的所有 mesh
                for a_mesh in load_scene.meshes:
                    # 处理面
                    for face in a_mesh.faces:
                        triangle = MeshTriangle()
                        
                        # 修正：兼容不同版本的 pyassimp 返回的数据结构
                        if hasattr(face, 'indices'):
                            indices = face.indices
                        else:
                            indices = face
                        
                        if len(indices) == 3:
                            triangle.vertex_indices = [int(indices[0]), int(indices[1]), int(indices[2])]
                            mesh_msg.triangles.append(triangle)
                        elif len(indices) == 4:
                            # 处理四边形
                            v0, v1, v2, v3 = int(indices[0]), int(indices[1]), int(indices[2]), int(indices[3])
                            tri1 = MeshTriangle()
                            tri1.vertex_indices = [v0, v1, v2]
                            mesh_msg.triangles.append(tri1)
                            tri2 = MeshTriangle()
                            tri2.vertex_indices = [v0, v2, v3]
                            mesh_msg.triangles.append(tri2)
                    
                    # 处理顶点位置
                    for vertex in a_mesh.vertices:
                        point = Point()
                        point.x = vertex[0] * scale[0]
                        point.y = vertex[1] * scale[1]
                        point.z = vertex[2] * scale[2]
                        mesh_msg.vertices.append(point)

                return mesh_msg
        except pyassimp.errors.AssimpError as e:
            self.get_logger().error(f'Assimp 加载错误: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'加载 mesh 失败: {mesh_path}, 错误: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathPlanningActionNode()
        rclpy.spin(node)
    except Exception as e:
        raise e
    finally:
        if rclpy.ok(): # 检查上下文是否还存活
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()