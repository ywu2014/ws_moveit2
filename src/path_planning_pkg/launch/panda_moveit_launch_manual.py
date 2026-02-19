from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz(可视化机器人模型)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", get_package_share_directory('panda_moveit_config') + '/config/moveit.rviz'],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # 启动控制器管理器
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # 启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 启动手臂关节轨迹控制器
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # 启动夹爪关节轨迹控制器
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"],
    )

    # 路径规划节点
    path_planning_node = Node(
        package='path_planning_pkg',
        executable='path_planning_moveitpy',
        name='path_planning_moveitpy',
        output='both',
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            # 如果不需要可视化的话, run_move_group_node和rviz_node可以不用启动
            # run_move_group_node,
            # rviz_node,
            # 如果不启动ros2_control_node, 会报Could not contact service /controller_manager/list_controllers错误
            ros2_control_node,
            # 如果不启动arm_controller_spawner, 路径规划能成功, 但执行轨迹会失败, 报Action client not connected to action server: arm_controller/follow_joint_trajectory, execute trajectory status: ABORTED
            arm_controller_spawner,
            hand_controller_spawner,
            joint_state_broadcaster_spawner,
            path_planning_node
        ]
    )
