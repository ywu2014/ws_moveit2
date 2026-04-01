from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    robot_config_file_name = LaunchConfiguration('robot_config_file_name', default='robot_franka_config.yaml')
    mjcf_name = LaunchConfiguration("mjcf_name", default='scene_cube_pick_place.xml')

    # franka moveit 节点
    incl_franka_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('panda_moveit_config'), 'launch', 'demo.launch.py')
        ),
    )

    # 路径规划节点
    path_planning_node = Node(
        package="pick_place_demo",
        executable="path_planning",
        output='both',
        parameters=[{
            'robot_config_file_name': robot_config_file_name
        }]
    )

    # mujoco仿真节点
    sim_mujoco_node = Node(
        package="pick_place_demo",
        executable="sim_mujoco_pick_place",
        output='both',
        parameters=[{
            'mjcf_name': mjcf_name
        }]
    )

    return LaunchDescription(
        [
            incl_franka_robot,
            path_planning_node,
            sim_mujoco_node
        ]
    )
