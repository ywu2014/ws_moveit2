from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 机器人结点(有了这个结点后就不需要再单独起动robot_state_publisher、joint_state_publisher、rviz了)
    incl_panda_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('panda_moveit_config'), 'launch', 'demo.launch.py')
        ),
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
            incl_panda_robot,
            path_planning_node
        ]
    )
