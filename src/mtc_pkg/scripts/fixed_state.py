#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.core import planning_scene
from moveit.core.robot_state import RobotState
from moveit.core.robot_model import JointModelGroup
from moveit.task_constructor import core, stages
import rclcpp
from moveit.core.planning_scene import PlanningScene
import time
from moveit_msgs.msg import RobotState as RobotStateMsg  # 导入 ROS 消息类型

def print_solution(solution):
    print('-'*30)
    print(f"Solution Cost: {solution.cost}")
    print(f"Comment: {solution.comment}")

    # 1. 获取 InterfaceState
    start_interface = solution.start
    # 2. 通过 scene 属性获取 PlanningScene
    # 注意：这里需要判断 scene 是否存在，以防万一
    if hasattr(start_interface, 'scene') and start_interface.scene:
        planning_scene = start_interface.scene
        
        # # 2. 打印 planning_scene 的所有属性，寻找状态相关的属性
        # print("PlanningScene Attributes:")
        # print(dir(planning_scene))
        # print("-" * 20)

        if hasattr(planning_scene, 'current_state'):
            robot_state: RobotState = planning_scene.current_state
            
            print(f"Robot State Details: {robot_state}")

            # 1. 获取 RobotModel
            # 通常 RobotState 内部会持有 RobotModel 的引用
            if hasattr(robot_state, 'getRobotModel'):
                robot_model = robot_state.getRobotModel()
            elif hasattr(robot_state, 'robot_model'):
                robot_model = robot_state.robot_model
            else:
                print("Cannot find RobotModel in RobotState")
                return

            # 2. 获取 JointModelGroup
            jmg:JointModelGroup = robot_model.get_joint_model_group("panda_arm")
            print(f'jmg: {jmg.name}')
            
            # 获取关节名称
            joint_names = jmg.active_joint_model_names
            print(f'joint_names: {joint_names}')
            
            # 获取关节位置
            joint_positions = robot_state.get_joint_group_positions(jmg.name)

            print("\nJoint Positions (panda_arm):")
            for name, value in zip(joint_names, joint_positions):
                print(f"{name}: {value:.4f}")

    print('-'*30)

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# Create a task
task = core.Task()
task.name = "fixed state"

# [initAndConfigFixedState]
# Initialize a PlanningScene for use in a FixedState stage
task.loadRobotModel(node)  # load the robot model (usually done in init())

# 1. 直接创建 ROS 消息对象
robot_state_msg = RobotStateMsg()

# 2. 设置关节名称和位置
# 注意：关节名称必须与 URDF 中的定义完全一致
robot_state_msg.joint_state.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
robot_state_msg.joint_state.position = [0.0, -0.785, 0.0, -2.356, 1.0, 1.571, 0.785]

planningScene = PlanningScene(task.getRobotModel())

planningScene.set_current_state(robot_state_msg)

# Create a FixedState stage and pass the created PlanningScene as its state
# FixedState 是 MTC 中的一种特殊阶段。它不进行任何运动规划或计算，而是向任务流中提供一个预先设定好的、确定的状态作为起点（或中间点）。
# 它的作用是作为一个强制起点。它告诉 MTC：“不管机器人现在在哪里，也不管之前规划了什么，从这个阶段开始，机器人的状态必须是我给你的这个 PlanningScene 里的样子。”
# 通常用于测试特定轨迹的可行性，或者作为复杂任务（如抓取-放置）的一个中间检查点。例如，你可以先规划一个从 A 到 B 的动作，然后插入一个 FixedState 强制机器人在 B 点，再规划从 B 到 C 的动作，以确保两段动作的衔接是完美的。
fixedState = stages.FixedState("fixed state")
# 将上一步创建的空 PlanningScene 设置为该阶段的内部状态。这意味着这个阶段强制任务从这个特定的（这里是初始的）状态开始。
fixedState.setState(planningScene)

# Add the stage to the task hierarchy
task.add(fixedState)
# [initAndConfigFixedState]

if task.plan():
    print_solution(task.solutions[0])
    task.publish(task.solutions[0])

del planningScene  # Avoid ClassLoader warning by destroying the RobotModel
time.sleep(1)
