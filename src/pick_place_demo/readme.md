# cube pick place demo
## 启动节点
### moveit节点
```bash
ros2 launch panda_moveit_config demo.launch.py
```

### 路径规划启动
```bash
ros2 run pick_place_demo path_planning --ros-args -p robot_config_file_name:=robot_franka_config.yaml
```

### 仿真启动
```bash
ros2 run pick_place_demo sim_mujoco_pick_place --ros-args -p mjcf_name:=scene_cube_pick_place.xml
```

### 一次性启动
```bash
ros2 launch pick_place_demo simple_cube_pick_place.launch.py
```

## 测试流程
### 打开夹爪
```bash
ros2 topic pub --once /path_planning_cmd std_msgs/msg/String 'data: "{\"planning_group\":\"hand\",\"target_state\":{\"state_name\":\"open\"},\"execute_trajectory\":true}"'
```

### 移动到方块位置
```bash
ros2 topic pub --once path_planning_cmd std_msgs/msg/String 'data: "{\"planning_group\":\"arm\",\"target_state\":{\"pose\":{\"position\":{\"x\":0.5,\"y\":0.0,\"z\":0.02},\"orientation\":{\"x\":1.0,\"y\":0.0,\"z\":0.0,\"w\":0.0},\"base_frame\":\"world\",\"end_link\":\"panda_hand_tcp\"}},\"execute_trajectory\":true}"'
```

### 关闭夹爪
```bash
ros2 topic pub --once /path_planning_cmd std_msgs/msg/String 'data: "{\"planning_group\":\"hand\",\"target_state\":{\"state_name\":\"close\"},\"execute_trajectory\":true}"'
```

### 移动到放置位置
```bash
ros2 topic pub --once path_planning_cmd std_msgs/msg/String 'data: "{\"planning_group\":\"arm\",\"target_state\":{\"pose\":{\"position\":{\"x\":0.3,\"y\":0.3,\"z\":0.2},\"orientation\":{\"x\":1.0,\"y\":0.0,\"z\":0.0,\"w\":0.0},\"base_frame\":\"world\",\"end_link\":\"panda_hand_tcp\"}},\"execute_trajectory\":true}"'
```

### 打开夹爪
```bash
ros2 topic pub --once /path_planning_cmd std_msgs/msg/String 'data: "{\"planning_group\":\"hand\",\"target_state\":{\"state_name\":\"open\"},\"execute_trajectory\":true}"'
```