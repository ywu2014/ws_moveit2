# track_link pick place demo
## 启动节点
```bash
ros2 launch pick_place_demo track_link_pick_place.launch.py
```

## 测试流程
### 到达预抓取点
```bash
ros2 topic pub --once path_planning_bridge_cmd std_msgs/msg/String 'data: "{\"bridge_type\":\"path_planning\",\"params\":{\"planning_group\":\"arm\",\"target_state\":{\"pose\":{\"base_frame\":\"world\",\"target_frame\":\"pre_grasp_pt_1\",\"end_link\":\"panda_hand_tcp\"}},\"execute_trajectory\":true}}
"'
```

### 打开夹爪
```bash
ros2 topic pub --once /path_planning_bridge_cmd std_msgs/msg/String 'data: "{\"bridge_type\":\"path_planning\",\"params\":{\"planning_group\":\"hand\",\"target_state\":{\"state_name\":\"open\"},\"execute_trajectory\":true}}"'
```

### 到达抓取点
```bash
ros2 topic pub --once /path_planning_bridge_cmd std_msgs/msg/String 'data: "{\"bridge_type\":\"path_planning\",\"params\":{\"planning_group\":\"arm\",\"target_state\":{\"pose\":{\"base_frame\":\"world\",\"target_frame\":\"grasp_pt_1\",\"end_link\":\"panda_hand_tcp\"}},\"cartesian\":{\"max_step\":0.01,\"jump_threshold\":0.0,\"avoid_collisions\":true},\"execute_trajectory\":true}}"'
```

### 关闭夹爪
```bash
ros2 topic pub --once /path_planning_bridge_cmd std_msgs/msg/String 'data: "{\"bridge_type\":\"path_planning\",\"params\":{\"planning_group\":\"hand\",\"target_state\":{\"state_name\":\"close\"},\"execute_trajectory\":true}}"'
```

### 抬起来
```bash
ros2 topic pub --once /path_planning_bridge_cmd std_msgs/msg/String 'data: "{\"bridge_type\":\"path_planning\",\"params\":{\"planning_group\":\"arm\",\"target_state\":{\"pose\":{\"position\":{\"x\":0.3,\"y\":0.0,\"z\":0.4},\"orientation\":{\"x\":1.0,\"y\":0.0,\"z\":0.0,\"w\":0.0},\"base_frame\":\"world\",\"end_link\":\"panda_hand_tcp\"}},\"cartesian\":{\"max_step\":0.01,\"jump_threshold\":0.0,\"avoid_collisions\":true},\"execute_trajectory\":true}}"'
```

### 移动到放置位置
```bash
ros2 topic pub --once /path_planning_bridge_cmd std_msgs/msg/String 'data: "{\"bridge_type\":\"path_planning\",\"params\":{\"planning_group\":\"arm\",\"target_state\":{\"pose\":{\"position\":{\"x\":0.0,\"y\":0.3,\"z\":0.4},\"orientation\":{\"x\":1.0,\"y\":0.0,\"z\":0.0,\"w\":0.0},\"base_frame\":\"world\",\"end_link\":\"panda_hand_tcp\"}},\"cartesian\":{\"max_step\":0.01,\"jump_threshold\":0.0,\"avoid_collisions\":true},\"execute_trajectory\":true}}"'
```

### 打开夹爪
```bash
ros2 topic pub --once /path_planning_bridge_cmd std_msgs/msg/String 'data: "{\"bridge_type\":\"path_planning\",\"params\":{\"planning_group\":\"hand\",\"target_state\":{\"state_name\":\"open\"},\"execute_trajectory\":true}}"'
```