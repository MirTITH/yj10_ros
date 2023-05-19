# end_effector_servo

末端实时伺服

## 使用说明

### 启动

```sh
roslaunch end_effector_servo end_effector_servo.launch
```

### 使用方法

#### 末端速度伺服

向 `/end_effector_servo/end_effector_velocity` 连续发布末端速度指令即可

Tips: 可以执行以下命令打开 rqt：

```sh
roslaunch yj10_driver rqt_monitor.launch
```

#### 运行到给定位置

向 `/end_effector_servo/move_to` 发送要运动到的位置

在 yj10_moveit_config/config/yj10.srdf 中指定位置和对应的名称，然后发送位置的名称即可

注意：虽然这个功能带有碰撞规避，但不会考虑夹爪夹着的东西

例如：

```sh
rostopic pub /end_effector_servo/move_to std_msgs/String "data: 'fold'"
```

### 注意事项

必须连续发布末端速度，如果间隔时间太长，则会自动停止机械臂运动，保证安全。这个时间由 launch 文件中的 `max_topic_interval` 指定。

如果发布的值为全0，本软件将不会控制 joint_group_position_controller。此时你可以使用其他软件控制机械臂。

只有当 `joint_group_position_controller` 处于 `running` 状态下，才能控制机械臂。

如果同时使用 moveit，moveit 可能会自动将控制器切换为 `position_trajectory_controller`，使得本软件不再能控制机械臂

可以使用以下指令查看状态：

```sh
rosservice call /controller_manager/list_controllers
```

如果 `joint_group_position_controller` 不在 `running`，使用以下命令切换：

```sh
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_position_controller']
stop_controllers: ['position_trajectory_controller']
strictness: 1
start_asap: false
timeout: 0.0"
```
