# yj10_driver

YJ10 机械臂的驱动，负责与实体机械臂通信。

## 使用说明

### yj10_control_HW.launch

连接实体机械臂，启动 ROS Controller 以接收关节命令，发布关节信息，启动夹持器控制相关的服务

#### 配置文件

- config/yj10_driver.yaml

在该文件中配置串口设备、是否开启假连接和初始位姿。

> 假连接打开后，程序将不会真正地连接机械臂，它会向 ROS 返回假数据，方便无机械臂调试。

- config/yj10_controllers.yaml

在该文件中配置与机械臂的通信频率，建议不超过 10 Hz

#### 启动 driver，连接机械臂

```bash
roslaunch yj10_driver yj10_control_HW.launch
```

##### Topic

在成功启动后，会发布如下 topic:

```text
/joint_states
/position_trajectory_controller/command
/position_trajectory_controller/follow_joint_trajectory/cancel
/position_trajectory_controller/follow_joint_trajectory/feedback
/position_trajectory_controller/follow_joint_trajectory/goal
/position_trajectory_controller/follow_joint_trajectory/result
/position_trajectory_controller/follow_joint_trajectory/status
/position_trajectory_controller/state
/tf
/tf_static
```

关于这些 topic 的意义，请搜索 ros control

##### Service

在成功启动后，会发布如下 service:

```text
# 夹持器相关（后续更新可能加入更多）
/clamp/close
/clamp/get_state
/clamp/open
/clamp/stop

# 来自 ros control
/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/switch_controller
/controller_manager/unload_controller
/position_trajectory_controller/query_state
/robot_state_publisher/get_loggers
/robot_state_publisher/set_logger_level
/yj10_hardware_interface/get_loggers
/yj10_hardware_interface/set_logger_level
```
