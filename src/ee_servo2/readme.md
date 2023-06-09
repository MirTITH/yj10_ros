# ee_servo2

末端实时伺服 第二代

注意，不能和 end_effector_servo 同时启动

## 使用说明

### 配置

本程序有两套解算方式，可以在 launch 文件中修改：

采用`只解算到腕关节`方法时，腕关节在平移的时候电机不会转，所以它在世界空间下的朝向会变

采用`解算到末端`方法时，腕关节在平移的时候电机会转，所以它在世界空间下的朝向不变，与一代表现相似

无论采用哪种方法，都可以发布旋转指令控制基座关节、腕关节和夹爪旋转关节的转动

launch 文件相关部分截取如下：

```xml
    <!-- 只解算到腕关节 -->
    <param name="chain_end" value="link4" />
    <param name="add_fake_joint_x" value="false" />
    <param name="add_fake_joint_y" value="true" />
    <param name="add_fake_joint_z" value="true" />
    <param name="add_fake_joint_num" value="2" />

    <!-- 解算到末端 -->
    <!-- <param name="chain_end" value="fake_link6" />
    <param name="add_fake_joint_x" value="false" />
    <param name="add_fake_joint_y" value="false" />
    <param name="add_fake_joint_z" value="false" />
    <param name="add_fake_joint_num" value="0" /> -->
```

### 启动

```sh
roslaunch ee_servo2 ee_servo2.launch
```

### 使用方法

#### 末端速度伺服

向 `/end_effector_servo/end_effector_velocity` 连续发布末端速度指令即可

Tips: 可以执行以下命令打开 rqt：

```sh
roslaunch yj10_driver rqt_monitor.launch
```

#### 运行到给定位置

向 `/end_effector_servo_moveit/move_to` 发送要运动到的位置，**注意，这个话题名字和一代不同**

在 yj10_moveit_config/config/yj10.srdf 中指定位置和对应的名称，然后发送位置的名称即可

注意：虽然这个功能带有碰撞规避，但不会考虑夹爪夹着的东西

例如：

```sh
rostopic pub /end_effector_servo_moveit/move_to std_msgs/String "data: 'fold'"
```

### 注意事项

必须连续发布末端速度，如果间隔时间太长，则会自动停止机械臂运动，保证安全。

如果发布的值为全0，本软件将不会控制 joint_group_position_controller。此时你可以使用其他软件控制机械臂。
