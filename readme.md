# YJ10 排爆机械臂 ROS 库

YJ10 排爆机械臂的 ROS 库。使用 RS485 与机械臂通信，通信协议为 Modbus-RTU.

## 注意事项

- 请注意，该机械臂没有关节反馈，所以发布的关节消息是假消息
- 尽量不要在关节运动时调用读写夹持器的服务
- 夹持器在闭合状态下不要再次调用闭合命令，在完全张开状态下不要再次调用张开命令

## 目录结构

| 文件夹 | 说明           |
| ------ | -------------- |
| docs   | 机械臂相关文档 |
| src    | ros 软件包代码 |

各个软件包的说明放在各自的子文件夹里

## 构建方法

本仓库为 catkin 标准工作空间结构。

构建前，需要安装依赖，在仓库根目录执行：

```bash
# 安装 modbus 库
sudo apt install libmodbus-dev

# 安装其他 ros 依赖。如果 $ROS_DISTRO 未定义，请将它改为你的 ros 版本
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
```

构建：

```bash
catkin_make
```

## 使用说明

仅列举常见使用方式，详细说明请查看各自软件包的 readme 文件

使用软件包前，需要 source:

对于bash用户:

```sh
source devel/setup.bash
```

对于zsh用户:

```sh
source devel/setup.zsh
```

### 显示机械臂的模型

```sh
roslaunch yj10_description display.launch
```

### 使用 Gazebo 仿真，在 Rviz 中使用 Moveit

```sh
roslaunch yj10_moveit_config demo_gazebo.launch
```

### 连接机械臂

**这会将机械臂复位到初始位姿**，同时启动 ros controller 并发布 joint state 关节信息

#### 准备工作

1. 使用串口连接机械臂
2. 在 [yj10_driver.yaml](src/yj10_driver/config/yj10_driver.yaml) 中配置串口设备
    > 请注意，关闭其中的 `fake_connect` 才会真正连接机械臂
3. 赋予串口设备访问权限（以下方法二选一）

    ```sh
    # 临时打开串口权限（注意将 /dev/ttyUSB0 改为对应串口）
    sudo chmod 666 /dev/ttyUSB0

    # 永久打开串口权限（注意把 `用户名` 换成你的用户名）
    sudo usermod -aG dialout 用户名
    ```

4. 给机械臂供电，24V 直流（注意，给机械臂供电后，机械臂会自动在3秒后展开）

#### 连接但不启动 Rviz

```sh
roslaunch yj10_driver yj10_connect.launch
```

#### 连接并启动 Rviz

```sh
roslaunch yj10_driver yj10_connect_rviz.launch
```

#### 开始控制机械臂

详情见 [src/yj10_driver/readme.md](src/yj10_driver/readme.md)

### 连接机械臂，在 Rviz 中使用 Moveit

连接串口后，执行以下命令：

```bash
roslaunch yj10_driver yj10_connect.launch
# 开一个新终端：
roslaunch yj10_moveit_config demo_hardware.launch
```

### 机械臂末端伺服

参见 [src/end_effector_servo](src/end_effector_servo)
