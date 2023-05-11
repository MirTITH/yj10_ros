# YJ10 排爆机械臂 ROS 库

YJ10 排爆机械臂的 ROS 库。使用 RS485 与机械臂通信，通信协议为 Modbus-RTU.

## 目录结构

- docs: 机械臂相关文档
- src: ros 软件包代码

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

### 显示机械臂的模型

```bash
source devel/setup.bash
roslaunch yj10_description display.launch
```

### 使用 Gazebo 仿真，在 Rviz 中使用 Moveit

```bash
source devel/setup.bash
roslaunch yj10_moveit_config demo_gazebo.launch
```

### 连接实体机械臂，在 Rviz 中使用 Moveit

#### 准备工作

1. 使用串口连接机械臂
2. 给机械臂供电，24V 直流
3. 在 [yj10_connection.yaml](src/yj10_driver/config/yj10_connection.yaml) 中配置串口设备
    > 请注意，关闭其中的 `fake_connect` 才会真正连接机械臂

#### 运行程序

执行以下命令，连接机械臂并运行 Rviz Moveit：

```bash
source devel/setup.bash
roslaunch yj10_moveit_config demo_hardware.launch
```

### 键盘控制机械臂末端

先按照上一节进行准备工作

#### 运行程序

参见 [yj10_moveit_group/readme.md](src/yj10_moveit_group/readme.md)
