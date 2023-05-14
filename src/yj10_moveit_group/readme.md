# yj10_moveit_group

调用 move_group_interface 实现键盘控制机械臂末端

响应实时性不佳，但可以防止碰撞

## 使用说明

```bash
roslaunch yj10_moveit_config demo_hardware.launch
rosrun yj10_moveit_group yj10_moveit_group_node
```
