# YJ10 workspace

## 安装方法

1. 进入 yj10_grpc_ws，按照该文件夹下的 readme.md 编译安装 gRPC 相关库
2. 进入 yj10_ros_ws，按照该文件夹下的 readme.md 构建 ros 工作空间
3. 配置 grasp_model docker （见下）
4. 安装 gazebo 模型文件：将 models 复制到 ~/.gazebo/ 下:
    ```shell
    cp -r ~/models ~/.gazebo/
    ```

### 配置 grasp_model docker

TODO

## Gazebo 抓取仿真
见 [yj10_ros_ws/readme.md#gazebo-抓取仿真)](./yj10_ros_ws/readme.md#gazebo-抓取仿真)