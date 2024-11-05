# YJ10 workspace

## 安装方法

1. 克隆本仓库
    
2. 解压 `yj10_ws_resources.tar.xz`：
    
    ```shell
    # 将 /path/to/repo 换成仓库路径（和-C之间不要加空格）
    # 例如 tar -xaf yj10_ws_resources.tar.xz -Cyj10_ws
    tar -xaf yj10_ws_resources.tar.xz -C/path/to/repo
    ```
    
    解压后仓库中应该会多两个文件夹：
    
    - grasp_model
    - models
    
    目录结构应该如下：
    
    ```
    yj10_ws
    ├── grasp_model
    ├── models
    ├── readme.md
    ├── run_grasp_model_docker.sh
    ├── yj10_grpc_ws
    └── yj10_ros_ws
    ```
    
3. 安装 gazebo 模型文件：将 models 复制到 ~/.gazebo/ 下:
    
    ```shell
    cd yj10_ws
    cp -r models ~/.gazebo/
    ```
    
4. 进入 yj10_grpc_ws，按照该文件夹下的 readme.md 编译安装 gRPC 相关库

5. 进入 yj10_ros_ws，按照该文件夹下的 readme.md 构建 ros 工作空间

6. 配置 grasp_model docker（见下）

### 配置 grasp_model docker

#### 安装 Docker

1. 安装 docker

    ```shell
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    ```

2. 使 docker 命令不需要 root 运行

    ```shell
    sudo groupadd docker
    sudo usermod -aG docker $USER
    ```

3. Log out and log back in so that your group membership is re-evaluated.

4. Test your docker installation

    ```shell
    docker run hello-world
    ```

5. 安装 Nvidia Container Toolkit：

    从 NVIDIA 官方安装文档截取的安装命令：
    ```shell
    # Installing Nvidia Container Toolkit with Apt
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    
    sudo apt-get update
    
    sudo apt-get install -y nvidia-container-toolkit
    
    # Configuring Docker
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```

    > 官方安装文档 https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

####  加载镜像
```shell
# 将 /path/to/grasp_model_he_docker.tar.xz 换成对应路径
xz -dvc /path/to/grasp_model_he_docker.tar.xz | docker load
```

#### 检查镜像
```shell
docker images
```

应该会输出：

```
REPOSITORY                 TAG                   IMAGE ID       CREATED             SIZE
...
grasp_model_he             latest                2d6faf0fd876   28 minutes ago      17.8GB
...
```

## Gazebo 抓取仿真
见 [yj10_ros_ws/readme.md#gazebo-抓取仿真](./yj10_ros_ws/readme.md#gazebo-抓取仿真)