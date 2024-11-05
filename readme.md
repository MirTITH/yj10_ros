# YJ10 workspace

## 安装方法

1. 安装 gazebo 模型文件：将 models 复制到 ~/.gazebo/ 下:
    ```shell
    cp -r ~/models ~/.gazebo/
    ```

2. 配置 grasp_model docker（见下）

3. 进入 yj10_grpc_ws，按照该文件夹下的 readme.md 编译安装 gRPC 相关库

4. 进入 yj10_ros_ws，按照该文件夹下的 readme.md 构建 ros 工作空间

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

    从 NVIDIA 官网截取的安装命令：
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

    > 如果上述方法无效，则参考官方安装文档 https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

####  加载镜像
```shell
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