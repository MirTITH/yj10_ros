# YJ10 gRPC workspace

这个目录下包含 gRPC 远程调用库和相应的消息定义。

## 安装方法

```shell
sudo apt install -y build-essential autoconf libtool pkg-config ninja-build
python3 -m pip install --upgrade pip
./build_install.sh
```

这个脚本会编译、安装下列包：

- grpc: gRPC 的 C++ 库
- grasp_model_grpc_msg

其中：

- 编译中间文件会放在 ./build 文件夹下
- C++ 相关的库会安装在 ./install 文件夹下
- Python 库会使用 `python3 -m pip install <path>` 安装

## 卸载方法

```shell
./clean.sh
python3 -m pip uninstall grasp_model_grpc_msg
```

