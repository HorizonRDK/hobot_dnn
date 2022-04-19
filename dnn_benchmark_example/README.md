Getting Started with Dnn benchmark Example
=======


# Intro

Dnn Benchmark example package是Dnn Node package的性能评测示例，通过继承DnnNode虚基类，使用模型和图像数据利用BPU处理器进行模型推理，输出帧率fps和单帧延迟latency性能指标。图像数据来源于本地图片回灌。

# Build

## Dependency

依赖库：

- dnn:1.8.4
- easydnn:0.3.3
- opencv:3.4.5
- rapidjson:1.1.0

ros package：

- dnn node

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式，并支持通过编译选项控制编译pkg的依赖和pkg的功能。

### X3 Ubuntu系统上编译

1、编译环境确认

- 当前编译终端已设置ROS环境变量：`source /opt/ros/foxy/setup.bash`。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`apt update; apt install python3-colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select dnn_benchmark_example

### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好tros。docker安装、交叉编译说明、tros编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译dnn node package

2、编译

- 编译命令：

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

  colcon build --packages-select dnn_benchmark_example \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
  ```

# Usage

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行。

## X3 Ubuntu系统上运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash

# config中为example使用的模型，输入的配置文件，回灌使用的本地图片
# 根据实际安装路径进行拷贝（docker中的安装路径为install/lib/dnn_benchmark_example/config/，拷贝命令为cp -r install/dnn_benchmark_example/lib/dnn_benchmark_example/config/ .）。
cp -r install/dnn_benchmark_example/lib/dnn_benchmark_example/config/ .

# 运行：使用本地jpg格式图片通过同步模式进行回灌预测，通过日志输出性能指标fps和latency，并设置log级别为warn
ros2 run dnn_benchmark_example dnn_benchmark_example --ros-args --log-level warn

# 运行参数配置：通过运行时-p选项更改配置参数
  1、show_fps_log(fps开关)默认为1(0:关闭 1:打开)
  2、show_latency_log(latency开关)默认为1(0:关闭 1:打开)
  3、statistic_cycle(处理周期图片个数)默认为500(即每处理500张图片输出一次性能指标)
  4、is_sync_mode(同步或异步模式)默认为1(0:异步 1:同步)
  5、config_file(配置文件路径)默认为"config/hobot_benchmark_config.json"
  6、model_file_name(模型文件)默认为"config/multitask_body_kps_960x544.hbm"
  7、model_name(模型名称)默认为"multitask_body_kps_960x544"
ros2 run dnn_benchmark_example dnn_benchmark_example --ros-args --log-level warn -p show_fps_log:=0 -p statistic_cycle:=1000

# 配置文件hobot_benchmark_config.json
{
"input_config": {
      "input_type": "image",
      "height": 544,
      "width": 960,
      "data_type": 1,   //0: BGR 1: NV12
      "image_list_file": "config/image.list", //图片配置 每行一个图片地址
      "need_pre_load": true,
      "limit": 2,  //input输入个数减去release个数 < 2
      "need_loop": true,  //循环输入
      "max_cache": 10
    }
}
