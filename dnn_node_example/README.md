Getting Started with Dnn Node Example
=======


# Intro

Dnn Node example package是Dnn Node package的使用示例，通过继承DnnNode虚基类，使用模型和图像数据利用BPU处理器进行模型推理。图像数据来源于本地图片回灌和订阅到的image msg。

# Build

## Dependency

依赖库，具体请参考dnn node package的dnn_node_deps中版本说明。

- dnn:1.5.1
- easydnn:0.3.3
- opencv:3.4.5
- hobotlog:1.0.5
- rapidjson:1.1.0

ros package：

- dnn node
- cv_bridge
- sensor_msgs

其中cv_bridge为ROS开源的package，需要手动安装，具体安装方法：

```cpp
# 方法1，直接使用apt安装，以cv_bridge安装举例
sudo apt-get install ros-foxy-cv-bridge -y

# 方法2，使用rosdep检查并自动安装pkg编译的依赖项
# 安装ros pkg依赖下载⼯具rosdep
sudo apt-get install python3-pip
sudo pip install rosdep
sudo rosdep init
rosdep update
# 在ros的⼯程路径下执⾏安装依赖，需要指定pkg所在路径。默认为所有pkg安装依赖，也可以指定为某个pkg安装依赖
rosdep install -i --from-path . --rosdistro foxy -y
```

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

- CMakeLists.txt中指定dnn node package的安装路径，默认为`../install/dnn_node`，即和dnn_node_example安装在同一install路径下。
- 编译：`colcon build --packages-select dnn_node_example`

# Usage

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:install/dnn_node_example/lib/dnn_node_example/
# config中为example使用的模型，回灌使用的本地图片
cp -r install/dnn_node_example/lib/dnn_node_example/config/ .
# 运行模式1：使用本地jpg格式图片通过同步模式进行回灌预测，并存储渲染后的图片
ros2 run dnn_node_example example --ros-args -p feed_type:=0 -p image:=config/test.jpg -p image_type:=0 -p dump_render_img:=1
# 运行模式2：使用订阅到的image msg(topic为/image_raw)通过异步模式进行预测，并设置log级别为warn
ros2 run dnn_node_example example --ros-args -p feed_type:=1 -p is_sync_mode:=0 --ros-args --log-level warn
```
