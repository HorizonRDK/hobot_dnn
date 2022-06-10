from argparse import Action
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    config_file_launch_arg = DeclareLaunchArgument(
        "config_file", default_value=TextSubstitution(text="config/multitask_body_kps_960x544.json")
    )

    img_file_launch_arg = DeclareLaunchArgument(
        "image", default_value=TextSubstitution(text="config/test.jpg")
    )


    # config中模型太大，创建一个软连接
    # 1. 创建config软连接
    dnn_node_example_path = os.path.join(
        get_package_prefix('dnn_node_example'),
        "lib/dnn_node_example")
    print("dnn_node_example_path is ", dnn_node_example_path)
    link_cmd = "ln -s " + dnn_node_example_path + "/config/ config"
    print("link_cmd is ", link_cmd)
    os.system(link_cmd)

    # 2.创建runtime软连接
    dnn_benchmark_example_path = os.path.join(
        get_package_prefix('dnn_benchmark_example'),
        "lib/dnn_benchmark_example")
    print("dnn_benchmark_example_path is ", dnn_benchmark_example_path)
    link_cmd = "ln -s " + dnn_benchmark_example_path + "/config/runtime config/runtime"
    print("link_cmd is ", link_cmd)
    os.system(link_cmd)


    return LaunchDescription([
        config_file_launch_arg,
        img_file_launch_arg,
        # 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
        Node(
            package='dnn_node_example',
            executable='example',
            output='screen',
            parameters=[
                {"feed_type": 0},
                {"config_file": LaunchConfiguration('config_file')},
                {"image": LaunchConfiguration('image')},
                {"image_type": 0},
                {"dump_render_img": 1}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
