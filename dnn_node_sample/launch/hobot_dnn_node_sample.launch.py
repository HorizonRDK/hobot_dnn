# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # include web launch file
    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/hobot_websocket_service.launch.py'))
    )

    # args that can be set from the command line or a default will be used
    image_width_launch_arg = DeclareLaunchArgument(
        "image_width", default_value=TextSubstitution(text="960")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "image_height", default_value=TextSubstitution(text="544")
    )

    # 本地图片发布
    fb_node = Node(
        package='hobot_image_publisher',
        executable='hobot_image_pub',
        output='screen',
        parameters=[
            {"image_source": "./config/target.jpg"},
            {"image_format": "jpg"},
            {"msg_pub_topic_name": "/hbmem_img"},
            # 不指定发布图片的分辨率
            # {"output_image_w": 320},
            # {"output_image_h": 320},
            {"fps": 10},
            {"is_loop": True},
            {"is_shared_mem": True}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    
    # mipi cam图片发布pkg
    mipi_node = Node(
        package='mipi_cam',
        executable='mipi_cam',
        output='screen',
        parameters=[
            # {"camera_calibration_file_path": "/opt/tros/lib/mipi_cam/config/F37_calibration.yaml"},
            {"image_width": LaunchConfiguration('image_width')},
            {"image_height": LaunchConfiguration('image_height')},
            {"out_format": "nv12"},
            {"io_method": "shared_mem"},
            {"video_device": "F37"} # 支持的配置项为"F37"和"GC4663"
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # usb cam图片发布pkg
    usb_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='hobot_usb_cam',
        parameters=[
                    {"frame_id": "default_usb_cam"},
                    {"image_height": 480},
                    {"image_width": 640},
                    {"zero_copy": False},
                    {"video_device": "/dev/video8"}
                    ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # jpeg图片编码&发布pkg
    jpeg_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"channel": 1},
            {"in_mode": "shared_mem"},
            {"in_format": "nv12"},
            {"out_mode": "ros"},
            {"out_format": "jpeg"},
            {"sub_topic": "/hbmem_img"},
            {"pub_topic": "/image"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # nv12图片解码&发布pkg
    nv12_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"channel": 1},
            {"in_mode": "ros"},
            {"in_format": "jpeg"},
            {"out_mode": "shared_mem"},
            {"out_format": "nv12"},
            {"sub_topic": "/image"},
            {"pub_topic": "/hbmem_img"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 算法pkg
    dnn_node_sample_node = Node(
        package='dnn_node_sample',
        executable='dnn_node_sample',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_sync_mode": 0},
            {"is_shared_mem_sub": 1},
            {"msg_pub_topic_name": "dnn_node_sample"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # web展示pkg
    web_node = Node(
        package='websocket',
        executable='websocket',
        output='screen',
        parameters=[
            {"image_topic": "/image"},
            {"image_type": "mjpeg"},
            {"smart_topic": "/dnn_node_sample"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)
    cam_node = mipi_node
    camera_type_mipi = True
    if camera_type == "usb":
        print("using usb cam")
        cam_node = usb_node
        camera_type_mipi = False
    elif camera_type == "mipi":
        print("using mipi cam")
        cam_node = mipi_node
        camera_type_mipi = True
    elif camera_type == "fb":
        print("using feedback")
        cam_node = fb_node
        camera_type_mipi = True
    else:
        print("invalid camera_type ", camera_type, ", which is set with export CAM_TYPE=usb/mipi/fb, using default mipi cam")
        cam_node = mipi_node
        camera_type_mipi = True

    if camera_type_mipi:
        return LaunchDescription([
            web_service_launch_include,
            image_width_launch_arg,
            image_height_launch_arg,
            # 图片发布pkg
            cam_node,
            # 图片编解码&发布pkg
            jpeg_codec_node,
            # 启动算法pkg
            dnn_node_sample_node,
            # 启动web展示pkg
            web_node
        ])
    else:
        return LaunchDescription([
            web_service_launch_include,
            image_width_launch_arg,
            image_height_launch_arg,
            # 图片发布pkg
            cam_node,
            # 图片编解码&发布pkg
            nv12_codec_node,
            # 启动算法pkg
            dnn_node_sample_node,
            # 启动web展示pkg
            web_node
        ])
