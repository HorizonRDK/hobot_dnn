from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # args that can be set from the command line or a default will be used
    config_file_launch_arg = DeclareLaunchArgument(
        "config_file", default_value=TextSubstitution(text="config/fcosworkconfig.json")
    )
    dump_render_launch_arg = DeclareLaunchArgument(
        "dump_render_img", default_value=TextSubstitution(text="0")
    )
    image_width_launch_arg = DeclareLaunchArgument(
        "image_width", default_value=TextSubstitution(text="960")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "image_height", default_value=TextSubstitution(text="544")
    )
    
    return LaunchDescription([
        config_file_launch_arg,
        dump_render_launch_arg,
        image_width_launch_arg,
        image_height_launch_arg,
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"image_width": LaunchConfiguration('image_width')},
                {"image_height": LaunchConfiguration('image_height')},
                {"out_format": "nv12"},
                {"io_method": "shared_mem"},
                {"video_device": "F37"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动jpeg图片编码&发布pkg
        Node(
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
                {"pub_topic": "/image_jpeg"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
        Node(
            package='dnn_node_example',
            executable='example',
            output='screen',
            parameters=[
                {"config_file": LaunchConfiguration('config_file')},
                {"dump_render_img": LaunchConfiguration('dump_render_img')},
                {"feed_type": 1},
                {"is_sync_mode": 0},
                {"is_shared_mem_sub": 1},
                {"msg_pub_topic_name": "hobot_dnn_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        # 启动web展示pkg
        Node(
            package='websocket',
            executable='websocket',
            output='screen',
            parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"smart_topic": "/hobot_dnn_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
