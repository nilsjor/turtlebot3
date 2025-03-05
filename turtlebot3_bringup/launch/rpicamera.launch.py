from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_size = LaunchConfiguration('image_size', default='[640,480]')
    camera_info_url = LaunchConfiguration('camera_info_url', 
        default='package://turtlebot3_bringup/param/rpicamera.yaml')
    output_encoding = LaunchConfiguration('output_encoding', default='yuv422_yuy2')

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            parameters=[
                {
                    'image_size': image_size,
                    'output_encoding': output_encoding,
                    'camera_info_url': camera_info_url
                }
            ]
        ),
    ])
