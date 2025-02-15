import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='livox_pointcloud_converter',
            executable='converter_node',
            name='livox_pointcloud_converter',
            output='screen',
            remappings=[
                ("input_cloud", "/livox/lidar"),
                ("output_cloud", "/livox/lidar/converted"),
            ]
        )
    ])
