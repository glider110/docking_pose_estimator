import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    package_dir = get_package_share_directory('docking_pose_estimator')

    # 定义参数文件的路径
    params_file = os.path.join(
        package_dir, 'config', 'deploy_params.yaml'
    )

    # 定义 RViz 配置文件的路径
    rviz_config_file = '/home/std/workspace/test_pkg_ws/src/docking_pose_estimator/rviz/deploy.rviz'

    # 声明启动参数（可选，允许在命令行中覆盖参数文件路径或 RViz 配置文件路径）
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to the YAML file with parameters for docking_pose_node'
    )

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=rviz_config_file,
        description='Path to the RViz configuration file'
    )

    # 启动 deploy_target_perception 节点，加载参数文件
    deploy_target_perception = Node(
        package='docking_pose_estimator',
        executable='deploy_target_perception',
        name='deploy_target_perception',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # 启动 RViz2 节点，加载指定的配置文件
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')]
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        params_file_arg,
        rviz_config_file_arg,
        deploy_target_perception,
        rviz_node
    ])