import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取包的路径
    package_dir = get_package_share_directory('docking_pose_estimator')

    # 定义 bag 文件路径
    bag_file_path = os.path.expanduser("/home/std/workspace/test_pkg_ws/src/docking_pose_estimator/data/bag/satic_workbin")
    
    # 验证 bag 文件是否存在
    if not os.path.exists(bag_file_path):
        return LaunchDescription([
            LogInfo(msg=f"Error: Bag file not found at {bag_file_path}")
        ])

    # 定义参数文件的路径
    params_file = os.path.join(package_dir, 'config', 'docking_param_entity.yaml')

    # 定义 RViz 配置文件的路径
    rviz_config_file = '/home/std/workspace/test_pkg_ws/src/docking_pose_estimator/rviz/dock.rviz'

    # 声明启动参数
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

    use_bag = DeclareLaunchArgument(
        'use_bag',
        default_value='true',
        description='Whether to play the ROS 2 bag file'
    )

    # 启动 docking_pose_node 节点
    docking_pose_node = Node(
        package='docking_pose_estimator',
        executable='docking_pose_node',
        name='docking_pose_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # 启动 RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')]
    )

    # 静态 TF 发布
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0.157', '0.01', '0.419', '-1.570', '0', '-1.570', 'base_footprint', 'camera_depth_optical_frame'],
        output='screen'
    )

    # 播放 ROS 2 bag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_path, '--clock', '--rate', '1.0', '--loop'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bag'))
    )

    # 添加 Orbbec 相机启动文件
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('orbbec_camera'),
            '/launch/dabai_dcw2.launch.py'
        ]),
        condition=UnlessCondition(LaunchConfiguration('use_bag'))
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        params_file_arg,
        rviz_config_file_arg,
        use_bag,
        static_transform_node,
        bag_play,
        orbbec_launch,
        docking_pose_node,
        rviz_node
    ]) 