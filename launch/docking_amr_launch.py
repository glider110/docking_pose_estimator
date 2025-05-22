#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 设置环境变量
    set_turtlebot_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )

    # 获取包路径
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    docking_pose_dir = get_package_share_directory('docking_pose_estimator')

    # 1. 启动 Gazebo 仿真环境
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_amr_world.launch.py')
        )
    )

    # 2. 启动对接姿态估计器
    docking_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(docking_pose_dir, 'launch', 'docking_pose_launch.py')
        )
    )

    # 3. 运行位姿更新脚本
    update_model_pose = ExecuteProcess(
        cmd=['python3', os.path.join(docking_pose_dir, 'script', 'update_model_pose.py')],
        output='screen'
    )

    # 4. 运行两阶段导航
    move_to_dock = ExecuteProcess(
        cmd=['python3', os.path.join(docking_pose_dir, 'script', 'move_to_dock_pose.py')],
        output='screen'
    )

    # 延时启动，确保依赖关系
    delayed_docking_pose_launch = TimerAction(
        period=5.0,  # 等待 Gazebo 启动
        actions=[docking_pose_launch]
    )

    delayed_update_pose = TimerAction(
        period=10.0,  # 等待姿态估计器运行
        actions=[update_model_pose]
    )

    delayed_move_to_dock = TimerAction(
        period=15.0,  # 等待位姿更新完成
        actions=[move_to_dock]
    )

    # 返回启动描述
    return LaunchDescription([
        # set_turtlebot_model,
        # gazebo_launch,
        delayed_docking_pose_launch,
        delayed_update_pose,
        delayed_move_to_dock
    ])