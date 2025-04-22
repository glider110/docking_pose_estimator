### AMR对接位姿估计

> 对接载具人为计算提取特征的方式在AMR行业过繁琐，不能适用大部份载具，此功能包用icp的方式匹配以概率的形式进行迭代估计载具的位姿



#### 运行：

**编译：**

```
colcon build --symlink-install --packages-select docking_pose_estimator、
source install/setup.zsh
```

**仿真环境：**

1.运行turtlebot3仿真环境

```
ros2 launch turtlebot3_gazebo turtlebot3_amr_world.launch.py
```

2.位姿估计节点

```
ros2 launch docking_pose_estimator docking_pose_launch.py
```

**真机环境：**





#### 问题：

- 模版全局不够问题
- 地面的影响



#### TODO LIST：

- [x] 载具对接功能包构建
- [x] 
