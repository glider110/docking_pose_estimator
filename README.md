### AMR对接位姿估计

> 对接载具人为计算提取特征的方式在AMR行业过繁琐，不能适用大部份载具，此功能包用icp的方式匹配以概率的形式进行迭代估计载具的位姿

#### 方案设计：

[**自然特征巡航对接（研发专项）**](https://standard-robots.yuque.com/group-rd/fgx8kc/mbwxz1ladbbry9ef)

#### 运行：

**编译：**

```
colcon build --symlink-install --packages-select docking_pose_estimator、
source install/setup.zsh
```

**仿真环境：**

1.运行turtlebot3仿真环境

```
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_amr_world.launch.py
```

2.感知目标部署节点（部署点对齐）

```
ros2 launch docking_pose_estimator deploy_target_perception.py
ros2 topic pub /save_pointcloud_command std_msgs/String "data: 'save'"   #保存部署点云文件
```

3.位姿估计节点

```
ros2 launch docking_pose_estimator docking_pose_launch.py
```

4.导航到部署点(对接点)

```
#模拟更新小车或者载具的位姿
#偏移场景 ('waffle',2.5,-0.262, 0.0, 0.0)('cart_model2_no_whell', 4.5, -0.262201, 0.0, 170) 
cd src/docking_pose_estimator/script
python3 update_model_pose.py
#运行两阶段导航
python3 move_to_dock_pose.py
```



**真机环境：**







#### TODO LIST：

- [x] 载具对接功能包构建
- [ ] 全景载具的模版制作和icp匹配效果
- [ ] 自动特征提取
- [ ] 标准对接动作方案
- [ ] 载具倾斜问题



#### 模版制作：

**1.建模软件导出stl格式，再用cloudcompare转换为pcd或者ply**

![image-20250423161422258](README.assets/image-20250423161422258.png)

**2.用rtabmap建图，再用cloudcompare来裁剪**

![image-20250422150653270](README.assets/image-20250422150653270.png)

**3.用高精度的三维扫描设备扫描**





#### **参考：**

[**基于ICP点云配准的工件位姿估计-Open3D**](https://www.bilibili.com/video/BV1wr4y1S7UB/?spm_id_from=333.337.search-card.all.click&vd_source=7f98e46af73470a39ad6b1a64611b176)
