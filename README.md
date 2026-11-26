# FAST-LIO2-ROS2
FAST-LIO2 激光惯性里程计（LIO-SLAM），移植适配 ROS2 Humble。
采用激光雷达与IMU紧耦合估计框架，基于 ikd-Tree 实现增量式动态点云地图，支持离线 rosbag2 回放，输出TUM格式轨迹，使用 evo 完成轨迹精度评估。
> 个人学习Demo，测绘工程背景，面向SLAM / LIO 导航岗位求职。

## 一、硬件适配
- 订阅标准 `sensor_msgs/msg/PointCloud2` 点云消息，**无需Livox私有驱动与SDK**
- 适配：Livox Mid360 / Mid40 固态激光雷达 + 6轴IMU组合
- 支持离线数据集回放，无实体雷达也可以复现建图效果

## 二、开发环境
- 操作系统：Ubuntu 22.04.5
- ROS版本：ROS2 Humble Hawksbill
- 依赖库：
  - PCL 1.12：点云滤波、配准、几何处理
  - Eigen3：矩阵运算、IMU预积分、状态求解
  - ikd-Tree：增量式KD-Tree，地图动态增删，提升配准效率
  - OpenCV：图像辅助处理
  - evo：轨迹离线评估，计算APE、RPE误差并绘图
  - RViz2：ROS2三维可视化工具，实时查看点云与里程计轨迹
  - CloudCompare：点云后处理、去噪、ICP配准、测绘断面/等高线生成

## 三、算法原理简述
FAST-LIO2 属于紧耦合激光惯性里程计，核心流程：
1. IMU高频预积分，预测雷达帧间运动，对点云做运动畸变补偿；
2. ikd-Tree维护全局增量地图，相比传统静态KD-Tree，支持点的插入、删除、查询，适合动态场景；
3. 将去畸变后的点云与全局地图进行迭代配准，联合优化LiDAR-IMU位姿；
4. 输出 `odom` 里程计话题，同时保存TUM格式轨迹文件，用于离线精度评估。

> 特点：计算轻量化、对运动畸变鲁棒性较好，适合固态激光雷达；
> 局限：原生仅里程计，**无回环检测模块，长时间大场景会产生漂移**。

## 四、仓库目录结构
```

fast_lio/
├── config/         # 参数配置文件：IMU 噪声、雷达 - IMU 外参、滤波器、配准阈值
├── launch/         # ROS2 launch 启动脚本
├── src/            # 核心源码：ikd-Tree 实现、点云预处理、配准、状态估计
├── CMakeLists.txt
└── package.xml

```

## 五、编译步骤
```bash
# 进入ROS2工作空间
cd ~/lio_ws
# 单独编译fast_lio包
colcon build --packages-select fast_lio
# 刷新环境变量，新开终端必须执行
source install/setup.bash
```

> 
> 可选：把 `source ~/lio_ws/install/setup.bash` 写入 `~/.bashrc`，开机自动加载环境

## 六、完整运行流程

1. 启动 FAST-LIO2 建图节点

```
ros2 launch fast_lio mapping.launch.py
```

2. 新开终端，回放 rosbag2 数据集

```
ros2 bag play your_dataset.db3
```

3. 新开终端，启动 RViz2 可视化

```
rviz2
```

RViz2 添加显示项：

- PointCloud2：查看实时点云地图
- Odometry：可视化里程计轨迹

4. 数据集播放结束，程序自动保存估计轨迹（TUM 格式）

## 七、轨迹评估（evo）

TUM 轨迹格式：`timestamp x y z qw qx qy qz`

```
# 绝对位姿误差 APE，衡量整体漂移
evo_ape tum groundtruth.txt estimate.txt --plot --save_results ape_result.zip

# 相对位姿误差 RPE，衡量局部里程计精度
evo_rpe tum groundtruth.txt estimate.txt --plot --save_results rpe_result.zip

# 真值与估计轨迹同屏对比可视化
evo_traj tum groundtruth.txt estimate.txt --ref groundtruth.txt -p
```

指标说明：

- APE：绝对位姿误差，评估全局定位漂移；
- RPE：相对位姿误差，评估短距离里程计精度；
- 误差单位：平移 (m)、旋转 (°)。

## 八、结果展示（后续跑完数据集补充截图）

- RViz2 实时建图截图：
- evo 轨迹对比误差图：
- APE / RPE 误差统计：

## 九、调参与注意事项

1. 外参标定：雷达与 IMU 的旋转平移外参写在 config 文件，外参不准会严重漂移；
2. IMU 噪声参数：陀螺、加速度计噪声参数直接影响预积分精度；
3. 运动畸变：高速运动场景依赖 IMU 预积分补偿；
4. 数据集要求：rosbag2 内包含点云话题 + IMU 话题；
5. 本项目无回环，适合短距离场景学习验证。
#### 十、后续学习计划

- 接入回环检测，移植 LIO-SAM 框架
- 增加一键导出全局点云地图为 pcd 文件
- 测试多组公开数据集，横向对比精度
