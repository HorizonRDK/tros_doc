---
sidebar_position: 1
---

# 4.1 SLAM建图

## 功能介绍

SLAM指即时定位与地图构建（Simultaneous Localization and Mapping，简称SLAM）。
本章节使用ROS2的SLAM-Toolbox作为建图算法，在Gazebo中控制小车行驶建立地图，并通过Rviz2观察建图效果。
其中SLAM-Toolbox运行在地平线RDK上，Gazebo和Rviz2运行在与地平线RDK同一网段的PC上。

## 支持平台

| 平台    | 运行方式     | 示例功能                       |
| ------- | ------------ | ------------------------------ |
| RDK X3, RDK X3 Module, RDK X5 Ultra| Ubuntu 20.04 | PC端启动仿真环境，并在地平线RDK进行SLAM建图，最后通过Rviz2展示建图效果 |

## 准备工作

### 地平线RDK平台

1. 地平线RDK已烧录好地平线提供的Ubuntu 20.04镜像。

2. 地平线RDK已成功安装TogetheROS.Bot。

3. tros.b成功安装后，安装SLAM-Toolbox

    ```bash
    sudo apt-get install ros-foxy-slam-toolbox
    ```

4. 建立软连接

    ```bash
    cd /opt/tros
    # 使用/opt/tros目录下的create_soft_link.py创建ROS package至tros.b的软连接
    sudo python3 create_soft_link.py --foxy /opt/ros/foxy/ --tros /opt/tros/
    ```

5. 和地平线RDK在同一网段的PC，PC已安装Ubuntu 20.04系统、ROS2 Foxy桌面版和仿真环境Gazebo，
数据可视化工具Rviz2。

    ROS2 Foxy安装参考：https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

    PC的ROS2安装成功后安装Gazebo和Turtlebot3相关的功能包，安装方法为：

    ```bash
    sudo apt-get install ros-foxy-gazebo-*
    sudo apt install ros-foxy-turtlebot3
    sudo apt install ros-foxy-turtlebot3-bringup
    sudo apt install ros-foxy-turtlebot3-simulations
    sudo apt install ros-foxy-teleop-twist-keyboard
    ```

## 使用介绍

### 地平线RDK平台

本小节介绍如何使用地平线RDK运行SLAM算法，并使用PC观察建图效果。

PC端启动仿真环境：

```bash
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

仿真环境如下图所示：
![](./image/slam/gazebo.jpg)

PC端开启另外一个控制台，启动Rviz2 用于观察建图效果：

```bash
source /opt/ros/foxy/setup.bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

打开Rviz2后，需要添加“map”可视化选项，用于展示建立的地图，步骤如下所示：
![](./image/slam/rvizsetting.jpg)

地平线RDK板端运行SLAM-Toolbox：

```bash
# 配置tros.b环境
source /opt/tros/setup.bash
source /opt/ros/foxy/local_setup.bash

#启动SLAM launch文件
ros2 launch slam_toolbox online_sync_launch.py
```

PC端开启另外一个控制台，PC端启动控制工具，通过键盘控制小车运动，控制方法见控制台打印的log，在此不再赘述：

```bash
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

控制小车行驶，随着小车雷达探测到更多的环境信息，SLAM算法也建立起环境地图，可以在Rviz2上观察到建图效果。
![](./image/slam/map.jpg)

## 结果分析

在地平线RDK板端运行终端输出如下信息：

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-06-10-06-40-34-204213-ubuntu-5390
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [sync_slam_toolbox_node-1]: process started with pid [5392]
[sync_slam_toolbox_node-1] [INFO] [1654843239.403931058] [slam_toolbox]: Node using stack size 40000000
[sync_slam_toolbox_node-1] [INFO] [1654843240.092340814] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[sync_slam_toolbox_node-1] [INFO] [1654843240.096554433] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
[sync_slam_toolbox_node-1] Info: clipped range threshold to be within minimum and maximum range!
[sync_slam_toolbox_node-1] [WARN] [1654843589.431524393] [slam_toolbox]: maximum laser range setting (20.0 m) exceeds the capabilities of the used Lidar (3.5 m)
[sync_slam_toolbox_node-1] Registering sensor: [Custom Described Lidar]
```
