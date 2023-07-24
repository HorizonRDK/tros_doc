---
sidebar_position: 6
---

# 1.6 版本发布记录

## 版本号：2.0-Release（2.0.1）

功能变更：

- 升级语音算法，优化ASR（语音识别）效果。
- 优化算法示例的`model_name`配置项，从模型文件中自动解析`model_name`配置，解决参数配置错误导致的加载模型失败问题，提升算法二次开发的易用性。
- tros.b安装包中不再包含nav2功能包，用户直接在RDK上使用apt命令安装ROS2最新版本的nav2功能包，解决老版本nav2存在的稳定性问题。

新增功能：

- 新增支持`RDK Ultra`平台。
- 新增Trigger事件触发并获取和可视化rosbag数据的`hobot_trigger`和`hobot_visalization`等node，帮助用户定位、复现和可视化机器人场景中的感知、规控等问题。同时用户可以二次开发实现数据触发、录制和实时回传的功能。
- USB图像采集node自适应USB摄像头的设备号，降低用户使用USB摄像头的门槛。
- 新增视觉惯性里程计（Visual Inertial Odometry，VIO）算法node，基于视觉实现低成本、鲁棒性高的机器人高精度定位算法。
- 新增文本转语音的`hobot_tts` node，实现将文本转化为语音进行播报的功能。
- 新增激光雷达目标检测算法`hobot_centerpoint` node。
- 新增BEV感知算法`hobot_bev` node。
- 新增双目深度估计算法`hobot_stereonet` node。

问题修复：

- 升级`RDK X3`的easydnn（版本号1.6.1）和dnn（版本号1.18.4），修复算子crash问题以及支持更多算子。
- 修复RGBD图像采集node发布的深度数据错误的问题。

其他更新：

- 优化人体检测和跟踪算法node，支持根据输入图像分辨率自适应输出的算法感知结果坐标。
- 修复orb_slam3算法编译脚本路径错误导致的编译失败问题。


## 版本号：2.0-Beta（2.0.0）

2.0-Beta（2.0.0）是第一个2.x版本tros.b，建议[1.x版本tros.b](https://developer.horizon.cc/api/v1/fileData/TogetherROS/index.html)的用户升级到2.x版本。

功能变更：

- 代码托管平台从Gitlab更换为GitHub，方便更多开发者进行二次开发。
- 集成更高效的包管理机制，加快版本升级效率，让机器人应用安装更加便捷。

新增功能：

- 支持全新的核心板开发套件RDK X3 Module。
- hobot_audio增加语音ASR识别结果输出，方便用于开发语音应用。

问题修复：

- 修复dnn_node内置的MobileNet_SSD模型后处理在多线程情况下崩溃问题。
- 修复X86平台下dnn_node使用DDR输入模型推理失败问题
- 修复X86平台下hobot_codec和hobot_image_publisher编译失败问题。

其他更新：

- 更新示例的launch启动脚本，应用引用依赖模块的launch脚本并配置参数。
- webscoket更新展示端的地平线logo。