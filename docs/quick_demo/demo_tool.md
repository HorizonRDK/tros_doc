---
sidebar_position: 7
---

# 3.7 图像发布工具

## 功能介绍

图片发布工具支持批量读取本地图片或视频文件，并按照ROS消息格式发布，从而提高算法调试和部署效率。目前支持JPEG/JPG/PNG/NV12格式的图片，并在读取后将其转换为NV12格式进行发布。对于视频文件，支持H264/H265/MP4格式，读取视频文件后提取相关的视频流进行发布。注意，由于目前X86版本的TogetheROS.Bot hobot_codec还不支持H.264、H.265和NV12之间的转换，因此该功能无法在X86平台展示。

代码仓库:<https://github.com/HorizonRDK/hobot_image_publisher.git>

## 支持平台

| 平台    | 运行方式     |
| ------- | ------------ |
| 地平线RDK| Ubuntu 20.04 |
| X86     | Ubuntu 20.04 |

## 准备工作

### 地平线RDK平台

1. 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像

2. 地平线RDK已成功安装tros.b

3. 可以通过网络访问地平线RDK的PC

### X86平台

1. X86环境已配置Ubuntu 20.04系统镜像

2. X86环境已安装X86版本 tros.b

## 图片发布使用介绍

循环读取本地的一张NV12格式图片并发布，使用图像编解码模块将图片压缩编码成JPEG格式，在PC的Web端展示图片。

### 地平线RDK/X86平台

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的图片文件
cp -r /opt/tros/lib/hobot_image_publisher/config/ .

# 启动launch文件
ros2 launch hobot_image_publisher hobot_image_publisher_demo.launch.py
```

## 图片发布结果分析

在运行终端输出如下信息：

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-08-19-12-58-02-288516-ubuntu-24492
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launch
[INFO] [hobot_image_pub-1]: process started with pid [24511]
[INFO] [hobot_codec_republish-2]: process started with pid [24513]
[INFO] [websocket-3]: process started with pid [24519]
```

输出log显示出webserver已启动，hobot_image_pub、hobot_codec_republish、websocket都正常运行

在PC端的浏览器输入<http://IP:8000> 即可查看图像展示效果（IP为地平线RDK/X86设备的IP地址）：

![hobot_img_pub](./image/demo_tool/show.png )

## 视频发布使用介绍

读取本地video.list文件，获取list文件中的视频文件路径，循环读取视频文件并发布，先使用图像编解码模块将视频流解码成NV12格式图片，再使用图像编解码模块将图片压缩编码成JPEG格式，在PC的Web端展示图片。

### 地平线RDK平台

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的图片文件
cp -r /opt/tros/lib/hobot_image_publisher/config/ .

# 启动launch文件
ros2 launch hobot_image_publisher hobot_image_publisher_videolist_demo.launch.py
```

### X86平台

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的图片文件
cp -r /opt/tros/lib/hobot_image_publisher/config/ .

# 启动图片发布节点，使用本地MP4格式视频文件进行发布（可以根据自己的需求进行参数配置），暂不支持Web端显示
/opt/tros/lib/hobot_image_publisher/hobot_image_pub --ros-args -p image_source:=./config/video.list -p fps:=30 -p image_format:=mp4
```

## 视频发布结果分析

在运行终端输出如下信息：

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-10-22-21-44-03-663907-ubuntu-702475
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launch
[INFO] [hobot_image_pub-1]: process started with pid [702597]
[INFO] [hobot_codec_republish-2]: process started with pid [702599]
[INFO] [hobot_codec_republish-3]: process started with pid [702601]
[INFO] [websocket-4]: process started with pid [702603]
```

输出log显示出webserver已启动，hobot_image_pub、hobot_codec_republish、websocket都正常运行

在PC端的浏览器输入<http://IP:8000> 即可查看图像展示效果（IP为地平线RDK/X86设备的IP地址）：

![hobot_img_pub](./image/demo_tool/mp4show.jpg )
