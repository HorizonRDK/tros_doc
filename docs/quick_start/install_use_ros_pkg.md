---
sidebar_position: 5
---

# 2.5 使用ROS2 package

前提：已成功安装TogetheROS.Bot

tros.b foxy版本接口完全兼容，能够复用ROS丰富工具包，这里以安装和使用ROS foxy版本ros-foxy-image-transport为例介绍如何在tros.b中使用ROS package。

## 安装ROS2 package

### 1 添加ROS apt源

添加ROS apt源

```shell
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

更新apt仓库

```shell
sudo apt update
```

### 2 安装packages

```shell
sudo apt install ros-foxy-image-transport
sudo apt install ros-foxy-image-transport-plugins
```

### 3 建立软连接

```shell
cd /opt/tros
## 使用/opt/tros目录下的create_soft_link.py创建ROS package至tros.b的软连接
sudo python3 create_soft_link.py --foxy /opt/ros/foxy/ --tros /opt/tros/
```

## 使用ROS2 package

与ROS使用一样

```shell
source /opt/tros/local_setup.bash
ros2 run image_transport list_transports
```

运行结果如下，展示了image_transport package支持的图像格式

```shell
root@ubuntu:/opt/tros# ros2 run image_transport list_transports
Declared transports:
image_transport/compressed
image_transport/compressedDepth
image_transport/raw
image_transport/theora

Details:
----------
"image_transport/compressed"
 - Provided by package: compressed_image_transport
 - Publisher:
      This plugin publishes a CompressedImage using either JPEG or PNG compression.

 - Subscriber:
      This plugin decompresses a CompressedImage topic.

----------
"image_transport/compressedDepth"
 - Provided by package: compressed_depth_image_transport
 - Publisher:
      This plugin publishes a compressed depth images using PNG compression.

 - Subscriber:
      This plugin decodes a compressed depth images.

----------
"image_transport/raw"
 - Provided by package: image_transport
 - Publisher:
      This is the default publisher. It publishes the Image as-is on the base topic.

 - Subscriber:
      This is the default pass-through subscriber for topics of type sensor_msgs/Image.

----------
"image_transport/theora"
 - Provided by package: theora_image_transport
 - Publisher:
      This plugin publishes a video packet stream encoded using Theora.

 - Subscriber:
      This plugin decodes a video packet stream encoded using Theora.
```
