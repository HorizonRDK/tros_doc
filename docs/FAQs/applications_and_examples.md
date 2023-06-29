---
sidebar_position: 2
---

# 6.2 应用和示例

## 第三方库在RDK X3上的安装/交叉编译和使用

如需交叉编译部署，请参考[交叉编译环境部署](https://developer.horizon.ai/forumDetail/112555549341653662)

## 编译大型程序过程中提示编译进程被kill错误日志如何解决？

参考[Swap使用教程](https://developer.horizon.ai/forumDetail/98129467158916281)

```shell
sudo mkdir -p /swapfile 
cd /swapfile 
sudo dd if=/dev/zero of=swap bs=1M count=1024 
sudo chmod 0600 swap 
sudo mkswap -f swap 
sudo swapon swap 
free

```

## 如何运行GC4633摄像头示例？

python已提供基于F37和GC4663 camera的FCOS算法示例，会自动检测camera进行算法推理。

```bash
cd /app/ai_inference/03_mipi_camera_sample
sudo python3 mipi_camera.py
```

然后HDMI接屏幕可以渲染图像和算法展示结果。

## 使用rqt_image_view查看RDK X3发布的RGB888 RAW图卡顿，甚至无法接收?

FastDDS在UDP协议层没有实现MTU分片，导致IP层进行分片，当UDP数据过大时由于常见路由器、网卡均无法缓冲大量分片，且某一分片丢失会导致重传所有分片，进而形成IP fragmentation attack，表现为同一网段机器通信卡顿。更换cycloneDDS（命令：export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp），或者发送小分辨率jpeg格式图片降低传输数据量。

## Linux镜像是否支持板端编译?

Linux镜像rootfs经过最小剪裁，无法支持板端编译。

## Linux镜像上如何运行示例

手册中的示例是以Ubuntu系统举例介绍，示例的运行依赖python，这些示例同样可以运行在烧录Linux镜像（无python）的RDK X3上。

- Ubuntu系统和Linux镜像启动示例说明

在Ubuntu系统中，启动一个示例分为三步：

1、使用`source /opt/tros/setup.bash`命令配置`tros.b`环境

2、拷贝需要的配置文件到执行路径下

3、使用`ros2 run`或者`ros2 launch`启动`tros.b`中的package

在Linux镜像上，这三步分别为：

1、使用`export LD_LIBRARY_PATH`命令配置`tros.b`环境

2、拷贝需要的配置文件到执行路径下

3、启动示例程序，并指定启动参数。对于使用C++编写的package，每个package对应于一个可执行程序。

下面使用算法推理示例举例，说明如何将launch脚本内容转换成Linux镜像上的命令。示例输入一张本地图片进行推理，得到渲染后的图片并保存在本地。

- Ubuntu系统上启动命令

命令如下：

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。config中为example使用的模型，回灌使用的本地图片
cp -r /opt/tros/lib/dnn_node_example/config/ .

# 使用本地jpg格式图片进行回灌预测，并存储渲染后的图片
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py
```

- launch脚本路径

此示例使用`ros2 launch`启动了`dnn_node_example package`，在RDK X3的`tros.b`安装路径`/opt/tros/`中查找launch脚本`dnn_node_example_feedback.launch.py`所在路径：

```shell
# find /opt/tros/ -name dnn_node_example_feedback.launch.py
/opt/tros/share/dnn_node_example/launch/dnn_node_example_feedback.launch.py
```

- launch脚本内容

launch脚本`dnn_node_example_feedback.launch.py`主要内容如下：

```python
def generate_launch_description():
    config_file_launch_arg = DeclareLaunchArgument(
        "dnn_example_config_file", default_value=TextSubstitution(text="config/fcosworkconfig.json")
    )

    img_file_launch_arg = DeclareLaunchArgument(
        "dnn_example_image", default_value=TextSubstitution(text="config/test.jpg")
    )

    # 拷贝config中文件
    dnn_node_example_path = os.path.join(
        get_package_prefix('dnn_node_example'),
        "lib/dnn_node_example")
    print("dnn_node_example_path is ", dnn_node_example_path)
    cp_cmd = "cp -r " + dnn_node_example_path + "/config ."
    print("cp_cmd is ", cp_cmd)
    os.system(cp_cmd)

    return LaunchDescription([
        config_file_launch_arg,
        img_file_launch_arg,
        # 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
        Node(
            package='dnn_node_example',
            executable='example',
            output='screen',
            parameters=[
                {"feed_type": 0},
                {"config_file": LaunchConfiguration(
                    'dnn_example_config_file')},
                {"image": LaunchConfiguration('dnn_example_image')},
                {"image_type": 0},
                {"dump_render_img": 1}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
```

- launch脚本说明

launch脚本支持通过参数`dnn_example_config_file`指定配置文件选择运行的算法，参数`dnn_example_image`指定算法推理使用的图片。示例启动时未指定，使用的是默认配置。

launch脚本中`package`参数指定了启动的package名为`dnn_node_example`，`executable`参数指定可执行程序名为`example`，`parameters`参数指定了传给可执行程序的参数。ros2 launch的详细使用说明参考[ROS2手册](http://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html)。

- package和可执行程序路径

在RDK X3的`tros.b`安装路径`/opt/tros/`中查找：

```shell
# find /opt/tros/ -name dnn_node_example -type d
/opt/tros/lib/dnn_node_example
/opt/tros/share/dnn_node_example

# ls /opt/tros/lib/dnn_node_example
config  example
```

可以看到可执行程序`example`在`/opt/tros/lib/dnn_node_example`路径下。

- Linux镜像上启动可执行程序

运行上一步查找到的可执行程序，并且带上launch脚本中`parameters`参数：`/opt/tros/lib/dnn_node_example/example --ros-args -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1`。

- 完整的Linux镜像上运行示例

```shell
# 配置tros.b环境
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/tros/lib/

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。config中为example使用的模型，回灌使用的本地图片
cp -r /opt/tros/lib/dnn_node_example/config/ .

# 使用本地jpg格式图片进行回灌预测，并存储渲染后的图片
/opt/tros/lib/dnn_node_example/example --ros-args -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1
```

## 交叉编译TogetheROS.Bot源码速度慢

tros.b的package较多，源码编译需要一些时间（8核CPU & 32G内存配置下编译完成大概需要20分钟）。有两种加速方法：

1 最小化编译

编译脚本中提供了两种编译方式，分别为`all_build.sh`完整编译（手册交叉编译章节中默认的编译方式）和`minimal_build.sh`最小化编译。其中最小化编译不编译算法示例和测试用例，编译速度快。

使用最小化编译的方法是将手册交叉编译章节中配置编译选项命令`./robot_dev_config/all_build.sh`替换为`./robot_dev_config/minimal_build.sh`。

2 手动忽略编译不需要的package

在package源码目录下生成`COLCON_IGNORE`文件，编译的时候将会忽略编译此package。

下载下来的package源码目录在`robot_dev_config/ros2_release.repos`中指定，例如下载`google_benchmark_vendor`时配置如下：

```text
  ament/google_benchmark_vendor:
    type: git
    url: https://github.com/ament/google_benchmark_vendor.git
    version: 0.0.7
```

说明`google_benchmark_vendor` package源码下载在了`src/ament/google_benchmark_vendor`路径下，因此执行命令`touch src/ament/google_benchmark_vendor/COLCON_IGNORE`忽略`google_benchmark_vendor` package的编译。

## 是否支持安装使用其他版本ROS?

支持。

RDK X3上安装了tros.b之后，还可以安装使用其他版本的ROS，包括ROS1。使用时需要注意，一个终端下只能source一个版本的ROS。

此外tros.b与ROS foxy版本接口完全兼容，不需要安装ROS foxy也能够复用ROS丰富工具包。

## colcon编译报错

如果使用`colcon build`命令编译pkg报如下错误：

```shell
root@ubuntu:~/hobot_cam# colcon build
[4.933s] ERROR:colcon.colcon_core.package_identification:Exception in package identification extension 'ros' in 'hobot_cam': module 'pyparsing' has no attribute 'operatorPrecedence'
Traceback (most recent call last):
  File "/usr/lib/python3/dist-packages/catkin_pkg/condition.py", line 23, in evaluate_condition
    expr = _get_condition_expression()
  File "/usr/lib/python3/dist-packages/catkin_pkg/condition.py", line 44, in _get_condition_expression
    _condition_expression = pp.operatorPrecedence(
AttributeError: module 'pyparsing' has no attribute 'operatorPrecedence'
```
可能是`python3-catkin-pkg`版本较低，condition功能支持不完备。

## 解决方法 ##

升级`python3-catkin-pkg`版本，步骤如下：

```shell
# 添加ROS apt源
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 删除老版本
sudo apt remove python3-catkin-pkg

# 安装新版本
sudo apt update
sudo apt install python3-catkin-pkg
```

## 如何查看tros.b版本

tros.b安装完成后，登录系统并使用命令`apt show tros`查看tros.b版本。

2.x版本（以2.0.0版本为例说明）tros.b信息如下：

```shell
root@ubuntu:~# apt show tros
Package: tros
Version: 2.0.0-20230523223852
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: unknown
Depends: hobot-models-basic, tros-ros-base, tros-ai-msgs, tros-audio-control, tros-audio-msg, tros-audio-tracking, tros-body-tracking, tros-dnn-benchmark-example, tros-dnn-node, tros-dnn-node-example, tros-dnn-node-sample, tros-elevation-net, tros-gesture-control, tros-hand-gesture-detection, tros-hand-lmk-detection, tros-hbm-img-msgs, tros-hobot-app-xrrobot-body-tracking, tros-hobot-app-xrrobot-gesture-control, tros-hobot-codec, tros-hobot-cv, tros-hobot-falldown-detection, tros-hobot-hdmi, tros-hobot-image-publisher, tros-hobot-mot, tros-hobot-usb-cam, tros-image-subscribe-example, tros-img-msgs, tros-imu-sensor, tros-line-follower-model, tros-line-follower-perception, tros-mipi-cam, tros-mono2d-body-detection, tros-mono2d-trash-detection, tros-mono3d-indoor-detection, tros-parking-perception, tros-parking-search, tros-rgbd-sensor, tros-websocket, tros-xrrobot, tros-xrrobot-msgs
Download-Size: 980 B
APT-Manual-Installed: yes
APT-Sources: http://archive.sunrisepi.tech/ubuntu-rdk focal/main arm64 Packages
Description: TogetheROS Bot

root@ubuntu:~#

```

1.x版本（以1.1.6版本为例说明）tros.b信息如下：

```shell
root@ubuntu:~# apt show tros
Package: tros
Version: 1.1.6
Section: utils
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: 1,536 MB
Pre-Depends: hhp-verify
Depends: symlinks, locales, hhp-verify, hobot-models-basic, hobot-arm64-libs (>= 1.1.6)
Apt-Sources: http://archive.sunrisepi.tech/ubuntu-ports focal/main arm64 Packages
Date: 2023-03-24_17-29-12
Download-Size: 116 MB
APT-Manual-Installed: yes
Description: TogetherROS

N: There are 7 additional records. Please use the '-a' switch to see them.
root@ubuntu:~#

```

## 1.x和2.x版本tros.b说明，以及和系统版本、RDK平台硬件对应关系

- 2.x版本tros.b：仅支持2.x版本系统；支持RDK X3、RDK X3 Module等全系列硬件；未来tros.b的新增功能将会发布在2.x版本tros.b；代码托管在github。

- [1.x版本tros.b](https://developer.horizon.ai/api/v1/fileData/TogetherROS/index.html)：历史版本；仅支持1.x版本系统和RDK X3；未来1.x版本tros.b仅发布问题修复版本；代码托管在gitlab。

**注意：1.x版本tros.b无法通过apt命令直接升级到2.x版本tros.b，需要以烧录镜像的方式重新[安装系统](https://developer.horizon.ai/api/v1/fileData/documents_rdk/getting_start/quick_start.html#id3)后再安装2.x版本tros.b。**
