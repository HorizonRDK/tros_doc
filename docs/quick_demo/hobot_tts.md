---
sidebar_position: 8
---

# 2.8 文本转语音

## 功能介绍

本章节介绍如何将一段文本转化为语音信号，并通过音频输出接口播放。

代码仓库：<https://github.com/HorizonRDK/hobot_tts.git>

## 支持平台

| 平台    | 运行方式     | 示例功能                       |
| ------- | ------------ | ------------------------------ |
| RDK X3 | Ubuntu 20.04 | 订阅文本消息，然后转化为语音数据，最后播放出去 |

**注意：仅支持RDK X3，RDK X3 Module暂不支持。**

## 准备工作

### 地平线RDK平台

1. 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像。
2. 地平线RDK已成功安装TogetheROS.Bot。
3. 已有地平线专用音频驱动板，并参考[智能语音章节](../boxs/box_adv#智能语音)搭建好硬件环境。
4. 音频板耳机接口连接耳机或音响。

## 使用方式

### 地平线RDK平台

1. 首次运行需要下载模型文件并解压，详细命令如下：

    ```bash
    wget http://sunrise.horizon.cc/tts-model/tts_model.tar.gz
    sudo tar -xf tts_model.tar.gz -C /opt/tros/lib/hobot_tts/
    ```

2. 上电后首次运行需要运行如下命令加载音频驱动：

    ```bash
    echo 112 >/sys/class/gpio/export
    echo out >/sys/class/gpio/gpio112/direction
    echo 1 >/sys/class/gpio/gpio112/value
    echo 118 >/sys/class/gpio/export
    echo out >/sys/class/gpio/gpio118/direction
    echo 1 >/sys/class/gpio/gpio118/value

    modprobe -r es7210
    modprobe -r es8156
    modprobe -r hobot-i2s-dma
    modprobe -r hobot-cpudai
    modprobe -r hobot-snd-7210

    modprobe es7210
    modprobe es8156
    modprobe hobot-i2s-dma
    modprobe hobot-cpudai
    modprobe hobot-snd-7210 snd_card=5
    ```

    运行上述命令后使用如下命令可检查是否加载成功：

    ```bash
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
    ```

    如果出现类似`pcmC0D1p`新增音频播放设备则表示加载成功。

3. 启动hobot_tts程序

    ```bash
    source /opt/tros/setup.bash

    # 屏蔽调式打印信息
    export GLOG_minloglevel=1

    ros2 run hobot_tts hobot_tts
    ```

    注意：加载音频驱动时，若新增音频设备不是`pcmC0D1p`，则需要使用参数`playback_device`指定播放音频设备。例如新增音频播放设备`pcmC1D1p`，启动命令为：`ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="hw:1,1"`

4. 新开一个终端，使用echo命令发布一条topic

   ```bash
   source /opt/tros/setup.bash
   ros2 topic pub --once /tts_text std_msgs/msg/String "{data: ""你知道地平线吗？是的，我知道地平线。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。""}"
   ```

5. 耳机或音响可以听到播放的声音

## 注意事项

目前仅支持中文和英文文本内容，切记勿发布其他语言文本消息。
