---
sidebar_position: 9
---

# 2.9 文本转语音

## 功能介绍

该章节介绍如何将一段文本转化为语音信号，并播放。

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
3. 参考[智能语音章节](../boxs/box_adv#智能语音)搭建好硬件环境。
4. 音频板耳机接口连接耳机或音响。

## 使用方式

### 地平线RDK平台

1. 首次运行需要下载模型文件并解压，详细命令如下：

    ```bash
    wget http://archive.sunrisepi.tech/tts-model/tts_model.tar.gz
    sudo tar -xf tts_model.tar.gz -C /opt/tros/lib/hobot_tts/
    ```

2. 启动hobot_gpt程序

    ```bash
    source /opt/tros/setup.bash
    ros2 run hobot_tts hobot_tts
    ```

3. 新开一个终端，使用echo命令发布一条topic

   ```bash
   source /opt/tros/setup.bash
   ros2 topic pub --once /tts_text std_msgs/msg/String "{data: "你知道地平线吗？是的，我知道地平线。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。"}"
   ```

4. 耳机或音响可以听到输出的声音

## 注意事项

目前仅支持中文和英文文本内容，切记勿发布其他语言文本消息。
