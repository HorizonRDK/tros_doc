---
sidebar_position: 9
---

# 4.9 智能语音聊天机器人

## 功能介绍

智能语音聊天机器人由智能语音，GPT交互和文本转语音组成，实现人和机器人语音聊天功能。

代码仓库：<https://github.com/HorizonRDK/chat_robot.git>

## 支持平台

| 平台    | 运行方式      | 示例功能                       |
| ------- | ------------ | ------------------------------ |
| RDK X3 | Ubuntu 20.04 | 启动智能语音，GPT交互和文本转语音模块，实现人和机器人语音聊天功能 |

**注意：仅支持RDK X3，RDK X3 Module暂不支持。**

## 准备工作

### 地平线RDK平台

1. 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像。
2. 地平线RDK已成功安装TogetheROS.Bot。
3. 地平线RDK已成功安装智能语音算法包，安装命令：`apt update; apt install tros-hobot-audio`。
4. 地平线RDK已成功接好适配的音频板，耳机接口连接上耳机或音响（参考[智能语音章节](../boxs/box_adv#智能语音)）。
5. 已拥有OpenAI API key，并可正常访问OpenAI服务。

## 使用介绍

### 地平线RDK平台

1. 配置hobot_audio

    ```bash
    cp -r /opt/tros/lib/hobot_audio/config/ .
    ```

    修改 *config/audio_config.json*，`asr_mode`字段为`1`，然后加载音频驱动。

    ```bash
    bash config/audio.sh
    ```

2. 配置hobot_gpt

    ```bash
    cp -rf /opt/tros/lib/hobot_gpt/config ./
    ```

    修改 *config/gpt_config.json* ，将`api_key`字段设置为自己的OpenAI API key。

    确认网络可以访问OpenAI服务。

3. 配置hobot_tts

    首次运行需要下载模型文件并解压，详细命令如下：

    ```bash
    wget http://archive.sunrisepi.tech//tts-model/tts_model.tar.gz
    sudo tar -xf tts_model.tar.gz -C /opt/tros/lib/hobot_tts/
    ```

4. 启动程序和机器人聊天

    ```bash
    source /opt/tros/setup.bash

    # 屏蔽调式打印信息
    export GLOG_minloglevel=3

    ros2 launch chat_robot chat_robot.launch.py
    ```

    和机器人聊天时，要先使用唤醒词“地平线你好”唤醒机器人，紧接着说话，然后等待机器人语音回答。

## 结果分析

以如下终端输出信息为例分析：

```bash
root@ubuntu:~# ros2 launch chat_robot chat_robot.launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-19-15-38-09-954439-ubuntu-61993
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_audio-1]: process started with pid [62007]
[INFO] [hobot_gpt-2]: process started with pid [62009]
[INFO] [hobot_tts-3]: process started with pid [62011]
[hobot_audio-1] alsa_device_init, snd_pcm_open. handle((nil)), name(hw:0,0), direct(1), mode(0)
[hobot_audio-1] snd_pcm_open succeed. name(hw:0,0), handle(0x5588fb4a60)
[hobot_audio-1] Rate set to 16000Hz (requested 16000Hz)
[hobot_audio-1] Buffer size range from 8 to 10240
[hobot_audio-1] Period size range from 8 to 5120
[hobot_audio-1] Requested period size 512 frames
[hobot_audio-1] Periods = 4
[hobot_audio-1] was set period_size = 512
[hobot_audio-1] was set buffer_size = 2048
[hobot_audio-1] alsa_device_init. hwparams(0x5588fb4d00), swparams(0x5588fb4f70)
[hobot_audio-1] hrsc sdk wakeup word is:
[hobot_audio-1] {
[hobot_audio-1]     "cmd_word": [
[hobot_audio-1]         "地平线你好",
[hobot_audio-1]         "向前走",
[hobot_audio-1]         "向后退",
[hobot_audio-1]         "向左转",
[hobot_audio-1]         "向右转",
[hobot_audio-1]         "停止运动"
[hobot_audio-1]     ]
[hobot_audio-1] }
[hobot_audio-1]
[hobot_audio-1] [EasyDNN]: EasyDNN version = 1.6.1_(1.18.4 DNN)
[hobot_audio-1] [BPU_PLAT]BPU Platform Version(1.3.3)!
[hobot_audio-1] [HBRT] set log level as 0. version = 3.15.25.0
[hobot_audio-1] [DNN] Runtime version = 1.18.4_(3.15.25 HBRT)
[hobot_audio-1] [A][DNN][packed_model.cpp:234][Model](2023-07-19,15:38:11.553.857) [HorizonRT] The model builder version = 1  .9.10
[hobot_audio-1] [A][DNN][packed_model.cpp:234][Model](2023-07-19,15:38:12.103.275) [HorizonRT] The model builder version = 1  .9.10
[hobot_audio-1] [A][DNN][packed_model.cpp:234][Model](2023-07-19,15:38:12.791.443) [HorizonRT] The model builder version = 1  .9.10
[hobot_audio-1] recv hrsc sdk event wakeup success, wkp count is 1
[hobot_audio-1] recv hrsc sdk doa data: 220
[hobot_audio-1] asr is: 你知道地平线吗
[hobot_gpt-2] Human: 你知道地平线吗
[hobot_gpt-2] AI: 是的，我知道地平线。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。

```

该段聊天内容为：聊天者首先使用唤醒词“***地平线你好***”唤醒机器人，然后提问：“*你知道地平线吗？*”，过一段时间机器人语音回答：“*是的，我知道地平线。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。*”。
