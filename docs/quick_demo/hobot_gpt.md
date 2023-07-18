---
sidebar_position: 8
---

# 2.8 GPT交互

## 功能介绍

本章节介绍如何调用OpenAI的API，实现和GPT交互功能。当前支持两种交互模式，一种是聊天模式，支持多轮对话，注意该方式消耗token较多，另一种方式是问答模式，该模式只支持单轮对话。交互模式可通过参数，默认模式为问答模式。

代码仓库：<https://github.com/HorizonRDK/hobot_gpt.git>

## 支持平台

| 平台    | 运行方式     | 示例功能                       |
| ------- | ------------ | ------------------------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 | 订阅文本消息，然后调用OpenAI API，最后将获取到的结果发布出去 |

## 准备工作

### 地平线RDK平台

1. 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像。
2. 地平线RDK已成功安装TogetheROS.Bot。
3. 已注册OpenAI账号，并创建API key。
4. 网络可以正常访问OpenAI服务。

## 使用方式

### 地平线RDK平台

1. 拷贝配置文件到当前目录

    ```bash
    cp -rf /opt/tros/lib/hobot_gpt/config ./
    ```

2. 修改 *config/gpt_config.json* ，将`api_key`字段设置为自己的OpenAI API key，同时设置交互模式，`chat_mode_enable`为`true`表示使用聊天模式，`false`表示问答模式。
3. 设置网络代理，确保可以访问OpenAI服务
4. 启动hobot_gpt程序

    ```bash
    source /opt/tros/setup.bash
    ros2 run hobot_gpt hobot_gpt
    ```

5. 新开一个终端，使用echo命令发布一条topic

   ```bash
   source /opt/tros/setup.bash
   ros2 topic pub --once /audio_asr std_msgs/msg/String "{data: "你知道地平线吗"}"
   ```

6. 在hobot_gpt终端页面，查看结果

  ```bash
  root@ubuntu:~# ros2 run hobot_gpt hobot_gpt
  Human: 你知道地平线吗
  AI: 是的，我知道地平线。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。
  ```

## 注意事项

调用OpenAI API会消耗账户余额，使用前需确认账户有可用余额。
