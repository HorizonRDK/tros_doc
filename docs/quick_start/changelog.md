---
sidebar_position: 6
---

# 2.6 版本发布记录

## 版本号：2.0-Beta（2.0.0）

2.0-Beta（2.0.0）是第一个2.x版本tros.b，建议[1.x版本tros.b](https://developer.horizon.ai/api/v1/fileData/TogetherROS/index.html)的用户升级到2.x版本。

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