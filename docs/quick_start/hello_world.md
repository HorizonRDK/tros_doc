---
sidebar_position: 4
---

# 1.4 运行“Hello World”

前提：已通过deb包或者源码安装的方式成功安装TogetheROS.Bot

启动两个终端，均ssh登陆至地平线RDK或X86平台设备

第一个终端运行

```shell
source /opt/tros/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

第二个终端运行

```shell
source /opt/tros/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

运行效果如下图

![hello world](./image/hello_world/hello_world.png "hello world")
可以看到左侧终端作为pub，在不断发送“'Hello, world! N”，右侧终端作为sub端不断收到“'Hello, world! N”

OK tros.b目前已成功安装并验证！
