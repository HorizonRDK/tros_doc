---
sidebar_position: 9
---

# 4.9 智能聊天机器人

## 功能介绍

智能聊天机器人由智能语音，GPT交互和文本转语音组成，实现人和机器人语音聊天功能。

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

    设置网络代理，确保可以访问OpenAI服务。

3. 配置hobot_tts

    首次运行需要下载模型文件并解压，详细命令如下：

    ```bash
    wget http://archive.sunrisepi.tech//tts-model/tts_model.tar.gz
    sudo tar -xf tts_model.tar.gz -C /opt/tros/lib/hobot_tts/
    ```

4. 启动程序和机器人聊天

    ```bash
    source /opt/tros/setup.bash
    ros2 launch chat_robot chat_robot.launch.py
    ```

    和机器人聊天时，要先使用唤醒词“地平线你好”唤醒机器人，紧接着说话，然后等待机器人语音回答。

## 结果分析

以如下终端输出信息为例分析：

```bash
root@ubuntu:~# ros2 launch chat_robot chat_robot.launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-14-18-18-11-919962-ubuntu-38023
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_audio-1]: process started with pid [38037]
[INFO] [hobot_gpt-2]: process started with pid [38039]
[INFO] [hobot_tts-3]: process started with pid [38041]
[hobot_audio-1] alsa_device_init, snd_pcm_open. handle((nil)), name(hw:0,0), direct(1), mode(0)
[hobot_audio-1] snd_pcm_open succeed. name(hw:0,0), handle(0x5589df03a0)
[hobot_audio-1] Rate set to 16000Hz (requested 16000Hz)
[hobot_audio-1] Buffer size range from 8 to 10240
[hobot_audio-1] Period size range from 8 to 5120
[hobot_audio-1] Requested period size 512 frames
[hobot_audio-1] Periods = 4
[hobot_audio-1] was set period_size = 512
[hobot_audio-1] was set buffer_size = 2048
[hobot_audio-1] alsa_device_init. hwparams(0x5589df0640), swparams(0x5589df08b0)
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
[hobot_audio-1] WARNING: Logging before InitGoogleLogging() is written to STDERR
[hobot_audio-1] I20230714 18:18:12.285497 38037 params.h:150] Resource dir ./config/hrsc/e2e_decoder_conf/
[hobot_audio-1] I20230714 18:18:12.285636 38037 params.h:151] Reading model rescore
[hobot_audio-1] [EasyDNN]: EasyDNN version = 1.6.1_(1.18.4 DNN)
[hobot_audio-1] [BPU_PLAT]BPU Platform Version(1.3.3)!
[hobot_audio-1] [HBRT] set log level as 0. version = 3.15.25.0
[hobot_audio-1] [DNN] Runtime version = 1.18.4_(3.15.25 HBRT)
[hobot_audio-1] [A][DNN][packed_model.cpp:234][Model](2023-07-14,18:18:13.493.887) [HorizonRT] The model builder version = 1.9.10
[hobot_audio-1] [A][DNN][packed_model.cpp:234][Model](2023-07-14,18:18:14.22.559) [HorizonRT] The model builder version = 1.9.10
[hobot_audio-1] [A][DNN][packed_model.cpp:234][Model](2023-07-14,18:18:14.711.958) [HorizonRT] The model builder version = 1.9.10
[hobot_audio-1] I20230714 18:18:14.840123 38037 bpu_asr_model.cc:273] Bpu Model Info:
[hobot_audio-1] I20230714 18:18:14.840202 38037 bpu_asr_model.cc:274]   chunk_size 8
[hobot_audio-1] I20230714 18:18:14.840219 38037 bpu_asr_model.cc:275]   num_left_chunks 4
[hobot_audio-1] I20230714 18:18:14.840232 38037 bpu_asr_model.cc:276]   subsampling_rate 8
[hobot_audio-1] I20230714 18:18:14.840246 38037 bpu_asr_model.cc:277]   right context 14
[hobot_audio-1] I20230714 18:18:14.840260 38037 bpu_asr_model.cc:278]   sos 5999
[hobot_audio-1] I20230714 18:18:14.840273 38037 bpu_asr_model.cc:279]   eos 5999
[hobot_audio-1] I20230714 18:18:14.840286 38037 bpu_asr_model.cc:280]   is bidirectional decoder 1
[hobot_audio-1] I20230714 18:18:14.840306 38037 bpu_asr_model.cc:281]   num_partial_chunks 8
[hobot_audio-1] I20230714 18:18:14.840320 38037 bpu_asr_model.cc:282]   num_partial_tokens 20
[hobot_audio-1] I20230714 18:18:14.840334 38037 bpu_asr_model.cc:283]   num_units 6000
[hobot_audio-1] I20230714 18:18:14.840348 38037 bpu_asr_model.cc:284]   hidden_dim 256
[hobot_audio-1] I20230714 18:18:14.840361 38037 bpu_asr_model.cc:285]   valid_mask_value 1
[hobot_audio-1] I20230714 18:18:14.840412 38037 bpu_asr_model.cc:286]   invalid_mask_value 0
[hobot_audio-1] I20230714 18:18:14.840436 38037 params.h:161] Using BpuAsrModel
[hobot_audio-1] I20230714 18:18:14.840452 38037 params.h:177] Reading unit table units.txt
[hobot_audio-1] I20230714 18:18:14.852633 38037 params.h:185] Reading fst lm/TLG.fst
[hobot_audio-1] I20230714 18:18:14.852851 38037 mapped-file.cc:40] memorymap: false source: "./config/hrsc/e2e_decoder_conf/lm/TLG.fst" size: 20958100 offset: 65
[hobot_audio-1] I20230714 18:18:14.873353 38037 mapped-file.cc:40] memorymap: false source: "./config/hrsc/e2e_decoder_conf/lm/TLG.fst" size: 59234720 offset: 20958165
[hobot_audio-1] I20230714 18:18:14.959647 38037 params.h:191] Reading symbol table lm/words.txt
[hobot_audio-1] W20230714 18:18:15.092751 38037 params.h:207] Load bpe model failed bpe_model.model
[hobot_audio-1] E20230714 18:18:15.092916 38037 post_text_process.cc:34] several paths are empty, tagger fst: , verbalizer fst:
[hobot_audio-1] E20230714 18:18:15.092952 38037 post_text_process.cc:34] several paths are empty, tagger fst: , verbalizer fst:
[hobot_audio-1] I20230714 18:18:15.093080 38037 post_replace.cc:44] h u d    hud
[hobot_audio-1] I20230714 18:18:15.093109 38037 post_replace.cc:44] l d w    ldw
[hobot_audio-1] I20230714 18:18:15.093128 38037 post_replace.cc:44] a p a    apa
[hobot_audio-1] I20230714 18:18:15.093144 38037 post_replace.cc:44] l k a    lka
[hobot_audio-1] I20230714 18:18:15.093163 38037 post_replace.cc:44] applecarplay    apple carplay
[hobot_audio-1] I20230714 18:18:15.093179 38037 post_replace.cc:44] 打开a c    打开ac
[hobot_audio-1] I20230714 18:18:15.093196 38037 post_replace.cc:44] 关闭a c    关闭ac
[hobot_audio-1] I20230714 18:18:15.093212 38037 post_replace.cc:44] 一档    一挡
[hobot_audio-1] I20230714 18:18:15.093227 38037 post_replace.cc:44] 二档    二挡
[hobot_audio-1] I20230714 18:18:15.093245 38037 post_replace.cc:44] 三档    三挡
[hobot_audio-1] I20230714 18:18:15.093261 38037 post_replace.cc:44] 四档    四挡
[hobot_audio-1] I20230714 18:18:15.093277 38037 post_replace.cc:44] 五档    五挡
[hobot_audio-1] I20230714 18:18:15.093293 38037 post_replace.cc:44] 六档    六挡
[hobot_audio-1] I20230714 18:18:15.093308 38037 post_replace.cc:44] 七档    七挡
[hobot_audio-1] I20230714 18:18:15.093324 38037 post_replace.cc:44] 八档    八挡
[hobot_audio-1] I20230714 18:18:15.093340 38037 post_replace.cc:44] 九档    九挡
[hobot_audio-1] I20230714 18:18:15.093355 38037 post_replace.cc:44] 十档    十挡
[hobot_audio-1] I20230714 18:18:15.093375 38037 post_replace.cc:44] 农一点    浓一点
[hobot_audio-1] I20230714 18:18:15.097651 38037 bpu_asr_model.h:32] bpu model`s chunk_size can`t be set dynamically, bpu will read fixed chunk_size from model.flag
[hobot_audio-1] I20230714 18:18:15.097786 38037 bpu_asr_model.h:37] bpu model`s num_left_chunk can`t be set dynamically, bpu will read fixed num_left_chunk from model.flag
[hobot_audio-1] E20230714 18:18:15.097821 38037 decoder_api.cc:119] PostWakeWord init failed
[hobot_audio-1] E20230714 18:18:15.101415 38037 decoder_api.cc:119] PostWakeWord init failed
[hobot_audio-1] E20230714 18:18:15.104702 38037 decoder_api.cc:119] PostWakeWord init failed
[hobot_audio-1] E20230714 18:18:15.120872 38037 decoder_api.cc:119] PostWakeWord init failed
[hobot_audio-1] I20230714 18:18:15.527793 38075 context_graph.cc:95] Num of contexts: 6
[hobot_tts-3] WARNING: Logging before InitGoogleLogging() is written to STDERR
[hobot_tts-3] I20230714 18:18:15.573621 38041 onnx_model.cc:56] Input 0 : name=input type=7 dims=1 -1
[hobot_tts-3] I20230714 18:18:15.573782 38041 onnx_model.cc:74] Output 0 : name=phone_output type=1 dims=-1 -1 876
[hobot_tts-3] I20230714 18:18:15.573812 38041 onnx_model.cc:74] Output 1 : name=prosody_output type=1 dims=-1 -1 5
[hobot_audio-1] recv hrsc sdk event wakeup success, wkp count is 1
[hobot_audio-1] recv hrsc sdk doa data: 220
[hobot_audio-1] asr is: 你知道地平线吗
[hobot_gpt-2] Human: 你知道地平线吗
[hobot_gpt-2] AI: 是的，我知道地平线。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。
[hobot_tts-3] I20230714 18:18:28.474123 38041 onnx_model.cc:56] Input 0 : name=input type=7 dims=-1 -1
[hobot_tts-3] I20230714 18:18:28.474218 38041 onnx_model.cc:56] Input 1 : name=input_lengths type=7 dims=-1
[hobot_tts-3] I20230714 18:18:28.474246 38041 onnx_model.cc:56] Input 2 : name=scales type=1 dims=-1 3
[hobot_tts-3] I20230714 18:18:28.474269 38041 onnx_model.cc:56] Input 3 : name=sid type=7 dims=-1
[hobot_tts-3] I20230714 18:18:28.474299 38041 onnx_model.cc:74] Output 0 : name=output type=1 dims=-1 -1 -1
[hobot_tts-3] I20230714 18:18:28.572036 38079 onnx_g2p_prosody.cc:76] Sentence after frontend:
[hobot_tts-3] text: 是的
[hobot_tts-3] norm_text: 是的
[hobot_tts-3] tokens: 是 的
[hobot_tts-3] token_ids: 3221 4638
[hobot_tts-3] seg_tokens:
[hobot_tts-3]   [CLS] 是 的 [SEP]
[hobot_tts-3] seg_token_ids:
[hobot_tts-3]   101 3221 4638 102
[hobot_tts-3] seg_pinyins:
[hobot_tts-3]   shi4 de5
[hobot_tts-3] seg_prosodys:
[hobot_tts-3]   #0 #2
[hobot_tts-3] pinyins: shi4 de5
[hobot_tts-3] prosodys: #0 #4
[hobot_tts-3] phonemes: sh iii4 #0 d e5 #4
[hobot_tts-3] I20230714 18:18:29.153384 38079 onnx_g2p_prosody.cc:76] Sentence after frontend:
[hobot_tts-3] text: 我知道地平线
[hobot_tts-3] norm_text: 我知道地平线
[hobot_tts-3] tokens: 我 知 道 地 平 线
[hobot_tts-3] token_ids: 2769 4761 6887 1765 2398 5296
[hobot_tts-3] seg_tokens:
[hobot_tts-3]   [CLS] 我 知 道 地 平 线 [SEP]
[hobot_tts-3] seg_token_ids:
[hobot_tts-3]   101 2769 4761 6887 1765 2398 5296 102
[hobot_tts-3] seg_pinyins:
[hobot_tts-3]   wo3 zhi1 dao4 di4 ping2 xian4
[hobot_tts-3] seg_prosodys:
[hobot_tts-3]   #1 #0 #0 #1 #0 #3
[hobot_tts-3] pinyins: wo3 zhi1 dao4 di4 ping2 xian4
[hobot_tts-3] prosodys: #1 #0 #0 #1 #0 #4
[hobot_tts-3] phonemes: ^ uo3 #1 zh iii1 #0 d ao4 #0 d i4 #1 p ing2 #0 x ian4 #4
[hobot_tts-3] I20230714 18:18:30.206856 38079 onnx_g2p_prosody.cc:76] Sentence after frontend:
[hobot_tts-3] text: 它是一条从地面延伸到天空的线
[hobot_tts-3] norm_text: 它是一条从地面延伸到天空的线
[hobot_tts-3] tokens: 它 是 一 条 从 地 面 延 伸 到 天 空 的 线
[hobot_tts-3] token_ids: 2124 3221 671 3340 794 1765 7481 2454 847 1168 1921 4958 4638 5296
[hobot_tts-3] seg_tokens:
[hobot_tts-3]   [CLS] 它 是 一 条 从 地 面 延 伸 到 天 空 的 线 [SEP]
[hobot_tts-3] seg_token_ids:
[hobot_tts-3]   101 2124 3221 671 3340 794 1765 7481 2454 847 1168 1921 4958 4638 5296 102
[hobot_tts-3] seg_pinyins:
[hobot_tts-3]   ta1 shi4 yi1 tiao2 cong2 di4 mian4 yan2 shen1 dao4 tian1 kong1 de5 xian4
[hobot_tts-3] seg_prosodys:
[hobot_tts-3]   #0 #2 #0 #2 #1 #0 #1 #0 #0 #1 #0 #0 #0 #0
[hobot_tts-3] pinyins: ta1 shi4 yi4 tiao2 cong2 di4 mian4 yan2 shen1 dao4 tian1 kong1 de5 xian4
[hobot_tts-3] prosodys: #0 #2 #0 #2 #1 #0 #1 #0 #0 #1 #0 #0 #0 #4
[hobot_tts-3] phonemes: t a1 #0 sh iii4 #2 ^ i4 #0 t iao2 #2 c ong2 #1 d i4 #0 m ian4 #1 ^ ian2 #0 sh en1 #0 d ao4 #1 t ian1 #0 k ong1 #0 d e5 #0 x ian4 #4
[hobot_tts-3] I20230714 18:18:32.292238 38079 onnx_g2p_prosody.cc:76] Sentence after frontend:
[hobot_tts-3] text: 它定义了地面和天空之间的分界线
[hobot_tts-3] norm_text: 它定义了地面和天空之间的分界线
[hobot_tts-3] tokens: 它 定 义 了 地 面 和 天 空 之 间 的 分 界 线
[hobot_tts-3] token_ids: 2124 2137 721 749 1765 7481 1469 1921 4958 722 7313 4638 1146 4518 5296
[hobot_tts-3] seg_tokens:
[hobot_tts-3]   [CLS] 它 定 义 了 地 面 和 天 空 之 间 的 分 界 线 [SEP]
[hobot_tts-3] seg_token_ids:
[hobot_tts-3]   101 2124 2137 721 749 1765 7481 1469 1921 4958 722 7313 4638 1146 4518 5296 102
[hobot_tts-3] seg_pinyins:
[hobot_tts-3]   ta1 ding4 yi4 le5 di4 mian4 he2 tian1 kong1 zhi1 jian1 de5 fen1 jie4 xian4
[hobot_tts-3] seg_prosodys:
[hobot_tts-3]   #1 #0 #0 #1 #0 #2 #1 #0 #1 #0 #0 #1 #0 #0 #3
[hobot_tts-3] pinyins: ta1 ding4 yi4 le5 di4 mian4 he2 tian1 kong1 zhi1 jian1 de5 fen1 jie4 xian4
[hobot_tts-3] prosodys: #1 #0 #0 #1 #0 #2 #1 #0 #1 #0 #0 #1 #0 #0 #4
[hobot_tts-3] phonemes: t a1 #1 d ing4 #0 ^ i4 #0 l e5 #1 d i4 #0 m ian4 #2 h e2 #1 t ian1 #0 k ong1 #1 zh iii1 #0 j ian1 #0 d e5 #1 f en1 #0 j ie4 #0 x ian4 #4

```

该段聊天内容为：聊天者首先使用唤醒词“*地平线你好*”唤醒机器人，然后提问“*你知道地平线吗？*”，过一段时间机器人回答“*是的，我知道地平线。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。*”。
