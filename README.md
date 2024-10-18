###### ![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=ZDg1YTU1MDZiMDZjYzQ5ZGZlOTA5NWU1YzdiMjE5YTZfdGVLOHFDSlZFNzM2UXlYcjBrNm4zTVgwM3Z0dWdLbVRfVG9rZW46TktZQmJ4RHJqbzE2WFN4bzhnaWN4ZkRHbmZjXzE3MjkyMzI4Nzg6MTcyOTIzNjQ3OF9WNA)

#                              **知存科技Denoise Demo使用说明**                                                                                                                  

​                                                                                                             **  文档版本V0.1**

​                                                                                                       **发布日期2024年10月17日**



# 一、背景介绍

## 1、WTMDK2101-X3介绍

WTMDK2101-X3是针对WTM2101 AI SOC设计的评估板，包含：

1. WTM2101核心板，即我们的存算芯片。
2. 和I/O 板：WTM2101运行需要的电源、以及应用I/O接口等.

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=NWFhNzRkZjdlYjk2YWEzZDQzZmEwMzNmZTA5YjA2YmNfYndlTU5nQ2Q1eVNCZlJlT0FCRWczVUVzUnhla3gyOHJfVG9rZW46RmNkRmJKN0lIb3lUbm14cHVPemNuTzNmblFhXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

核心板示意图

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=MWUwMGM4ZTU4YWU2ODZhODlmMzczODA4YjdjY2I3YjdfRVBrMnZxWW9BSFF5cklKdVlWVTZXVTFLVFFwTWtjaXNfVG9rZW46Q2FsSGJDVTBtb2ZzUnZ4QnBpMmNZQmZmblFmXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

WTMDK2101-X3 I/O 板示意图

二、开发说明

###### **Tools 下载：**

1. amaoComConfig.ini
2. amaoComV4.9
3. JLink_Windows_V786e_x86_64
4. Miniconda3-py311_24.7.1-0-Window-x86_64
5. WSSetup_1.5.7
6. Xftp-7.0.0162p
7. Xshell-7.0.0164p



**目录结构:**

witmem_denoise_demo_v0.1

|——  doc         一些工具或工程的说明文档

|——  mapper      工具链的输入及产物示例

|——  npu_modelregister.c目录

|——  python      模型训练、onnx模型转换代码

|——  tools        串口工具，环境.whl文件等

|——  upper_machine灌数据软件

|——  witinkws_WTM2101_P1板端工程示例

|——  WitinProgramTool_WTM2101npu烧录板工具

|——  WitmemOTATool_Jlinkota烧录工具

|——  WTM2101_SDK wtm sdk

## 1、模型训练

本demo提供简单的dnn网络结构用于降噪演示：

1. python环境安装，参考文档doc/python环境安装.txt，推荐在windows下运行。
2. 进入代码目录cd python，运行train.py，训练降噪模型，默认50 epoch。

（3）训练完毕后，模型权重保存在在python\output\checkpoints目录下。

（4）运行onnx_converter.py，构建用于转mapper的onnx文件，输入 ./output/checkpoints/denoise_50.pth，输出model/Model_denoise.onnx。

## 2、知存onnx格式模型转芯片mapper格式

mapper是知存工具链制定的一套工具，目的在于将模型转换至芯片所需格式。

### 2.1、连接mapper环境

1. 安装Xshell/Xftp，安装包在tools下
2. 通过Xshell连接到mapper服务器

- 服务器IP地址： [101.126.129.212](http://101.126.129.212/)
- 端口： 8090
- 账号:   guest
- 密码:   guest111

1. 输入命令：source ~/.bashrc
2. 在/home/guest/csdn新建一个文件夹，用于存放后续相关文件，如/home/guest/csdn/yourname

### 2.2、准备mapper输入文件

进入mapper文件夹下, input文件夹下为转mapper需要的的输入。

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=Mjg3NTgwOTMyNDc3MmMxNjZmODE2ZTU2MjE5MjY3OTRfZ0hlYnRUb1B1azIwcExxbXk3WExnaXJaZlpQZzdYSmxfVG9rZW46QU5zRWJCMjhHb0xiMzh4NmtLTmNLcUVqbkpFXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

其中：

- Model_denoise.onnx为步骤1.（4）生成的模型。
- opt.protobuf为模型优化策略配置文件，是可选项。
- quant_data.npy用于模型精度校准。
- gen_mapper.py为转换脚本，其中网络输入形状、名称等，需与模型适配（如图所示）。

 

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=NDU1NDMwMjMzZWY5NGExM2FhOWYxMjg1MDYyZDA5ODlfRG5FYnRDbENhUGp2SlVJMjRYd0lpRTFvYXJpTWVaVmJfVG9rZW46RE9NMWI4N1lkb2pMWEN4Z0JyMGNXd1ZRblZlXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=Y2I1YmJjMTVlODg2NTc5YTEzMzYxNTJmOTk5MmM4YWVfNEowdzB5dzBLQldFeHdoTTAxeWx1aFlzWGU1SzdWNWdfVG9rZW46T25IMGJ2WFVCb01GR0h4WWxEOWN5dFVHbkpmXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

### 2.3、生成map

（1）通过Xftp将mapper/input文件夹传输到2.1.（4）建立的文件夹下，如/home/guest/csdn/yourname/input，执行gen_mapper.py，转换输出在/home/guest/csdn/yourname/output，将output文件夹传输回本地。

（2）mapper的生成产物示例位于mapper/output：

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=MjBhZDM1NjcxZDY2ZmVkMTg1ZTBhZjFmNWZmM2U4MmJfcjJHOWhZSE80N2J2YXY1SkYzS0pKQk9naVEyNjVpa1lfVG9rZW46SUdxeWJYeUh0b3d1eXd4V3R2MmNuSUZObmlRXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

## 3、烧写NPU权重

NPU权重烧写有两种方式，二选一即可。

### 3.1、方式一：使用NPU烧写板（NPU烧录器）

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=MTYwMjNhMWVjMzFjYTY3YmYxMTExNzI4YWQzNzcyZGRfQXhCOHJFZVRtdW5jN0JRNFh3NXlneFh3bGpndmdwak9fVG9rZW46SEV2aWI2VW1ob1h2TWN4OTQzV2NIbVUybmJiXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

（1）系统连接：进行模型烧录和开发时，我们需要将JTAG，核心板，NPU烧写板连接好，并打开开关，如系统连接示意图所示。

（2）跳线帽连接：如跳线帽连接示意图所示，按照红框标注进行跳线连接。      

含义解释：

| 跳线 | 编号     | 跳帽连接 | 含义                        |
| ---- | -------- | -------- | --------------------------- |
| 1    | 3.3V     | AVDD     | WTM2101芯片模拟供电，3.3V   |
| 2    | 3.3V     | IOVDD    | WTM2101芯片I/O供电，3.3V    |
| 3    | 0.9/1.2V | DVDD     | 用 WTM2101BC 芯片时需接跳帽 |
| 4    | GND      | BOOT0    | 启动模式，SRAM启动          |
| 5    | 32K      | XTAL     | 晶振                        |
| 6    | RXD      | P17      | 串口                        |
| 7    | TXD      | P16      |                             |

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=NGZlZWYwNDBiZDQ2ZDgwMzYzMWNkMzBhMjY5YjY0M2ZfZ3FvZ0I3TWh1dEgxenQxZ0poa2RDek5pNkxFUzVIeWZfVG9rZW46TDhON2I5MVRib0dPY2x4RE91OWNXckZpblhmXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

跳线帽连接示意图

（3）使用WitinProgramTool_WTM2101下的WitinProgramTool.exe进行模型权重烧写。

（4）烧写指令:

.\WitinProgramTool.exe -m init

.\WitinProgramTool.exe -m program -i XXXX\map.csv -k 2

其中XXXX为步骤2.（3）中生成的mapper/output/map

示例:

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=ZTRiZTIyMDE5M2E2ZjYzMWQ1NDUxZWJlZDgxZWI2ZDBfcmQxTzNOa0l0N0JZYlprazVKYzZYNVJveVhHc3dZY1JfVG9rZW46THR0bGJMZTR4b3hIeHV4bEs1a2NVM2hSbjVjXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

### 3.2、方式二：不用NPU烧录器

如果没有npu烧录器，用jlink ota进行烧写。除了不用连接NPU烧录器，前3步和方式一一样。

（4）使用WitmemOTATool_Jlink下的WitmemOTATool_Jlink.exe进行模型权重烧写。

（5）烧写步骤：

  先使用WitinProgramTool_WTM2101/生成 ota_generate.bin，命令是：

​      ./WitinProgramTool.exe -m ota -i XXXX\map.csv

  将生成的ota_generate.bin拷贝到project/ WitmemOTATool_Jlink中

  运行指令 ./WitmemOTATool_Jlink --init   或 双击运行 init.bat

  运行指令 WitmemOTATool_Jlink --ota ota_generate.bin  或 双击运行 prog.bat

  烧写完成

## 4、板端工程编译及测试

（1）安装知存IDE Witmem Studio，安装包在tools下WSSetup_1.5.7.exe。

（2）将步骤2.（3）生成的mapper\output\register.c放在npu_model下，使用Witmem Studio打开工程witinkws_WTM2101_P1\SES-RISCV\Demo.wmproject。

(3) 编译工程, 快捷方式：ALT + F7。

（4）Target->Download下载工程（ctrl + D）：

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=NzAzNjU0OGJlNmY2MGUwOGRlNjRmZWZjNjU1NjE4NTNfaGlydXpvakptSmhtdnczUG9KSjRtVzNsN3IxQ0wzOXFfVG9rZW46UVYxcGJYRVhsb3FoSXB4ZXFvTmM0YXd4bjJlXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

注：本demo用x3开发板，所用串口GPIO为16,17。若是其他开发板，请根据情况修改。

（5）打开tools中的amaoComV4.9.exe串口工具，设置波特率921600，查看准确率输出。这些串口打印信息会在（4）download工程完成后开始打印，如果没有看到完整的打印信息，可以将板子重新下电上电后，先打开串口，再进行(4)download.

**正确输出见下图：**

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=NGU0ZDhmMGM2YzhjNDM4YmZlZTVhODJmYmI5Y2JhZDlfSFQ0dnZMdk5MUzJwdFR6R1dtb2lET1RReWRYVEhYNHdfVG9rZW46VzhIZmJsQTRsb21KeFd4eTVzNGMwU2RSbmhBXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

(6) **灌数据测试**

进入upper_machine目录，双击AudioEnhancement-v4.1.3.exe，按照如下步骤设置后点击“Run”，开始往开发板下发数据用于测试，测试完后降噪后的音频保存在upper_machine\wav_out 目录下，**至此demo完成。**

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=NjU4ZDliOTkzOTMwNjVlNjFmOWVlMjBlYWNiMGM5MDdfYVJTWnIzYWExTlVDWmFMM0kzbUxCVU9SSUVaVFJORktfVG9rZW46RnRJVGJpZ0dEb200MFh4SXJIVWNQU0hQbmhnXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

![img](https://iucylxooqp.feishu.cn/space/api/box/stream/download/asynccode/?code=NDAyMWQyYWQ3NzZiNDNjMDhkNTkwYzI0ODVkZDg1MDNfaVJ4dWQzRjIzcjVQTTk4RnFheUhRNVdGV2lvZXBSNDlfVG9rZW46TzJpVmJ3aHVOb1Z4dlF4b2NRM2NGUlZCbkUxXzE3MjkyMzI4NDA6MTcyOTIzNjQ0MF9WNA)

修订历史:

| Time       | Version | Description |
| ---------- | ------- | ----------- |
| 2024.10.17 | 0.1     | 初始版本    |
|            |         |             |
