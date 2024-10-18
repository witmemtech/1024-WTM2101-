# WitmemOTATool_Jlink User Guide

WitmemOTATool_Jlink for WTM2101 [(Based on WitinMem Program Tool for WTM2101Release) ](http://192.168.4.230:3000/zhangjian/WitinProgramTool_WTM2101)


1.	#### 工具简介：
    
    WitmemOTATool_Jlink是通过Jlink调试器以OTA方式实现对WTM2101芯片的NPU网络编程功能的工具软件。
    
2. #### 硬件需求:

   * WTM2101 chip and demo board
   * Jlink适配器

3. #### 软件需求:

   * Jlink驱动
   * WitmemOTATool_Jlink.exe
   * JLink_x64.dll
   * wtm2101_2w.JLinkScript
   * wtm2101_4w.JLinkScript

4. #### 用法

   ##### Initialization:

   ```C
   /*Common usage*/
   WitmemOTATool_Jlink --init
   /*Select jlink by serial number*/
   WitmemOTATool_Jlink --init	--sn 123456789  
   /*Select cJTAG:*/
   WitmemOTATool_Jlink --init	--cjtag
   /*Select cJTAG && Select jlink by serial number*/
   WitmemOTATool_Jlink --init	--cjtag	--sn 123456789  
   ```

   ##### Initialization and write bootloader:

   ```C
   /*Common usage*/
   WitmemOTATool_Jlink --init --boot bootlaoder.txt
   /*Select jlink by serial number*/
   WitmemOTATool_Jlink --init --boot bootlaoder.txt  --sn 123456789  
   /*Select cJTAG:*/
   WitmemOTATool_Jlink --init --boot bootlaoder.txt  --cjtag
   /*Select cJTAG && Select jlink by serial number*/
   WitmemOTATool_Jlink --init --boot bootlaoder.txt  --cjtag	--sn 123456789 
   ```

   ##### Programe(OTA):

   ```C
   /*Common usage*/
   WitmemOTATool_Jlink --ota  ota_generate.bin
   /*Select jlink by serial number*/
   WitmemOTATool_Jlink --ota  ota_generate.bin  --sn 123456789  
   /*Select cJTAG:*/
   WitmemOTATool_Jlink --ota  ota_generate.bin  --cjtag
   /*Select cJTAG && Select jlink by serial number*/
   WitmemOTATool_Jlink --ota  ota_generate.bin  --cjtag	--sn 123456789 
   ```

   **Check OTA:**

   ```C
   /*Common usage*/
   WitmemOTATool_Jlink --check  ota_generate.bin
   /*Select jlink by serial number*/
   WitmemOTATool_Jlink --check  ota_generate.bin  --sn 123456789  
   /*Select cJTAG:*/
   WitmemOTATool_Jlink --check  ota_generate.bin  --cjtag
   /*Select cJTAG && Select jlink by serial number*/
   WitmemOTATool_Jlink --check  ota_generate.bin  --cjtag	--sn 123456789 
   ```

   **OSC Calibration:**

   ```C
   /*Common usage*/
   WitmemOTATool_Jlink --osc_calibrate
   /*Select jlink by serial number*/
   WitmemOTATool_Jlink --osc_calibrate  --sn 123456789  
   /*Select cJTAG:*/
   WitmemOTATool_Jlink --osc_calibrate  --cjtag
   /*Select cJTAG && Select jlink by serial number*/
   WitmemOTATool_Jlink --osc_calibrate  --cjtag	--sn 123456789 
   ```

   **Other Options：**

   ```C
   --help (-h): help information on commands and options.
   
   --version (-v): version info.
   ```

------

**注意事项：**

- 该版本工具对应的W2101固件程序已经固化在工具内部，不需要使用其他工具进行固件下载；

- init模式下指定bootloader.txt文件的话则会同时将指定的bootloader写入NPU，如果不指定则会写入随机数据；在init模式下如果返回‘-1’异常，则需要进行时钟校准后再执行init操作；

- OSC Calibration模式下需要连接外部32k时钟才可以获取准确的时钟校准值；

- Programe(OTA)模式下的输入参数为对应网络的OTA升级数据，通过对应版本的[WitinProgramTool](http://192.168.4.230:3000/zhangjian/WitinProgramTool_WTM2101/src/master/UserGuide.docx)工具生成；

  ```shell
  WitinProgramTool -m ota -i map.csv
  ```

- 通过”--sn=<s/n>“指定选择jlink设备，当连接设备数为1个时，不需要指定；

- 当选择cJTAG连接方式时需要增加“--cjtag”命令，默认采用4线JTAG方式。

