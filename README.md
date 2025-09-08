# BMCU370x 项目简介
这是修改版，以下为原项目的简介搬运，项目链接：[4061N/BMCU](https://gitee.com/at_4061N/BMCU)。

## 本项目相关链接
* 1. 此项目WiKi [BMCU370x Wiki](https://bmcu.wanzii.cn/doc/build/bmcu370x.html)
* 2. 此项目硬件 [BMCU370x oshwhub.com](https://oshwhub.com/xingcc1/bmcu-370x)
* 3. **硬件技术文档** [Hardware Documentation](docs/README.md) - 详细的硬件传感器和组件技术文档

## 定制固件
克隆仓库，并使用Vscode打开`src`文件夹，使用PlatformIO插件进行编译。

### 修改退料时间
查看`src/Motion_control.cpp`文件中的第463-464行代码，修改为自定义退料时间。

### 修改马达速度
查看`src/Motion_control.cpp`文件中的第142-184行代码，修改对应状态的速度，正数为送出，负数为回退。
# BMCU

#### 介绍

BMCU以四通道为一个单位，目前以CH32单片机为主控设计。其设计所需资料均参考于网络公开资料及个人测试，程序基于Platform IO平台下CH32单片机的Arduino支持库设计，调用了robtillaart的CRC库。
 **注意:本项目遵循GPL2.0开源协议，但需要额外补充的是本项目禁止商业用途。** 

#### 使用说明及安装教程

 **1.  制造所需资料见BMCU整合打包文件** 

BMCU打包文件目前发布在群内，或可以从[https://oshwhub.com/bamboo-shoot-xmcu-pcb-team/bmcu](https://oshwhub.com/bamboo-shoot-xmcu-pcb-team/bmcu)的附件部分，找到V1.1版打包文件。

 **2.  所需刷写的固件见release** 

 **3.  安装及使用教程参见wiki(由群友@丸子 牵头搭建，社区共建中)** 

地址：[https://bmcu.wanzii.cn/](https://bmcu.wanzii.cn/) 


#### 软件架构
主要文件：

main.cpp/h：               负责调度各个模块

many_soft_AS5600.cpp/h：   IO模拟的，可同时和多个AS5600通讯的驱动

Motion_control.cpp/h：     负责硬件和运动状态调度

Flash_saves.cpp/h：        用于保存数据到flash

time64.cpp/h：             将Arduino的32位时基转为64位，防止连续运行几个月后溢出

Debug_log.cpp/h：          用于DMA发送DEBUG数据到串口3

BambuBus.cpp/h：           用于支持拓竹打印机的通讯，使用了串口0

Klipper.cpp/h：            （因klipper机型众多，停止开发）用于支持klipper的通讯

ws2812.cpp/h：             IO模拟的WS2812驱动

调用的库：

robtillaart/CRC@^1.0.3     用于计算CRC校验

#### 参与贡献

设计师：

4061N-程序员，优化机械部分，设计PCB

括号-参与BMCU组件框架及BMCU到打印机支架的设计

部分群友也参与了设计，这里暂未列出

测试和数据提供者：

风雪-提供打印机与AMS通讯的数据，参与部分测试

二月喵-提供少量高质量的通讯数据，参与部分测试

其他成员：

婆老-负责在线摸鱼

其他未列出的群友-提供了宝贵的测试数据和建议

## Hardware Documentation / 硬件技术文档

Comprehensive technical documentation for the BMCU370x hardware sensors and components is available in the [`docs/`](docs/) directory:

详细的BMCU370x硬件传感器和组件技术文档位于[`docs/`](docs/)目录中：

### Documentation Contents / 文档内容

- **[Hardware Overview](docs/HARDWARE_DOCUMENTATION.md)** - Complete system architecture and component specifications / 完整的系统架构和组件规格
- **[AS5600 Magnetic Sensors](docs/AS5600_SENSORS.md)** - Position sensing and encoder details / 位置传感和编码器详情  
- **[Pressure Sensing System](docs/PRESSURE_SENSING.md)** - ADC-based filament monitoring / 基于ADC的耗材监控
- **[PWM Motor Control](docs/PWM_MOTOR_CONTROL.md)** - Motor control with PID feedback / 带PID反馈的电机控制
- **[BambuBus Protocol](docs/BAMBUBUS_PROTOCOL.md)** - Communication with Bambu Lab printers / 与拓竹打印机的通信协议
- **[System Integration](docs/SYSTEM_INTEGRATION.md)** - Advanced features and coordination / 高级功能和协调控制

### Key Hardware Features / 主要硬件特性

- **4-Channel Operation** - Independent filament management per channel / 4通道独立耗材管理
- **Magnetic Position Sensing** - AS5600 encoders for precise filament tracking / AS5600编码器实现精确耗材跟踪
- **Pressure Monitoring** - Real-time filament tension feedback / 实时耗材张力反馈
- **Dual Microswitch Detection** - Redundant filament presence sensing / 冗余耗材检测系统
- **RGB Status Indicators** - Visual feedback for each channel / 每个通道的可视状态反馈
- **BambuBus Integration** - Seamless printer communication / 无缝打印机通信
