# Ciallo!👋 这是牢雄的电控代码
## 项目介绍
25赛季PHOENIX英雄机器人电控代码，分为两个文件夹，分别用于底盘C板和云台C板。
## 使用流程
1. 编译代码：
点击 Build（F7）按钮，确保无错误（0 Error, 0 Warning）（warning必须重视！！！）。
2. 连接硬件：
用J-Link或CMISI-DAP调试器连接开发板和电脑。
3. 下载程序：
点击 Download（F8）按钮，将程序烧录到C板。
## 嵌入式架构
### 软件架构
#### CHASSIS
+ Application <u>CUBEMX生成的应用层代码</u>
    + MDK-ARM <u>下面单拎出来讲</u>
    + User
        + main <u>主函数，所有全局变量在此文件定义</u>
        + stm32f4xx_it <u>部分中断函数写在这个文件下，USART3_IRQHandler中遥控器接收中断Remote_Handler，USART6_IRQHandler中裁判系统接收中断USART6_IRQHandler_1</u>
+ Drivers <u>CUBEMX生成驱动层代码，包括HAL库和CMSIS库</u>
    + STM32F4xx_HAL_Driver
    + CMSIS
+ Middlewares <u>CUBEMX生成中间文件</u>
    + FreeRTOS
    + USB_DEVICE_Library
以下为MDK-ARM文件夹中(以后这些文件夹必须建在跟MDK-ARM同一级文件夹下！！！这份代码与24赛季代码放在MDK-ARM文件夹下是有问题的！！！）
+ bsp
    +bsp_rc,remote_control <u>遥控器数据处理相关函数</u>
    +bsp_can <u>can通信相关函数，与电机的通信，板间通信
+ calcula
    +
#### GIMBAL

### 硬件架构