# Ciallo!👋 这是牢雄的电控代码
## 项目介绍
25赛季PHOENIX英雄机器人电控代码，分为两个文件夹，分别用于底盘C板和云台C板。
## 使用流程
1. 编译代码：  
点击 Build（F7）按钮，确保无错误（0 Error, 0 Warning, warning必须重视！！！）。
2. 连接硬件：  
用J-Link或CMISI-DAP调试器连接开发板和电脑。
3. 下载程序：  
点击 Download（F8）按钮，将程序烧录到C板。
## 嵌入式架构
### 软件架构
#### ***CHASSIS***
+ Application <u>CUBEMX生成的应用层代码</u>
    + MDK-ARM <u>下面单拎出来讲</u>
    + User
        + main <u>主函数，绝大多数全局变量在此文件定义</u>
        + stm32f4xx_it <u>部分中断函数写在这个文件下，USART3_IRQHandler中遥控器接收中断Remote_Handler，USART6_IRQHandler中裁判系统接收中断USART6_IRQHandler_1</u>
+ Drivers <u>CUBEMX生成驱动层代码，包括HAL库和CMSIS库</u>
    + STM32F4xx_HAL_Driver
    + CMSIS
+ Middlewares <u>CUBEMX生成中间文件</u>
    + FreeRTOS
    + USB_DEVICE_Library

**以下为MDK-ARM文件夹中**  
*以后这些文件夹必须建在跟MDK-ARM同一级文件夹下！！！这份代码与24赛季代码放在MDK-ARM文件夹下是有问题的！！！*
+ bsp <u>通信相关文件</u>
    + bsp_rc,remote_control <u>遥控器数据处理相关函数</u>
    + bsp_can <u>can通信相关函数，与电机的通信，板间通信</u>
+ calculate <u>计算相关文件</u>
    + motion <u>底盘运动结算函数</u>
    + pid <u>pid计算函数</u>
    + Filter <u>低通滤波器</u>
    + ShootFun <u>拨弹盘功能函数</u>
    + ChassisFun <u>底盘功能函数</u>
    + GimbalFun <u>yaw轴功能函数</u>
+ judge <u>裁判系统相关文件</u>
    + CRC8_CRC16,CRCs <u>CRC校验函数</u>
    + fifo,judge <u>裁判系统通信函数</u>
    + tuxin <u>绘制UI用函数</u>
+ task <u>Freertos任务文件</u>  *后面命名还是统一规范较好*
    + Chassis_Task
    + Gimbal_task
    + ShootTask
    + UI_task
+ motor <u>电机相关函数</u>
    + DM4310 <u>达妙4310通信相关函数</u>
---

#### ***GIMBAL***
+ Application <u>CUBEMX生成的应用层代码</u>
    + MDK-ARM <u>下面单拎出来讲</u>
    + User
        + main <u>主函数，绝大多数全局变量在此文件定义</u>
        + stm32f4xx_it <u>USART6_IRQHandler中外加陀螺仪接收中断USER_UART_IRQHandler</u>
+ Drivers <u>CUBEMX生成驱动层代码，包括HAL库和CMSIS库</u>
    + STM32F4xx_HAL_Driver
    + CMSIS
+ Middlewares <u>CUBEMX生成中间文件</u>
    + FreeRTOS
    + USB_DEVICE_Library
+ USB_DEVICE <u>CUBEMX生成虚拟串口文件

**以下为MDK-ARM文件夹中**  
*以后这些文件夹必须建在跟MDK-ARM同一级文件夹下！！！这份代码与24赛季代码放在MDK-ARM文件夹下是有问题的！！！*
+ bsp <u>通信相关文件</u>
    + bsp_can <u>can通信相关函数，与电机的通信，板间通信</u>
+ calculate <u>计算相关文件</u>
    + pid <u>pid计算函数</u>
    + vision <u>视觉通信相关函数</u>
    + HWT606 <u>外加陀螺仪相关函数</u>
    + GimbalFun <u>pitch轴功能函数</u>
+ task <u>Freertos任务文件</u>
    + Shoot_task
    + Gimbal_task
    + INS_task <u>原配置C板内置陀螺仪相关函数，本份代码中仅用于虚拟串口发送及板间通信发送，后续如不使用内置陀螺仪需优化这部分代码</u>
+ Motor <u>电机相关函数</u>
    + DM4310 <u>达妙4310通信相关函数</u>
    + DM4310multi <u>达妙4310一拖四模式通信相关函数 *该模式几乎没用*</u>
### 硬件架构
+ 电源线
    + Chassis <u>四个底盘电机（M3508*4）</u>
    + Gimbal <u>两个云台电机（Yaw：DM4310，Pitch：DM4310）</u>
    + MiniPC <u>小电脑和C板供电</u>
    + Ammo-Booster <u>发射机构供电（摩擦轮：M3508*4，拨弹盘：DM4310）</u>
+ CAN线
    + Chassis
        + CAN1 <u>M3508* 4,DM4310* 2</u>
        + CAN2 <u>板间通信</u>
    + Gimbal
        + CAN1 <u>M3508* 4,DM4310* 1</u>
        + CAN2 <u>板间通信</u>
+ UART通信
    + Chassis
        + uart3 <u>接收遥控器数据串口</u>
        + uart6 <u>裁判系统通信串口</u>
+ 裁判系统线材 <u>详见裁判系统手册</u>
    + 航空线
    + GH线材
## 未来优化方向
- [ ] 连接小电脑的同时稳定can通信🌟🌟🌟
- [ ] 云台Yaw, Pitch跟随情况和响应速度🌟🌟🌟
- [ ] 增加超电并使其功能稳定🌟🌟🌟
- [ ] 接线优化（线材外露）🌟🌟
- [ ] 切换模式时拨弹盘略微转动的问题🌟

