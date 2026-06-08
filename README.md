1. # 智能麦轮小车控制系统

   ## 项目概述

   这是一个基于STM32F103ZET6微控制器的智能麦轮小车控制系统，采用FreeRTOS实时操作系统，支持多种控制模式和通信协议。
   代码用的clion写的，编译用的gcc,下载调试用的openocd,当然也可以用其他的比如jlink只要修改cfg文件就行
   ### 硬件平台
   - **微控制器**: STM32F103ZET6 (ARM Cortex-M3)
   - **主频**: 72MHz
   - **Flash**: 512KB
   - **RAM**: 64KB

   ## 项目结构

   ```
   MySmartcar/
   ├── Core/                    # STM32核心代码
   │   ├── Inc/                # 头文件
   │   └── Src/                # 源代码
   │       ├── main.c          # 主程序入口
   │       ├── freertos.c      # FreeRTOS任务定义
   │       ├── usart.c         # 串口通信
   │       ├── gpio.c          # GPIO配置
   │       └── ...
   ├── Drivers/                 # HAL驱动
   ├── Middlewares/             # 中间件
   │   └── Third_Party/FreeRTOS/# FreeRTOS系统
   └── include/                 # 应用层代码
       ├── motor/              # 电机控制
       │   ├── car_control.c   # 麦轮运动控制
       │   ├── encoder.c       # 编码器读取
       │   └── pid.c          # PID控制器
       ├── fifo/               # 数据缓存和协议解析
       ├── oled/               # OLED显示
       ├── MPU6050/            # 陀螺仪姿态解算
       ├── NRF24L01/           # 无线通信
       └── sensor/             # 传感器驱动
   ```

   ## 主要功能模块

   ### 1. 电机控制 (Motor Control)
   - **麦轮运动学**: 支持四轮麦克纳姆轮运动控制
   - **编码器读取**: 四个编码器实时测速
   - **PID速度控制**: 独立的PID控制器用于每个电机

   ### 2. 通信系统 (Communication)
   - **USART1**: 主通信串口（连接上位机）
     - 波特率: 115200
     - 数据格式: 8N1
   - **USART2**: 备用串口
   - **协议格式**: `09 XX ... 0D` (包头0x09, 包尾0x0D)

   ### 3. 传感器系统 (Sensors)
   - **MPU6050/MPU9250**: 六轴陀螺仪（支持DMP姿态解算）
   - **编码器**: 电机速度反馈
   - **OLED显示**: 128x64 OLED显示系统状态

   ### 4. 显示系统 (Display)
   - **OLED驱动**: 基于I2C的软件模拟
   - **界面显示**: 支持参数显示、调试信息

   ## 控制模式

   ### 模式0: 速度矢量模式 (Remote Mode)
   ```c
   vx_set: X轴速度
   vy_set: Y轴速度
   vw_set: 旋转角速度
   
   // 麦轮逆运动学
   w1 = vx - vy - vw
   w2 = vx + vy + vw
   w3 = vx - vy + vw
   w4 = vx + vy - vw
   ```

   ### 模式1: 精准路径模式 (Precision Mode)
   - **直线行走**: `Move_Distance_Precision(float distance_mm)`
   - **精准旋转**: `Rotate_Degree_Precision(float angle_deg)`

   ## 通信协议

   ### 命令格式
   ```
   包头(1字节) + 命令(1字节) + 数据(N字节) + 包尾(1字节)
      0x09          0xA1         ...              0x0D
   ```

   ### 命令列表

   | 命令 | 功能 | 数据格式 |
   |------|------|----------|
   | 0xA0 | 设置PID参数 | P(float), I(float), D(float), TarSpeed(int) |
   | 0xA1 | 控制模式 | mode(int), 参数1-3 |

   ### 示例数据包
   ```
   09 F5 A0 04 3F 80 00 00 40 00 00 00 40 40 00 00 00 00 00 04 0D
        └─ 0xA0 ─┘ └──────── P=1.0f ──────┘ └───── I=2.0f ────┘
                 └────────────────── D=3.0f ────────┘ └─ TarSpeed=4 ─┘
   ```

   ## FreeRTOS任务调度

   | 任务名 | 优先级 | 周期 | 栈大小 | 功能 |
   |--------|--------|------|--------|------|
   | defaultTask | Normal | - | 256 | 默认任务 |
   | pidTask | High | 10ms | 256 | PID控制计算 |
   | sensorTask | AboveNormal | 10ms | 256 | 传感器采集 |
   | uart2Task | Normal | 10ms | 256 | 串口数据解析 |
   | oledTask | Normal | 100ms | 256 | OLED显示更新 |

   ## 系统初始化流程

   ```c
   1. HAL_Init()                    // HAL库初始化
   2. SystemClock_Config()          // 系统时钟配置 (72MHz)
   3. MX_GPIO_Init()                // GPIO初始化
   4. MX_TIMx_Init()               // 定时器初始化（PWM、编码器）
   5. MX_USARTx_Init()             // 串口初始化
   6. MX_I2C1_Init()               // I2C初始化（OLED）
   7. app_init()                   // 应用层初始化
      - FIFO初始化
      - 电机初始化
      - OLED初始化
      - 编码器启动
      - 定时器中断启动
   8. MX_FREERTOS_Init()           // FreeRTOS任务创建
   9. osKernelStart()             // 启动调度器
   ```

   ## 引脚配置

   ### 电机控制 (PWM输出)
   | 引脚 | 功能 | 定时器 |
   |------|------|--------|
   | PA0-PA3 | 电机PWM | TIM2 |
   | PC6-PC9 | 电机PWM | TIM3/TIM8 |

   ### 编码器接口
   | 定时器 | 通道A | 通道B | 功能 |
   |--------|-------|-------|------|
   | TIM3 | PC6 | PC7 | 左前编码器 |
   | TIM4 | PD12 | PD13 | 右前编码器 |
   | TIM5 | PA0 | PA1 | 左后编码器 |
   | TIM8 | PC6 | PC7 | 右后编码器 |

   ### 串口通信
   | 串口 | TX | RX | 用途 |
   |------|-----|-----|------|
   | USART1 | PA9 | PA10 | 主通信 |
   | USART2 | PD5 | PD6 | 备用 |

   ### OLED显示 (软件I2C)
   | 引脚 | 功能 |
   |------|------|
   | PD11 | SCL |
   | PD12 | SDA |
   | PD13 | RES |

   ### MPU6050 (硬件I2C)
   | 引脚 | 功能 |
   |------|------|
   | PB8 | I2C1_SCL |
   | PB9 | I2C1_SDA |

   ## 编译与下载

   ### 开发环境
   - **IDE**: CLion + STM32CubeMX
   - **编译器**: ARM GNU Toolchain
   - **构建工具**: CMake + Make

   ### 编译命令
   ```bash
   cd cmake-build-debug-mingw-_stm32
   make
   ```

   ### 下载
   使用ST-Link或J-Link通过SWD接口下载

   ## 使用说明

   ### 1. 上位机连接
   - 使用串口调试助手连接USART1
   - 波特率: 115200
   - 数据格式: 8N1

   ### 2. 发送控制命令
   - 发送格式: `09 XX ... 0D`
   - 例如: 设置P=1.0, I=2.0, D=3.0, TarSpeed=100
     ```
     09 F5 A0 04 3F 80 00 00 40 00 00 00 40 40 00 00 00 00 00 64 0D
     ```

   ### 3. 监控状态
   - 观察OLED显示的系统参数
   - 串口返回确认信息

   ## 调试指南

   ### 常见问题

   1. **OLED不显示**
      - 检查OLED初始化是否在FreeRTOS启动前完成
      - 检查I2C引脚配置是否正确

   2. **串口接收不到数据**
      - 检查回调函数是否正确实现
      - 检查FIFO缓冲区是否正确初始化

   3. **任务调度异常**
      - 检查任务栈大小是否足够
      - 避免在任务中使用阻塞操作（HAL_Delay）

   4. **MPU6050初始化失败**
      - 检查I2C通信是否正常
      - 检查MPU6050地址是否正确（0x68）

   ### 调试技巧

   1. **使用LED指示程序状态**
   ```c
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // 翻转LED
   ```

   2. **串口打印调试信息**
   ```c
   char buf[64];
   sprintf(buf, "Value: %d\r\n", value);
   HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
   ```

   ## 技术参数

   ### 系统参数
   - 系统时钟: 72MHz
   - SysTick频率: 1000Hz
   - 任务调度周期: 1ms

   ### 电机参数
   - 电机数量: 4个直流减速电机
   - 编码器分辨率: 编码器脉冲数/轮转
   - PWM分辨率: 16位

   ### 通信参数
   - 串口波特率: 115200
   - I2C速率: 100kHz

   ## 未来扩展

   - [ ] 添加MPU6050姿态控制
   - [ ] 实现NRF24L01无线控制
   - [ ] 添加超声波避障
   - [ ] 实现路径规划算法

   ## 版本历史

   - **v1.0**: 基础麦轮控制框架
   - **v1.1**: 添加FreeRTOS多任务调度
   - **v1.2**: 完善PID控制和协议解析

   ## 许可证

   本项目采用MIT许可证

   ## 作者

   MySmartcar Development Team

   ## 联系方式

   如有问题，请提交Issue或联系开发者
