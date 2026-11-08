# STM32 FreeRTOS 电机控制学习项目

基于 STM32F103C8T6 和 FreeRTOS 的直流减速电机闭环调速项目。项目使用编码器测速、PID 速度闭环和 PWM 电机驱动，并通过 USART1 以 VoFA+ JustFloat 协议上传目标转速、实际转速和 PID 输出。

## 项目功能

- 编码器测速：TIM2 编码器接口读取电机转速
- PID 速度闭环：根据目标转速和实际转速计算 PWM 输出
- PWM 电机驱动：TIM3 输出 PWM，GPIO 控制电机方向
- FreeRTOS 多任务：电机控制、正弦目标生成、串口上传三个任务
- 上位机波形：USART1 发送 VoFA+ JustFloat 数据帧

## 任务设计

| 任务 | 函数 | 周期 | 优先级 | 作用 |
|---|---|---|---|---|
| 电机控制任务 | `TaskMotor` | 10 ms | 4 | 编码器测速、PID 计算、PWM 输出 |
| 正弦目标任务 | `TaskSin` | 20 ms | 3 | 生成 180 ± 70 rpm、0.2 Hz 的正弦目标转速 |
| 串口上传任务 | `TaskVofa` | 50 ms | 2 | 通过 USART1 上传三路浮点数据 |

优先级设计原则：控制周期越短、实时性要求越高的任务，优先级越高。电机控制任务优先级最高，串口上传任务优先级最低。

## 硬件平台

- STM32F103C8T6 最小系统板
- JGB37-520 12V 333rpm 直流减速电机
- 霍尔编码器：11 线，四倍频，减速比 18
- 电机驱动模块
- ST-Link 下载器
- USB 转串口模块

## 软件环境

- STM32CubeMX
- Keil MDK
- FreeRTOS（CMSIS-RTOS v1 接口）
- VoFA+ 上位机

## 目录结构

```text
stm32-rtos-lab/
├─ Core/
│  ├─ Inc/                  # 外设和 FreeRTOS 配置头文件
│  └─ Src/                  # 主程序、外设初始化、FreeRTOS 配置
├─ Drivers/
│  ├─ CMSIS/                # CMSIS 内核和器件头文件
│  └─ STM32F1xx_HAL_Driver/ # STM32 HAL 驱动
├─ Middlewares/
│  └─ Third_Party/FreeRTOS/ # FreeRTOS 内核和 CMSIS-RTOS 封装
├─ MDK-ARM/
│  ├─ app_main.c            # 电机控制、PID、三个任务
│  ├─ startup_stm32f103xb.s # 启动文件
│  └─ xiaozurenwu2.uvprojx  # Keil 工程文件
├─ docs/
│  └─ 任务设计说明.md
├─ xiaozurenwu2.ioc         # STM32CubeMX 工程配置
└─ README.md
```

## 编译和运行

1. 使用 Keil MDK 打开：

```text
MDK-ARM/xiaozurenwu2.uvprojx
```

2. 点击 **Rebuild** 编译工程。
3. 连接 ST-Link，将程序下载到 STM32F103C8T6。
4. 使用 USART1 连接上位机：
   - 波特率：`115200`
   - 数据位：8
   - 停止位：1
   - 校验位：无
5. 打开 VoFA+，选择 **JustFloat** 协议，添加 3 个通道：
   - 通道 1：目标转速
   - 通道 2：实际转速
   - 通道 3：PID 输出（duty / 10）

## 串口数据格式

每帧发送 3 个 `float`（12 字节），帧尾为：

```text
00 00 80 7F
```

对应 VoFA+ 的 JustFloat 协议。

## 学习记录

- 2026-11-08：创建仓库，完成 README、`.gitignore` 和第一次提交。
- 2026-11-08：加入 STM32F103 FreeRTOS 电机控制工程。
- 2026-11-08：补充项目说明和任务设计文档。
