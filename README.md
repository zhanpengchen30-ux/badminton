# 高坚果 RoboMaster 电控工程

这是一个基于 **STM32F407 + HAL + FreeRTOS** 的 RoboMaster 电控工程，当前代码包含云台控制、五轴达妙电机控制、GM6020 挥拍机构、BMI088 姿态解算、IST8310 磁力计、DR16 遥控器以及双 CAN 通信。

> **安全提示**：首次上电、修改电机方向或调整 PID 前，必须拆下危险负载或架空机械结构，并准备物理急停。本文档中的 ID 和引脚以当前源码及 `ShaoBin2.0.ioc` 为准；未在代码中明确使用的接口请在实车接线前用原理图和万用表复核。

## 1. 工程信息

| 项目 | 配置 |
|---|---|
| MCU | STM32F407IGHx / Cortex-M4F |
| 工具链 | Keil MDK，ARMCC V5 工程 |
| HAL/CubeMX | STM32 HAL，CubeMX 6.8.1 配置 |
| RTOS | FreeRTOS，通过 CMSIS-RTOS V1 接口创建任务 |
| 主频 | 168 MHz（HSE=25 MHz，PLL 配置见 `Core/Src/main.c`） |
| CAN 速率 | CAN1、CAN2 均为 1 Mbps |
| 遥控器 | DR16，USART3 + DMA，18 字节帧 |
| IMU | BMI088，SPI1 + DMA + 数据就绪中断 |
| 磁力计 | IST8310，I2C3 |
| 工程入口 | `PVT/PVT/MDK-ARM/ShaoBin2.0.uvprojx` |
| CubeMX 配置 | `PVT/PVT/ShaoBin2.0.ioc` |

## 2. 目录结构

```text
PVT/PVT/
├── Core/                  CubeMX 生成的启动、外设和 FreeRTOS 文件
├── Drivers/               STM32F4 HAL 与 CMSIS
├── Middlewares/           FreeRTOS 源码
├── MDK-ARM/
│   ├── BSP/               板级驱动、串口和 CAN 辅助代码
│   ├── User/              DR16、达妙电机、GM6020 等用户驱动
│   ├── TASK/              姿态、云台、校准和动作控制任务
│   ├── PID/               PID 控制器
│   └── ShaoBin2.0.uvprojx Keil 工程
└── ShaoBin2.0.ioc         CubeMX 工程
```

## 3. 硬件连线

### 3.1 关键 MCU 引脚

| 功能 | MCU 引脚 | 配置/说明 |
|---|---|---|
| CAN1_RX | PD0 | CAN1 接收 |
| CAN1_TX | PD1 | CAN1 发送 |
| CAN2_RX | PB5 | CAN2 接收 |
| CAN2_TX | PB6 | CAN2 发送 |
| USART3_TX | PC10 | 调试/串口发送 |
| USART3_RX | PC11 | DR16 接收，DMA1 Stream1 |
| SPI1_SCK | PB3 | BMI088 SPI 时钟 |
| SPI1_MISO | PB4 | BMI088 SPI 输入 |
| SPI1_MOSI | PA7 | BMI088 SPI 输出 |
| BMI088 加速度计 CS | PA4 | `CS1_ACCEL`，低有效 |
| BMI088 陀螺仪 CS | PB0 | `CS1_GYRO`，低有效 |
| BMI088 加速度计 DRDY | PC4 | 下降沿 EXTI |
| BMI088 陀螺仪 DRDY | PC5 | 下降沿 EXTI |
| IST8310 INT | PG3 | 下降沿 EXTI |
| I2C3_SCL | PA8 | IST8310 |
| I2C3_SDA | PC9 | IST8310 |

### 3.2 电源与总线

1. **CAN 总线**使用 CANH/CANL 差分线，所有节点共地，物理总线两端各保留一个 120 Ω 终端电阻。
2. CAN1、CAN2 的波特率均为 **1 Mbps**，不能把 CAN1 与 CAN2 的设备混接后仍按同一个滤波配置理解。
3. 电机电源与控制板逻辑电源分开确认；接线顺序建议为：共地、CAN、信号线、最后接动力电源。
4. BMI088 的 SPI 线应尽量短，CS 与 DRDY 不要与大电流电机线平行长距离走线。
5. DR16 接收器输出连接 USART3_RX（PC11），确认接收器电平与控制板 IO 电平兼容。

### 3.3 当前代码中的总线归属

| 总线 | 当前代码使用情况 |
|---|---|
| CAN1 | `bsp_can_DM.c` 中发送达妙控制帧、接收达妙反馈和 GM6020 反馈；GM6020 电压也从 CAN1 的 `0x1FF` 发送 |
| CAN2 | CubeMX 已初始化并启动接收中断；当前用户代码中未形成明确的设备 ID 分配表，接线前需结合整车原理图复核 |

## 4. CAN ID 分配

### 4.1 CAN1：达妙电机

达妙电机控制 ID 在 `DM_Ctrl_ID[]` 中定义：

| 电机序号 | 控制标准帧 ID | 反馈标准帧 ID | 代码数组下标 | 默认姿态数组 |
|---:|---:|---:|---:|---|
| 1 | `0x301` | `0x11` | 0 | `FRONT_READY[0]` |
| 2 | `0x302` | `0x12` | 1 | `FRONT_READY[1]` |
| 3 | `0x303` | `0x13` | 2 | `FRONT_READY[2]` |
| 4 | `0x304` | `0x14` | 3 | `FRONT_READY[3]` |
| 5 | `0x305` | `0x15` | 4 | `FRONT_READY[4]` |

控制帧由 `psi_ctrl()` 发送 8 字节位置、速度和力矩参数。当前 `Motor_control()` 每次只发送一个电机，并通过 `Selection` 在 0～4 之间轮询。

达妙使能相关命令使用同一个电机控制 ID：

| 命令 | 数据帧 |
|---|---|
| 清除错误 | `FF FF FF FF FF FF FF FB` |
| 使能 | `FF FF FF FF FF FF FF FC` |
| 失能 | `FF FF FF FF FF FF FF FD` |
| 保存零点 | `FF FF FF FF FF FF FF FE` |

### 4.2 CAN1：GM6020

| 功能 | 标准帧 ID | 数据说明 |
|---|---:|---|
| GM6020 反馈 | `0x206` | Byte0~1：编码器值；Byte2~3：转速 RPM；Byte4~5：给定电流 |
| GM6020 电压控制 | `0x1FF` | Byte0~1：电机 1；Byte2~3：电机 2；Byte4~5：电机 3；Byte6~7：电机 4 |

当前代码把 GM6020 反馈按 `0x206` 解析，并将 `total_ecd` 作为连续多圈位置使用。GM6020 控制函数当前只填充 `0x1FF` 的 Byte2~3，因此实际使用的是该帧对应的第二个电机槽位；如硬件电机编号不同，必须同步修改发送槽位和反馈 ID。

### 4.3 CAN2

当前提交中可以确认 CAN2 已按 1 Mbps 初始化，并启用了 FIFO0 接收中断，但没有在用户层形成稳定的设备 ID 使用表。请不要仅凭 CAN2 外设已开启就直接接入执行器。建议实车前补充：

- CAN2 每个节点的控制 ID 与反馈 ID；
- 设备型号和节点编号；
- 终端电阻位置；
- CAN2 接收回调与超时策略。

## 5. DR16 遥控器映射

### 5.1 数据结构

`DR16_Decode()` 将 18 字节接收帧解码为：

- `rc_ctrl.rc.ch[0]`～`rc_ctrl.rc.ch[3]`：四个摇杆通道，范围约为 `-660`～`660`；
- `rc_ctrl.rc.ch[4]`：右上角拨轮，范围约为 `-660`～`660`；
- `rc_ctrl.rc.s[0]`：左侧三档开关 S1；
- `rc_ctrl.rc.s[1]`：右侧三档开关 S2。

### 5.2 当前动作控制映射

| 遥控器输入 | 代码字段 | 当前用途 |
|---|---|---|
| 左摇杆横向 | `rc_ctrl.rc.ch[0]` | 达妙击球姿态的 Yaw 偏转 |
| 左摇杆纵向 | `rc_ctrl.rc.ch[1]` | 击球动作比例，正向推动进入 HIT 姿态 |
| 右摇杆横向 | `rc_ctrl.rc.ch[2]` | 当前 `Control_Task` 未作为主要动作输入使用 |
| 右摇杆纵向 | `rc_ctrl.rc.ch[3]` | 当前 `Control_Task` 未作为主要动作输入使用 |
| 右上拨轮 | `rc_ctrl.rc.ch[4]` | GM6020 挥拍位置，范围限制在约 `3200`～`7800` 编码器值 |
| 左侧 S1 向上 | `rc_ctrl.rc.s[0] == 1` | 选择左侧击球姿态 |
| 左侧 S1 中间 | `rc_ctrl.rc.s[0] == 3` | 选择正面击球姿态 |
| 左侧 S1 向下 | `rc_ctrl.rc.s[0] == 2` | 选择右侧击球姿态 |
| 右侧 S2 中间 | `rc_ctrl.rc.s[1] == 3` | 进入动作使能控制分支 |
| 右侧 S2 非中间 | `rc_ctrl.rc.s[1] != 3` | GM6020 输出 0，达妙电机失能 |

> 注意：工程中还保留一套旧的 `DR16` / `DR16_Export_Data` 云台控制接口，位于 `Gimbal_control.c`。后续建议统一为 `rc_ctrl` 一套命名，避免两个遥控器数据结构产生不一致。

## 6. FreeRTOS 任务

| 任务 | 优先级 | 周期/作用 |
|---|---:|---|
| `Gimbal_task` | Normal | 1 ms，执行云台闭环 |
| `IMU_Send` | High | 1 ms，发送姿态与陀螺仪数据 |
| `INS_task` | AboveNormal | BMI088 采样、滤波与姿态解算 |
| `calibrate_task` | Low | 传感器校准与参数保存 |
| `Gimbal1_Task` | Low | 当前为空壳任务 |
| `Control_Task` | Normal | 1 ms，执行达妙和 GM6020 动作控制 |

## 7. 安全保护

当前版本加入了两级 P0 保护：

1. **DR16 超时保护**：超过 `RC_TIMEOUT_MS` 未收到有效 18 字节帧时，云台输出清零、达妙电机失能、GM6020 输出清零。
2. **GM6020 浮点输出限幅**：先在浮点阶段将位置 PD 输出限制到 `±GM6020_VOLTAGE_LIMIT`，再转换为 `int16_t`，避免大误差下先转换溢出。
3. **GM6020 待命基准一次性捕获**：不再把每次挥拍的目标固定计算为 `5527.0f + 偏移`；首次收到有效 `0x206` 反馈时，捕获当时的 `total_ecd` 作为待命基准，后续挥拍沿用该基准。

建议实车继续增加 CAN 反馈超时、IMU 数据有效性和电机温度/错误码保护。

## 8. 编译与烧录

1. 使用 Keil MDK 打开 `PVT/PVT/MDK-ARM/ShaoBin2.0.uvprojx`。
2. 安装对应的 STM32F4 Device Pack，并确认工程使用 ARMCC V5 工具链。
3. 编译前检查电机 ID、方向、零点和遥控器映射。
4. 第一次上电时拆除负载或架空机构。
5. 先验证 DR16 数据，再验证 CAN 反馈，最后逐个使能电机。
6. 不要直接使用仓库中历史编译产物判断当前源码是否可编译；应在目标 Keil 环境中执行全量 Rebuild。

## 9. 调试检查清单

- [ ] 控制板、接收器、电机和 CAN 收发器共地。
- [ ] CANH/CANL 没有接反，且总线两端终端电阻正确。
- [ ] DR16 能稳定收到 18 字节帧。
- [ ] 遥控器断电后 100 ms 内所有执行器停止。
- [ ] GM6020 反馈 `0x206` 的编码器值和速度变化方向正确。
- [ ] 达妙反馈 `0x11`～`0x15` 与电机顺序一致。
- [ ] 所有动作目标位置均在机械限位内。
- [ ] GM6020 输出没有超过软件限幅。
- [ ] 发送邮箱满、CAN 错误和电机失联均有可观察现象。

## 10. 关键源文件

| 文件 | 作用 |
|---|---|
| `Core/Src/main.c` | HAL、CAN、DMA、FreeRTOS 初始化 |
| `Core/Src/freertos.c` | 任务创建与调度配置 |
| `MDK-ARM/BSP/bsp_usart.c` | DR16 DMA 接收、在线状态与回调 |
| `MDK-ARM/User/dr16.c` | DR16 数据解码 |
| `MDK-ARM/User/bsp_can_DM.c` | 达妙/GM6020 CAN 控制与反馈解析 |
| `MDK-ARM/TASK/control_task.c` | 击球动作、GM6020 位置控制 |
| `MDK-ARM/TASK/Gimbal_control.c` | 旧云台控制路径 |
| `MDK-ARM/TASK/INS_task.c` | BMI088 与姿态解算 |
