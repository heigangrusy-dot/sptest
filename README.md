# sptest

> 同济大学 SuperPower 战队电控组 · **第一阶段个人考核**实施记录
>
> 本工程由 STM32CubeMX 从零生成（未基于任何现有模板），用于逐项实现并验证考核要求的软件功能。

---

## 一、考核总览

第一阶段个人考核共四部分，本仓库负责其中的**软件部分**；硬件部分为独立的嘉立创 EDA 工程。

| 编号 | 考核项 | 内容 | 本仓库 | 状态 |
|------|--------|------|--------|------|
| 4.4.1.1 | **C 板基本功能** | 蜂鸣器、LED、串口打印 IMU、遥控器通讯 | ✅ 是 | 🚧 进行中 |
| 4.4.1.2 | **姿态与电机联动** | DT7 拨杆模式切换 + C 板 IMU 与两台 GM6020 联动 | ✅ 是 | ⬜ 待做 |
| 4.4.1.3 | **24V 转 5V 降压模块** | 基于 MPS **MP4420A** 的降压变换器，嘉立创 EDA 原理图 + PCB | ❌ 独立 EDA 工程 | ⬜ 待做 |
| 4.4.1.4 | **软件与版本管理** | Git 版本管理、可追溯的提交历史、README | ✅ 是 | 🚧 进行中 |

---

## 二、主要任务

### 4.4.1.1 C 板基本功能

> 基本功能考核作为晋级的必要条件；未完成本项要求者，不予晋级。

| # | 任务 | 板载资源 / 引脚 | 验证方式 | 状态 |
|---|------|----------------|---------|--------|
| 1 | **蜂鸣器控制**：成功烧录或上电后有提示音，音乐可自定义 | 贴片无源蜂鸣器，`TIM4_CH3` → **PD14**，额定 4000 Hz，需 PWM 驱动 | 上电听到可辨认的旋律，且旋律能替换 | ⬜ |
| 2 | **LED 控制**：三色呼吸流水灯，并以此判断程序是否阻塞 | 共阳 RGB LED，`TIM5_CH1/2/3` → **PH10(蓝) / PH11(绿) / PH12(红)**，IO 高电平点亮 | 多色流水 + 呼吸；能解释「灯定住」对应哪类阻塞 | 🚧 |
| 3 | **串口打印**：在上位机软件中打印 IMU 三轴数据 | BMI088，`SPI1`：CLK **PB3** / MISO **PB4** / MOSI **PA7**；片选 `CS1_Accel` **PA4**、`CS1_Gyro` **PB0**；加热 `TIM10_CH1` **PF6**；输出经 `USART1` | SerialPlot 上出现随时间变化的曲线 | ⬜ |
| 4 | **遥控器控制**：C 板与 DT7 遥控器正常通讯 | 接收机 DR16 接 DBUS 接口，信号反相后入 `USART3_RX` = **PC11**，100 kbps，8E1，18 字节/帧 | 摇杆/拨轮/开关读数随操作变化，可判断是否在线 | ⬜ |

### 4.4.1.2 姿态与电机联动

**硬件构成**：C 板 + 两台 GM6020 电机，构成三个旋转输入端
（输入量分别为：C 板 IMU 的 yaw 角度、两台电机的编码器数值）。

与 C 板保持 **1:1** 联动比例的电机记为 **A 电机**，另一台记为 **B 电机**。

**右拨杆决定工作模式：**

| 右拨杆档位 | 模式 | 行为 |
|-----------|------|------|
| 下档 | **失能模式** | 所有电机处于**无力状态**（不输出力矩） |
| 中档 | **姿态联动模式** | 按下方联动规则运行 |
| 上档 | **复位模式** | 两台电机的指向标箭头重新对齐 C 板上指向标箭头的方向 |

**左拨杆决定 B 电机的联动比例：**

| 左拨杆档位 | B 电机 : C 板 | 示例（C 板绕 yaw 逆时针转 60°） |
|-----------|--------------|------------------------------|
| 下档 | **1 : 0.5** | A 电机逆时针 60°，B 电机逆时针 30° |
| 中档 | **1 : −1** | A 电机逆时针 60°，B 电机顺时针 60°（负号表示反向） |
| 上档 | **1 : 3** | A 电机逆时针 60°，B 电机逆时针 180° |

**联动控制要求：**

1. 将 C 板绕 yaw 轴转动时，两台电机按设定比例跟随运动。
2. 手动转动任一电机时，另一台电机按对应比例跟随运动，且 **C 板的实际偏航角（yaw）保持不变**。
   手动转动电机后，系统应同步调整 C 板的**联动参考零点**，该参考零点随 **A 电机**的位置变化。
3. 再次转动 C 板时，两台电机应基于调整后的参考零点继续联动，**不应自动返回原零位**（即不会像方向盘一样回正）。
4. C 板与 A 电机保持 1:1 联动比；C 板与 B 电机的联动比由左拨杆档位决定（见上表）。

**本项涉及的技术点**：

- GM6020 电流/电压控制与编码器反馈读取
- C 板 IMU 的姿态解算（yaw）
- 模式状态机（失能 / 联动 / 复位）
- 联动的**零点跟随**处理：手动转动电机时不改变 C 板实际 yaw，仅在内部平移参考零点

### 4.4.1.3 24V 转 5V 降压模块（独立 EDA 工程）

> 本项为硬件设计，不在本仓库内，相关文件为嘉立创 EDA 工程。

1. 基于 MPS **MP4420A** 降压变换器，依据培训课程内容自行设计 24V → 5V 降压模块
2. 用嘉立创 EDA 完成**原理图**设计
3. 用嘉立创 EDA 完成 **PCB** 设计
4. 入电/出电端使用 **XT30** 接口（**公头入电、母头出电**）
5. PCB 上需带有 **MPS 商标**（商标图片由组委会提供）
6. 设计中途/完成后可提交队内导师审核；每位选手有**不少于两次**审图答疑机会

### 4.4.1.4 软件与版本管理

| 要求 | 落实方式 |
|------|---------|
| 代码使用 Git 版本管理，按组委会要求上传至指定 GitHub 仓库 | 本仓库；⚠️ 需与队内确认「指定仓库」的具体指向 |
| 提交历史应反映开发过程，**不得仅在验收前一次性上传全部成果** | 按功能分次提交，见第七节进度记录 |
| 提交信息简洁说明变更内容；重要功能保留可回退的稳定版本 | 提交信息格式 `<type>(<scope>): <subject>`；每个里程碑打 **git tag** |
| 仓库应包含 README，至少说明主要任务和操作注意事项 | 本文件 |

---

## 三、硬件平台

| 项目 | 型号 / 说明 |
|------|------------|
| 主控板 | RoboMaster 开发板 C 型（STM32F407IGH6，UFBGA176，168 MHz） |
| 电机 | GM6020 直流无刷电机 × 2（CAN 通信，需设定 ID） |
| 烧录器 | 无线烧录器（CMSIS-DAP） |
| 遥控器 | RoboMaster DT7 + DR16 接收机 |
| 串口转接 | USB 转 TTL（CH340），用于 `USART1` 上位机通信 |
| 电源 | 24V（XT30，**红正黑负**） |

---

## 四、开发环境

| 工具 | 版本 |
|------|------|
| STM32CubeMX | 6.18.1 |
| STM32Cube 固件包 | STM32Cube FW_F4 **V1.28.3** |
| STM32CubeCLT（交叉编译工具链） | GCC `arm-none-eabi` 14.3.1（GNU Tools for STM32 14.3.rel1） |
| CMake | 4.3.1 |
| Ninja | 1.13.2 |
| OpenOCD | 0.12.0 |
| Visual Studio Code | 1.140.0 |
| VS Code 插件 | C/C++ 1.34.4、CMake Tools 1.24.42、Cortex-Debug 1.12.1 |
| Git | 2.56.0 |
| 上位机软件 | SerialPlot |
| 中间件 | `sp_middleware`（战队自研，命名空间 `sp::`） |
| 硬件 EDA | 嘉立创 EDA（4.4.1.3 使用） |

### 工程生成配置（CubeMX）

| 配置项 | 值 | 说明 |
|--------|-----|------|
| MCU | STM32F407IGH6 / UFBGA176 | — |
| Toolchain / IDE | **CMake** | 配合 VSCode + OpenOCD 使用 |
| `SYS → Debug` | **Serial Wire** | 保留 PA13/PA14，释放 PB3/PB4 给 SPI1 |
| `SYS → Timebase Source` | **TIM14** | SysTick 交由 FreeRTOS 调度使用 |
| `RCC → HSE` | Crystal/Ceramic Resonator，**12 MHz** | 板载晶振 |
| FreeRTOS 接口 | **CMSIS_V1** | `sp_middleware` 使用 v1 API |
| 时钟树 | HSE 12 MHz → PLLM 6 / PLLN 168 / PLLP 2 → SYSCLK **168 MHz** | AHB /1、APB1 /4、APB2 /2 |
| 定时器时钟 | APB1 = **84 MHz**（TIM4/TIM5）、APB2 = **168 MHz**（TIM1/TIM8） | 写 PWM 代码时直接用到 |
| 任务 | `defaultTask`（保留，不可删除） | 其余任务按功能逐个添加 |

---

## 五、操作注意事项

### 常用操作

| 操作 | 快捷键 / 命令 |
|------|--------------|
| 编译 | `F7` |
| 烧录（先自动编译） | `Ctrl+Shift+B` |
| 调试 | `F5` |
| 选择构建预设（首次必须） | `Ctrl+Shift+P` → `cmake: select configure preset` → `Debug` |
| 命令行编译 | `cmake --preset Debug && cmake --build --preset Debug` |
| 打里程碑标签 | `git tag -a <名称> -m "<说明>"` 后 `git push origin <名称>` |

### 硬件相关

- ⚠️ **24V 电源线红正黑负，接反会瞬间烧板。**
- ⚠️ `SYS → Debug` 必须保持 **Serial Wire**。若改回 JTAG，PB3/PB4 会被 JTAG 占用，SPI1 将无法使用。
- ⚠️ micro-USB 只给 `VCC_5V` 供电，带不动 PWM 接口等由 `VCC_5V_M` 供电的外设，完整功能验证需接 24V。
- ⚠️ `BOOT` 跳线帽默认均为低电平（从 Flash 启动），不要随意改动。
- ⚠️ **电机调试安全**：GM6020 通电后会输出力矩。上电前确认电机已固定、周围无人，失能模式（右拨杆下档）应作为上电默认状态。
- 无线烧录器的接口类型在 `openocd.cfg` 中配置。当前：
  ```tcl
  source [find interface/cmsis-dap.cfg]   # 无线烧录器
  # source [find interface/stlink.cfg]    # 若改用 ST-Link，把这两行注释对调
  source [find target/stm32f4x.cfg]
  ```
- ⚠️ 本仓库使用 `git submodule` 引入 `sp_middleware`，
  **克隆时必须加 `--recursive`**，否则 `sp_middleware/` 为空、编译失败：
  ```bash
  git clone --recursive https://github.com/heigangrusy-dot/sptest.git

### 代码相关

- **不要手改** `Core/`、`Drivers/`、`Middlewares/` 下由 CubeMX 生成的任何文件。
- 自己的代码写在 `applications/` 目录下（本工程约定），并**手动加进根目录 `CMakeLists.txt`** 的 `target_sources`，否则不会被编译。
- CubeMX 重新生成代码时只会覆盖 `/* USER CODE BEGIN xxx */` 与 `/* USER CODE END xxx */` **之间以外**的内容，因此自定义代码必须写在 `BEGIN/END` 之间。
- 新增 FreeRTOS 任务要在 **CubeMX 的 `Tasks and Queues`** 里添加，不要直接手写 `freertos.c` —— 手写内容会在下次重新生成时丢失。
- ⚠️ `FREERTOS → Tasks and Queues` 中必须**保留名为 `defaultTask` 的默认任务**。清空该页会导致 CubeMX 代码模板渲染失败，生成的 `freertos.c` 会出现 `#n` 等非法内容，且整个项目生成中断。
- **LED 是独占资源**：同一时刻只应有一个任务控制 RGB LED，否则多个任务抢着改颜色会看起来像乱闪。建议由一个任务统一读取状态并显示。
- 编译产物在 `build/`（已被 `.gitignore` 忽略，不纳入版本管理）。

### 排查手段

| 现象 | 处理 |
|------|------|
| CubeMX 生成失败或"卡住" | 查看 `%USERPROFILE%\.stm32cubemx\STM32CubeMX.log` 末尾若干行，`[ERROR]` / `IOException` 即原因 |
| CubeMX 窗口无法关闭 | 进程名为 `javaw.exe`；通常是模态对话框卡住导致主窗口被禁用，可 `Stop-Process -Name javaw -Force` |
| 编译报找不到 `arm-none-eabi-gcc` | CubeCLT 未安装或未加入 PATH，安装后需重开 VSCode / 终端 |
| IMU 三轴恒为 0 | 检查 SPI1 片选引脚（PA4 / PB0）、`CPOL=High / CPHA=2Edge`；SPI 预分频可先由 8 改为 16 降低速率 |
| 遥控器只能收到第一帧 | `HAL_UARTEx_RxEventCallback` 中漏了重新调用接收（`remote.request()`） |

---

## 六、代码规范

遵循战队《代码风格和命名规范》：

- 代码风格基于 **ROS2 / Google**，使用仓库根目录的 `.clang-format`
- `.vscode/settings.json` 已开启保存时自动格式化；**`.h` 文件被特意排除**（避免改动 CubeMX 生成的代码），因此**自己写的头文件一律使用 `.hpp` 后缀**
- 命名要点：
  - 文件夹 / 源文件 / 变量 / 函数：`snake_case`
  - 类名：`CamelCase`（缩写类名用下划线分割，如 `DM_Motor`）；private 成员结尾加 `_`
  - 编译期常量：`constexpr` + `UPPER_CASE`，**禁止使用 `#define` 定义常量**
  - 命名空间：全小写 + 下划线；**禁止 `using namespace`**
  - 有物理含义的数值使用国际单位制；非 SI 单位需体现在变量名中（如 `speed_rpm`、`angle_deg`）
- Git 提交信息格式：`<type>(<scope>): <subject>`，例如
  - `feat(led): 用 TIM5 PWM 实现流水灯`
  - `fix(imu): 修正 BMI088 片选引脚`

---

## 七、目录结构

```
sptest/
├── Core/                  # CubeMX 生成：main.c / freertos.c / 各外设初始化
│   ├── Inc/               #   .h 头文件（不参与自动格式化）
│   └── Src/
├── Drivers/               # CubeMX 生成：STM32F4xx HAL + CMSIS
├── Middlewares/           # CubeMX 生成：FreeRTOS 等
├── applications/          # 【自己写】各功能任务实现（.cpp/.hpp）
├── cmake/                 # CubeMX 生成的构建脚本与工具链配置
├── CMakeLists.txt         # ← 新增源文件需要在这里登记
├── CMakePresets.json      # 构建预设（Debug / Release）
├── sptest.ioc             # CubeMX 工程配置（务必纳入版本管理）
├── openocd.cfg            # 烧录器 / 目标芯片配置
├── startup_stm32f407xx.s  # 启动文件
├── STM32F407xx_FLASH.ld   # 链接脚本
├── .clang-format          # 代码格式化规则
├── .vscode/               # 调试 / 烧录 / 格式化设置
└── build/                 # 编译产物（git 忽略）
```

---

## 八、开发进度记录

| 日期 | 提交 / 标签 | 内容 |
|------|------------|------|
| 2026-10-03 | `v0.1.0-baseline` | 生成可编译的 C 板工程基线（时钟树 168 MHz / FreeRTOS CMSIS_V1 / Serial Wire），补充 README |
| | | |

### 里程碑规划

```
v0.1.0-baseline   空工程可编译
v0.2.0-led        LED 流水灯 + 呼吸灯              (4.4.1.1-2)
v0.3.0-buzzer     蜂鸣器提示音                     (4.4.1.1-1)
v0.4.0-imu        SPI1 + BMI088 + SerialPlot 打印   (4.4.1.1-3)
v0.5.0-dbus       USART3 + DBUS 遥控器通讯          (4.4.1.1-4)
v0.6.0-linkage    IMU 与双 GM6020 姿态联动          (4.4.1.2)
```
