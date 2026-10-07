#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// 实验室验证基础功能期间设 false（不跑 CAN）
// 以后验证 CAN / 电机时改回 true
constexpr bool ENABLE_CAN = true;

constexpr float K_RATIO_DOWN = 0.5f;  // 左拨杆下
constexpr float K_RATIO_MID = -1.0f;  // 左拨杆中
constexpr float K_RATIO_UP = 3.0f;    // 左拨杆上

// 遥控器 yaw 灵敏度（摇杆满偏 → φ 变化多少 rad）
constexpr float YAW_SCALE = 0.05f;

// 手动拖拽吸收增益（0=不吸收，越大越跟随手）
constexpr float K_DRAG = 0.02f;
}  // namespace

// 两个GM6020：拨码开关把 ID 设成 1 和 2
sp::RM_Motor motor_A(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_B(2, sp::RM_Motors::GM6020);

sp::CAN can1(&hcan1);

// 自己一个plotter
extern sp::Plotter plotter;
extern sp::DBus remote;

float phi = 0.0f;
float yaw_last = 0.0f;

sp::PID pid_A(0.001f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, false, true);
//           dt=1ms    kp    ki   kd  max_out  max_iout  alpha  angular  dynamic
sp::PID pid_B(0.001f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, false, true);

float target_angle = 0.0f;

extern "C" void linkage_task(void const * argument)
{
  (void)argument;

  if (!ENABLE_CAN) {
    while (true) {
      osDelay(1000);
    }
  }

  osDelay(500);   // 等待CAN总线稳定
  can1.config();  // 使用C板官方示例的CAN过滤器配置
  can1.start();   // 启动CAN总线

  // ★★★ 等两个电机都有反馈了，再采零点（否则会采到初值 0！）
  while (!motor_A.is_alive(osKernelSysTick()) || !motor_B.is_alive(osKernelSysTick())) {
    osDelay(10);
  }
  osDelay(50);

  const float a_zero = motor_A.angle;
  const float b_zero = motor_B.angle;
  phi = 0.0f;
  yaw_last = remote.ch_rh;

  while (true) {
    if (!motor_A.is_alive(osKernelSysTick()) || !motor_B.is_alive(osKernelSysTick())) {
      motor_A.cmd(0.0f);
      motor_B.cmd(0.0f);
      motor_A.write(can1.tx_data);
      motor_B.write(can1.tx_data);
      can1.send(motor_A.tx_id);
      osDelay(1);
      continue;
    }

    //遥控器在线检查
    // ★ 简化版：φ 直接跟随 A 的转角，B 跟随 k 倍
    constexpr float k = 3.0f;
    phi = motor_A.angle - a_zero;

    pid_A.calc(phi + a_zero, motor_A.angle);  // A 目标=自己 → 手能拖
    pid_B.calc(k * phi + b_zero, motor_B.angle);

    motor_A.cmd(pid_A.out);
    motor_B.cmd(pid_B.out);
    motor_A.write(can1.tx_data);
    motor_B.write(can1.tx_data);
    can1.send(motor_A.tx_id);

    plotter.plot(motor_A.angle, motor_B.angle, phi + a_zero, k * phi + b_zero);

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_A.rx_id) motor_A.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_B.rx_id) motor_B.read(can1.rx_data, stamp_ms);
    }
  }
}
