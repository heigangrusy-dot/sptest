#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// 实验室验证基础功能期间设 false（不跑 CAN）
// 以后验证 CAN / 电机时改回 true
constexpr bool ENABLE_CAN = true;
}  // namespace

// 两个GM6020：拨码开关把 ID 设成 1 和 2
sp::RM_Motor motor_A(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_B(2, sp::RM_Motors::GM6020);

sp::CAN can1(&hcan1);

// 自己一个plotter
extern sp::Plotter plotter;

sp::PID pid_A(0.001f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, true, true);
//           dt=1ms    kp    ki   kd  max_out  max_iout  alpha  angular  dynamic

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

  target_angle = motor_A.angle;

  while (true) {
    pid_A.calc(target_angle, motor_A.angle);
    motor_A.cmd(pid_A.out);
    motor_A.write(can1.tx_data);
    motor_B.write(can1.tx_data);
    can1.send(motor_A.tx_id);
    plotter.plot(motor_A.angle, pid_A.out, motor_A.speed, target_angle);
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
