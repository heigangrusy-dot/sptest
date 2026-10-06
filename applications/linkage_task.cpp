#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// 两个GM6020：拨码开关把 ID 设成 1 和 2
sp::RM_Motor motor_A(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_B(2, sp::RM_Motors::GM6020);

sp::CAN can1(&hcan1);

// 自己一个plotter
sp::Plotter plotter(&huart1);

extern "C" void linkage_task(void const * argument)
{
  (void)argument;

  osDelay(500);   // 等待CAN总线稳定
  can1.config();  // 使用C板官方示例的CAN过滤器配置
  can1.start();   // 启动CAN总线

  while (true) {
    // ---- 第一阶段：只发0力矩，验证能否收到反馈 ----
    motor_A.cmd(0.0f);
    motor_B.cmd(0.0f);

    //send 一次
    motor_A.write(can1.tx_data);
    motor_B.write(can1.tx_data);
    can1.send(motor_A.tx_id);

    // ---- 用SerialPlot 观察（8通道） ----
    plotter.plot(
      motor_A.angle, motor_A.speed, motor_A.torque, motor_A.temperature, motor_B.angle,
      motor_B.speed, motor_B.torque, motor_B.temperature);

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