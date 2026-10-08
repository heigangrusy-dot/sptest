#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// 实验室验证基础功能期间设 false（不跑 CAN）
// 以后验证 CAN / 电机时改回 true
constexpr bool ENABLE_CAN = true;
constexpr float K_RATIO_DOWN = 0.5f;  // 左拨杆下
constexpr float K_RATIO_MID = -1.0f;  // 左拨杆中
constexpr float K_RATIO_UP = 3.0f;    // 左拨杆上

// 手动拖拽吸收增益（0=不吸收，越大越跟随手）
constexpr float K_DRAG = 0.025f;

// 复位目标角
constexpr float A_HOME = -3.50208f;
constexpr float B_HOME = -0.368155f;
constexpr float YAW_CALIB = 0.0107f;
}  // namespace

// 两个GM6020：拨码开关把 ID 设成 1 和 2
sp::RM_Motor motor_A(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_B(2, sp::RM_Motors::GM6020);

sp::CAN can1(&hcan1);

extern sp::Plotter plotter;
extern sp::DBus remote;
extern sp::Mahony imu;

float phi = 0.0f;
float yaw_last = 0.0f;
float k_last = 0.0f;
sp::DBusSwitchMode mode_last = sp::DBusSwitchMode::DOWN;

sp::PID pid_A(0.001f, 1.3f, 0.0f, 0.045f, 1.0f, 0.0f, 0.5f, false, true);
//           dt=1ms    kp    ki   kd  max_out  max_iout  alpha  angular  dynamic
sp::PID pid_B(0.001f, 1.3f, 0.0f, 0.045f, 1.0f, 0.0f, 0.5f, false, true);

extern "C" void linkage_task(void const * argument)
{
  (void)argument;

  if (!ENABLE_CAN) {
    while (true) {
      osDelay(1000);
    }
  }

  osDelay(500);
  can1.config();
  can1.start();

  // 等两个电机都上线再采零点
  while (!motor_A.is_alive(osKernelSysTick()) || !motor_B.is_alive(osKernelSysTick())) {
    osDelay(10);
  }
  osDelay(50);
  float a_zero = motor_A.angle;
  float b_zero = motor_B.angle;
  phi = 0.0f;
  yaw_last = imu.yaw;

  while (true) {
    //失联保护
    if (!motor_A.is_alive(osKernelSysTick()) || !motor_B.is_alive(osKernelSysTick())) {
      motor_A.cmd(0.0f);
      motor_B.cmd(0.0f);
      motor_A.write(can1.tx_data);
      motor_B.write(can1.tx_data);
      can1.send(motor_A.tx_id);
      osDelay(1);
      continue;
    }

    //读拨杆
    const bool rc_ok = remote.is_alive(osKernelSysTick());
    const sp::DBusSwitchMode mode_r = rc_ok ? remote.sw_r : sp::DBusSwitchMode::DOWN;
    const sp::DBusSwitchMode mode_l = rc_ok ? remote.sw_l : sp::DBusSwitchMode::DOWN;

    //左拨杆
    float k = K_RATIO_UP;
    if (mode_l == sp::DBusSwitchMode::DOWN) {
      k = K_RATIO_DOWN;
    }
    else if (mode_l == sp::DBusSwitchMode::MID) {
      k = K_RATIO_MID;
    }

    const bool entering_mid =
      (mode_r == sp::DBusSwitchMode::MID) && (mode_last != sp::DBusSwitchMode::MID);
    if (entering_mid || k != k_last) {
      a_zero = motor_A.angle;
      b_zero = motor_B.angle;
      phi = 0.0f;
      yaw_last = imu.yaw;
    }
    mode_last = mode_r;
    k_last = k;

    // 右拨杆
    switch (mode_r) {
      case sp::DBusSwitchMode::DOWN:  //失能
        motor_A.cmd(0.0f);
        motor_B.cmd(0.0f);
        pid_A.clear();
        pid_B.clear();
        phi = motor_A.angle - a_zero;
        yaw_last = imu.yaw;
        break;

      case sp::DBusSwitchMode::MID: {  //联动
        // 遥控器 yaw 的增量驱动 φ7
        const float d_yaw = sp::limit_angle(imu.yaw - yaw_last);
        yaw_last = imu.yaw;
        phi += d_yaw;

        // 手动拖拽
        if (std::abs(d_yaw) < 0.0005f) {
          phi += K_DRAG * (motor_A.angle - (phi + a_zero));
          phi += K_DRAG * ((motor_B.angle - b_zero) / k - phi);
        }

        // 两个电机的目标
        pid_A.calc(phi + a_zero, motor_A.angle);
        pid_B.calc(k * phi + b_zero, motor_B.angle);
        motor_A.cmd(pid_A.out);
        motor_B.cmd(pid_B.out);
        break;
      }

      case sp::DBusSwitchMode::UP: {  //复位
        // R 标对齐位置 = 校准值 + C 板当前 yaw
        const float home_A = A_HOME + (imu.yaw - YAW_CALIB);
        const float home_B = B_HOME + (imu.yaw - YAW_CALIB);
        const float set_A = motor_A.angle + sp::limit_angle(home_A - motor_A.angle);
        const float set_B = motor_B.angle + sp::limit_angle(home_B - motor_B.angle);
        pid_A.calc(set_A, motor_A.angle);
        pid_B.calc(set_B, motor_B.angle);
        motor_A.cmd(pid_A.out);
        motor_B.cmd(pid_B.out);

        // 复位后，联动零点 = 当前（R 标）位置
        a_zero = motor_A.angle;
        b_zero = motor_B.angle;
        phi = 0.0f;
        yaw_last = imu.yaw;
        break;
      }
    }

    //一帧控两个电机
    motor_A.write(can1.tx_data);
    motor_B.write(can1.tx_data);
    can1.send(motor_A.tx_id);

    //绘图（6 通道
    const float err_A = (phi + a_zero) - motor_A.angle;
    const float err_B = (k * phi + b_zero) - motor_B.angle;
    plotter.plot(err_A, err_B, phi, imu.yaw, motor_A.angle, motor_B.angle);
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
