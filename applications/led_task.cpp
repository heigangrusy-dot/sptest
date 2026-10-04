#include "cmsis_os.h"
#include "main.h"

namespace
{
//每个颜色点亮的时间，单位：毫秒
constexpr uint32_t LED_ON_MS = 300;
}  //namespace

extern "C" void led_task(void const * argument)
{
  (void)argument;

  while (true) {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    osDelay(LED_ON_MS);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    osDelay(LED_ON_MS);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    osDelay(LED_ON_MS);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  }
}