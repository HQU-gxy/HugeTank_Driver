#include "main.h"
#include <cmsis_os.h>
#include "st7735.h"

void app_main()
{
  
  while (1)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    osDelay(100);
  }
}
