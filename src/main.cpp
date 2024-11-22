#include <Arduino.h>
#include <ulog.h>
#include <STM32FreeRTOS.h>
#include <semphr.h>

#include "Motor.h"
#include "IMU.h"
#include "config.h"

void app_main(void *)
{
  Motor rightMotor(MOTOR_RIGHT_ALM_PIN, MOTOR_RIGHT_EN_PIN, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_FG, MOTOR_RIGHT_PWM);
  Motor leftMotor(MOTOR_LEFT_ALM_PIN, MOTOR_LEFT_EN_PIN, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_FG, MOTOR_LEFT_PWM);

  rightMotor.enable();
  leftMotor.enable();
  while (1)
  {
    ULOG_INFO("left frequency: %d", leftMotor.getInputFrequency());
    vTaskDelay(1000);
  }
}

void setup()
{
  Serial1.setTx(UART1_TX_PIN);
  Serial1.setRx(UART1_RX_PIN);
  Serial1.begin(115200);

  Serial2.setTx(UART2_TX_PIN);
  Serial2.setRx(UART2_RX_PIN);
  Serial2.begin(115200);

  static auto loggerMutex = xSemaphoreCreateMutex();
  static auto my_console_logger = [](ulog_level_t severity, char *msg)
  {
    // Not interrupt safe, DO NOT USE ULOG IN ISR
    if (xSemaphoreTake(loggerMutex, 100) == pdTRUE)
    {
      Serial2.printf("%d [%s]: %s\n", millis(), ulog_level_name(severity), msg);
      xSemaphoreGive(loggerMutex);
    }
  };
  ulog_init();
  ulog_subscribe(my_console_logger, ULOG_DEBUG_LEVEL);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);


  ULOG_INFO("I'm fucking coming");
  xTaskCreate(app_main, "app_main", 1024, NULL, osPriorityNormal, NULL);
  vTaskStartScheduler();
}

void loop() { /* Never fuck into this*/ }
