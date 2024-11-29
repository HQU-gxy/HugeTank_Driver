#include <Arduino.h>
#include <ulog.h>
#include <STM32FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <TFT_eSPI.h>
#include <vector>

#include "Motor.h"
#include "IMU.h"
#include "config.h"

void app_main(void *)
{
  static TFT_eSPI screen;
  screen.init();
  screen.setRotation(3);
  screen.fillScreen(TFT_BLACK);
  screen.setTextSize(2);
  screen.setTextColor(TFT_RED, TFT_BLACK);

  Motor rightMotor(MOTOR_RIGHT_ALM_PIN, MOTOR_RIGHT_EN_PIN, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_FG, MOTOR_RIGHT_PWM);
  // Motor leftMotor(MOTOR_LEFT_ALM_PIN, MOTOR_LEFT_EN_PIN, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_FG, MOTOR_LEFT_PWM);

  rightMotor.enable();
  // leftMotor.enable();

  auto aliveLEDTimer = xTimerCreate("Alive", pdMS_TO_TICKS(1000), true, (void *)233, [](TimerHandle_t)
                                    { digitalToggle(LED1_PIN); });
  xTimerStart(aliveLEDTimer, 0);

  while (1)
  {
#ifdef PID_TUNING
    struct __attribute__((packed)) PIDConfig
    {
      uint8_t header;
      float kp;
      float ki;
      float kd;
      char shit[4];
    };

    struct __attribute__((packed)) SpeedSet
    {
      uint8_t header;
      float speed;
      char shit[4];
    };

    static String inputStr;
    if (Serial2.available())
    {
      char c = Serial2.read();
      inputStr += c;
      if (inputStr.endsWith("shit"))
      {
        if (inputStr[0] == 0x96)
        {
          if (inputStr.length() != sizeof(PIDConfig))
            continue;
          auto parsed = reinterpret_cast<const PIDConfig *>(inputStr.c_str());
          rightMotor.setPID(parsed->kp, parsed->ki, parsed->kd);
        }
        else if (inputStr[0] == 0x97)
        {
          if (inputStr.length() != sizeof(SpeedSet))
            continue;
          auto parsed = reinterpret_cast<const SpeedSet *>(inputStr.c_str());
          rightMotor.setSpeed(parsed->speed);
        }
        inputStr = "";
      }
    }
#endif
#ifdef IMU_TEST
    IMU::IMUData readData;
    IMU::getIMUData(&readData);

    struct __attribute__((packed))
    {
      uint8_t header = 0x11;
      IMU::IMUData data;
      char shit[4] = {'s', 'h', 'i', 't'};
    } imuData = {
        .data = readData,
    };
    Serial2.write(reinterpret_cast<char *>(&imuData), sizeof(imuData));

#endif
    vTaskDelay(50);
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

#if defined(PID_TUNING) || defined(IMU_TEST)
  // To reduce data tranmitted
  ulog_subscribe(my_console_logger, ULOG_ERROR_LEVEL);
#else
  ulog_subscribe(my_console_logger, ULOG_DEBUG_LEVEL);
#endif

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  if (!IMU::begin())
  {
    while (1)
    {
      ULOG_ERROR("IMU initialization failed");
      while (1)
      {
        digitalWrite(LED2_PIN, 1);
        delay(100);
        digitalWrite(LED2_PIN, 0);
        delay(400);
      }
    }
  }

  ULOG_INFO("I'm fucking coming");
  xTaskCreate(app_main, "app_main", 1024, NULL, osPriorityNormal, NULL);
  vTaskStartScheduler();
}

void loop() { /* Never fuck into this*/ }
