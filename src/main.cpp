#include <Arduino.h>
#include <ulog.h>
#include <STM32FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <TFT_eSPI.h>
#include <vector>
#include <utility>
#include <sbus.h>

#include "Motor.h"
#include "IMU.h"
#include "config.h"
#include "UpLink.h"

/**
 * @brief Convert the linear and angular speed of the car to the speed of the left and right motors
 *
 * @param linear Linear speed of the car in m/s
 * @param angular Angular speed of the car in rad/s
 *
 * @return A pair of left and right motor speed in m/s
 */
std::pair<float, float> carSpeedToMotorSpeed(float linear, float angular)
{
  float leftSpeed = linear - angular * WHEEL_DISTANCE / 2;
  float rightSpeed = linear + angular * WHEEL_DISTANCE / 2;

  return {leftSpeed, rightSpeed};
}

/**
 * @brief Convert the speed of the left and right motors to the linear and angular speed of the car
 *
 * @param left Speed of the left motor in m/s
 * @param right Speed of the right motor in m/s
 *
 * @return A pair of linear and angular speed of the car in m/s and rad/s
 */
std::pair<float, float> motorSpeedToCarSpeed(float left, float right)
{
  float linear = (left + right) / 2;
  float angular = (right - left) / WHEEL_DISTANCE;

  return {linear, angular};
}

void app_main(void *)
{
  static TFT_eSPI screen;
  screen.init();
  screen.setRotation(3);
  screen.fillScreen(TFT_BLACK);
  screen.setTextColor(TFT_RED, TFT_BLACK);

  Motor rightMotor(MOTOR_RIGHT_ALM_PIN, MOTOR_RIGHT_EN_PIN, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_FG, MOTOR_RIGHT_PWM);
  Motor leftMotor(MOTOR_LEFT_ALM_PIN, MOTOR_LEFT_EN_PIN, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_FG, MOTOR_LEFT_PWM);

  rightMotor.enable();
  leftMotor.enable();

  // Wait a while to get the IMU offset
  vTaskDelay(pdMS_TO_TICKS(300));
  IMU::resetOffset();

  Serial3.setRx(UART3_RX_PIN);
  Serial3.setTx(UART3_TX_PIN);
  bfs::SbusRx sbus(&Serial3);
  sbus.Begin();

  std::pair<float, float> upLinkTargetSpeed = {0, 0}; // Linear and angular speed from up-link in m/s and rad/s

  auto getStatus = [&leftMotor, &rightMotor]() -> std::tuple<float, float, IMU::IMUData>
  {
    IMU::IMUData imuData;
    IMU::getIMUData(&imuData);
    auto [linear, angular] = motorSpeedToCarSpeed(leftMotor.getSpeed(), rightMotor.getSpeed());
    return {linear, angular, imuData};
  };
  UpLink::setGetStatusFunc(getStatus);

  auto onCommand = [&upLinkTargetSpeed](float linear, float angular)
  {
    upLinkTargetSpeed = {linear, angular};
  };
  UpLink::setOnCmdCallback(onCommand);
  UpLink::begin();

  auto aliveLEDTimer = xTimerCreate("Alive", pdMS_TO_TICKS(500), true, (void *)233, [](TimerHandle_t)
                                    { digitalToggle(LED1_PIN); });
  xTimerStart(aliveLEDTimer, 0);

  screen.setCursor(0, 0);
  screen.print("Mode: UPLINK");

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

#if (!defined(IMU_TEST)) && (!defined(PID_TUNING)) // Normally working

    screen.fillScreen(TFT_BLACK);
    screen.setCursor(0, 0);
    screen.print("Mode: ");

    float targetLinear, targetAngular;
    static uint8_t noDataCnt = 0;

    if (!sbus.Read())
    {
      if (noDataCnt < 10)
        noDataCnt++;
      else // Switch to uplink if there's no data from sbus for long
      {
        screen.print("Uplink");
        targetLinear = upLinkTargetSpeed.first;
        targetAngular = upLinkTargetSpeed.second;
      }
    }
    else
    {
      // Roll: 0, Throttle: 1, Pitch: 2, Yaw: 3, *Mode*: 6
      noDataCnt = 0;
      auto sbusData = sbus.data();
      if (sbusData.ch[6] < 600) // Stop
      {
        screen.print("Stop");
        targetLinear = targetAngular = 0;
      }
      else if (sbusData.ch[6] < 1400) // Uplink
      {
        screen.print("Uplink");
        targetLinear = upLinkTargetSpeed.first;
        targetAngular = upLinkTargetSpeed.second;
      }
      else // Sbus
      {
        screen.print("SBus");
        if (sbusData.ch[2] > 1400) // Forward
          targetLinear = static_cast<float>(2000 - sbusData.ch[1]) / 800;
        else if (sbusData.ch[2] > 600) // Dead zone
          targetLinear = 0;
        else // Backward
          targetLinear = static_cast<float>(sbusData.ch[1] - 2000) / 800;

        if (sbusData.ch[0] > 1300 || sbusData.ch[0] < 700) // Turn
          targetAngular = static_cast<float>(1000 - sbusData.ch[0]) / 400;
        else
          targetAngular = 0;
      }
    }

    auto [leftSpeed, rightSpeed] = carSpeedToMotorSpeed(targetLinear, targetAngular);
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);

    screen.setCursor(0, 30);
    screen.println("Linear: ");
    screen.println(targetLinear);
    screen.println("Angular: ");
    screen.print(targetAngular);
#endif
    vTaskDelay(50);
  }
}

void setup()
{
  Serial2.setTx(UART2_TX_PIN);
  Serial2.setRx(UART2_RX_PIN);
  Serial2.begin(115200);

  auto logger = [](ulog_level_t severity, char *msg)
  { Serial2.printf("%d [%s]: %s\n", xTaskGetTickCount(), ulog_level_name(severity), msg); };

  ulog_init();
#if defined(PID_TUNING) || defined(IMU_TEST)
  // To reduce data tranmitted
  ulog_subscribe(logger, ULOG_ERROR_LEVEL);
#else
  ulog_subscribe(logger, ULOG_DEBUG_LEVEL);
#endif

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  if (!IMU::begin())
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

  // Press the button to reset the IMU offset
  attachInterrupt(BUTTON_PIN, []
                  { IMU::resetOffset(); }, FALLING);

  ULOG_INFO("I'm fucking coming");
  xTaskCreate(app_main, "app_main", 2048, NULL, osPriorityNormal, NULL);
  vTaskStartScheduler();
}

void loop() { /* Never fuck into this*/ }
