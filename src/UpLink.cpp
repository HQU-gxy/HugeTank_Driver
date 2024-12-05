#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <timers.h>
#include <ulog.h>

#include "config.h"

namespace UpLink
{
  static String buf;

  static bool avail;
  static std::pair<float, float> targetSpeed;

  struct __attribute__((packed)) UpLinkCommand
  {
    uint8_t header;
    uint8_t RESERVED1;
    uint8_t RESERVED2;

    // Note: Speed in mm/s
    uint8_t targetLinear_H;
    uint8_t targetLinear_L;

    // Not used in differential drive
    uint8_t targetY_H;
    uint8_t targetY_L;

    // Note: Speed in mrad/s
    uint8_t targetAngular_H;
    uint8_t targetAngular_L;

    uint8_t checksum;
    uint8_t ass;
  };

  struct __attribute__((packed)) UpLinkCarStatus
  {
    uint8_t header = 0x7b;
    uint8_t isDisabled; // True if the car is not ready

    // Note: Speed in mm/s
    uint8_t linear_H;
    uint8_t linear_L;

    // Not used in differential drive
    uint8_t Y_H;
    uint8_t Y_L;

    // Note: Speed in mrad/s
    uint8_t angular_H;
    uint8_t angular_L;

    uint8_t checksum;
    uint8_t ass = 0x7d;
  };

  void init()
  {
    Serial1.setTx(UART1_TX_PIN);
    Serial1.setRx(UART1_RX_PIN);
    Serial1.begin(115200);
  }

  void readCmdAndSendStat(TimerHandle_t)
  {

    if (buf.length() >= 11)
    {
      if (buf[0] == 0x7b)
      {
        auto parsed = reinterpret_cast<const UpLinkCommand *>(buf.c_str());
        uint8_t sum;
        for (uint8_t i = 0; i < 9; i++)
        {
          sum ^= buf[i];
        }

        if (sum == parsed->checksum)
        {
          ULOG_WARNING("UpLink command checksum error");
        }
        else
        {
          targetSpeed.first = ((parsed->targetLinear_H << 8) | parsed->targetLinear_L) / 1000.0f;
          targetSpeed.second = ((parsed->targetAngular_H << 8) | parsed->targetAngular_L) / 1000.0f;
        }
      }
      buf = "";
    }

    if (!Serial1.available())
      return;

    while (Serial1.available())
    {
      buf += Serial1.read();
    }
  }

} // namespace UpLink
