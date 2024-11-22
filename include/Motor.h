#pragma once

#include <Arduino.h>
#include <memory>

#include "config.h"

class Motor
{
private:
    uint8_t alarmPin;
    uint8_t enablePin;
    uint8_t directionPin;

    std::shared_ptr<HardwareTimer> feedbackTimer;
    std::shared_ptr<HardwareTimer> pwmTimer;
    uint8_t feedbackChannel;
    uint8_t pwmChannel;

    bool direction;

    // For input capture
    static constexpr uint32_t OVERFLOW_VALUE = 0x10000;
    static constexpr uint16_t PRESCALE_FACTOR = 1600;
    volatile uint32_t freqMeasured, lastCapture = 0, currentCapture;
    uint32_t timerFreq = 0;
    volatile uint32_t rolloverCompareCount = 0;

    /***
     * Set the duty cycle of the PWM signal
     * @param duty: duty cycle between 0 and 255
     */
    inline void setDuty(uint8_t duty)
    {
        pwmTimer->setCaptureCompare(pwmChannel, duty, RESOLUTION_8B_COMPARE_FORMAT);
    }

    /***
     * Callback function for the input capture interrupt
     */
    void feedbackInputCallback();

public:
    Motor(uint8_t alarm_pin,
          uint8_t enable_pin,
          uint8_t direction_pin,
          TimerCfg feedback_timer,
          TimerCfg pwm_timer);

    ~Motor();
    /***
     * Set the direction signal
     * @param direction: direction signal value
     */
    void setDirection(bool direction);
    void setSpeed(int32_t speed);

    /***
     * Enable the ESC, making it fuckable
     */
    inline void enable()
    {
        digitalWrite(enablePin, 0);
    }

    /***
     * Disable the ESC, unpower the motor
     */
    inline void disable()
    {
        digitalWrite(enablePin, 1);
    }

    inline uint32_t getInputFrequency()
    {
        return freqMeasured;
    }
};
