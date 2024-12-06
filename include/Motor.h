#pragma once

#include <Arduino.h>
#include <memory>
#include <STM32FreeRTOS.h>
#include <timers.h>

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

    bool direction = 0;                       // 0 for forward, 1 for backward
    uint32_t targetFreq = 0;                  // Target encoder frequency in Hz
    static constexpr uint8_t PID_PERIOD = 50; // Fuck PID every 50ms
    static constexpr uint8_t MIN_DUTY = 10;
    static constexpr uint8_t MAX_DUTY = 200;       // To avoid Flying away
    static constexpr uint8_t MIN_TARGET_FREQ = 15; // Whether to park

    float PID_KP = 0.3;
    float PID_KI = 0.2;
    float PID_KD = 0.5;

    static constexpr uint32_t SPEED_SCALE = 56; // Linear speed(m/s) to encoder output frequency(Hz)

    // For input capture
    static constexpr uint32_t OVERFLOW_VALUE = 0x10000;
    static constexpr uint16_t PRESCALE_FACTOR = 1600;
    volatile uint32_t freqMeasured, lastCapture = 0, currentCapture;
    uint32_t timerFreq = 0;
    uint32_t rolloverCompareCount = 0;

    /**
     * @brief Set the duty cycle of the PWM signal
     *
     * @param duty: duty cycle between 0 and 255
     */
    inline void setDuty(uint8_t duty)
    {
        pwmTimer->setCaptureCompare(pwmChannel, duty, RESOLUTION_8B_COMPARE_FORMAT);
    }

    /**
     * @brief Get the encoder input frequency
     *
     * @return the frequency measured
     */
    inline uint32_t getInputFrequency()
    {
        return freqMeasured;
    }

    /**
     * @brief Callback function for the input capture interrupt
     */
    void feedbackInputCallback();

public:
    Motor(uint8_t alarm_pin,
          uint8_t enable_pin,
          uint8_t direction_pin,
          TimerCfg feedback_timer,
          TimerCfg pwm_timer);

    ~Motor();

    /**
     * @brief Set the direction signal
     *
     * @param direction: direction signal value
     */
    inline void setDirection(bool direction)
    {
        digitalWrite(directionPin, direction);
    }

    /**
     * @brief Enable the ESC, making it fuckable
     */
    inline void enable()
    {
        digitalWrite(enablePin, 0);
    }

    /**
     * @brief Disable the ESC, unpower the motor
     */
    inline void disable()
    {
        digitalWrite(enablePin, 1);
    }

    /**
     * @brief Set the PID controller arguments
     *
     * @param kp: Proportional gain
     * @param ki: Integral gain
     * @param kd: Derivative gain
     */
    inline void setPID(float kp, float ki, float kd)
    {
        PID_KP = kp;
        PID_KI = ki;
        PID_KD = kd;
    }

    inline float getSpeed()
    {
        return static_cast<float>(freqMeasured) / SPEED_SCALE;
    }

    /**
     * @brief Set the target speed of the motor
     *
     * @param speed: target speed in m/s, positive for forward, negative for backward
     */
    void setSpeed(float speed);

    /**
     * @brief The PID implementation function
     *
     * Used in an OS timer, don't call it manually
     */
    friend void fuckPID(TimerHandle_t);
};
