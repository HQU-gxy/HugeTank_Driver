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
    static constexpr uint8_t MIN_DUTY = 5;
    static constexpr uint8_t MAX_DUTY = 200;

    float PID_KP = 0.3;
    float PID_KI = 0.1;
    float PID_KD = 0.5;

    static constexpr uint32_t SPEED_SCALE = 100; // Linear speed(m/s) to encoder output frequency(Hz)

    // For input capture
    static constexpr uint32_t OVERFLOW_VALUE = 0x10000;
    static constexpr uint16_t PRESCALE_FACTOR = 1600;
    volatile uint32_t freqMeasured, lastCapture = 0, currentCapture;
    uint32_t timerFreq = 0;
    uint32_t rolloverCompareCount = 0;

    /**
     * Set the duty cycle of the PWM signal
     * @param duty: duty cycle between 0 and 255
     */
    inline void setDuty(uint8_t duty)
    {
        pwmTimer->setCaptureCompare(pwmChannel, duty, RESOLUTION_8B_COMPARE_FORMAT);
    }

    /**
     * Get the encoder input frequency
     * @return the frequency measured
     */
    inline uint32_t getInputFrequency()
    {
        return freqMeasured;
    }

    /**
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

    /**
     * Set the direction signal
     * @param direction: direction signal value
     */
    void setDirection(bool direction);

    /**
     * Set the target speed of the motor
     * @param speed: target speed in m/s, positive for forward, negative for backward
     */
    void setSpeed(float speed);

    /**
     * Enable the ESC, making it fuckable
     */
    inline void enable()
    {
        digitalWrite(enablePin, 0);
    }

    /**
     * Disable the ESC, unpower the motor
     */
    inline void disable()
    {
        digitalWrite(enablePin, 1);
    }

    /**
     * Set the PID controller arguments
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

    /**
     * The PID implementation function
     * 
     * Used in an OS timer, don't call it manually
     */
    friend void fuckPID(TimerHandle_t);
};
