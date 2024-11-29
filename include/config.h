#pragma once
#include <Arduino.h>

struct TimerCfg
{
    uint16_t pin;
    TIM_TypeDef *timer;
    uint8_t channel;
};

constexpr uint8_t LED1_PIN = PA0;
constexpr uint8_t LED2_PIN = PA1;

// Uplink UART
constexpr uint8_t UART1_TX_PIN = PB6;
constexpr uint8_t UART1_RX_PIN = PB7;

// Debug UART
constexpr uint8_t UART2_TX_PIN = PA2;
constexpr uint8_t UART2_RX_PIN = PA3;

// SBUS UART
constexpr uint8_t UART3_RX_PIN = PB11;

// ESCs
constexpr uint8_t MOTOR_RIGHT_ALM_PIN = PC10;
constexpr uint8_t MOTOR_RIGHT_EN_PIN = PC11;
constexpr uint8_t MOTOR_RIGHT_DIR_PIN = PB3;
constexpr TimerCfg MOTOR_RIGHT_FG = {PB4, TIM3, 1};
constexpr TimerCfg MOTOR_RIGHT_PWM = {PB5_ALT2, TIM17, 1};

constexpr uint8_t MOTOR_LEFT_ALM_PIN = PA9;
constexpr uint8_t MOTOR_LEFT_EN_PIN = PA10;
constexpr uint8_t MOTOR_LEFT_DIR_PIN = PA11;
constexpr TimerCfg MOTOR_LEFT_FG = {PA12_ALT2, TIM16, 1};
constexpr TimerCfg MOTOR_LEFT_PWM = {PA15, TIM2, 1};

// IMU
constexpr uint8_t BMI088_ACCEL_CS = PB1;
constexpr uint8_t BMI088_GYRO_CS = PB0;