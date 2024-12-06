#include <BMI088.h>
#include <ulog.h>
#include <STM32FreeRTOS.h>
#include <timers.h>
#include <arm_math.h>

#include "IMU.h"
#include "config.h"

namespace IMU
{
    // `SPI` stands for SPI1 by default
    static Bmi088Accel accel(SPI, BMI088_ACCEL_CS);
    static Bmi088Gyro gyro(SPI, BMI088_GYRO_CS);

    constexpr uint8_t AVR_SAMPLES_COUNT = 8;
    constexpr uint8_t SAMPLE_PERIOD = 20; // Fuck IMU every 20ms
    IMUData collectedData[AVR_SAMPLES_COUNT]{IMUData{0}};

    IMUData imuOffset;
    void collectData(TimerHandle_t)
    {
        if (!accel.getDrdyStatus())
        {
            ULOG_WARNING("Accel data not ready");
            return;
        }
        if (!gyro.getDrdyStatus())
        {
            ULOG_WARNING("Gyro data not ready");
            return;
        }

        accel.readSensor();
        gyro.readSensor();

        for (uint8_t i = 0; i < AVR_SAMPLES_COUNT - 1; i++)
        {
            collectedData[i] = collectedData[i + 1];
        }
        collectedData[AVR_SAMPLES_COUNT - 1] = IMUData{
            accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss(),
            gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads()};
    }

    bool begin()
    {
        if (accel.begin() < 0)
        {
            ULOG_ERROR("Accel initialization failed");
            return false;
        }
        accel.setRange(Bmi088Accel::RANGE_6G);
        accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ);
        ULOG_INFO("Accel initialized");

        if (gyro.begin() < 0)
        {
            ULOG_ERROR("Gyro initialization failed");
            return false;
        }
        gyro.setRange(Bmi088Gyro::RANGE_500DPS);
        gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
        ULOG_INFO("Gyro initialized");

        static auto collectTimer = xTimerCreate("Read IMU", pdMS_TO_TICKS(SAMPLE_PERIOD), pdTRUE, (void *)114514, collectData);
        xTimerStart(collectTimer, 0);
        ULOG_INFO("IMU collection started");

        return true;
    }

    void getIMUDataRaw(IMUData *data)
    {
        float32_t temp[6]{0};
        for (auto &p : collectedData)
        {
            arm_add_f32(temp, reinterpret_cast<float32_t *>(&p), temp, 6);
        }
        arm_scale_f32(temp, 1.0f / AVR_SAMPLES_COUNT, reinterpret_cast<float32_t *>(data), 6);
    }

    void resetOffset()
    {
        getIMUDataRaw(&imuOffset);
        imuOffset.accelZ += 9.81f;
    }

    void getIMUData(IMUData *data)
    {
        IMUData raw;
        getIMUDataRaw(&raw);
        arm_sub_f32(reinterpret_cast<float32_t *>(&raw), reinterpret_cast<float32_t *>(&imuOffset), reinterpret_cast<float32_t *>(data), 6);
    }

} // namespace IMU