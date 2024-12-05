#pragma once

namespace IMU
{
    struct IMUData
    {
        float accelX;
        float accelY;
        float accelZ;
        float gyroX;
        float gyroY;
        float gyroZ;
    };

    bool begin();

    void getIMUData(IMUData *data);
    void resetOffset();
} // namespace IMU
