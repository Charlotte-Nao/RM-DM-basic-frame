//
// Created by charlotte on 7/12/26.
//

#include "sensor_task.h"
#include "cmsis_os2.h"

#include "../../bsp/uart/uart.h"
#include "../../device/BMI088/BMI088driver.h"
#include "../../dsp/MahonyAHRS/MahonyAHRS.h"
#include "../../application/global_data.h"

#include <math.h>
#include <stdint.h>

#define SENSOR_SAMPLE_PERIOD_MS       1U
#define GYRO_CALIBRATION_SAMPLES      1000U

static void sensor_calibrate_gyro(float gyro_bias[3])
{
    float gyro[3];
    float accel[3];
    float temperature;
    uint32_t index;

    gyro_bias[0] = 0.0f;
    gyro_bias[1] = 0.0f;
    gyro_bias[2] = 0.0f;

    /* Keep the board still during this approximately one-second calibration. */
    for (index = 0U; index < GYRO_CALIBRATION_SAMPLES; ++index) {
        BMI088_read(gyro, accel, &temperature);
        gyro_bias[0] += gyro[0];
        gyro_bias[1] += gyro[1];
        gyro_bias[2] += gyro[2];
        osDelay(SENSOR_SAMPLE_PERIOD_MS);
    }

    gyro_bias[0] /= (float)GYRO_CALIBRATION_SAMPLES;
    gyro_bias[1] /= (float)GYRO_CALIBRATION_SAMPLES;
    gyro_bias[2] /= (float)GYRO_CALIBRATION_SAMPLES;
}

static void quaternion_to_euler(const float q[4], float *roll,
                                float *pitch, float *yaw)
{
    float sin_pitch;

    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                   1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));

    sin_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    *pitch = asinf(sin_pitch);

    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}


void sensor_task(void)
{
    struct uart_device *uart1;
    float gyro[3];
    float accel[3];
    float gyro_bias[3];
    float temperature;
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float roll;
    float pitch;
    float yaw;

    uint8_t bmi088_status;
    uart1 = uart_get_device("uart1_dma");
    bmi088_status = BMI088_init();
    if (bmi088_status != 0U) {
        for (;;) {
            if (uart1 != NULL) {
                (void)uart1->uart_printf(uart1, "BMI088 init error: %u\r\n",
                                         (unsigned int)bmi088_status);
            }
            osDelay(1000U);
        }
    }

    sensor_calibrate_gyro(gyro_bias);

    for (;;)
    {
        BMI088_read(gyro, accel, &temperature);

        /* BMI088 gyro output is degrees/s; Mahony requires radians/s. */
        MahonyAHRSupdateIMU(quaternion,
                            (gyro[0] - gyro_bias[0]) ,
                            (gyro[1] - gyro_bias[1]) ,
                            (gyro[2] - gyro_bias[2]) ,
                            accel[0], accel[1], accel[2]);

        /* Publish the newest attitude every sample for motor-control users. */
        quaternion_to_euler(quaternion, &roll, &pitch, &yaw);
        global_data.imu_roll_rad = pitch;
        global_data.imu_pitch_rad = roll;
        global_data.imu_yaw_rad = yaw;
        global_data.imu_update_tick = osKernelGetTickCount();
        global_data.imu_ready = 1U;

        osDelay(SENSOR_SAMPLE_PERIOD_MS);
    }

}
