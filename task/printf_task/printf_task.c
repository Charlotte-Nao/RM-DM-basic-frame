//
// Created by charlotte on 7/14/26.
//

#include "printf_task.h"
#include "cmsis_os2.h"
#include "../../device/motor/motor.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../bsp/uart/uart.h"
#include "../../application/global_data.h"
#include "../../dsp/calculation/calculation.h"

#define RAD_TO_DEG                    57.295779513082320876f
#define DEG_TO_RAD                    0.01745329251994329577f


static float yb_sd15m_position_to_angle_float(uint16_t target_position)
{
    float position_range;
    float angle_range;

    if (target_position < YB_SD15M_POSITION_MIN) {
        target_position = YB_SD15M_POSITION_MIN;
    } else if (target_position > YB_SD15M_POSITION_MAX) {
        target_position = YB_SD15M_POSITION_MAX;
    }

    position_range = (float)(YB_SD15M_POSITION_MAX - YB_SD15M_POSITION_MIN);
    angle_range = (float)(YB_SD15M_ANGLE_MAX - YB_SD15M_ANGLE_MIN);

    return (float)YB_SD15M_ANGLE_MIN
         + ((float)(target_position - YB_SD15M_POSITION_MIN) * angle_range)
         / position_range;
}

void printf_task(void)
{
    struct motor_device *gm6020_pitch = motor_get_device("GM6020_PITCH");
    struct motor_device *gm6020_yaw = motor_get_device("GM6020_YAW");
    struct yb_sd15m_device *servo_1 = yb_sd15m_get_device("YB_SD15M_1");
    struct yb_sd15m_device *servo_2 = yb_sd15m_get_device("YB_SD15M_2");
    struct yb_sd15m_device *servo_3 = yb_sd15m_get_device("YB_SD15M_3");
    struct yb_sd15m_device *servo_4 = yb_sd15m_get_device("YB_SD15M_4");
    struct uart_device *uart1 = uart_get_device("uart1_dma");

    if (uart1 == NULL ) return;

    float position_rad = 0.0f;
    float velocity_rpm = 0.0f;
    uint8_t temperature = 0U;
    uint16_t encoder = 0U;
    int16_t servo_target[FOUR_AXIS_SERVO_COUNT] = {0};
    float servo_angle[FOUR_AXIS_SERVO_COUNT] = {0.0f};
    float servo_pose[FOUR_AXIS_TARGET_POSE_SIZE] = {0.0f};

    for (;;)
    {
        // if (gm6020_yaw == NULL) {(void)uart1->uart_printf(uart1,"GM6020_yaw未连接");}
        // else
        // {
        //     gm6020_yaw->get_status(gm6020_yaw, "POS", &position_rad);
        //     gm6020_yaw->get_status(gm6020_yaw, "VEL", &velocity_rpm);
        //     gm6020_yaw->get_status(gm6020_yaw, "TEMP", &temperature);
        //     gm6020_yaw->get_status(gm6020_yaw, "ENC", &encoder);
        //
        //     (void)uart1->uart_printf(uart1,
        //     "GM6020_yaw: online=%u "
        //     "pos=%.3f "
        //     "rad vel=%.1f rpm "
        //     "temp=%u  "
        //     "encoder=%u\r\n",
        //     motor_is_online(gm6020_yaw) ? 1U : 0U,
        //     position_rad,
        //     velocity_rpm,
        //     (unsigned int)temperature,
        //     (unsigned int)encoder
        //     );
        // }
        //
        // osDelay(200U);
        //
        // if (gm6020_pitch == NULL) {(void)uart1->uart_printf(uart1,"GM6020_pitch未连接");}
        // else
        // {
        //     gm6020_pitch->get_status(gm6020_pitch, "POS", &position_rad);
        //     gm6020_pitch->get_status(gm6020_pitch, "VEL", &velocity_rpm);
        //     gm6020_pitch->get_status(gm6020_pitch, "TEMP", &temperature);
        //     gm6020_pitch->get_status(gm6020_pitch, "ENC", &encoder);
        //
        //     (void)uart1->uart_printf(uart1,
        //              "GM6020_pitch: online=%u "
        //              "pos=%.3f "
        //              "rad vel=%.1f rpm "
        //              "temp=%u  "
        //              "encoder=%u\r\n",
        //              motor_is_online(gm6020_yaw) ? 1U : 0U,
        //              position_rad, velocity_rpm,
        //              (unsigned int)temperature,
        //              (unsigned int)encoder);
        //
        // }
        //
        // osDelay(200U);
        //
        // (void)uart1->uart_printf(uart1,
        // "roll:%.2f,pitch:%.2f,yaw:%.2f,temp:%u\r\n",
        // global_data.imu_roll_rad * RAD_TO_DEG,
        // global_data.imu_pitch_rad * RAD_TO_DEG,
        // global_data.imu_yaw_rad * RAD_TO_DEG,
        // temperature);
        //
        // osDelay(200U);

        if (servo_1 == NULL) {(void)uart1->uart_printf(uart1,"YB_SD15M_1未连接");}
        if (servo_2 == NULL) {(void)uart1->uart_printf(uart1,"YB_SD15M_2未连接");}
        if (servo_3 == NULL) {(void)uart1->uart_printf(uart1,"YB_SD15M_3未连接");}
        if (servo_4 == NULL) {(void)uart1->uart_printf(uart1,"YB_SD15M_4未连接");}
        if (servo_1 != NULL && servo_2 != NULL && servo_3 != NULL && servo_4 != NULL)
        {
            servo_1->get_status(servo_1, "TARGET_ANGLE", &servo_target[0]);
            servo_2->get_status(servo_2, "TARGET_ANGLE", &servo_target[1]);
            servo_3->get_status(servo_3, "TARGET_ANGLE", &servo_target[2]);
            servo_4->get_status(servo_4, "TARGET_ANGLE", &servo_target[3]);

            Four_degree_of_freedom_positive_calculation(&arm, servo_target, servo_pose);

            (void)uart1->uart_printf(uart1,
            "YB_SD15M: angle1=%d angle2=%d angle3=%d angle4=%d "
            "x=%.2f y=%.2f z=%.2f phi=%.2f\r\n",
            servo_target[0],
            servo_target[1],
            servo_target[2],
            servo_target[3],
            servo_pose[0],
            servo_pose[1],
            servo_pose[2],
            servo_pose[3]
            );
        }

        osDelay(200U);
    }
}
