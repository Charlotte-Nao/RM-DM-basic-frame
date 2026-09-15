/**
* @file test_task.c
 * @brief just test
 */

#include "test_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "../../device/motor/motor.h"
#include "../../dsp/math.h"
#include "../../application/global_data.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/pwm/pwm.h"
#include "../../bsp/usb/usb.h"

void test_task(void)
{
    struct motor_device *motor = motor_get_device("M3508_1");
    const float zero = 0.0f;
    const float target_rpm = 300.0f;

    osDelay(2000);
    motor->send_enable_cmd(motor);

    while (!motor_is_online(motor))
    {
        LED_PINK_SET();
        osDelay(1);
    }

    /* 关闭位置环，使用速度环控制转子转速。 */
    motor->set_para(motor, "POS_KP", &zero);
    motor->set_para(motor, "POS_KI", &zero);
    motor->set_para(motor, "POS_KD", &zero);


    for (;;) {
        motor->set_target(motor, 2, 0.0f, target_rpm);
        osDelay(1);
    }
}
