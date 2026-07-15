/**
 * @file test_task.c
 * @brief GM6020 CAN1 position--velocity closed-loop test.
 */

#include "../test_task/test_task.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/uart/uart.h"
#include "cmsis_os2.h"

#include "fdcan.h"
#include "../../application/global_data.h"
#include "../../device/motor/motor.h"


void test_task(void)
{

    for (;;) {

        osDelay(1U);
    }
}
