//
// Created by charlotte on 7/12/26.
//

#include "test_task.h"
#include "cmsis_os2.h"
#include "../bsp/LED/LED.h"


void test_task(void)
{

    for (;;)
    {
        // LED_BLUE_RESET();
        //LED_GREEN_RESET();
        LED_SKY_SET();

        osDelay(1);
    }

}
