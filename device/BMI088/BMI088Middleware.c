#include "BMI088Middleware.h"
#include "spi.h"


void BMI088_GPIO_init(void)
{
    /*
     * 两个片选都是低电平有效。
     * 初始化时先全部拉高，避免两个器件同时选中。
     */
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
}


void BMI088_com_init(void)
{
    /*
     * SPI2 已由 CubeMX 的 MX_SPI2_Init() 完成初始化。
     *
     * 此处开启 DWT 周期计数器，
     * 用于实现比较准确的微秒延时。
     */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


void BMI088_delay_ms(uint16_t ms)
{
    /*
     * 初始化阶段使用 HAL_Delay。
     *
     * 这样无论 BMI088_init() 是在 RTOS 启动前，
     * 还是在任务中调用，都可以工作。
     */
    HAL_Delay(ms);
}


void BMI088_delay_us(uint16_t us)
{
    uint32_t start;
    uint32_t delay_cycles;

    start = DWT->CYCCNT;

    delay_cycles =
        (SystemCoreClock / 1000000U) * (uint32_t)us;

    while ((uint32_t)(DWT->CYCCNT - start) < delay_cycles)
    {
        __NOP();
    }
}


uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata = 0xFFU;

    if (HAL_SPI_TransmitReceive(&hspi2,
                               &txdata,
                               &rxdata,
                               1U,
                               10U) != HAL_OK)
    {
        return 0xFFU;
    }

    return rxdata;
}