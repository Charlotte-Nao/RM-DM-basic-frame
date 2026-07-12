#include "BMI088Middleware.h"
#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void) {
    // CubeMX 已经生成，此处可留空或进行特定初始化
}

void BMI088_com_init(void) {
    // SPI 初始化已在 main.c 完成
}

void BMI088_delay_ms(uint16_t ms) {
    vTaskDelay(ms);
}

void BMI088_delay_us(uint16_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    uint32_t tcnt = 0;
    uint32_t told = SysTick->VAL;
    uint32_t tnow;
    uint32_t reload = SysTick->LOAD;

    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) tcnt += told - tnow;
            else tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks) break;
        }
    }
}

uint8_t BMI088_read_write_byte(uint8_t txdata) {
    uint8_t rx_data;
    // 使用超时机制，防止 SPI 硬件故障导致死机
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 10);
    return rx_data;
}