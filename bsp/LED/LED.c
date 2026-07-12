/**
 * @file LED.c
 * @brief DM-MC-Board02 onboard WS2812B RGB LED driver by GPIO bit-bang.
 *
 * Board connection:
 *   WS2812 DIN -> PA7
 *
 * This version does not use SPI6.
 * It directly drives PA7 as GPIO output.
 */

#include "LED.h"

#include <stdbool.h>

#include "main.h"

#define WS2812_PORT        GPIOA
#define WS2812_PIN         GPIO_PIN_7

#define WS2812_DEFAULT_ON  255U

/*
 * WS2812 typical timing:
 *
 * Bit total : about 1.25 us
 * 0 high    : about 0.35 us
 * 1 high    : about 0.70 us
 *
 * We use DWT cycle counter to generate timing.
 */
#define WS2812_T0H_NS      350U
#define WS2812_T1H_NS      700U
#define WS2812_BIT_NS      1250U

static bool led_initialized = false;

static uint8_t led_red = 0U;
static uint8_t led_green = 0U;
static uint8_t led_blue = 0U;

static uint32_t ws2812_t0h_cycles = 0U;
static uint32_t ws2812_t1h_cycles = 0U;
static uint32_t ws2812_bit_cycles = 0U;

static uint32_t ns_to_cycles(uint32_t ns)
{
    return (uint32_t)(((uint64_t)SystemCoreClock * ns) / 1000000000ULL);
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void ws2812_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = WS2812_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(WS2812_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_RESET);
}

static inline void ws2812_set_high(void)
{
    WS2812_PORT->BSRR = WS2812_PIN;
}

static inline void ws2812_set_low(void)
{
    WS2812_PORT->BSRR = ((uint32_t)WS2812_PIN << 16U);
}

static inline void ws2812_wait_until(uint32_t target)
{
    while ((int32_t)(DWT->CYCCNT - target) < 0)
    {
    }
}

static void ws2812_send_bit(uint8_t bit_value)
{
    uint32_t start;
    uint32_t high_cycles;

    high_cycles = (bit_value != 0U) ?
                  ws2812_t1h_cycles :
                  ws2812_t0h_cycles;

    start = DWT->CYCCNT;

    ws2812_set_high();
    ws2812_wait_until(start + high_cycles);

    ws2812_set_low();
    ws2812_wait_until(start + ws2812_bit_cycles);
}

static void ws2812_send_byte(uint8_t value)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        ws2812_send_bit((value & mask) ? 1U : 0U);
    }
}

static void ws2812_show(void)
{
    uint32_t primask;

    if (!led_initialized)
    {
        return;
    }

    /*
     * WS2812 一帧时间很短，发送期间关闭中断，避免 FreeRTOS 或 SysTick
     * 打断时序。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    /*
     * 标准 WS2812 顺序一般是：
     *
     *   Green -> Red -> Blue
     *
     * 如果后面发现红绿蓝顺序不对，只需要调换下面三行。
     */
    ws2812_send_byte(led_green);
    ws2812_send_byte(led_red);
    ws2812_send_byte(led_blue);

    __set_PRIMASK(primask);

    /*
     * WS2812 需要低电平复位/锁存。
     */
    ws2812_set_low();
    HAL_Delay(1U);
}

void LED_init(void)
{
    dwt_init();

    ws2812_t0h_cycles = ns_to_cycles(WS2812_T0H_NS);
    ws2812_t1h_cycles = ns_to_cycles(WS2812_T1H_NS);
    ws2812_bit_cycles = ns_to_cycles(WS2812_BIT_NS);

    ws2812_gpio_init();

    led_initialized = true;

    led_red = 0U;
    led_green = 0U;
    led_blue = 0U;

    /*
     * 连续发送两次黑色，清掉 WS2812 之前锁存的颜色。
     */
    ws2812_show();
    ws2812_show();
}

void LED_SET_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!led_initialized)
    {
        LED_init();
    }

    led_red = red;
    led_green = green;
    led_blue = blue;

    ws2812_show();
}

void LED_RED_SET(void)
{
    LED_SET_RGB(WS2812_DEFAULT_ON, 0U, 0U);
}

void LED_RED_RESET(void)
{
    LED_SET_RGB(0U, 0U, 0U);
}

void LED_RED_TOGGLE(void)
{
    if ((led_red != 0U) && (led_green == 0U) && (led_blue == 0U))
    {
        LED_off();
    }
    else
    {
        LED_RED_SET();
    }
}

void LED_RDE_TOGGLE(void)
{
    LED_RED_TOGGLE();
}

void LED_GREEN_SET(void)
{
    LED_SET_RGB(0U, WS2812_DEFAULT_ON, 0U);
}

void LED_GREEN_RESET(void)
{
    LED_SET_RGB(0U, 0U, 0U);
}

void LED_GREEN_TOGGLE(void)
{
    if ((led_red == 0U) && (led_green != 0U) && (led_blue == 0U))
    {
        LED_off();
    }
    else
    {
        LED_GREEN_SET();
    }
}

void LED_BLUE_SET(void)
{
    LED_SET_RGB(0U, 0U, WS2812_DEFAULT_ON);
}

void LED_BLUE_RESET(void)
{
    LED_SET_RGB(0U, 0U, 0U);
}

void LED_BLUE_TOGGLE(void)
{
    if ((led_red == 0U) && (led_green == 0U) && (led_blue != 0U))
    {
        LED_off();
    }
    else
    {
        LED_BLUE_SET();
    }
}

void LED_SKY_SET(void)
{
    LED_SET_RGB(0U, WS2812_DEFAULT_ON / 2, WS2812_DEFAULT_ON);
}

void LED_SKY_RESET_TOGGLE(void)
{
    if ((led_red == 0U) && (led_green == WS2812_DEFAULT_ON / 2) && (led_blue == WS2812_DEFAULT_ON))
    {
        LED_off();
    }
    else
        LED_SKY_SET();
}

void LED_off(void)
{
    LED_SET_RGB(0U, 0U, 0U);
}