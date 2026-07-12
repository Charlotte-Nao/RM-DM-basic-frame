/**
 * @file LED.c
 * @brief DM-MC-Board02 onboard WS2812B RGB LED driver.
 *
 * The board manual connects LED DIN to SPI6_MOSI (PA7).  SPI6 runs at
 * 2.5 MHz (80 MHz / 32); each WS2812 bit is encoded as three SPI bits:
 * 0 -> 100, 1 -> 110.  This produces the required 0.4/0.8 us high times.
 */

#include "LED.h"

#include <stdbool.h>

#include "spi.h"

#define WS2812_CHANNEL_ON       129U
#define WS2812_ENCODED_BYTES    9U
#define WS2812_RESET_TIME_MS    1U

static bool led_initialized;
static uint8_t led_red;
static uint8_t led_green;
static uint8_t led_blue;

static void ws2812_encode_byte(uint8_t value, uint8_t *data, uint8_t *bit_offset)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; ++bit)
    {
        uint8_t symbol = (value & (uint8_t)(0x80U >> bit)) ? 0x06U : 0x04U;
        uint8_t symbol_bit;

        for (symbol_bit = 0U; symbol_bit < 3U; ++symbol_bit)
        {
            if ((symbol & (uint8_t)(0x04U >> symbol_bit)) != 0U)
            {
                data[*bit_offset / 8U] |= (uint8_t)(0x80U >> (*bit_offset % 8U));
            }
            ++(*bit_offset);
        }
    }
}

static void ws2812_show(void)
{
    uint8_t data[WS2812_ENCODED_BYTES] = {0};
    uint8_t bit_offset = 0U;

    if (!led_initialized)
    {
        return;
    }

    /* WS2812 transfers colour data in green, red, blue order. */
    ws2812_encode_byte(led_green, data, &bit_offset);
    ws2812_encode_byte(led_red, data, &bit_offset);
    ws2812_encode_byte(led_blue, data, &bit_offset);

    if (HAL_SPI_Transmit(&hspi6, data, sizeof(data), HAL_MAX_DELAY) == HAL_OK)
    {
        /* MOSI must remain low for at least 50 us to latch the new colour. */
        HAL_Delay(WS2812_RESET_TIME_MS);
    }
}

void LED_init(void)
{
    if (hspi6.State == HAL_SPI_STATE_RESET)
    {
        MX_SPI6_Init();
    }

    led_initialized = true;
    led_red = 0U;
    led_green = 0U;
    led_blue = 0U;
    ws2812_show();
}

void LED_RED_SET(void)
{
    led_red = WS2812_CHANNEL_ON ;
    led_green = 0U;
    led_blue = 0U;
    ws2812_show();
}

void LED_RED_RESET(void)
{
    led_red = 0U;
    ws2812_show();
}

void LED_RED_TOGGLE(void)
{
    led_red = (led_red == 0U) ? WS2812_CHANNEL_ON : 0U;
    ws2812_show();
}

void LED_RDE_TOGGLE(void)
{
    LED_RED_TOGGLE();
}

void LED_GREEN_SET(void)
{

    led_red = 0U;
    led_green = WS2812_CHANNEL_ON ;
    led_blue = 0U;

    ws2812_show();
}

void LED_GREEN_RESET(void)
{
    led_green = 0U;
    ws2812_show();
}

void LED_GREEN_TOGGLE(void)
{
    led_green = (led_green == 0U) ? WS2812_CHANNEL_ON : 0U;
    ws2812_show();
}

void LED_BLUE_SET(void)
{
    led_red = 0U;
    led_green = 0U;
    led_blue = WS2812_CHANNEL_ON ;

    ws2812_show();
}

void LED_BLUE_RESET(void)
{
    led_blue = 0U;
    ws2812_show();
}

void LED_BLUE_TOGGLE(void)
{
    led_blue = (led_blue == 0U) ? WS2812_CHANNEL_ON : 0U;
    ws2812_show();
}

void LED_off(void)
{
    led_red = 0U;
    led_green = 0U;
    led_blue = 0U;
    ws2812_show();
}
