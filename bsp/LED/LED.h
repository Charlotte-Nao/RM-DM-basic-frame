/**
 * @file LED.h
 * @brief RGB LED public interface.
 */

#ifndef DM_LED_H
#define DM_LED_H

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise SPI6 and turn the onboard WS2812 RGB LED off. */
void LED_init(void);

/** Turn the red channel on, off, or toggle it. */
void LED_RED_SET(void);
void LED_RED_RESET(void);
void LED_RED_TOGGLE(void);

/* Kept for compatibility with the requested (typoed) interface name. */
void LED_RDE_TOGGLE(void);

/** Turn the green channel on, off, or toggle it. */
void LED_GREEN_SET(void);
void LED_GREEN_RESET(void);
void LED_GREEN_TOGGLE(void);

/** Turn the blue channel on, off, or toggle it. */
void LED_BLUE_SET(void);
void LED_BLUE_RESET(void);
void LED_BLUE_TOGGLE(void);

/** Turn off all channels and release the SPI6 resources used by the LED. */
void LED_off(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_LED_H */
