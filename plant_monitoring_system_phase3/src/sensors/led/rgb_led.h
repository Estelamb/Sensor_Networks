/**
 * @file rgb_led.h
 * @brief Interface for controlling an RGB LED using GPIO pins.
 *
 * This module provides functions to initialize and control an RGB LED
 * connected via GPIO. Each color channel (Red, Green, Blue) is represented
 * by a separate GPIO pin, allowing mixed color combinations through
 * bitmask-based operations.
 */

#ifndef RGB_LED_H
#define RGB_LED_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/** @brief Number of GPIO pins used for an RGB LED (R, G, B). */
#define BUS_SIZE 3

/**
 * @brief Structure representing an RGB LED connected through GPIO pins.
 *
 * Each LED color channel is mapped to a GPIO pin. The structure defines
 * the device tree pin specifications and the total number of pins used.
 */
struct bus_rgb_led {
    struct gpio_dt_spec pins[BUS_SIZE];  /**< GPIO pin specifications for R, G, B. */
    size_t pin_count;                    /**< Number of pins in use (should be 3). */
};

/**
 * @brief Initialize all GPIO pins used by the RGB LED.
 *
 * Verifies that each associated GPIO device is ready, then configures
 * all pins as outputs with an initial inactive (off) state.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If initialization succeeded.
 * @retval -ENODEV If a GPIO device is not ready.
 * @retval Other Negative error code from @ref gpio_pin_configure_dt.
 */
int rgb_led_init(struct bus_rgb_led *rgb_led);

/**
 * @brief Write a bitmask value to the RGB LED pins.
 *
 * Each bit controls one color channel:
 * - Bit 0 → Red
 * - Bit 1 → Green
 * - Bit 2 → Blue
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @param value Bitmask (0–7) defining active color channels.
 * @retval 0 If the operation succeeded.
 * @retval -EIO If a GPIO write failed.
 */
int rgb_led_write(struct bus_rgb_led *rgb_led, int value);

/**
 * @brief Turn on all RGB LED channels (white color).
 *
 * Equivalent to setting the bitmask to `0x7`.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_led_on(struct bus_rgb_led *rgb_led);

/**
 * @brief Turn off all RGB LED channels (black/off state).
 *
 * Equivalent to setting the bitmask to `0x0`.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_led_off(struct bus_rgb_led *rgb_led);

/** @brief Set LED color to red only. */
int rgb_red(struct bus_rgb_led *rgb_led);

/** @brief Set LED color to green only. */
int rgb_green(struct bus_rgb_led *rgb_led);

/** @brief Set LED color to blue only. */
int rgb_blue(struct bus_rgb_led *rgb_led);

/** @brief Set LED color to yellow (red + green). */
int rgb_yellow(struct bus_rgb_led *rgb_led);

/** @brief Set LED color to cyan (green + blue). */
int rgb_cyan(struct bus_rgb_led *rgb_led);

/** @brief Set LED color to purple (red + blue). */
int rgb_purple(struct bus_rgb_led *rgb_led);

/** @brief Set LED color to white (all channels on). */
int rgb_white(struct bus_rgb_led *rgb_led);

/** @brief Turn off all channels (black/off). */
int rgb_black(struct bus_rgb_led *rgb_led);

/**
 * @brief Apply one PWM step to the RGB LED.
 *
 * Each duty cycle is compared with the current time slice `t`
 * to determine whether to turn each color channel on or off.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @param r_value Value for the red channel (0 or 1).
 * @param g_value Value for the green channel (0 or 1).
 * @param b_value Value for the blue channel (0 or 1).
 */
void rgb_led_pwm_step(struct bus_rgb_led *rgb_led, int r_value, int g_value, int b_value);

#endif // RGB_LED_H
