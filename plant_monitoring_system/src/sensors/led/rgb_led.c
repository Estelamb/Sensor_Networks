/**
 * @file rgb_led.c
 * @brief Implementation of RGB LED control using GPIO pins.
 *
 * This module provides initialization and color-setting functions for
 * an RGB LED connected to three GPIO pins (Red, Green, Blue). It supports
 * individual color control and mixed combinations using bitmask-based writes.
 */

#include "rgb_led.h"
#include <zephyr/sys/printk.h>

/**
 * @brief Initialize all GPIO pins used by the RGB LED.
 *
 * Verifies that each GPIO device is ready and configures the associated pins
 * as outputs with an initial inactive (off) state.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If initialization succeeded.
 * @retval -ENODEV If a GPIO device is not ready.
 * @retval Other Negative error code from @ref gpio_pin_configure_dt.
 */
int rgb_led_init(struct bus_rgb_led *rgb_led) {
    printk("[RGB LED] - Initializing RGB LED...\n");

    for (size_t i = 0; i < rgb_led->pin_count; i++) {
        if (!device_is_ready(rgb_led->pins[i].port)) {
            printk("[RGB LED] - GPIO device not ready for pin %d\n", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&rgb_led->pins[i], GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            printk("[RGB LED] - Failed to configure output pin %d (code %d)\n", i, ret);
            return ret;
        }
    }

    printk("[RGB LED] - RGB LED initialized successfully\n");
    return 0;
}

/**
 * @brief Write a bitmask value to the RGB LED GPIO pins.
 *
 * Each bit in the input value corresponds to a color channel:
 * - Bit 0 → Red
 * - Bit 1 → Green
 * - Bit 2 → Blue
 *
 * Example:
 * - `0x1` → Red ON
 * - `0x3` → Yellow (Red + Green)
 * - `0x7` → White (all channels ON)
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @param value Bitmask (0–7) controlling the color combination.
 * @retval 0 If the operation succeeded.
 * @retval -EIO If a GPIO write failed.
 */
int rgb_led_write(struct bus_rgb_led *rgb_led, int value) {
    for (size_t i = 0; i < rgb_led->pin_count; i++) {
        int pin_value = (value >> i) & 0x1;
        int ret = gpio_pin_set_dt(&rgb_led->pins[i], pin_value);
        if (ret != 0) {
            printk("Error: Failed to set pin %d (code %d)\n", i, ret);
            return ret;
        }
    }
    return 0;
}

/**
 * @brief Turns on all RGB LED channels (white light).
 *
 * Equivalent to setting the bitmask to `0x7`.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_led_on(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x7); }

/**
 * @brief Turns off all RGB LED channels (black/off state).
 *
 * Equivalent to setting the bitmask to `0x0`.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_led_off(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x0); }

/**
 * @brief Sets the LED color to red only.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_red(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x1); }

/**
 * @brief Sets the LED color to green only.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_green(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x2); }

/**
 * @brief Sets the LED color to blue only.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_blue(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x4); }

/**
 * @brief Sets the LED color to yellow (red + green).
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_yellow(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x3); }

/**
 * @brief Sets the LED color to cyan (green + blue).
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_cyan(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x6); }

/**
 * @brief Sets the LED color to purple (red + blue).
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_purple(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x5); }

/**
 * @brief Sets the LED color to white (all channels on).
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_white(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x7); }

/**
 * @brief Turns off all LED channels (black/off state).
 *
 * Equivalent to setting the bitmask to `0x0`.
 *
 * @param rgb_led Pointer to the RGB LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int rgb_black(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x0); }

/**
 * @brief Apply one PWM step to the RGB LED.
 *
 * Each duty cycle is compared with the current time slice `t`
 * to determine if the corresponding channel must be ON or OFF.
 *
 * @param rgb_led Pointer to the RGB LED structure.
 * @param r_value Value for the red channel (0 or 1).
 * @param g_value Value for the green channel (0 or 1).
 * @param b_value Value for the blue channel (0 or 1).
 */
void rgb_led_pwm_step(struct bus_rgb_led *rgb_led, int r_value, int g_value, int b_value)
{
    gpio_pin_set_dt(&rgb_led->pins[0], r_value);
    gpio_pin_set_dt(&rgb_led->pins[1], g_value);
    gpio_pin_set_dt(&rgb_led->pins[2], b_value);
}