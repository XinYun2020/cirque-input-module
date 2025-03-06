/**
 * @file drivers/input/cirque_glidepoint.h
 *
 * @brief Public APIs for the Cirque GlidePoint touchpad driver
 *
 * Copyright (c) 2023 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

 #ifndef ZEPHYR_INCLUDE_DRIVERS_INPUT_CIRQUE_GLIDEPOINT_H_
 #define ZEPHYR_INCLUDE_DRIVERS_INPUT_CIRQUE_GLIDEPOINT_H_
 
 #include <zephyr/device.h>
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Set the acceleration profile for the touchpad
  *
  * @param dev Pointer to the device structure for the driver instance
  * @param profile Acceleration profile (0: none, 1: linear, 2: sigmoid)
  * @param factor Acceleration factor (1.0 - 5.0, where 1.0 is minimal acceleration)
  *
  * @retval 0 If successful
  * @retval -EINVAL If the profile or factor is invalid
  */
 int cirque_glidepoint_set_accel_profile(const struct device *dev, uint8_t profile, float factor);
 
 /**
  * @brief Set the scroll sensitivity for circular scrolling
  *
  * @param dev Pointer to the device structure for the driver instance
  * @param sensitivity Scroll sensitivity (0.1 - 10.0, where 1.0 is default)
  *
  * @retval 0 If successful
  * @retval -EINVAL If the sensitivity is invalid
  */
 int cirque_glidepoint_set_scroll_sensitivity(const struct device *dev, float sensitivity);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* ZEPHYR_INCLUDE_DRIVERS_INPUT_CIRQUE_GLIDEPOINT_H_ */