/**
 * @file
 * @brief Driver for Cirque GlidePoint touchpad
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <zephyr/device.h>
 #include <zephyr/drivers/i2c.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/input/input.h>
 #include <zephyr/kernel.h>
 #include <zephyr/sys/byteorder.h>
 #include <zephyr/logging/log.h>
 #include <math.h>
 #include "cirque_glidepoint.h"
 
 LOG_MODULE_REGISTER(cirque_glidepoint, CONFIG_CIRQUE_GLIDEPOINT_LOG_LEVEL);
 
 /* Cirque GlidePoint Register addresses */
 #define REG_FIRMWARE_ID       0x00
 #define REG_STATUS            0x02
 #define REG_COMMAND           0x04
 #define REG_REPORT_MODE       0x05
 #define REG_REPORT_DATA       0x10
 #define REG_FEED_CONFIG       0x21
 #define REG_CALIBRATION       0x22
 
 /* Commands */
 #define CMD_SOFT_RESET        0x01
 #define CMD_REPORT_ENABLE     0x02
 #define CMD_REPORT_DISABLE    0x03
 #define CMD_SET_REPORT_MODE   0x04
 
 /* Report modes */
 #define REPORT_MODE_ABSOLUTE  0x01
 #define REPORT_MODE_RELATIVE  0x02
 
 /* Status bits */
 #define STATUS_DATA_READY     BIT(0)
 
 /* Constants for acceleration */
 #define SIGMOID_MIDPOINT      20.0f  // Midpoint for sigmoid function
 #define SIGMOID_STEEPNESS     0.1f   // Steepness control for sigmoid curve
 #define MIN_ACCEL_THRESHOLD   5      // Minimum movement to apply acceleration
 #define MAX_ACCEL_FACTOR      5.0f   // Maximum acceleration multiplier
 
 /* Constants for circular scrolling */
 #define ANGLE_HISTORY_SIZE    3      // Size of angle history buffer for smoothing
 #define MIN_SCROLL_THRESHOLD  0.05f  // Minimum angle change to trigger scroll
 #define MAX_SCROLL_AMOUNT     10     // Maximum scroll amount to prevent jumps
 
// Change in the struct
struct cirque_glidepoint_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec dr; // FIXME
    // struct gpio_dt_spec dr_gpio;  // Changed from dr
#ifdef CONFIG_CIRQUE_GLIDEPOINT_CIRCULAR_SCROLL
    uint8_t circular_scroll_radius;
    uint8_t circular_scroll_deadzone;
#endif
};

 
 struct cirque_glidepoint_data {
     const struct device *dev;
     struct gpio_callback dr_cb;
     struct k_work work;
     int16_t prev_x;
     int16_t prev_y;
     bool    contact;
     bool    scroll_mode;
     float   accel_factor;
     float   scroll_sensitivity;
     
     // Acceleration parameters
     uint8_t accel_profile; // 0: none, 1: linear, 2: sigmoid
     float   accel_history[3]; // Movement history for smoothing
     uint8_t history_idx;
     
 #ifdef CONFIG_CIRQUE_GLIDEPOINT_CIRCULAR_SCROLL
     int16_t center_x;
     int16_t center_y;
     float   prev_angle;
     float   angle_history[ANGLE_HISTORY_SIZE];
     uint8_t angle_idx;
     bool    circular_scroll_active;
     uint8_t scroll_accumulator;  // For fractional scroll accumulation
 #endif
 };
 
 // Improved sigmoid function for acceleration
 static float apply_sigmoid_accel(float magnitude, float accel_factor) {
     if (magnitude < MIN_ACCEL_THRESHOLD) {
         return 1.0f; // No acceleration for very small movements
     }
     
     // Apply sigmoid function: 2/(1+e^(-x*steepness)) - 1
     // This produces values from 0 to 1 as x increases
     float sigmoid = 2.0f / (1.0f + expf(-(magnitude - SIGMOID_MIDPOINT) * SIGMOID_STEEPNESS)) - 1.0f;
     
     // Clamp sigmoid result between 0 and 1
     sigmoid = sigmoid < 0.0f ? 0.0f : (sigmoid > 1.0f ? 1.0f : sigmoid);
     
     // Scale by accel_factor (user configurable) and add 1 (so minimum is 1.0)
     float result = 1.0f + (sigmoid * (accel_factor - 1.0f));
     
     // Ensure we don't exceed maximum acceleration
     return result > MAX_ACCEL_FACTOR ? MAX_ACCEL_FACTOR : result;
 }
 
 static void cirque_glidepoint_work_handler(struct k_work *work)
 {
     struct cirque_glidepoint_data *data = CONTAINER_OF(work, struct cirque_glidepoint_data, work);
     const struct device *dev = data->dev;
     const struct cirque_glidepoint_config *config = dev->config;
     uint8_t status;
     uint8_t report[8];
     
     // Read status register
     if (i2c_reg_read_byte_dt(&config->i2c, REG_STATUS, &status) < 0) {
         LOG_ERR("Failed to read status register");
         return;
     }
     
     // Check if data is ready
     if (!(status & STATUS_DATA_READY)) {
         return;
     }
     
     // Read report data
     if (i2c_burst_read_dt(&config->i2c, REG_REPORT_DATA, report, sizeof(report)) < 0) {
         LOG_ERR("Failed to read report data");
         return;
     }
     
     bool contact = (report[0] & BIT(0));
     int16_t x = (report[2] << 8) | report[1];
     int16_t y = (report[4] << 8) | report[3];
     
     if (contact) {
         if (!data->contact) {
             // First touch, initialize tracking
             data->contact = true;
             data->prev_x = x;
             data->prev_y = y;
             
 #ifdef CONFIG_CIRQUE_GLIDEPOINT_CIRCULAR_SCROLL
             // Store center position for circular scrolling
             data->center_x = x;
             data->center_y = y;
             data->circular_scroll_active = false;
             
             // Initialize angle history
             for (int i = 0; i < ANGLE_HISTORY_SIZE; i++) {
                 data->angle_history[i] = 0.0f;
             }
             data->angle_idx = 0;
 #endif
             return;
         }
         
         int16_t dx = x - data->prev_x;
         int16_t dy = y - data->prev_y;
         
 #ifdef CONFIG_CIRQUE_GLIDEPOINT_CIRCULAR_SCROLL
         // Check for circular scrolling
         int16_t dist_from_center_x = x - data->center_x;
         int16_t dist_from_center_y = y - data->center_y;
         uint16_t distance = sqrt(dist_from_center_x * dist_from_center_x +
                                 dist_from_center_y * dist_from_center_y);
         
         // Only activate circular scroll if beyond radius and outside deadzone
         if (distance > config->circular_scroll_radius && 
             distance > config->circular_scroll_deadzone) {
             
             // Calculate angle in radians
             float angle = atan2f(dist_from_center_y, dist_from_center_x);
             
             if (!data->circular_scroll_active) {
                 data->circular_scroll_active = true;
                 data->prev_angle = angle;
                 
                 // Initialize angle history with current angle
                 for (int i = 0; i < ANGLE_HISTORY_SIZE; i++) {
                     data->angle_history[i] = angle;
                 }
             } else {
                 // Store angle in history buffer
                 data->angle_history[data->angle_idx] = angle;
                 data->angle_idx = (data->angle_idx + 1) % ANGLE_HISTORY_SIZE;
                 
                 // Calculate average angle change for smoothing
                 float angle_diff = angle - data->prev_angle;
                 
                 // Handle wrap-around
                 if (angle_diff > M_PI) {
                     angle_diff -= 2 * M_PI;
                 } else if (angle_diff < -M_PI) {
                     angle_diff += 2 * M_PI;
                 }
                 
                 // Only scroll if angle change exceeds threshold
                 if (fabsf(angle_diff) > MIN_SCROLL_THRESHOLD) {
                     // Convert angle change to scroll amount
                     float scroll_float = angle_diff * 100.0f * data->scroll_sensitivity;
                     
                     // Apply a small amount of acceleration to circular scrolling
                     float scroll_magnitude = fabsf(scroll_float);
                     if (scroll_magnitude > 1.0f) {
                         float accel = 1.0f + (scroll_magnitude / 20.0f);
                         scroll_float *= accel > 2.0f ? 2.0f : accel;
                     }
                     
                     // Add to accumulator (for fractional scrolling)
                     data->scroll_accumulator += (int8_t)scroll_float;
                     
                     // Report scroll when accumulator exceeds threshold
                     int16_t scroll_amount = data->scroll_accumulator;
                     if (scroll_amount != 0) {
                         // Limit maximum scroll amount to prevent jumps
                         if (scroll_amount > MAX_SCROLL_AMOUNT) {
                             scroll_amount = MAX_SCROLL_AMOUNT;
                         } else if (scroll_amount < -MAX_SCROLL_AMOUNT) {
                             scroll_amount = -MAX_SCROLL_AMOUNT;
                         }
                         
                         input_report_rel(dev, INPUT_REL_WHEEL, scroll_amount, true);
                         data->scroll_accumulator = 0;
                     }
                 }
                 
                 data->prev_angle = angle;
             }
         } else {
             data->circular_scroll_active = false;
 #endif
             
 #ifdef CONFIG_CIRQUE_GLIDEPOINT_SIGMOID_ACCEL
             // Apply sigmoid acceleration if not in circular scroll mode
             if (!data->circular_scroll_active && data->accel_profile == 2) {
                 float dx_f = (float)dx;
                 float dy_f = (float)dy;
                 
                 // Calculate magnitude of movement
                 float magnitude = sqrtf(dx_f * dx_f + dy_f * dy_f);
                 
                 // Store in history buffer for potential smoothing
                 data->accel_history[data->history_idx] = magnitude;
                 data->history_idx = (data->history_idx + 1) % 3;
                 
                 // Apply sigmoid acceleration
                 float accel_factor = apply_sigmoid_accel(magnitude, data->accel_factor);
                 
                 // Apply acceleration
                 dx = (int16_t)(dx_f * accel_factor);
                 dy = (int16_t)(dy_f * accel_factor);
             } else if (!data->circular_scroll_active && data->accel_profile == 1) {
                 // Linear acceleration as a fallback option
                 float dx_f = (float)dx;
                 float dy_f = (float)dy;
                 float magnitude = sqrtf(dx_f * dx_f + dy_f * dy_f);
                 
                 if (magnitude > MIN_ACCEL_THRESHOLD) {
                     float accel_factor = 1.0f + (magnitude / 100.0f) * data->accel_factor;
                     accel_factor = accel_factor > MAX_ACCEL_FACTOR ? MAX_ACCEL_FACTOR : accel_factor;
                     
                     dx = (int16_t)(dx_f * accel_factor);
                     dy = (int16_t)(dy_f * accel_factor);
                 }
             }
 #endif
             
             if (!data->scroll_mode && !data->circular_scroll_active) {
                 if (dx != 0) {
                     input_report_rel(dev, INPUT_REL_X, dx, false);
                 }
                 if (dy != 0) {
                     input_report_rel(dev, INPUT_REL_Y, dy, false);
                 }
             }
             
 #ifdef CONFIG_CIRQUE_GLIDEPOINT_CIRCULAR_SCROLL
         }
 #endif
         
         data->prev_x = x;
         data->prev_y = y;
     } else if (data->contact) {
         // Touch released
         data->contact = false;
         data->scroll_mode = false;
 #ifdef CONFIG_CIRQUE_GLIDEPOINT_CIRCULAR_SCROLL
         data->circular_scroll_active = false;
         data->scroll_accumulator = 0;
 #endif
     }
 }
 
static void cirque_glidepoint_dr_callback(const struct device *port,
        struct gpio_callback *cb,
        gpio_port_pins_t pins)
    {
    struct cirque_glidepoint_data *data = CONTAINER_OF(cb, struct cirque_glidepoint_data, dr_cb);
    k_work_submit(&data->work);
    }
 
 // Set acceleration profile
 int cirque_glidepoint_set_accel_profile(const struct device *dev, uint8_t profile, float factor)
 {
     struct cirque_glidepoint_data *data = dev->data;
     
     if (profile > 2) {
         return -EINVAL;
     }
     
     data->accel_profile = profile;
     data->accel_factor = factor;
     
     return 0;
 }
 
 // Set scroll sensitivity
 int cirque_glidepoint_set_scroll_sensitivity(const struct device *dev, float sensitivity)
 {
     struct cirque_glidepoint_data *data = dev->data;
     
     if (sensitivity < 0.1f || sensitivity > 10.0f) {
         return -EINVAL;
     }
     
     data->scroll_sensitivity = sensitivity;
     
     return 0;
 }
 
 static int cirque_glidepoint_init(const struct device *dev)
 {
     const struct cirque_glidepoint_config *config = dev->config;
     struct cirque_glidepoint_data *data = dev->data;
     uint8_t firmware_id[2];
     int ret;
     
     data->dev = dev;
     data->contact = false;
     data->scroll_mode = false;
     data->accel_factor = 2.0f;  // Default acceleration factor
     data->scroll_sensitivity = 1.0f;  // Default scroll sensitivity
     data->accel_profile = 2;  // Default to sigmoid acceleration
     data->history_idx = 0;
     
 #ifdef CONFIG_CIRQUE_GLIDEPOINT_CIRCULAR_SCROLL
     data->circular_scroll_active = false;
     data->angle_idx = 0;
     data->scroll_accumulator = 0;
 #endif
     
     // Initialize I2C
     if (!device_is_ready(config->i2c.bus)) {
         LOG_ERR("I2C bus not ready");
         return -ENODEV;
     }
     
     // Initialize DR GPIO
     if (!device_is_ready(config->dr.port)) {
         LOG_ERR("IRQ GPIO not ready");
         return -ENODEV;
     }
     
     // Read firmware ID
     ret = i2c_burst_read_dt(&config->i2c, REG_FIRMWARE_ID, firmware_id, sizeof(firmware_id));
     if (ret < 0) {
         LOG_ERR("Failed to read firmware ID");
         return ret;
     }
     
     LOG_INF("Cirque GlidePoint firmware ID: %02x %02x", firmware_id[0], firmware_id[1]);
     
     // Soft reset
     ret = i2c_reg_write_byte_dt(&config->i2c, REG_COMMAND, CMD_SOFT_RESET);
     if (ret < 0) {
         LOG_ERR("Failed to reset device");
         return ret;
     }
     
     // Wait for reset to complete
     k_sleep(K_MSEC(50));
     
     // Set report mode to absolute
     ret = i2c_reg_write_byte_dt(&config->i2c, REG_REPORT_MODE, REPORT_MODE_ABSOLUTE);
     if (ret < 0) {
         LOG_ERR("Failed to set report mode");
         return ret;
     }
     
     // Enable reporting
     ret = i2c_reg_write_byte_dt(&config->i2c, REG_COMMAND, CMD_REPORT_ENABLE);
     if (ret < 0) {
         LOG_ERR("Failed to enable reporting");
         return ret;
     }
     
     // Initialize work queue
     k_work_init(&data->work, cirque_glidepoint_work_handler);
     
     // Configure and enable IRQ
     ret = gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
     if (ret < 0) {
         LOG_ERR("Failed to configure DR pin");
         return ret;
     }
     
     gpio_init_callback(&data->dr_cb, cirque_glidepoint_dr_callback,
                       BIT(config->dr.pin));
                       
     ret = gpio_add_callback(config->dr.port, &data->dr_cb);
     if (ret < 0) {
         LOG_ERR("Failed to add DR callback");
         return ret;
     }
     
     ret = gpio_pin_interrupt_configure_dt(&config->dr, GPIO_INT_EDGE_FALLING);
     if (ret < 0) {
         LOG_ERR("Failed to configure DR interrupt");
         return ret;
     }
     
     return 0;
 }
 
 #define CIRQUE_GLIDEPOINT_INIT(inst)                                                    \
    static const struct cirque_glidepoint_config cirque_glidepoint_config_##inst = {    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                           \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(inst), dr_gpios, {}),                 \
        IF_ENABLED(CONFIG_CIRQUE_GLIDEPOINT_CIRCULAR_SCROLL, (                        \
            .circular_scroll_radius = DT_INST_PROP_OR(inst, circular_scroll_radius, 35), \
            .circular_scroll_deadzone = DT_INST_PROP_OR(inst, circular_scroll_deadzone, 10), \
        ))                                                                            \
    };                                                                            \
                                                                                     \
     static struct cirque_glidepoint_data cirque_glidepoint_data_##inst;              \
                                                                                     \
     DEVICE_DT_INST_DEFINE(inst,                                                     \
                         cirque_glidepoint_init,                                      \
                         NULL,                                                        \
                         &cirque_glidepoint_data_##inst,                              \
                         &cirque_glidepoint_config_##inst,                            \
                         POST_KERNEL,                                                \
                         CONFIG_INPUT_INIT_PRIORITY,                                 \
                         NULL);
 
 DT_INST_FOREACH_STATUS_OKAY(CIRQUE_GLIDEPOINT_INIT)

/* 
    devicetree overlay Circular scroll configuration example:
    
    irq-gpios = <&gpio0 13 GPIO_ACTIVE_LOW>; // why do i need irq-gpios? why do i need irq-gpios, I was using dr-gpios = <&gpio0 6 (GPIO_ACTIVE_HIGH)>;
    irq-gpios (Interrupt Request GPIO):

    circular-scroll-radius = <40>;  // Radius in pixels to activate circular scroll 
    circular-scroll-deadzone = <15>; // Inner deadzone to prevent accidental activation 

*/