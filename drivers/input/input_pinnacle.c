#define DT_DRV_COMPAT cirque_pinnacle

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>

#include "input_pinnacle.h"

LOG_MODULE_REGISTER(pinnacle, CONFIG_INPUT_LOG_LEVEL);

static int pinnacle_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                             const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return config->seq_read(dev, addr, buf, len);
}
static int pinnacle_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return config->write(dev, addr, val);
}

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int pinnacle_i2c_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return i2c_burst_read_dt(&config->bus.i2c, PINNACLE_READ | addr, buf, len);
}

static int pinnacle_i2c_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->bus.i2c, PINNACLE_WRITE | addr, val);
}

#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int pinnacle_spi_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[len + 3], rx_dummy[3];
    tx_buffer[0] = PINNACLE_READ | addr;
    memset(&tx_buffer[1], PINNACLE_AUTOINC, len + 2);

    const struct spi_buf tx_buf[2] = {
        {
            .buf = tx_buffer,
            .len = len + 3,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 1,
    };
    struct spi_buf rx_buf[2] = {
        {
            .buf = rx_dummy,
            .len = 3,
        },
        {
            .buf = buf,
            .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 2,
    };
    int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    return ret;
}

static int pinnacle_spi_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[2] = {PINNACLE_WRITE | addr, val};
    uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    if (ret < 0) {
        LOG_ERR("spi ret: %d", ret);
    }

    if (rx_buffer[1] != PINNACLE_FILLER) {
        LOG_ERR("bad ret val %d - %d", rx_buffer[0], rx_buffer[1]);
        return -EIO;
    }

    k_usleep(50);

    return ret;
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int set_int(const struct device *dev, const bool en) {
    const struct pinnacle_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static int pinnacle_clear_status(const struct device *dev) {
    int ret = pinnacle_write(dev, PINNACLE_STATUS1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

static int pinnacle_era_read(const struct device *dev, const uint16_t addr, uint8_t *val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_READ);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_VALUE, val, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static int pinnacle_era_write(const struct device *dev, const uint16_t addr, uint8_t val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_VALUE, val);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}


// Apply sigmoid acceleration to the input delta
static int8_t apply_sigmoid_acceleration(const struct device *dev, int8_t delta) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;
    
    if (!config->sigmoid_acceleration || abs(delta) < config->acceleration_threshold) {
        return delta;
    }
    
    // Get time since last movement
    int64_t now = k_uptime_get();
    int64_t time_delta = now - data->last_timestamp;
    data->last_timestamp = now;
    
    // If it's been too long, reset acceleration
    if (time_delta > 100) {
        return delta;
    }
    
    // Calculate acceleration using sigmoid function
    // sigmoid(x) = x / (1 + abs(x))
    float accel = (float)delta / (1.0f + fabsf((float)delta / config->acceleration_threshold));
    
    // Apply acceleration factor
    accel = accel * config->acceleration_factor;
    
    // Scale based on the original direction
    return (delta >= 0) ? (int8_t)(accel + 0.5f) : (int8_t)(accel - 0.5f);
}

// Process circular scroll
static void process_circular_scroll(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;
    
    if (!config->circular_scroll) {
        return;
    }
    
    // If we're not currently in a circular scroll and a button is pressed, initiate scroll
    if (!data->in_circular_scroll && data->btn_cache) {
        data->in_circular_scroll = true;
        data->scroll_center_x = 0;
        data->scroll_center_y = 0;
        data->last_angle = 0;
        data->scroll_direction = PINNACLE_SCROLL_NONE;
        *dx = 0;
        *dy = 0;
        return;
    }
    
    // If we were in a circular scroll but button was released, exit scroll mode
    if (data->in_circular_scroll && !data->btn_cache) {
        data->in_circular_scroll = false;
        return;
    }
    
    // If we're in circular scroll mode
    if (data->in_circular_scroll) {
        // Accumulate movement to track position relative to center
        data->scroll_center_x += *dx;
        data->scroll_center_y += *dy;
        
        // Calculate distance from center (approximation of sqrt)
        int16_t abs_x = abs(data->scroll_center_x);
        int16_t abs_y = abs(data->scroll_center_y);
        int16_t distance = abs_x > abs_y ? abs_x + abs_y/2 : abs_y + abs_x/2;

        // Check if we're inside the central deadzone radius
        if (distance < config->circular_scroll_deadzone_radius) {
            // Inside central deadzone - no scrolling
            *dx = 0;
            *dy = 0;
            return;
        }
        
        // If we're outside the minimum radius, process the scroll
        if (distance >= config->circular_scroll_radius) {
            // Calculate angle (in 0-255 range, where 0 is right, 64 is top, etc.)
            int16_t angle = 0;
            if (abs_x > abs_y) {
                // More horizontal than vertical
                angle = data->scroll_center_x > 0 ? 0 : 128;  // Right or left
                int16_t ratio = (abs_y * 64) / abs_x;
                if (data->scroll_center_y > 0) {
                    angle = (angle == 0) ? ratio : 256 - ratio;
                } else {
                    angle = (angle == 0) ? 256 - ratio : ratio;
                }
            } else {
                // More vertical than horizontal
                angle = data->scroll_center_y > 0 ? 64 : 192;  // Up or down
                int16_t ratio = (abs_x * 64) / abs_y;
                if (data->scroll_center_x > 0) {
                    angle = (angle == 64) ? 64 - ratio : 192 + ratio;
                } else {
                    angle = (angle == 64) ? 64 + ratio : 192 - ratio;
                }
            }
            angle &= 0xFF;  // Ensure 0-255 range
            
            // Determine direction of rotation
            if (data->last_angle != 0) {
                int16_t angle_diff = angle - data->last_angle;
                
                // Handle wrapping
                if (angle_diff > 128) angle_diff -= 256;
                if (angle_diff < -128) angle_diff += 256;
                
                // Only process if outside deadzone
                if (abs(angle_diff) > config->circular_scroll_deadzone) {
                    // Get the direction (positive = clockwise, negative = counterclockwise)
                    bool is_clockwise = angle_diff > 0;
                    if (config->circular_scroll_invert_direction) {
                        is_clockwise = !is_clockwise;
                    }
                    
                    // In key mode, send key events instead of scroll events
                    if (config->circular_scroll_key_mode) {
                        // Reset scroll values
                        *dx = 0;
                        *dy = 0;
                        
                        // Send appropriate key event
                        if (is_clockwise && config->circular_scroll_key_clockwise != 0) {
                            input_report_key(dev, config->circular_scroll_key_clockwise, 1, false, K_FOREVER);
                            input_report_key(dev, config->circular_scroll_key_clockwise, 0, true, K_FOREVER);
                        } else if (!is_clockwise && config->circular_scroll_key_counterclockwise != 0) {
                            input_report_key(dev, config->circular_scroll_key_counterclockwise, 1, false, K_FOREVER);
                            input_report_key(dev, config->circular_scroll_key_counterclockwise, 0, true, K_FOREVER);
                        }
                        
                        // Reset last angle to avoid repeat triggers too quickly
                        angle_diff = 0;
                    } else {
                        // Normal scroll mode - determine direction based on initial movement
                        if (data->scroll_direction == PINNACLE_SCROLL_NONE) {
                            data->scroll_direction = (angle >= 32 && angle < 96) || 
                                                    (angle >= 160 && angle < 224) 
                                                  ? PINNACLE_SCROLL_VERTICAL 
                                                  : PINNACLE_SCROLL_HORIZONTAL;
                        }
                        
                        // Map scroll direction and amount
                        if (data->scroll_direction == PINNACLE_SCROLL_VERTICAL) {
                            *dx = 0;
                            *dy = angle_diff > 0 ? -1 : 1;  // Adjust direction as needed
                        } else {
                            *dx = angle_diff > 0 ? 1 : -1;  // Adjust direction as needed
                            *dy = 0;
                        }
                    }
                } else {
                    *dx = 0;
                    *dy = 0;
                }
            }
            
            data->last_angle = angle;
        } else {
            // Not far enough from center yet
            *dx = 0;
            *dy = 0;
        }
    }
}

static void pinnacle_report_data(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    uint8_t packet[3];
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 1, "Pinnacle Status1");

    // Ignore 0xFF packets that indicate communcation failure, or if SW_DR isn't asserted
    if (packet[0] == 0xFF || !(packet[0] & PINNACLE_STATUS1_SW_DR)) {
        return;
    }
    ret = pinnacle_seq_read(dev, PINNACLE_2_2_PACKET0, packet, 3);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 3, "Pinnacle Packets");

    struct pinnacle_data *data = dev->data;
    uint8_t btn = packet[0] &
                  (PINNACLE_PACKET0_BTN_PRIM | PINNACLE_PACKET0_BTN_SEC | PINNACLE_PACKET0_BTN_AUX);

    int8_t dx = (int8_t)packet[1];
    int8_t dy = (int8_t)packet[2];

    if (packet[0] & PINNACLE_PACKET0_X_SIGN) {
        WRITE_BIT(dx, 7, 1);
    }
    if (packet[0] & PINNACLE_PACKET0_Y_SIGN) {
        WRITE_BIT(dy, 7, 1);
    }

    if (data->in_int) {
        LOG_DBG("Clearing status bit");
        ret = pinnacle_clear_status(dev);
        data->in_int = true;
    }

    // Process circular scrolling if enabled
    if (config->circular_scroll) {
        process_circular_scroll(dev, &dx, &dy);
    }
    
    // Apply sigmoid acceleration if enabled
    if (config->sigmoid_acceleration && !data->in_circular_scroll) {
        dx = apply_sigmoid_acceleration(dev, dx);
        dy = apply_sigmoid_acceleration(dev, dy);
    }
    
    // Update last delta values
    data->last_dx = dx;
    data->last_dy = dy;

    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
            }
        }
    }

    data->btn_cache = btn;

    input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);

    return;
}

static void pinnacle_work_cb(struct k_work *work) {
    struct pinnacle_data *data = CONTAINER_OF(work, struct pinnacle_data, work);
    pinnacle_report_data(data->dev);
}

static void pinnacle_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct pinnacle_data *data = CONTAINER_OF(cb, struct pinnacle_data, gpio_cb);

    LOG_DBG("HW DR asserted");
    data->in_int = true;
    k_work_submit(&data->work);
}

static int pinnacle_adc_sensitivity_reg_value(enum pinnacle_sensitivity sensitivity) {
    switch (sensitivity) {
    case PINNACLE_SENSITIVITY_1X:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    case PINNACLE_SENSITIVITY_2X:
        return PINNACLE_TRACKING_ADC_CONFIG_2X;
    case PINNACLE_SENSITIVITY_3X:
        return PINNACLE_TRACKING_ADC_CONFIG_3X;
    case PINNACLE_SENSITIVITY_4X:
        return PINNACLE_TRACKING_ADC_CONFIG_4X;
    default:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    }
}

static int pinnacle_tune_edge_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    int ret;

    uint8_t x_val;
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, &x_val);
    if (ret < 0) {
        LOG_WRN("Failed to read X val");
        return ret;
    }

    LOG_WRN("X val: %d", x_val);

    uint8_t y_val;
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, &y_val);
    if (ret < 0) {
        LOG_WRN("Failed to read Y val");
        return ret;
    }

    LOG_WRN("Y val: %d", y_val);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, config->x_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set X-Axis Min-Z %d", ret);
        return ret;
    }
    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, config->y_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set Y-Axis Min-Z %d", ret);
        return ret;
    }
    return 0;
}

static int pinnacle_set_adc_tracking_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;

    uint8_t val;
    int ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    val &= 0x3F;
    val |= pinnacle_adc_sensitivity_reg_value(config->sensitivity);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, val);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
    }
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    return ret;
}

static int pinnacle_force_recalibrate(const struct device *dev) {
    uint8_t val;
    int ret = pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to get cal config %d", ret);
    }

    val |= 0x01;
    ret = pinnacle_write(dev, PINNACLE_CAL_CFG, val);
    if (ret < 0) {
        LOG_ERR("Failed to force calibration %d", ret);
    }

    do {
        pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    } while (val & 0x01);

    return ret;
}

int pinnacle_set_sleep(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & PINNACLE_SYS_CFG_EN_SLEEP) != 0) == enabled) {
        return 0;
    }

    LOG_DBG("Setting sleep: %s", (enabled ? "on" : "off"));
    WRITE_BIT(sys_cfg, PINNACLE_SYS_CFG_EN_SLEEP_BIT, enabled ? 1 : 0);

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write sleep config %d", ret);
        return ret;
    }

    return ret;
}

static int pinnacle_init(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    const struct pinnacle_config *config = dev->config;
    
    // Initialize tracking variables
    data->last_dx = 0;
    data->last_dy = 0;
    data->last_timestamp = k_uptime_get();
    data->in_circular_scroll = false;
    data->scroll_center_x = 0;
    data->scroll_center_y = 0;
    data->last_angle = 0;
    data->scroll_direction = PINNACLE_SCROLL_NONE;
    
    int ret;

    uint8_t fw_id[2];
    ret = pinnacle_seq_read(dev, PINNACLE_FW_ID, fw_id, 2);
    if (ret < 0) {
        LOG_ERR("Failed to get the FW ID %d", ret);
    }

    LOG_DBG("Found device with FW ID: 0x%02x, Version: 0x%02x", fw_id[0], fw_id[1]);

    data->in_int = false;
    k_msleep(10);
    ret = pinnacle_write(dev, PINNACLE_STATUS1, 0); // Clear CC
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    k_usleep(50);
    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_RESET);
    if (ret < 0) {
        LOG_ERR("can't reset %d", ret);
        return ret;
    }
    k_msleep(20);
    ret = pinnacle_write(dev, PINNACLE_Z_IDLE, 0x05); // No Z-Idle packets
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    ret = pinnacle_set_adc_tracking_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
        return ret;
    }

    ret = pinnacle_tune_edge_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to tune edge sensitivity %d", ret);
        return ret;
    }
    ret = pinnacle_force_recalibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to force recalibration %d", ret);
        return ret;
    }

    if (config->sleep_en) {
        ret = pinnacle_set_sleep(dev, true);
        if (ret < 0) {
            return ret;
        }
    }

    uint8_t packet[1];
    ret = pinnacle_seq_read(dev, PINNACLE_SLEEP_INTERVAL, packet, 1);

    if (ret >= 0) {
        LOG_DBG("Default sleep interval %d", packet[0]);
    }

    ret = pinnacle_write(dev, PINNACLE_SLEEP_INTERVAL, 255);
    if (ret <= 0) {
        LOG_DBG("Failed to update sleep interaval %d", ret);
    }

    uint8_t feed_cfg2 = PINNACLE_FEED_CFG2_EN_IM | PINNACLE_FEED_CFG2_EN_BTN_SCRL;
    if (config->no_taps) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_TAP;
    }

    if (config->no_secondary_tap) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_SEC;
    }

    if (config->rotate_90) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_ROTATE_90;
    }
    ret = pinnacle_write(dev, PINNACLE_FEED_CFG2, feed_cfg2);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    uint8_t feed_cfg1 = PINNACLE_FEED_CFG1_EN_FEED;
    if (config->x_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_X;
    }

    if (config->y_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_Y;
    }
    if (feed_cfg1) {
        ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);
    }
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    data->dev = dev;

    pinnacle_clear_status(dev);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, pinnacle_gpio_cb, BIT(config->dr.pin));
    ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, pinnacle_work_cb);

    pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);

    set_int(dev, true);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int pinnacle_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return set_int(dev, false);
    case PM_DEVICE_ACTION_RESUME:
        return set_int(dev, true);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define PINNACLE_DIRECTION_INVERTED 1

#define PINNACLE_INST(n)                                                                           \
    static struct pinnacle_data pinnacle_data_##n;                                                 \
    static const struct pinnacle_config pinnacle_config_##n = {                                    \
        COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                                        \
                    (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n)}, .seq_read = pinnacle_i2c_seq_read,   \
                     .write = pinnacle_i2c_write),                                                 \
                    (.bus = {.spi = SPI_DT_SPEC_INST_GET(n,                                        \
                                                         SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |    \
                                                             SPI_TRANSFER_MSB | SPI_MODE_CPHA,     \
                                                         0)},                                      \
                     .seq_read = pinnacle_spi_seq_read, .write = pinnacle_spi_write)),             \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                                   \
        .x_invert = DT_INST_PROP(n, x_invert),                                                     \
        .y_invert = DT_INST_PROP(n, y_invert),                                                     \
        .sleep_en = DT_INST_PROP(n, sleep),                                                        \
        .no_taps = DT_INST_PROP(n, no_taps),                                                       \
        .no_secondary_tap = DT_INST_PROP(n, no_secondary_tap),                                     \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 5),                                       \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 4),                                       \
        .sensitivity = DT_INST_ENUM_IDX_OR(n, sensitivity, PINNACLE_SENSITIVITY_1X),               \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                   \
        .sigmoid_acceleration = DT_INST_PROP_OR(n, sigmoid_acceleration, false),                   \
        .acceleration_factor = DT_INST_PROP_OR(n, acceleration_factor, 2.0),                       \
        .acceleration_threshold = DT_INST_PROP_OR(n, acceleration_threshold, 5.0),                 \
        .circular_scroll = DT_INST_PROP_OR(n, circular_scroll, false),                             \
        .circular_scroll_radius = DT_INST_PROP_OR(n, circular_scroll_radius, 20),                  \
        .circular_scroll_deadzone = DT_INST_PROP_OR(n, circular_scroll_deadzone, 4),               \
        .circular_scroll_deadzone_radius = DT_INST_PROP_OR(n, circular_scroll_deadzone_radius, 15), \
        .circular_scroll_invert_direction = DT_INST_PROP_OR(n, circular_scroll_direction, 0) == PINNACLE_DIRECTION_INVERTED, \
        .circular_scroll_key_mode = DT_INST_PROP_OR(n, circular_scroll_key_mode, false),           \
        .circular_scroll_key_clockwise = DT_INST_PROP_OR(n, circular_scroll_key_clockwise, 0),     \
        .circular_scroll_key_counterclockwise = DT_INST_PROP_OR(n, circular_scroll_key_counterclockwise, 0), \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, pinnacle_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, pinnacle_init, PM_DEVICE_DT_INST_GET(n), &pinnacle_data_##n,          \
                          &pinnacle_config_##n, POST_KERNEL, CONFIG_INPUT_PINNACLE_INIT_PRIORITY,  \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PINNACLE_INST)
