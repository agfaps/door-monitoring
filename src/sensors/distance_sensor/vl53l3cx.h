#ifndef VL53L3CX_H
#define VL53L3CX_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

#define DISTANCE_INT_NODE          DT_ALIAS(extint2)
#define DISTANCE_XSHUT_NODE        DT_ALIAS(distancexshut)
#define DISTANCE_SENSOR_INT_NODE   DT_CHILD(DISTANCE_INT_NODE, distance_sensor_int)
#define DISTANCE_SENSOR_XSHUT_NODE DT_CHILD(DISTANCE_XSHUT_NODE, distance_sensor_xshut)
#define DISTANCE_I2C_DEV_NODE      DT_ALIAS(distancei2c)

#if !DT_NODE_HAS_STATUS(DISTANCE_INT_NODE, okay)
#error "Unsupported board: distance-int devicetree alias is not defined"
#endif

// VL53L3CX I²C device address: 0x52
#define VL53L3CX_I2C_ADDR          0x52U
#define VL53L3CX_REG_WHO_AM_I_VAL  0xEAU

// Register Addresses
#define VL53L3CX_REG_WHO_AM_I      0x010FU
#define VL53L3CX_REG_MODULE_TYPE   0x0110U
#define VL53L3CX_REG_SYSTEM_START  0x0087U
#define VL53L3CX_REG_RESULT_RANGE  0x0096U
#define VL53L3CX_REG_INT_CONFIG    0x0046U

// NOTE: tBOOT (FWBOOT) is 1.2 ms max
#define VL53L3CX_BOOT_TIME_MS      3U


typedef struct {
    const struct device       *i2c_dev;
    const struct gpio_dt_spec distance_sensor_int;
    const struct gpio_dt_spec distance_xshut;
    struct gpio_callback      int_cb;
    bool                      sensor_ready;
    bool                      sensor_active;
    uint16_t                  distance_mm;
    struct k_sem              distance_int_sem;
} vl53l3cx_data_t;

#endif // VL53L3CX_H