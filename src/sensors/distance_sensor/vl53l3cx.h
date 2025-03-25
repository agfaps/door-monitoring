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
#define VL53L3CX_I2C_ADDR           0x52U
#define VL53L3CX_REG_WHO_AM_I_VAL   0xEAU
#define VL53L3CX_REG_INT_CONFIG_VAL 0x01U  // Enable interrupt on new measurement

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


// VL53L0X internal registers
#define REG_RESULT_INTERRUPT_STATUS 		0x13
#define RESULT_RANGE_STATUS      		0x14
#define ALGO_PHASECAL_LIM                       0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT            0x30

#define GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48

#define PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57

#define REG_MSRC_CONFIG_CONTROL                 0x60
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44
#define SYSRANGE_START                          0x00
#define SYSTEM_SEQUENCE_CONFIG                  0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define RESULT_INTERRUPT_STATUS                 0x13
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define SYSTEM_INTERRUPT_CLEAR                  0x0B

#define SEQUENCE_ENABLE_FINAL_RANGE 0x80
#define SEQUENCE_ENABLE_PRE_RANGE   0x40
#define SEQUENCE_ENABLE_TCC         0x10
#define SEQUENCE_ENABLE_DSS         0x08
#define SEQUENCE_ENABLE_MSRC        0x04

// VL53L0X internal registers
#define REG_IDENTIFICATION_MODEL_ID		0xc0
#define REG_IDENTIFICATION_REVISION_ID		0xc2
#define REG_SYSRANGE_START			0x00

#endif // VL53L3CX_H