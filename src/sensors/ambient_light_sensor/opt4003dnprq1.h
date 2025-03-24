#ifndef OPT4003DNPRQ1_H
#define OPT4003DNPRQ1OPT4003DNPRQ1_H

#define LIGHT_INT_NODE DT_ALIAS(extint1)
#define LIGHT_SENSOR_INT_NODE DT_CHILD(LIGHT_INT_NODE, light_sensor_int)

#if !DT_NODE_HAS_STATUS(LIGHT_INT_NODE, okay)
#error "Unsupported board: light-int devicetree alias is not defined"
#endif

#define LIGHT_I2C_DEV_NODE DT_ALIAS(lighti2c)

// Table 6-10. ADDR Pin Addresses
// ADDR PIN CONNECTION DEVICE I2C ADDRESS
// GND                 1000100 (0x44)
// VDD                 1000101 (0x45)
// SDA                 1000110 (0x46)
// SCL                 1000111 (0x46)

/* OPT4003 I2C Address (Default: 0x44) */
#define OPT4003Q1_I2C_ADDR      0x44

// register addresses
#define OPT4003_REG_THRESHOLD_L 0x08
#define OPT4003_REG_THRESHOLD_H 0x09
#define OPT4003_REG_CONFIG      0x0A
#define OPT4003_REG_INT_CONFIG  0x0B
#define OPT4003_REG_STATUS      0x0C

// config register
#define CONVERSION_TIME 6
#define OPERATING_MODE  4
#define LATCH           3
#define INT_POL         2

// int config register
#define INT_DIR         4
#define INT_CFG         2
#define I2C_BURST       0


#endif // OPT4003DNPRQ1_H