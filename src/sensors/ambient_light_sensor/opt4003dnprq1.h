#ifndef OPT4003DNPRQ1_H
#define OPT4003DNPRQ1_H

#define LIGHT_INT_NODE DT_ALIAS(extint1)
#define LIGHT_SENSOR_INT_NODE DT_CHILD(LIGHT_INT_NODE, light_sensor_int)

#if !DT_NODE_HAS_STATUS(LIGHT_INT_NODE, okay)
#error "Unsupported board: light-int devicetree alias is not defined"
#endif

#define LIGHT_I2C_DEV_NODE DT_ALIAS(lighti2c)

// address
#define OPT4003DNPRQ1_I2C_ADDR                 0x4A    /* If SDO/SA0 is pulled to GND, use 0x49 */

// registers

#endif // OPT4003DNPRQ1_H