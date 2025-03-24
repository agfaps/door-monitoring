#ifndef LIS2DW12TR_H
#define LIS2DW12TR_H

#define MOTION_INT_NODE DT_ALIAS(extint0)
#define MOTION_SENSOR_INT_NODE DT_CHILD(MOTION_INT_NODE, motion_sensor_int)

#if !DT_NODE_HAS_STATUS(MOTION_INT_NODE, okay)
#error "Unsupported board: motion-int devicetree alias is not defined"
#endif

#define MOTION_I2C_DEV_NODE DT_ALIAS(motioni2c)

// address
#define LIS2DW12TR_I2C_ADDR                 0x19    /* If SDO/SA0 is pulled to GND, use 0x18 */

// registers
#define LIS2DW12TR_OUT_T_L_REG              0x0D
#define LIS2DW12TR_OUT_T_H_REG              0x0E
#define LIS2DW12TR_WHO_AM_I_REG             0x0F
#define LIS2DW12TR_CTRL1_REG                0x20
#define LIS2DW12TR_CTRL2_REG                0x21
#define LIS2DW12TR_CTRL3_REG                0x22
#define LIS2DW12TR_CTRL4_INT1_PAD_CTRL_REG  0x23
#define LIS2DW12TR_CTRL5_INT2_PAD_CTRL_REG  0x24
#define LIS2DW12TR_CTRL6_REG                0x25
#define LIS2DW12TR_OUT_T_REG                0x26
#define LIS2DW12TR_STATUS_REG               0x27
#define LIS2DW12TR_OUT_X_L_REG              0x28
#define LIS2DW12TR_OUT_X_H_REG              0x29
#define LIS2DW12TR_OUT_Y_L_REG              0x2A
#define LIS2DW12TR_OUT_Y_H_REG              0x2B
#define LIS2DW12TR_OUT_Z_L_REG              0x2C
#define LIS2DW12TR_OUT_Z_H_REG              0x2D
#define LIS2DW12TR_FIFO_CTRL_REG            0x2E
#define LIS2DW12TR_FIFO_SAMPLES_REG         0x2F
#define LIS2DW12TR_TAP_THS_X_REG            0x30
#define LIS2DW12TR_TAP_THS_Y_REG            0x31
#define LIS2DW12TR_TAP_THS_Z_REG            0x32
#define LIS2DW12TR_INT_DUR_REG              0x33
#define LIS2DW12TR_WAKE_UP_THS_REG          0x34
#define LIS2DW12TR_WAKE_UP_DUR_REG          0x35
#define LIS2DW12TR_FREE_FALL_REG            0x36
#define LIS2DW12TR_STATUS_DUP_REG           0x37
#define LIS2DW12TR_WAKE_UP_SRC_REG          0x38
#define LIS2DW12TR_TAP_SRC_REG              0x39
#define LIS2DW12TR_SIXD_SRC_REG             0x3A
#define LIS2DW12TR_ALL_INT_SRC_REG          0x3B
#define LIS2DW12TR_X_OFS_USR_REG            0x3C
#define LIS2DW12TR_Y_OFS_USR_REG            0x3D
#define LIS2DW12TR_Z_OFS_USR_REG            0x3E
#define LIS2DW12TR_CTRL7_REG                0x3F

#define LIS2DW12TR_WHO_AM_I_VAL             0x44

// CTRL1 register
#define ODR3        7
#define ODR2        6
#define ODR1        5
#define ODR0        4

#define LP_MODE1    1
#define LP_MODE0    0

// CTRL2 register
#define BDU         3
#define IF_ADD_INC  2

// CTRL3 register
#define H_LACTIVE   3

// LIS2DW12TR_CTRL4_INT1_PAD_CTRL_REG register
#define INT1_WU     5

// LIS2DW12TR_CTRL6_REG register
#define BW_FILT     6

#define ODR_20      3
#define ODR_10      2
#define ODR_4       1
#define ODR_2       0

// LIS2DW12TR_WAKE_UP_THS_REG register
#define WK_THS      0

#define MAX_WK_THS  63

// LIS2DW12TR_WAKE_UP_DUR_REG register
#define WAKE_DUR        5

#define THREE_ODR_CYCLE 3

// LIS2DW12TR_CTRL7_REG register
#define INTERRUPTS_ENABLE 5

#endif // LIS2DW12TR_H