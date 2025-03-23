#ifndef LIS2DW12TR_H
#define LIS2DW12TR_H

#define MOTION_INT_NODE DT_ALIAS(extint0)
#define MOTION_SENSOR_INT_NODE DT_CHILD(MOTION_INT_NODE, motion_sensor_int)

#if !DT_NODE_HAS_STATUS(MOTION_INT_NODE, okay)
#error "Unsupported board: motion-int devicetree alias is not defined"
#endif

#define EDGE (GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)

#endif // LIS2DW12TR_H