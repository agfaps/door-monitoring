#ifndef SENSORS_CONFIG_H
#define SENSORS_CONFIG_H

#define AMBIENT_LIGHT_SENSOR_IMPLEMENTATION OPT4003DNPRQ1
#define MOTION_SENSOR_IMPLEMENTATION        LIS2DW12TR
#define DISTANCE_SENSOR_IMPLEMENTATION      VL53L3CX

// Threshold values for sensors
#define MOTION_THRESHOLD_G          4
#define LIGHT_LOW_THRESHOLD_LUX     10
#define LIGHT_HIGH_THRESHOLD_LUX    40
#define DISTANCE_THRESHOLD_MM       100
#define DISTANCE_READING_TIMEOUT_MS 5000    // 5 seconds in milliseconds

#endif // SENSORS_CONFIG_H