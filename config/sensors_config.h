#ifndef SENSORS_CONFIG_H
#define SENSORS_CONFIG_H

#define AMBIENT_LIGHT_SENSOR_IMPLEMENTATION OPT4003DNPRQ1
#define MOTION_SENSOR_IMPLEMENTATION        LIS2DW12TR
#define DISTANCE_SENSOR_IMPLEMENTATION      VL53L3CX

// Threshold values for sensors
#define MOTION_THRESHOLD_G              4       // one step is 0.0625g
#define LIGHT_LOW_THRESHOLD_LUX         10
#define LIGHT_HIGH_THRESHOLD_LUX        40
#define DISTANCE_THRESHOLD_MM           100
#define DISTANCE_READING_TIMEOUT_MS     3000    // 3 seconds in milliseconds
#define DISTANCE_MINIMUM_SAMPLES_COUNT  3       // min samples before deciding the door state

#endif // SENSORS_CONFIG_H