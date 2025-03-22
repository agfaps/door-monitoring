#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

void distance_sensor_init() {}
bool distance_sensor_is_active() { return false; }
void distance_sensor_activate() {}
bool distance_sensor_is_ready() { return true; }
uint32_t distance_sensor_get() { return 0; }
void distance_sensor_deactivate() {}

// void distance_sensor_init();
// bool distance_sensor_is_active();
// void distance_sensor_activate();
// bool distance_sensor_is_ready();
// uint32_t distance_sensor_get();
// void distance_sensor_deactivate();

#endif // DISTANCE_SENSOR_H