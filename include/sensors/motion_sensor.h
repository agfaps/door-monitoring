#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include <stdbool.h>

void motion_sensor_init() {}
void motion_sensor_set_threshold(float threshold) {}
bool motion_sensor_is_interrupt_triggered() { return true; }
void motion_sensor_clear_interrupt() {}

// void motion_sensor_init();
// void motion_sensor_set_threshold(float threshold);
// bool motion_sensor_is_interrupt_triggered();
// void motion_sensor_clear_interrupt();

#endif // MOTION_SENSOR_H