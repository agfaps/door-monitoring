#ifndef AMBIENT_LIGHT_SENSOR_H
#define AMBIENT_LIGHT_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

void ambient_light_sensor_init() {}
void ambient_light_sensor_set_threshold(uint16_t low_threshold, uint16_t high_threshold) {}
bool ambient_light_sensor_is_interrupt_triggered() { return true; }
void ambient_light_sensor_clear_interrupt() {}

// void ambient_light_sensor_init();
// void ambient_light_sensor_set_threshold(uint16_t low_threshold, uint16_t high_threshold);
// bool ambient_light_sensor_is_interrupt_triggered();
// void ambient_light_sensor_clear_interrupt();

#endif // AMBIENT_LIGHT_SENSOR_H