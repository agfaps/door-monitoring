#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "sensors_config.h"

typedef struct
{
    void (*init)(void);
    void (*set_threshold)(uint8_t threshold);
    bool (*is_interrupt_triggered)(void);
    void (*clear_interrupt)(void);
} motion_sensor_api_t;

#if MOTION_SENSOR_IMPLEMENTATION == LIS2DW12TR
    #include "lis2dw12tr.h"
#else
    #error "Unsupported motion sensor implementation"
#endif

extern const motion_sensor_api_t *motion_sensor_get_api(void);

static inline void motion_sensor_init(void)
{
    motion_sensor_get_api()->init();
}

static inline void motion_sensor_set_threshold(uint8_t threshold)
{
    motion_sensor_get_api()->set_threshold(threshold);
}

static inline bool motion_sensor_is_interrupt_triggered(void)
{
    return motion_sensor_get_api()->is_interrupt_triggered();
}

static inline void motion_sensor_clear_interrupt(void)
{
    motion_sensor_get_api()->clear_interrupt();
}

#endif // MOTION_SENSOR_H