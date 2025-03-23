#ifndef AMBIENT_LIGHT_SENSOR_H
#define AMBIENT_LIGHT_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "sensors_config.h"

typedef struct
{
    void (*init)(void);
    void (*set_threshold)(uint16_t low_threshold, uint16_t high_threshold);
    bool (*is_interrupt_triggered)(void);
    void (*clear_interrupt)(void);
} ambient_light_sensor_api_t;

#if AMBIENT_LIGHT_SENSOR_IMPLEMENTATION == OPT4003DNPRQ1
    #include "opt4003dnprq1.h"
#else
    #error "Unsupported ambient light sensor implementation"
#endif

extern const ambient_light_sensor_api_t *ambient_light_sensor_get_api(void);

static inline void ambient_light_sensor_init(void)
{
    ambient_light_sensor_get_api()->init();
}

static inline void ambient_light_sensor_set_threshold(uint16_t low_threshold, uint16_t high_threshold)
{
    ambient_light_sensor_get_api()->set_threshold(low_threshold, high_threshold);
}

static inline bool ambient_light_sensor_is_interrupt_triggered(void)
{
    return ambient_light_sensor_get_api()->is_interrupt_triggered();
}

static inline void ambient_light_sensor_clear_interrupt(void)
{
    ambient_light_sensor_get_api()->clear_interrupt();
}

#endif // AMBIENT_LIGHT_SENSOR_H