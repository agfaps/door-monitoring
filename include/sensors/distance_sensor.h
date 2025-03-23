#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    void (*init)(void);
    bool (*is_active)(void);
    void (*activate)(void);
    bool (*is_ready)(void);
    uint32_t (*get)(void);
    void (*deactivate)(void);
} distance_sensor_api_t;

#if DISTANCE_SENSOR_IMPLEMENTATION == VL53L3CX
    #include "vl53l3cx.h"
#else
    #error "Unsupported distance sensor implementation"
#endif

extern const distance_sensor_api_t *distance_sensor_get_api(void);

static inline void distance_sensor_init(void)
{
    distance_sensor_get_api()->init();
}

static inline bool distance_sensor_is_active(void)
{
    return distance_sensor_get_api()->is_active();
}

static inline void distance_sensor_activate(void)
{
    distance_sensor_get_api()->activate();
}

static inline bool distance_sensor_is_ready(void)
{
    return distance_sensor_get_api()->is_ready();
}

static inline uint32_t distance_sensor_get(void)
{
    return distance_sensor_get_api()->get();
}

static inline void distance_sensor_deactivate(void)
{
    distance_sensor_get_api()->deactivate();
}

#endif // DISTANCE_SENSOR_H