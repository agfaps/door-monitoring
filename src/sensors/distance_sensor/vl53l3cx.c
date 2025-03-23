#include <zephyr/kernel.h>

#include "distance_sensor.h"
#include "vl53l3cx.h"

static void vl53l3cx_init(void)
{
    printf("vl53l3cx_init\n");
}

static bool vl53l3cx_is_active(void)
{
    printf("vl53l3cx_is_active\n");
    return true;
}

static void vl53l3cx_activate(void)
{
    printf("vl53l3cx_activate\n");
}

static bool vl53l3cx_is_ready(void)
{
    printf("vl53l3cx_is_ready\n");
    return true;
}

static uint32_t vl53l3cx_get(void)
{
    printf("vl53l3cx_get\n");
    return 0;
}

static void vl53l3cx_deactivate(void)
{
    printf("vl53l3cx_deactivate\n");
}

static const distance_sensor_api_t vl53l3cx_api =
{
    .init        = vl53l3cx_init,
    .is_active   = vl53l3cx_is_active,
    .activate    = vl53l3cx_activate,
    .is_ready    = vl53l3cx_is_ready,
    .get         = vl53l3cx_get,
    .deactivate  = vl53l3cx_deactivate,
};

const distance_sensor_api_t *distance_sensor_get_api(void)
{
    return &vl53l3cx_api;
}