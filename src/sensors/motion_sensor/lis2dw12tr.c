#include <zephyr/kernel.h>

#include "motion_sensor.h"
#include "lis2dw12tr.h"

static void lis2dw12tr_init(void)
{
    printf("lis2dw12tr_init\n");
}

static void lis2dw12tr_set_threshold(float threshold)
{
    printf("lis2dw12tr_set_threshold\n");
}

static bool lis2dw12tr_is_interrupt_triggered(void)
{
    printf("lis2dw12tr_is_interrupt_triggered\n");
    return true;
}

static void lis2dw12tr_clear_interrupt(void)
{
    printf("lis2dw12tr_clear_interrupt\n");
}

static const motion_sensor_api_t lis2dw12tr_api =
{
    .init                   = lis2dw12tr_init,
    .set_threshold          = lis2dw12tr_set_threshold,
    .is_interrupt_triggered = lis2dw12tr_is_interrupt_triggered,
    .clear_interrupt        = lis2dw12tr_clear_interrupt,
};

const motion_sensor_api_t *motion_sensor_get_api(void)
{
    return &lis2dw12tr_api;
}