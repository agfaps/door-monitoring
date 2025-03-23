#include <zephyr/kernel.h>

#include "ambient_light_sensor.h"
#include "opt4003dnprq1.h"

static void opt4003dnprq1_init(void)
{
    printf("opt4003dnprq1_init\n");
}

static void opt4003dnprq1_set_threshold(uint16_t low_threshold, uint16_t high_threshold)
{
    printf("opt4003dnprq1_set_threshold\n");
}

static bool opt4003dnprq1_is_interrupt_triggered(void)
{
    printf("opt4003dnprq1_is_interrupt_triggered\n");
    return true;
}

static void opt4003dnprq1_clear_interrupt(void)
{
    printf("opt4003dnprq1_clear_interrupt\n");
}

static const ambient_light_sensor_api_t opt4003dnprq1_api = {
    .init = opt4003dnprq1_init,
    .set_threshold = opt4003dnprq1_set_threshold,
    .is_interrupt_triggered = opt4003dnprq1_is_interrupt_triggered,
    .clear_interrupt = opt4003dnprq1_clear_interrupt
};

const ambient_light_sensor_api_t *ambient_light_sensor_get_api(void)
{
    return &opt4003dnprq1_api;
}