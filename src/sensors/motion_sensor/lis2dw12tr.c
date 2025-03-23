#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "motion_sensor.h"
#include "lis2dw12tr.h"

static const struct gpio_dt_spec motion_sensor_int = GPIO_DT_SPEC_GET(MOTION_SENSOR_INT_NODE, gpios);

// use binary semaphore
K_SEM_DEFINE(motion_int_sem, 0, 1);

// for debugging purposes
static volatile uint32_t motion_int_count           = 0;
static volatile uint32_t motion_int_processed_count = 0;
static volatile uint32_t motion_int_missed_count    = 0;

static struct gpio_callback motion_int_cb;

static void motion_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    motion_int_count++;

    if (k_sem_count_get(&motion_int_sem) > 0)
    {
        motion_int_missed_count++;
        printk("Motion interrupt count #%d triggered, prev data skipped\n", motion_int_count);
    }
    else
    {
        printk("Motion interrupt count #%d triggered\n", motion_int_count);
    }

    k_sem_give(&motion_int_sem);
}

static int interrupt_setup(void)
{
    if (!device_is_ready(motion_sensor_int.port))
    {
        printf("Motion sensor interrupt is not ready\n");
        return -1;
    }

    ret = gpio_pin_configure_dt(&motion_sensor_int, GPIO_INPUT | GPIO_PULL_UP);
    if (ret != 0)
    {
        printf("Failed to configure motion sensor pin\n");
        return -1;
    }

    ret = gpio_pin_interrupt_configure_dt(&motion_sensor_int, GPIO_INT_EDGE_TO_ACTIVE);

    if (ret != 0)
    {
        printf("Failed to configure motion sensor interrupt\n");
        return -1;
    }

    gpio_init_callback(&motion_int_cb, motion_int_callback, BIT(motion_sensor_int.pin));
    gpio_add_callback(motion_sensor_int.port, &motion_int_cb);

    printf("Motion interrupt is ready\n");

    return 0;
}

static void lis2dw12tr_init(void)
{
    int ret;
    printf("lis2dw12tr_init\n");

    if (interrupt_setup() != 0)
    {
        printf("Motion interrupt is not ready\n");

        return;
    }

    // TODO
    // set accelerometer to continuous comparing with threshold
}

static void lis2dw12tr_set_threshold(float threshold)
{
    printf("lis2dw12tr_set_threshold\n");
}

static bool lis2dw12tr_is_interrupt_triggered(void)
{
    printf("lis2dw12tr_is_interrupt_triggered\n");

    k_sem_take(&motion_int_sem, K_FOREVER);
    motion_int_processed_count++;

    printf("Processing accelerometer data #%d\n", motion_int_processed_count);

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