#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "ambient_light_sensor.h"
#include "opt4003dnprq1.h"

static const struct gpio_dt_spec light_sensor_int = GPIO_DT_SPEC_GET(LIGHT_SENSOR_INT_NODE, gpios);
static const struct device *light_sensor_i2c_dev  = DEVICE_DT_GET(LIGHT_I2C_DEV_NODE);

// use binary semaphore
K_SEM_DEFINE(light_int_sem, 0, 1);

// for debugging purposes
static volatile uint32_t light_int_count           = 0;
static volatile uint32_t light_int_processed_count = 0;
static volatile uint32_t light_int_missed_count    = 0;

static struct gpio_callback light_int_cb;

static void light_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    light_int_count++;

    if (k_sem_count_get(&light_int_sem) > 0)
    {
        light_int_missed_count++;
        printk("Light interrupt count #%d triggered, prev data skipped\n", light_int_count);
    }
    else
    {
        printk("Light interrupt count #%d triggered\n", light_int_count);
        k_sem_give(&light_int_sem);
    }
}

static int interrupt_setup(void)
{
    int ret = 0;

    if (!device_is_ready(light_sensor_int.port))
    {
        printf("Ambient light sensor interrupt is not ready\n");
        return -1;
    }

    ret = gpio_pin_configure_dt(&light_sensor_int, GPIO_INPUT | GPIO_PULL_UP);
    if (ret != 0)
    {
        printf("Failed to configure ambient light sensor pin\n");
        return -1;
    }

    ret = gpio_pin_interrupt_configure_dt(&light_sensor_int, GPIO_INT_EDGE_TO_ACTIVE);

    if (ret != 0)
    {
        printf("Failed to configure ambient light sensor interrupt\n");
        return -1;
    }

    gpio_init_callback(&light_int_cb, light_int_callback, BIT(light_sensor_int.pin));
    gpio_add_callback(light_sensor_int.port, &light_int_cb);

    printf("Ambient light interrupt is ready\n");

    return 0;
}

static int opt4003dnprq1_i2c_read_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t *value)
{
    
}

static int opt4003dnprq1_i2c_write_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t value)
{
    
}

static void initial_register_setup(void)
{
    printf("initial_register_setup\n");
}

static void opt4003dnprq1_init(void)
{
    printf("opt4003dnprq1_init\n");

    if (interrupt_setup() != 0)
    {
        printf("Ambient light interrupt is not ready\n");

        return;
    }

    initial_register_setup();
}

static void opt4003dnprq1_set_threshold(uint16_t low_threshold, uint16_t high_threshold)
{
    printf("opt4003dnprq1_set_threshold\n");
}

static bool opt4003dnprq1_is_interrupt_triggered(void)
{
    int ret = 1;
    printf("opt4003dnprq1_is_interrupt_triggered\n");

    ret = k_sem_take(&light_int_sem, K_NO_WAIT);
    if (ret == 0)
    {
        light_int_processed_count++;

        // if we need to read and process ambient light data
        printf("Processing ambient light data #%d\n", light_int_processed_count);        
    }
    else
    {
        printf("No light interrupt semaphore\n");
    }

    return true;
}

static void opt4003dnprq1_clear_interrupt(void)
{
    printf("opt4003dnprq1_clear_interrupt\n");
}

static const ambient_light_sensor_api_t opt4003dnprq1_api =
{
    .init                   = opt4003dnprq1_init,
    .set_threshold          = opt4003dnprq1_set_threshold,
    .is_interrupt_triggered = opt4003dnprq1_is_interrupt_triggered,
    .clear_interrupt        = opt4003dnprq1_clear_interrupt
};

const ambient_light_sensor_api_t *ambient_light_sensor_get_api(void)
{
    return &opt4003dnprq1_api;
}