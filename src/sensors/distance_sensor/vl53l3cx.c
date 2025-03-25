#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "distance_sensor.h"
#include "vl53l3cx.h"

static vl53l3cx_data_t *vl53l3cx_data;

// for debugging purposes
static volatile uint32_t distance_int_count           = 0;
static volatile uint32_t distance_int_processed_count = 0;
static volatile uint32_t distance_int_missed_count    = 0;

/**
 * @brief Create a new instance of the VL53L3CX data structure.
 *
 * This function allocates and initializes a new VL53L3CX data structure.
 * The structure is initialized with the I2C device, GPIO pin for the
 * distance sensor interrupt, GPIO pin for the XSHUT pin, and a zeroed out
 * distance measurement.
 *
 * @return A pointer to the new instance of the VL53L3CX data structure.
 */
static vl53l3cx_data_t *create_vl53l3cx_data(void)
{
    static vl53l3cx_data_t data = {
        .i2c_dev             = DEVICE_DT_GET(DISTANCE_I2C_DEV_NODE),
        .distance_sensor_int = GPIO_DT_SPEC_GET(DISTANCE_SENSOR_INT_NODE, gpios),
        .distance_xshut      = GPIO_DT_SPEC_GET(DISTANCE_SENSOR_XSHUT_NODE, gpios),
        .sensor_ready        = false,
        .sensor_active       = false,
        .distance_mm         = 0
    };

    return &data;
}

static void distance_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    distance_int_count++;

    if (k_sem_count_get(&vl53l3cx_data->distance_int_sem) > 0)
    {
        distance_int_missed_count++;

        printf("Distance interrupt count #%d triggered, prev data skipped\n", distance_int_count);
    }
    else
    {
        printf("Distance interrupt count #%d triggered\n", distance_int_count);

        k_sem_give(&vl53l3cx_data->distance_int_sem);
    }
}

static int interrupt_setup(void)
{
    int ret = 0;

    if (!device_is_ready(vl53l3cx_data->distance_sensor_int.port))
    {
        printf("Distance sensor interrupt is not ready\n");

        return -1;
    }

    ret = gpio_pin_configure_dt(&vl53l3cx_data->distance_sensor_int, GPIO_INPUT | GPIO_PULL_UP);
    if (ret != 0)
    {
        printf("Failed to configure distance sensor pin\n");

        return -1;
    }

    ret = gpio_pin_interrupt_configure_dt(&vl53l3cx_data->distance_sensor_int, GPIO_INT_EDGE_TO_ACTIVE);

    if (ret != 0)
    {
        printf("Failed to configure distance sensor interrupt\n");

        return -1;
    }

    gpio_init_callback(&vl53l3cx_data->int_cb, distance_int_callback, BIT(vl53l3cx_data->distance_sensor_int.pin));
    gpio_add_callback(vl53l3cx_data->distance_sensor_int.port, &vl53l3cx_data->int_cb);

    printf("Distance interrupt is ready\n");

    return 0;
}

static void initial_register_setup(void)
{
    printf("initial_register_setup\n");
}

static void vl53l3cx_init(void)
{
    int ret_value = 0;

    printf("vl53l3cx_init\n");

    vl53l3cx_data = create_vl53l3cx_data();  

    k_sem_init(&vl53l3cx_data->distance_int_sem, 0, 1);

    if (!gpio_is_ready_dt(&vl53l3cx_data->distance_xshut))
    {
        printf("Distance sensor XSHUT is not ready\n");

        return;
    }

    // Sets the initial state to LOW (0)
    // Enables the internal pull-up resistor of the mcu
    ret_value = gpio_pin_configure_dt(&vl53l3cx_data->distance_xshut, GPIO_OUTPUT_INACTIVE | GPIO_PULL_UP);

    if (ret_value < 0)
    {
        printf("Failed to configure distance sensor XSHUT\n");

        return;
    }

    if (interrupt_setup() != 0)
    {
        printf("Distance interrupt is not ready\n");

        return;
    }

    initial_register_setup();

    vl53l3cx_data->sensor_ready = true;
}



static bool vl53l3cx_is_active(void)
{
    printf("vl53l3cx_is_active\n");

    return vl53l3cx_data->sensor_active;
}

static void vl53l3cx_activate(void)
{
    int ret = 0;

    printf("vl53l3cx_activate\n");

    // Drive XSHUT high to enable the sensor (active low)
    ret = gpio_pin_set_dt(&vl53l3cx_data->distance_xshut, 1);

    if (ret < 0)
    {
        printf("Failed to enable sensor via XSHUT\n");
    }
    else
    {
        // delay for tBOOT (FWBOOT)
        k_msleep(VL53L3CX_BOOT_TIME_MS);

        vl53l3cx_data->sensor_active = true;
    }
}

static bool vl53l3cx_is_ready(void)
{
    printf("vl53l3cx_is_ready\n");

    return vl53l3cx_data->sensor_ready;
}

static uint32_t vl53l3cx_get_distance_mm(void)
{
    printf("vl53l3cx_get\n");

    return vl53l3cx_data->distance_mm;
}

static void vl53l3cx_deactivate(void)
{
    int ret = 0;
    printf("vl53l3cx_deactivate\n");
    
    // Drive XSHUT low to disable the sensor (active low)
    ret = gpio_pin_set_dt(&vl53l3cx_data->distance_xshut, 0);
    
    if (ret < 0)
    {
        printf("Failed to disable sensor via XSHUT\n");
    }
    else
    {
        vl53l3cx_data->sensor_active = false;
    }
}

static const distance_sensor_api_t vl53l3cx_api =
{
    .init        = vl53l3cx_init,
    .is_active   = vl53l3cx_is_active,
    .activate    = vl53l3cx_activate,
    .is_ready    = vl53l3cx_is_ready,
    .get         = vl53l3cx_get_distance_mm,
    .deactivate  = vl53l3cx_deactivate,
};

const distance_sensor_api_t *distance_sensor_get_api(void)
{
    return &vl53l3cx_api;
}