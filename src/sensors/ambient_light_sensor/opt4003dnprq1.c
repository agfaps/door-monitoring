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
        printf("Light interrupt count #%d triggered, prev data skipped\n", light_int_count);
    }
    else
    {
        printf("Light interrupt count #%d triggered\n", light_int_count);
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

    // printf("Ambient light interrupt is ready\n");

    return 0;
}

static void initial_register_setup(void)
{
    uint8_t config[3];
    uint8_t int_config[3];

    // printf("initial_register_setup\n");

    // ===== OPT4003_REG_CONFIG =====
    // Field Breakdown:
    // QWAKE (Bit 15)    : 0
    // Reserved (Bit 14) : 0
    // RANGE (Bits 13-10): 0 (0000 binary)
    // CONVERSION_TIME (Bits 9-6): 1 (0001 binary)  max is 800ms in 11 decimal
    // OPERATING_MODE (Bits 5-4) : 3 (11 binary) Continuous
    // LATCH (Bit 3)   : 1 (need to clear, good for mcu in stop mode)
    // INT_POL (Bit 2) : 1 (for active high/rising edge detection)
    // FAULT_COUNT (Bits 1-0): 0 (00 binary)

    // Understanding the Scenario
    // Dark Room: The ambient light levels will be very low when the door is closed.
    // Door Open: When the door opens, even a small amount of light from an adjacent hallway
    // or light seepage around the door frame will cause a noticeable increase in light level.
    // Rapid Detection: We want to detect the door's state change quickly.
    // Optimizing Range and Conversion Time

    // Range (Bits 13-10):
    // We need the sensor to be very sensitive to small changes in light. Therefore, we should choose the lowest possible range.
    // The lowest range is typically represented by a low value in the RANGE bits.
    // Because the reset value is 'C' (1100), this is a high range. Therefore we will want to lower this value.
    // I would recommend a range value of 0x00. This will give the highest sensitivity.

    // Conversion Time (Bits 9-6):
    // For rapid detection, we need a short conversion time.
    // The shortest conversion time is 600 µs (0).
    // However, if there is a lot of noise in the light signal, then a longer conversion time will help to average out the noise.
    // Therefore I would recommend a conversion time of 1ms (0x01). This is still a very fast conversion time, but it will help to reduce noise.

    // Putting it all together
    // To make the register value, we will keep the other values the same as the previous example.
    // 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0
    // 0  0  0  0  0  0  0 0 0 1 1 1 1 1 0 0
    // Converting this binary value to hexadecimal:
    // 0000 0000 0111 1100
    // 0x007C
    // Therefore, the hexadecimal value to write to register 0Ah is 0x013C.

    uint16_t config_value = (1 << CONVERSION_TIME) | (3 << OPERATING_MODE) | (1 << LATCH) | (1 << INT_POL);
    config[0] = OPT4003_REG_CONFIG;
    config[1] = config_value >> 8;
    config[2] = config_value & 0xFF;
    i2c_write(light_sensor_i2c_dev, config, 3, OPT4003Q1_I2C_ADDR);

    // uint16_t config_value = (1 << CONVERSION_TIME) | (3 << OPERATING_MODE) | (1 << LATCH) | (1 << INT_POL);
    // uint8_t config[3];

    // config[0] = OPT4003_REG_CONFIG;
    // config[1] = config_value >> 8;
    // config[2] = config_value & 0xFF;

    // struct i2c_msg msgs[] = {
    //     {
    //         .buf = config,
    //         .len = sizeof(config),
    //         .flags = I2C_MSG_WRITE | I2C_MSG_STOP,
    //     },
    // };

    // int ret = i2c_transfer(light_sensor_i2c_dev, msgs, 1, OPT4003Q1_I2C_ADDR);

    // ===== OPT4003_REG_THRESHOLD_L and OPT4003_REG_THRESHOLD_H =====

    // In a dark room that can receive light from outside when the door is open, 
    // a common lux level would be between 20-50 lux for public areas or general lighting, 
    // and 50-100 lux for areas like bedrooms or living rooms. 
    // Here's a more detailed breakdown:
    // Public Areas/Dark Surroundings: A lux level of 20-50 is suitable for public areas with dark surroundings. 
    // Family Living Room: A lux level of 50 is a common level for family living rooms. 
    // Bedrooms: Commonly, bedrooms have a lux level of 60-100. 
    // Living Rooms: Living rooms are often lit to a lux level of 100-150. 
    // Kitchens: Kitchens, especially working areas, often require higher lux levels, around 250-300 lux. 
    // Bathrooms: Bathrooms are commonly lit to a lux level of 150-300. 
    // Outdoor Light Levels:
    // Direct Sunlight: 32,000 to 100,000 lux 
    // Full Daylight: 10,000 to 25,000 lux 
    // Overcast Daylight: 1000 lux 
    // Sunset & Sunrise: 400 lux 
    // Indoor Light Levels:
    // Office: 500-1000 lux 
    // General Lighting: 100-300 lux 
    // Precise Work: 1500-2000 lux 

    // Set threshold values (example: low = 1000, high = 2000)
    uint16_t threshold_low          = LIGHT_LOW_THRESHOLD_LUX;
    uint16_t threshold_high         = LIGHT_HIGH_THRESHOLD_LUX;
    uint8_t  threshold_low_data[3]  = {OPT4003_REG_THRESHOLD_L, threshold_low >> 8, threshold_low & 0xFF};
    uint8_t  threshold_high_data[3] = {OPT4003_REG_THRESHOLD_H, threshold_high >> 8, threshold_high & 0xFF};
    i2c_write(light_sensor_i2c_dev, threshold_low_data, 3, OPT4003Q1_I2C_ADDR);
    i2c_write(light_sensor_i2c_dev, threshold_high_data, 3, OPT4003Q1_I2C_ADDR);

    // ===== OPT4003_REG_INT_CONFIG =====
    // Channel 0 (Visible Light) Only:
    // THRESHOLD_CH_SEL (Bit 5) = 0
    // Interrupt from Device to MCU:
    // INT_DIR (Bit 4) = 1 (output)
    // I2C Burst Always Set:
    // I2C_BURST (Bit 0) = 1
    // INT Config for Every Conversion:
    // INT_CFG (Bits 3-2) = 1 (01 binary)

    // Bits 15-6 (Reserved): 128 (200h)
    // Bit  5    (THRESHOLD_CH_SEL): 0
    // Bit  4    (INT_DIR): 1
    // Bits 3-2  (INT_CFG): 1 (01 binary)
    // Bit  1    (Reserved): 0
    // Bit  0    (I2C_BURST): 1
    // 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0
    // 1  0  0  0  0  0  0 0 0 0 0 1 0 1 0 1
    uint16_t int_config_value = 128 | (1 << INT_DIR) | (1 << INT_CFG) | (1 << I2C_BURST);
    int_config[0] = OPT4003_REG_INT_CONFIG;
    int_config[1] = int_config_value >> 8;
    int_config[2] = int_config_value & 0xFF;
    i2c_write(light_sensor_i2c_dev, int_config, 3, OPT4003Q1_I2C_ADDR);
}

static void opt4003dnprq1_init(void)
{
    // printf("opt4003dnprq1_init\n");

    if (interrupt_setup() != 0)
    {
        printf("Ambient light interrupt is not ready\n");

        return;
    }

    initial_register_setup();
}

static void opt4003dnprq1_set_threshold(uint16_t low_threshold, uint16_t high_threshold)
{
    // printf("opt4003dnprq1_set_threshold\n");

    uint8_t  threshold_low_data[3]  = {OPT4003_REG_THRESHOLD_L, low_threshold >> 8, low_threshold & 0xFF};
    uint8_t  threshold_high_data[3] = {OPT4003_REG_THRESHOLD_H, high_threshold >> 8, high_threshold & 0xFF};

    i2c_write(light_sensor_i2c_dev, threshold_low_data, 3, OPT4003Q1_I2C_ADDR);
    i2c_write(light_sensor_i2c_dev, threshold_high_data, 3, OPT4003Q1_I2C_ADDR);
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
        // using latched mode, we need to read 0x0C register to clear the interrupt
        uint8_t reg_addr = OPT4003_REG_STATUS;
        uint8_t status_data[2] = {0};
        int ret_val;
        
        // Read from the status register to clear the interrupt
        ret_val = i2c_write_read(
                    light_sensor_i2c_dev,
                    OPT4003Q1_I2C_ADDR, 
                    &reg_addr,
                    1,     // Write phase: register address
                    status_data,
                    2
                );  // Read phase: 2 bytes of data

        if (ret_val != 0)
        {
            printf("Failed to read status register: %d\n", ret_val);
            return ret_val;
        }
        
        // The interrupt is now cleared
        // OPT4003 (like most I2C devices) transmits data in big-endian format
        uint16_t status = (status_data[0] << 8) | status_data[1];
        status = status;    // to remove warning when not printed
        // printf("Status register value: 0x%04x\n", status);

        return true;
    }
    else
    {
        // printf("No light interrupt semaphore\n");

        return false;
    }
}

static void opt4003dnprq1_clear_interrupt(void)
{
    // printf("opt4003dnprq1_clear_interrupt\n");
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