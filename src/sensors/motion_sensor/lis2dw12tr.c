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

static void lis2dw12tr_setup(void)
{
    // Correct Code for CTRL1 Configuration
    // For a door movement detector with low power consumption, we would want:

    // ODR = 12.5 Hz (ODR[3:0] = 0001)
    // Low-power mode (MODE[1:0] = 00)
    // Low-power mode 4 for lowest power (LP_MODE[1:0] = 11)

    // This would give us:
    // 0001 00 11 = 0x13
    // So the correct register write would be:
    // /* CTRL1: Configure for ODR 12.5 Hz, Low-power mode 4 (lowest power) */
    // ret = lis2dw12_write_reg(i2c_dev, LIS2DW12_REG_CTRL1, 0x13);
    // if (ret < 0) return false;

    // For the CTRL2 register (0x21), the correct bit mapping is:
    // Bit 0: SIM (SPI serial interface mode selection)
    // Bit 1: I2C_DISABLE (disable I2C interface)
    // Bit 2: IF_ADD_INC (register address auto-increment)
    // Bit 3: BDU (block data update)
    // Bit 4: CS_PU_DISC (pull-up on CS pin disabled)
    // Bit 5: Reserved (should be set to 0)
    // Bit 6: SOFT_RESET (software reset)
    // Bit 7: BOOT (reboot memory content)
    
    // Given this correct mapping, the value for CTRL2 depends on your interface needs:
    // If using 4-wire SPI: SIM=0 (bit 0)
    // If using 3-wire SPI: SIM=1 (bit 0)
    // For I2C mode: I2C_DISABLE=0 (bit 1)
    // Auto-increment is usually helpful: IF_ADD_INC=1 (bit 2)
    // BDU is recommended for consistent readings: BDU=1 (bit 3)
    
    // For a typical I2C configuration with auto-increment and BDU, you would use:
    // CTRL2 = 0b00001100 = 0x0C
    // The reserved bit (bit 5) should indeed be kept at 0 for proper operation as you mentioned.

    // CTRL3
    // The value 0x20 in binary is 00100000, which means only bit 5 is set to 1 (bits are numbered 7 down to 0).
    // Looking at the datasheet fields you provided:
    
    // Bit 5 corresponds to H_LACTIVE (High/Low active)
    
    // When H_LACTIVE = 1 (which is what your code is setting), it configures the interrupt
    // signal on INT1 pin to be active high. This means:
    
    // The interrupt pin will be pulled HIGH when an interrupt is triggered
    // In the inactive state, the pin remains LOW
    
    // So the purpose of this line:
    // ret = lis2dw12_write_reg(i2c_dev, LIS2DW12_REG_CTRL3, 0x20);
    // Is specifically to configure the INT1 pin to be active high (signal goes high when an interrupt is triggered).
    // All other bits in this register are being set to 0, which means:
    
    // ST1, ST2: Self-test disabled
    // PP_OD: Push-pull output (not open drain)
    // LIR: Latched interrupt disabled
    // SLP_MODE_SEL, SLP_MODE_1: Sleep mode options disabled

    // CTRL4

    // The value 0x20 corresponds to setting bit 5 of the LIS2DW12_REG_CTRL4_INT1 register.
    // Looking at the bit mapping you provided:
    
    // INT1_6D (bit 7): 0x80
    // INT1_SINGLE_TAP (bit 6): 0x40
    // INT1_WU (bit 5): 0x20 ← This is what's being set
    // INT1_FF (bit 4): 0x10
    // INT1_TAP (bit 3): 0x08
    // INT1_DIFF5 (bit 2): 0x04
    // INT1_FTH (bit 1): 0x02
    // INT1_DRDY (bit 0): 0x01
    
    // By writing 0x20 to this register, the code is enabling the INT1_WU (Wake-Up) interrupt on the INT1 pin.
    // This means the LIS2DW12 accelerometer will generate an interrupt signal on INT1 when a wake-up event is detected
    // (typically when acceleration exceeds a certain threshold).
    // All other interrupt sources for the INT1 pin are disabled since their corresponding bits are set to 0.

    // CTRL5 is skipped because we do not use INT2

    // CTRL6 reg:
    // Setting 0x00 means all bits are set to 0, which configures:

    // BW_FILT = 00: Default bandwidth filter setting
    // FS = 00: ±2g full-scale range (lowest sensitivity setting)
    // FDS = 0: Low-pass filter path selected
    // LOW_NOISE = 0: Low-noise mode disabled (normal power mode)
    
    // This represents a basic, conservative configuration with:
    
    // The lowest full-scale range (±2g), which provides the highest resolution for small movements
    // Standard filtering without special noise reduction
    // Normal power consumption (not low-noise mode)

    // WAKE_UP_THS -> this will be using separate method. we will use default value in here, max +- 2g
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