#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "motion_sensor.h"
#include "lis2dw12tr.h"

static const struct gpio_dt_spec motion_sensor_int = GPIO_DT_SPEC_GET(MOTION_SENSOR_INT_NODE, gpios);
static const struct device *motion_sensor_i2c_dev  = DEVICE_DT_GET(MOTION_I2C_DEV_NODE);

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
    int ret;

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

static int lis2dw12tr_i2c_read_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t *value)
{
    printf("LIS2DW12TR I2C read reg: 0x%02x\n", reg_addr);
    // return i2c_reg_read_byte(i2c_dev, LIS2DW12TR_I2C_ADDR, reg_addr, value);

    return 0;
}

static int lis2dw12tr_i2c_write_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t value)
{
    printf("LIS2DW12TR I2C write reg: 0x%02x val: 0x%02x\n", reg_addr, value);
    return i2c_reg_write_byte(i2c_dev, LIS2DW12TR_I2C_ADDR, reg_addr, value);
}

static void initial_register_setup(void)
{
    uint8_t who_am_i;
    int ret;

    if (!device_is_ready(motion_sensor_i2c_dev))
    {
        printf("Motion sensor I2C is not ready\n");
        return;
    }

    // check who am i register (skipped for testing without real hardware)
    ret = lis2dw12tr_i2c_read_reg(motion_sensor_i2c_dev, LIS2DW12TR_WHO_AM_I_REG, &who_am_i);

    if ((ret < 0) || (who_am_i != LIS2DW12TR_WHO_AM_I_VAL))
    {
        printf("Failed to read LIS2DW12TR_WHO_AM_I_REG register\n");

        // skipped for testing
        // return;
    }

    // ====== CTRL1 Configuration ======
    // For the CTRL1 register (0x20), the correct bit mapping is:
    // Bit 7: ODR3 (Output data rate selection bit 3)
    // Bit 6: ODR2 (Output data rate selection bit 2)
    // Bit 5: ODR1 (Output data rate selection bit 1)
    // Bit 4: ODR0 (Output data rate selection bit 0)
    // Bit 3: MODE1 (Operating mode selection bit 1)
    // Bit 2: MODE0 (Operating mode selection bit 0)
    // Bit 1: LP_MODE1 (Low-power mode selection bit 1)
    // Bit 0: LP_MODE0 (Low-power mode selection bit 0)

    // For a door movement detector with low power consumption:
    // ODR = 12.5 Hz (ODR[3:0] = 0001)
    // Low-power mode (MODE[1:0] = 00)
    // Low-power mode 4 for lowest power (LP_MODE[1:0] = 11)

    // This would be:
    // 0001 00 11 = 0x13
    // CTRL1: Configure for ODR 12.5 Hz, Low-power mode 4 (lowest power) 14-bit resolution
    ret =   lis2dw12tr_i2c_write_reg(
                motion_sensor_i2c_dev,
                LIS2DW12TR_CTRL1_REG,
                (1 << ODR0) | (1 << LP_MODE1) | (3 << LP_MODE0)
            );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_CTRL1_REG register\n");
        // skipped for testing
        // return;
    }

    // ====== CTRL2 Configuration ======
    // For the CTRL2 register (0x21), the correct bit mapping is:
    // Bit 7: BOOT (reboot memory content)
    // Bit 6: SOFT_RESET (software reset)
    // Bit 5: Reserved (should be set to 0)
    // Bit 4: CS_PU_DISC (pull-up on CS pin disabled)
    // Bit 3: BDU (block data update)
    // Bit 2: IF_ADD_INC (register address auto-increment)
    // Bit 1: I2C_DISABLE (disable I2C interface)
    // Bit 0: SIM (SPI serial interface mode selection)

    // SIM settings depends on interface needs:
    // If using 4-wire SPI: SIM=0 (bit 0)
    // If using 3-wire SPI: SIM=1 (bit 0)
    // For I2C mode: I2C_DISABLE=0 (bit 1)
    // Auto-increment is usually helpful: IF_ADD_INC=1 (bit 2)
    // BDU is recommended for consistent readings: BDU=1 (bit 3)
    
    // For a typical I2C configuration with auto-increment and BDU:
    // CTRL2 = 0b00001100 = 0x0C
    ret =   lis2dw12tr_i2c_write_reg(
                motion_sensor_i2c_dev,
                LIS2DW12TR_CTRL2_REG,
                (1 << BDU) | (1 << IF_ADD_INC)
            );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_CTRL2_REG register\n");
        // skipped for testing
        // return;
    }

    // ====== CTRL3 Configuration ======
    // For the CTRL3 register (0x22), the correct bit mapping is:
    // Bit 7: ST2 (Self-test bit 2)
    // Bit 6: ST1 (Self-test bit 1)
    // Bit 5: PP_OD (Push-pull/open drain selection for interrupt pins)
    // Bit 4: LIR (Latched interrupt)
    // Bit 3: H_LACTIVE (Interrupt active high/low)
    // Bit 2: 0 (Reserved bit, should be set to 0)
    // Bit 1: SLP_MODE_SEL (Sleep mode selection)
    // Bit 0: SLP_MODE_1 (Sleep mode bit 1)

    // To generate interrupts on the rising edge of INT1, we need:
    // 1. H_LACTIVE = 1 (bit 3): Sets interrupt signal to active high
    // 2. PP_OD = 0 (bit 5): Uses push-pull output mode (recommended for edge detection)
    // 3. ST1, ST2: Self-test disabled
    // 4. LIR     : Latched interrupt disabled
    // 5. SLP_MODE_SEL, SLP_MODE_1: Sleep mode options disabled
    ret =   lis2dw12tr_i2c_write_reg(
                motion_sensor_i2c_dev,
                LIS2DW12TR_CTRL3_REG,
                (1 << H_LACTIVE)
            );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_CTRL3_REG register\n");
        // skipped for testing
        // return;
    }

    // ====== LIS2DW12TR_CTRL4_INT1_PAD_CTRL_REG Configuration ======
    // For the LIS2DW12TR_CTRL4_INT1_PAD_CTRL_REG register (0x23), the correct bit mapping is:
    // Bit 7: INT1_6D (6D orientation detection interrupt on INT1 pin)
    // Bit 6: INT1_SINGLE_TAP (Single-tap detection interrupt on INT1 pin)
    // Bit 5: INT1_WU (Wake-up detection interrupt on INT1 pin)
    // Bit 4: INT1_FF (Free-fall detection interrupt on INT1 pin)
    // Bit 3: INT1_TAP (Double-tap detection interrupt on INT1 pin)
    // Bit 2: INT1_DIFF5 (DIFF5 interrupt on INT1 pin)
    // Bit 1: INT1_FTH (FIFO threshold interrupt on INT1 pin)
    // Bit 0: INT1_DRDY (Data-ready interrupt on INT1 pin)

    // The value 0x20 corresponds to setting bit 5 of the LIS2DW12_REG_CTRL4_INT1 register.
    // By writing 0x20 to this register, the code is enabling the INT1_WU (Wake-Up) interrupt on the INT1 pin.
    // This means the LIS2DW12 accelerometer will generate an interrupt signal on INT1 when a wake-up event is detected
    // (typically when acceleration exceeds a certain threshold).
    // All other interrupt sources for the INT1 pin are disabled since their corresponding bits are set to 0.
    ret =   lis2dw12tr_i2c_write_reg(
                motion_sensor_i2c_dev,
                LIS2DW12TR_CTRL4_INT1_PAD_CTRL_REG,
                (1 << INT1_WU)
            );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_CTRL4_INT1_PAD_CTRL_REG register\n");
        // skipped for testing
        // return;
    }

    // ====== LIS2DW12TR_CTRL5_INT2_PAD_CTRL_REG Configuration ======
    // LIS2DW12TR_CTRL5_INT2_PAD_CTRL_REG is skipped because we do not use INT2

    // ====== LIS2DW12TR_CTRL6_REG Configuration ======
    // For the CTRL6 register (0x25), the correct bit mapping is:
    // Bit 7: BW_FILT1 (Bandwidth filter bit 1)
    // Bit 6: BW_FILT0 (Bandwidth filter bit 0)
    // Bit 5: FS1 (Full-scale selection bit 1)
    // Bit 4: FS0 (Full-scale selection bit 0)
    // Bit 3: FDS (Filtered data selection)
    // Bit 2: LOW_NOISE (Low-noise configuration)
    // Bit 1: 0 (Reserved bit, should be set to 0)
    // Bit 0: 0 (Reserved bit, should be set to 0)

    // Setting 0x00 means all bits are set to 0, which configures:
    // BW_FILT   = 00: Default bandwidth filter setting (can be adjusted later)
    // FS        = 00: ±2g full-scale range (lowest sensitivity setting)
    // FDS       = 0: Low-pass filter path selected
    // LOW_NOISE = 0: Low-noise mode disabled (normal power mode)
    
    // This represents a basic, conservative configuration with:
    // The lowest full-scale range (±2g), which provides the highest resolution for small movements
    // Standard filtering without special noise reduction
    // Normal power consumption (not low-noise mode)

    // Optimal Setting for 12.5 Hz ODR:
    // Recommended: BW_FILT = 01 (ODR/4)
    // This gives you a cutoff frequency of approximately 3.125 Hz (12.5 Hz ÷ 4),
    // which is ideal for door swing detection for these reasons:
    // 1. Appropriate frequency range: A door swing typically has a frequency signature below 3 Hz,
    //    so a 3.125 Hz cutoff will capture the entire movement pattern while filtering out higher-frequency noise.
    // 2. Sampling considerations: At 12.5 Hz ODR, you're already sampling relatively slowly (one sample every 80ms).
    //    Using BW_FILT = 10 (ODR/10) or BW_FILT = 11 (ODR/20) would filter too aggressively for this low sampling rate,
    //    potentially missing important aspects of the door movement.
    // 3. Power vs. detection balance: The ODR/4 setting provides good noise rejection while maintaining sufficient
    //    signal detail for reliable door swing detection, even at your low sampling rate.
    // 4. Minimal delay: With your already conservative sampling rate of 12.5 Hz, the ODR/4 filter introduces minimal
    //    additional delay in detecting the swing event compared to more aggressive filtering options.
    ret =   lis2dw12tr_i2c_write_reg(
            motion_sensor_i2c_dev,
            LIS2DW12TR_CTRL6_REG,
            (ODR_4 << BW_FILT)
        );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_CTRL6_REG register\n");
        // skipped for testing
        // return;
    }

    // ====== LIS2DW12TR_WAKE_UP_THS_REG Configuration ======
    // The WAKE_UP_THS register (address 0x34) serves two main purposes:
    // It configures the wake-up threshold (WK_THS[5:0] bits)
    // It contains configuration bits for tap detection and sleep mode (bit 7 and bit 6)
    
    // Writing 0x04 (binary 00000100) to this register, which means:
    
    // SINGLE_DOUBLE_TAP (bit 7) = 0: This disables single/double tap distinction
    // SLEEP_ON (bit 6) = 0: Sleep mode is disabled
    // WK_THS[5:0] (bits 5-0) = 000100 (decimal 4): Sets the wake-up threshold to 4
    
    // How the threshold works:
    // The wake-up threshold determines how much acceleration is needed to trigger a wake-up event.
    // Each step is 1/64 of full scale (±2g), so threshold of 4 = ~0.125g

    // The calculation works like this:
    // The threshold value is in steps of 1/64 of the selected full scale
    // If your sensor is configured for ±2g full scale range, then:
    // 1 LSB (step) = (2g × 2) ÷ 64 = 4g ÷ 64 = 0.0625g
    // Threshold of 4 = 4 × 0.0625g = 0.25g

    // To set up this register:
    // Decide on the desired wake-up threshold based on the application's sensitivity needs
    // Calculate the register value:
    // If you want the threshold to be X g, and your full scale is FS:
    // WK_THS = (X × 64) ÷ (2 × FS)
    
    // Set bits 7 and 6 as needed for tap detection and sleep mode
    // Write the combined value to the LIS2DW12_REG_WAKE_UP_THS register
    
    // For a door movement detector, a threshold around 3-5 makes sense as it would detect meaningful
    // motion while ignoring minor vibrations. The value of 4 would trigger the wake-up
    // interrupt when acceleration exceeds approximately 0.25g, which is reasonable for detecting door movement.

    // NOTE: we initially set the highest value and then will use a separate method to set the desired value
    ret =   lis2dw12tr_i2c_write_reg(
                motion_sensor_i2c_dev,
                LIS2DW12TR_WAKE_UP_THS_REG,
                (MAX_WK_THS << WK_THS)
            );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_WAKE_UP_THS_REG register\n");
        // skipped for testing
        // return;
    }

    // ====== LIS2DW12TR_WAKE_UP_DUR_REG Configuration ======
    // For the WAKE_UP_DUR register (0x35), the correct bit mapping is:
    // Bit 7: FF_DUR5 (Free-fall duration bit 5)
    // Bit 6: WAKE_DUR1 (Wake-up duration bit 1)
    // Bit 5: WAKE_DUR0 (Wake-up duration bit 0)
    // Bit 4: STATIONARY (Stationary detection)
    // Bit 3: SLEEP_DUR3 (Sleep duration bit 3)
    // Bit 2: SLEEP_DUR2 (Sleep duration bit 2)
    // Bit 1: SLEEP_DUR1 (Sleep duration bit 1)
    // Bit 0: SLEEP_DUR0 (Sleep duration bit 0)

    // Based on detecting door swing movements with an ODR of 12.5 Hz, 
    // here's the recommended setting for the WAKE_UP_DUR register:

    // Recommended value: 0x60 (binary: 01100000)
    // This sets:
    // FF_DUR5 = 0: Not using free-fall detection for door swing
    // WAKE_DUR[1:0] = 11: Set wake-up duration to maximum (3 ODR cycles = 240ms at 12.5Hz)
    // STATIONARY = 0: Stationary detection disabled (not needed for door swing)
    // SLEEP_DUR[3:0] = 0000: Sleep duration set to minimum

    // This configuration:
    // Maximizes the wake-up duration (WAKE_DUR = 11), which helps prevent missing the beginning
    // of a door swing movement
    // Disables stationary detection since we're focused on detecting motion rather than lack of motion
    // Sets sleep duration to minimum to ensure faster response to door movement
    // Doesn't use free-fall detection since it's not relevant for door swing monitoring

    // For door swing detection, the wake-up duration is particularly important as it determines
    // how quickly the device responds to the initial movement of the door. The maximum setting
    // (3 ODR cycles) gives you reliable detection without excessive power consumption.
    ret =   lis2dw12tr_i2c_write_reg(
                motion_sensor_i2c_dev,
                LIS2DW12TR_WAKE_UP_DUR_REG,
                (THREE_ODR_CYCLE << WAKE_DUR)
            );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_WAKE_UP_DUR_REG register\n");
        // skipped for testing
        // return;
    }


    // ====== LIS2DW12TR_CTRL7_REG Configuration ======
    // For the CTRL7 register (0x3F), the correct bit mapping is:
    // Bit 7: DRDY_PULSED (Data-ready pulsed configuration)
    // Bit 6: INT2_ON_INT1 (All interrupts to INT1 pin)
    // Bit 5: INTERRUPTS_ENABLE (Interrupts enable)
    // Bit 4: USR_OFF_ON_OUT (User offset on output registers)
    // Bit 3: USR_OFF_ON_WU (User offset on wake-up functions)
    // Bit 2: USR_OFF_W (Offset weight)
    // Bit 1: HP_REF_MODE (High-pass filter reference mode)
    // Bit 0: LPASS_ON6D (Low-pass filter on 6D function)

    // Recommended value: 0x20 (binary: 00100000)
    // This configuration:
    // 1. Enables interrupts (INTERRUPTS_ENABLE = 1), which is essential for the accelerometer
    //    to generate interrupt signals based on movement detection
    // 2. Uses latched data-ready mode (DRDY_PULSED = 0) to ensure the MCU can reliably detect the data-ready signal
    // 3. Keeps the normal interrupt routing rather than combining all interrupts to INT1
    // 4. Disables offset adjustments which are unnecessary for basic door swing detection
    // 5. Disables high-pass filter reference mode as we're using the bandwidth filter (BW_FILT) settings we discussed earlier
    // 6. Doesn't enable the low-pass filter for 6D detection as we're not using 6D orientation detection for door swing monitoring
    
    // This setting focuses on enabling the core interrupt functionality needed for door swing detection
    // while keeping other advanced features disabled to minimize power consumption and configuration complexity.
    ret =   lis2dw12tr_i2c_write_reg(
                motion_sensor_i2c_dev,
                LIS2DW12TR_CTRL7_REG,
                (1 << INTERRUPTS_ENABLE)
            );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_CTRL7_REG register\n");
        // skipped for testing
        // return;
    }
}

static void lis2dw12tr_init(void)
{
    printf("lis2dw12tr_init\n");

    if (interrupt_setup() != 0)
    {
        printf("Motion interrupt is not ready\n");

        return;
    }

    initial_register_setup();
}

static void lis2dw12tr_set_threshold(uint8_t threshold)
{
    int ret;

    printf("lis2dw12tr_set_threshold\n");
    if (threshold > MAX_WK_THS)
    {
        printf("Threshold value is out of range, set to MAX_WK_THS\n");
        threshold = MAX_WK_THS;
    }

    ret =   lis2dw12tr_i2c_write_reg(
                motion_sensor_i2c_dev,
                LIS2DW12TR_WAKE_UP_THS_REG,
                (threshold << WK_THS)
            );

    if (ret < 0)
    {
        printf("Failed to write LIS2DW12TR_WAKE_UP_THS_REG register\n");
        return;
    }
}

static bool lis2dw12tr_is_interrupt_triggered(void)
{
    printf("lis2dw12tr_is_interrupt_triggered\n");

    k_sem_take(&motion_int_sem, K_FOREVER);
    motion_int_processed_count++;

    // if we need to read and process acceleration data
    printf("Processing accelerometer data #%d\n", motion_int_processed_count);

    return true;
}

static void lis2dw12tr_clear_interrupt(void)
{
    printf("lis2dw12tr_clear_interrupt\n");
    // for now it is auto clear
    // LIR     : Latched interrupt disabled
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