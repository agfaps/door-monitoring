#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "distance_sensor.h"
#include "vl53l3cx.h"

// Register init lists consist of the count followed by register/value pairs
unsigned char ucI2CMode[]     = {4, 0x88, 0x00, 0x80, 0x01, 0xff, 0x01, 0x00, 0x00};
unsigned char ucI2CMode2[]    = {3, 0x00, 0x01, 0xff, 0x00, 0x80, 0x00};
unsigned char ucSPAD0[]       = {4, 0x80, 0x01, 0xff, 0x01, 0x00, 0x00, 0xff, 0x06};
unsigned char ucSPAD1[]       = {5, 0xff, 0x07, 0x81, 0x01, 0x80, 0x01, 0x94, 0x6b, 0x83, 0x00};
unsigned char ucSPAD2[]       = {4, 0xff, 0x01, 0x00, 0x01, 0xff, 0x00, 0x80, 0x00};
unsigned char ucSPAD[]        = {5, 0xff, 0x01, 0x4f, 0x00, 0x4e, 0x2c, 0xff, 0x00, 0xb6, 0xb4};
unsigned char ucDefTuning[]   = {
    80,
    0xff, 0x01, 0x00, 0x00, 0xff, 0x00, 0x09, 0x00,
    0x10, 0x00, 0x11, 0x00, 0x24, 0x01, 0x25, 0xff,
    0x75, 0x00, 0xff, 0x01, 0x4e, 0x2c, 0x48, 0x00,
    0x30, 0x20, 0xff, 0x00, 0x30, 0x09, 0x54, 0x00,
    0x31, 0x04, 0x32, 0x03, 0x40, 0x83, 0x46, 0x25,
    0x60, 0x00, 0x27, 0x00, 0x50, 0x06, 0x51, 0x00,
    0x52, 0x96, 0x56, 0x08, 0x57, 0x30, 0x61, 0x00,
    0x62, 0x00, 0x64, 0x00, 0x65, 0x00, 0x66, 0xa0,
    0xff, 0x01, 0x22, 0x32, 0x47, 0x14, 0x49, 0xff,
    0x4a, 0x00, 0xff, 0x00, 0x7a, 0x0a, 0x7b, 0x00,
    0x78, 0x21, 0xff, 0x01, 0x23, 0x34, 0x42, 0x00,
    0x44, 0xff, 0x45, 0x26, 0x46, 0x05, 0x40, 0x40,
    0x0e, 0x06, 0x20, 0x1a, 0x43, 0x40, 0xff, 0x00,
    0x34, 0x03, 0x35, 0x44, 0xff, 0x01, 0x31, 0x04,
    0x4b, 0x09, 0x4c, 0x05, 0x4d, 0x04, 0xff, 0x00,
    0x44, 0x00, 0x45, 0x20, 0x47, 0x08, 0x48, 0x28,
    0x67, 0x00, 0x70, 0x04, 0x71, 0x01, 0x72, 0xfe,
    0x76, 0x00, 0x77, 0x00, 0xff, 0x01, 0x0d, 0x01,
    0xff, 0x00, 0x80, 0x01, 0x01, 0xf8, 0xff, 0x01,
    0x8e, 0x01, 0x00, 0x01, 0xff, 0x00, 0x80, 0x00 };

#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

static vl53l3cx_data_t *vl53l3cx_data;

// for debugging purposes
static volatile uint32_t distance_int_count             = 0;
static volatile uint32_t distance_int_processed_count = 0;
static volatile uint32_t distance_int_missed_count    = 0;

static unsigned char stop_variable;
static uint32_t measurement_timing_budget_us;

typedef enum vcselperiodtype
{
    VcselPeriodPreRange,
    VcselPeriodFinalRange
} vcselPeriodType;

typedef struct tagSequenceStepTimeouts
{
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,     pre_range_us,     final_range_us;
} SequenceStepTimeouts;

static int performSingleRefCalibration(uint8_t vhv_init_byte);
static int setMeasurementTimingBudget(uint32_t budget_us);
static bool vl53l3cx_is_ready(void);

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


/**
 * @brief Interrupt callback for the VL53L3CX distance sensor.
 *
 * This function is triggered when an interrupt occurs on the distance sensor.
 * It increments the interrupt count and checks the semaphore count to determine
 * if any interrupt data was missed. If data was missed, it increments the missed
 * count and logs a message indicating that the previous data was skipped.
 * Otherwise, it logs the interrupt count and gives the semaphore for processing
 * the interrupt data.
 *
 * @param dev The device structure for the distance sensor.
 * @param cb The GPIO callback structure.
 * @param pins The pin mask for the triggered interrupt.
 */
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

/**
 * @brief Sets up the interrupt for the VL53L3CX distance sensor.
 *
 * Configures the GPIO pin for the distance sensor interrupt, enabling edge-to-active 
 * interrupt. If the device or pin is not ready, or if the configuration fails, 
 * appropriate error messages are printed and the function returns -1.
 * Upon successful setup, the interrupt callback is initialized and added, and a 
 * confirmation message is printed.
 *
 * @return 0 on successful setup, -1 on failure.
 */
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

    // printf("Distance interrupt is ready\n");

    return 0;
}

/**
 * @brief Reads an 8-bit value from a register address of the VL53L3CX distance sensor.
 *
 * This function sends a 16-bit register address (MSB first) and then reads the 8-bit value
 * from the same address. The register address is sent as a write message, and the value
 * is read as a read message with the RESTART flag set.
 *
 * @param reg_addr The 16-bit register address to read from.
 * @param value The 8-bit value read from the register address.
 *
 * @return 0 if successful, negative errno otherwise.
 */
static int vl53l3cx_i2c_reg_read_8(uint16_t reg_addr, uint8_t *value)
{
    struct i2c_msg msgs[2];
    uint8_t reg_buf[2];

    // Convert 16-bit register address to 2 bytes (MSB first)
    reg_buf[0] = (reg_addr >> 8) & 0xFF;   // High byte
    reg_buf[1] = reg_addr & 0xFF;          // Low byte

    // First message: send 16-bit register address
    msgs[0].buf   = reg_buf;
    msgs[0].len   = 2;
    msgs[0].flags = I2C_MSG_WRITE;

    // Second message: read 8-bit data
    msgs[1].buf   = value;
    msgs[1].len   = 1;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;

    return i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 2, VL53L3CX_I2C_ADDR);
}

/**
 * @brief Reads an 8-bit value from a register address of the VL53L3CX distance sensor
 * and returns it directly.
 *
 * This function sends a 16-bit register address (MSB first) and then reads the 8-bit value
 * from the same address. The register address is sent as a write message with the STOP flag set,
 * and the value is read as a read message with the STOP flag set as well.
 *
 * @param reg_addr The 16-bit register address to read from.
 *
 * @return The 8-bit value read from the register if successful, or a negative errno otherwise.
 */

static int vl53l3cx_i2c_reg_read_8_ret(uint16_t reg_addr)
{
    struct i2c_msg msgs[2];
    uint8_t reg_buf[2];
    uint8_t value;

    // Convert 16-bit register address to 2 bytes (MSB first)
    reg_buf[0] = (reg_addr >> 8) & 0xFF;   // High byte
    reg_buf[1] = reg_addr & 0xFF;         // Low byte

    // First message: send 16-bit register address
    msgs[0].buf = reg_buf;
    msgs[0].len = 2;
    msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP; // Add STOP

    // Second message: read 8-bit data
    msgs[1].buf = &value;
    msgs[1].len = 1;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP; // Add STOP, and remove RESTART.

    int ret = i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 2, VL53L3CX_I2C_ADDR);

    if (ret == 0)
    {
        return value; // Return the read value
    }
    else
    {
        return ret; // Return the error code
    }
}

/**
 * @brief Reads a 16-bit value from a register address of the VL53L3CX distance sensor.
 *
 * This function sends a 16-bit register address (MSB first) and then reads the 16-bit value
 * from the same address. The register address is sent as a write message, and the value
 * is read as a read message with the RESTART flag set.
 *
 * @param reg_addr The 16-bit register address to read from.
 * @param data The 16-bit value read from the register address.
 *
 * @return 0 if successful, negative errno otherwise.
 */
// static int vl53l3cx_i2c_reg_read_16(uint16_t reg_addr, uint16_t *data)
// {
//     struct i2c_msg msgs[2];
//     uint8_t reg_buf[2];
//     uint8_t data_buf[2];

//     // Convert 16-bit register address to 2 bytes (MSB first)
//     reg_buf[0] = (reg_addr >> 8) & 0xFF;   // High byte
//     reg_buf[1] = reg_addr & 0xFF;          // Low byte

//     // First message: send 16-bit register address
//     msgs[0].buf   = reg_buf;
//     msgs[0].len   = 2;
//     msgs[0].flags = I2C_MSG_WRITE;

//     // Second message: read 16-bit data
//     msgs[1].buf   = data_buf;
//     msgs[1].len   = 2;
//     msgs[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;

//     int ret = i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 2, VL53L3CX_I2C_ADDR);

//     if (ret == 0)
//     {
//         // Combine bytes (big-endian)
//         *data = (data_buf[0] << 8) | data_buf[1];
//     }

//     return ret;
// }

// NOTE: legacy example for reference
// static int vl53l3cx_i2c_reg_write_8(uint16_t reg_addr, uint8_t value)
// {
//     uint8_t buf[3] = {0};

//     buf[0] = (reg_addr >> 8) & 0xFF;
//     buf[1] = reg_addr & 0xFF;
//     buf[2] = value;

//     return i2c_write(vl53l3cx_data->i2c_dev, buf, 3, VL53L3CX_I2C_ADDR);
// }

/**
 * @brief Writes an 8-bit value to a register address of the VL53L3CX distance sensor.
 *
 * This function sends a 16-bit register address (MSB first) and then writes the 8-bit value
 * to the same address. The register address is sent as a write message, and the value
 * is written as a write message with the STOP flag set.
 *
 * @param reg_addr The 16-bit register address to write to.
 * @param value The 8-bit value to write to the register address.
 *
 * @return 0 if successful, negative errno otherwise.
 */
// static int vl53l3cx_i2c_reg_write_8(uint16_t reg_addr, uint8_t value)
// {
//     uint8_t buf[3];

//     buf[0] = (reg_addr >> 8) & 0xFF;
//     buf[1] = reg_addr & 0xFF;
//     buf[2] = value;

//     struct i2c_msg msgs[] =
//     {
//         {
//             .buf   = buf,
//             .len   = sizeof(buf),
//             .flags = I2C_MSG_WRITE | I2C_MSG_STOP,
//         },
//     };

//     return i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 1, VL53L3CX_I2C_ADDR);
// }

static int vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(uint8_t reg_addr, uint8_t value) {
    uint8_t buf[2];

    buf[0] = reg_addr;
    buf[1] = value;

    struct i2c_msg msgs[] = {
        {
            .buf = buf,
            .len = sizeof(buf),
            .flags = I2C_MSG_WRITE | I2C_MSG_STOP,
        },
    };

    return i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 1, VL53L3CX_I2C_ADDR);
}

static int vl53l3cx_i2c_reg_write_16bit_val_8bit_addr(uint8_t reg_addr, uint16_t value)
{
    uint8_t buf[3];

    buf[0] = reg_addr;
    buf[1] = (value >> 8) & 0xFF; // MSB of 16-bit value
    buf[2] = value & 0xFF;        // LSB of 16-bit value

    struct i2c_msg msgs[] = {
        {
            .buf = buf,
            .len = sizeof(buf),
            .flags = I2C_MSG_WRITE | I2C_MSG_STOP,
        },
    };

    return i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 1, VL53L3CX_I2C_ADDR);
}

static int writeReg16(unsigned char ucAddr, unsigned short usValue) {
    unsigned char ucTemp[3];
    struct i2c_msg msgs[1];
    int ret;

    ucTemp[0] = ucAddr;
    ucTemp[1] = (unsigned char)(usValue >> 8); // MSB first
    ucTemp[2] = (unsigned char)usValue;

    msgs[0].buf = ucTemp;
    msgs[0].len = 3;
    msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    ret = i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 1, VL53L3CX_I2C_ADDR);

    return ret; // Return the result of i2c_transfer (0 for success, negative for error).
}

static int writeRegList(unsigned char *ucList)
{
    unsigned char ucCount = *ucList++; // count is the first element in the list
    int ret;
    struct i2c_msg msg;
    unsigned char buf[2];

    while (ucCount)
    {
        buf[0] = ucList[0]; // address
        buf[1] = ucList[1]; // value

        msg.buf = buf;
        msg.len = 2;
        msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

        ret = i2c_transfer(vl53l3cx_data->i2c_dev, &msg, 1, VL53L3CX_I2C_ADDR);
        if (ret < 0)
        {
            printf("I2C writeRegList failed: %d", ret);

            return -1; // Indicate error
        }

        ucList += 2;
        ucCount--;
    }

    return 0; // Indicate success
}

static int writeMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount)
{
    unsigned char ucTemp[17]; // Increased size for address and data
    struct i2c_msg msgs[1];
    int ret;

    if (iCount > 16) {
        return -1; // Error: Data too large
    }

    ucTemp[0] = ucAddr;
    memcpy(&ucTemp[1], pBuf, iCount);

    msgs[0].buf = ucTemp;
    msgs[0].len = iCount + 1; // Address + data
    msgs[0].flags = I2C_MSG_WRITE;

    ret = i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 1, VL53L3CX_I2C_ADDR);

    if (ret < 0) {
        return ret; // Return the error code from i2c_transfer
    }

    return 0; // Success
}

static unsigned short readReg16(unsigned char ucAddr)
{
    struct i2c_msg msgs[2];
    uint8_t data_buf[2];
    unsigned short data = 0;

    // First message: send 8-bit register address
    msgs[0].buf = &ucAddr;
    msgs[0].len = 1;
    msgs[0].flags = I2C_MSG_WRITE;

    // Second message: read 16-bit data
    msgs[1].buf = data_buf;
    msgs[1].len = 2;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    int ret = i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 2, VL53L3CX_I2C_ADDR);

    if (ret == 0)
    {
        // Combine bytes (big-endian)
        data = (data_buf[0] << 8) | data_buf[1];
    }

    return data;
}

static int readMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount)
{
    struct i2c_msg msgs[2];
    int ret;

    msgs[0].buf = &ucAddr;
    msgs[0].len = 1;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = pBuf;
    msgs[1].len = iCount;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    ret = i2c_transfer(vl53l3cx_data->i2c_dev, msgs, 2, VL53L3CX_I2C_ADDR);

    if (ret < 0)
    {
        printf("I2C readMulti failed: %d", ret);
        return -1; // Indicate error
    }

    return 0; // Indicate success
}

static int getSpadInfo(unsigned char *pCount, unsigned char *pTypeIsAperture)
{
    int iTimeout;
    unsigned char ucTemp;
    #define MAX_TIMEOUT 50

    writeRegList(ucSPAD0); // Write a list of register/value pairs to configure SPAD (Single Photon Avalanche Diode) settings.

    // Modify register 0x83.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x83, vl53l3cx_i2c_reg_read_8_ret(0x83) | 0x04);
    // Reads the current value of register 0x83, sets bit 2 (0x04), and writes the modified value back.
    // This bit likely enables a specific mode for SPAD information retrieval.

    writeRegList(ucSPAD1); // Write another list of register/value pairs to configure SPAD settings.

    iTimeout = 0;
    while(iTimeout < MAX_TIMEOUT) // Loop with a timeout.
    {
        if (vl53l3cx_i2c_reg_read_8_ret(0x83) != 0x00) break; // Read register 0x83. If it's not 0, break the loop.
        // This likely waits for a specific status flag to indicate SPAD information readiness.
        iTimeout++;
        k_msleep(5); // Wait for 5 milliseconds. (k_msleep suggests this is for Zephyr RTOS or similar)
    }
    if (iTimeout == MAX_TIMEOUT) // Check for timeout.
    {
        // printf("Timeout while waiting for SPAD info\n");
        return 0; // Return 0 indicating failure.
    }

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x83,0x01); // Write 0x01 to register 0x83.
    // This likely sets a flag to indicate the start of SPAD data retrieval.

    ucTemp = vl53l3cx_i2c_reg_read_8_ret(0x92); // Read the SPAD information from register 0x92.

    *pCount = (ucTemp & 0x7f); // Extract the SPAD count (lower 7 bits) and store it in *pCount.
    *pTypeIsAperture = (ucTemp & 0x80); // Extract the SPAD type (bit 7) and store it in *pTypeIsAperture.
    // Bit 7 indicates if the SPAD type is an aperture SPAD.

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x81,0x00); // Write 0x00 to register 0x81.
    // This likely disables or resets a specific SPAD configuration.

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xff,0x06); // Write 0x06 to register 0xFF.
    // This likely selects a specific register bank or page within the VL53L0X.

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x83, vl53l3cx_i2c_reg_read_8_ret(0x83) & ~0x04);
    // Reads register 0x83, clears bit 2 (0x04), and writes the modified value back.
    // This likely reverts the change made earlier to enable SPAD information retrieval.

    writeRegList(ucSPAD2); // Write another list of register/value pairs to configure SPAD settings.

    return 1; // Return 1 indicating success.
}

static int performSingleRefCalibration(uint8_t vhv_init_byte)
{
    int iTimeout;
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    iTimeout = 0;
    while ((vl53l3cx_i2c_reg_read_8_ret(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        iTimeout++;
        // usleep(5000);
        k_msleep(5);
        if (iTimeout > 100) { return 0; }
    }

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_INTERRUPT_CLEAR, 0x01);

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSRANGE_START, 0x00);

    return 1;
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static void getSequenceStepTimeouts(uint8_t enables, SequenceStepTimeouts * timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = ((vl53l3cx_i2c_reg_read_8_ret(PRE_RANGE_CONFIG_VCSEL_PERIOD) +1) << 1);

    timeouts->msrc_dss_tcc_mclks = vl53l3cx_i2c_reg_read_8_ret(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks =
    decodeTimeout(readReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = ((vl53l3cx_i2c_reg_read_8_ret(FINAL_RANGE_CONFIG_VCSEL_PERIOD) +1) << 1);

    timeouts->final_range_mclks =
    decodeTimeout(readReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                timeouts->final_range_vcsel_period_pclks);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
static int setMeasurementTimingBudget(uint32_t budget_us)
{
    uint32_t used_budget_us;
    uint32_t final_range_timeout_us;
    uint16_t final_range_timeout_mclks;

    uint8_t enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) { return 0; }

    used_budget_us = StartOverhead + EndOverhead;

    enables = vl53l3cx_i2c_reg_read_8_ret(SYSTEM_SEQUENCE_CONFIG);
    getSequenceStepTimeouts(enables, &timeouts);

    if (enables & SEQUENCE_ENABLE_TCC)
    {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables & SEQUENCE_ENABLE_DSS)
    {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables & SEQUENCE_ENABLE_MSRC)
    {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables & SEQUENCE_ENABLE_FINAL_RANGE)
    {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
        // "Requested timeout too big."
        return 0;
    }

    final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    final_range_timeout_mclks =
        timeoutMicrosecondsToMclks(final_range_timeout_us,
                                    timeouts.final_range_vcsel_period_pclks);

    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
        final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
    }
    return 1;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
static int setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    uint8_t enables;
    SequenceStepTimeouts timeouts;

    enables = vl53l3cx_i2c_reg_read_8_ret(SYSTEM_SEQUENCE_CONFIG);
    getSequenceStepTimeouts(enables, &timeouts);

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."


    if (type == VcselPeriodPreRange)
    {
    // "Set phase check limits"
    switch (period_pclks)
    {
        case 12:
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

        case 14:
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

        case 16:
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

        case 18:
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

        default:
        // invalid period
        return 0;
    }
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
        timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
        timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(MSRC_CONFIG_TIMEOUT_MACROP,
        (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
    }
    else if (type == VcselPeriodFinalRange)
    {
    switch (period_pclks)
    {
        case 8:
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x01);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(ALGO_PHASECAL_LIM, 0x30);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x00);
        break;

        case 10:
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x01);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(ALGO_PHASECAL_LIM, 0x20);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x00);
        break;

        case 12:
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x01);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(ALGO_PHASECAL_LIM, 0x20);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x00);
        break;

        case 14:
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x01);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(ALGO_PHASECAL_LIM, 0x20);
        vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x00);
        break;

        default:
        // invalid period
        return 0;
    }

    // apply new VCSEL period
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
        timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
        new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
    }
    else
    {
    // invalid type
    return 0;
    }

    // "Finally, the timing budget must be re-applied"

    setMeasurementTimingBudget(measurement_timing_budget_us);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    uint8_t sequence_config = vl53l3cx_i2c_reg_read_8_ret(SYSTEM_SEQUENCE_CONFIG);
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_SEQUENCE_CONFIG, 0x02);
    performSingleRefCalibration(0x0);
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return 1;
}

static uint32_t getMeasurementTimingBudget(void)
{
    uint8_t enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    enables = vl53l3cx_i2c_reg_read_8_ret(SYSTEM_SEQUENCE_CONFIG);
    getSequenceStepTimeouts(enables, &timeouts);

    if (enables & SEQUENCE_ENABLE_TCC)
    {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables & SEQUENCE_ENABLE_DSS)
    {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables & SEQUENCE_ENABLE_MSRC)
    {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables & SEQUENCE_ENABLE_FINAL_RANGE)
    {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    measurement_timing_budget_us = budget_us; // store for internal reuse
    return budget_us;
}

static int initSensor(int bLongRangeMode)
{
    unsigned char spad_count = 0;
    unsigned char spad_type_is_aperture = 0;
    unsigned char ref_spad_map[6];
    unsigned char ucFirstSPAD;
    unsigned char ucSPADsEnabled;
    int i;

    // Set 2.8V mode for VL53L3CX
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(
        VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, // Address of the VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV register
        (vl53l3cx_i2c_reg_read_8_ret(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) & 0xFB) | 0x01 // Reads the current value, clears bit 2, sets bit 0.
    ); // Writes the modified value back to the register. This configures the voltage regulator.

    // Set I2C standard mode
    writeRegList(ucI2CMode); // Writes a series of register/value pairs to configure I2C mode.
    stop_variable = vl53l3cx_i2c_reg_read_8_ret(0x91); // Reads the value of register 0x91, storing it in stop_variable.
    writeRegList(ucI2CMode2); // Writes another series of register/value pairs, further configuring I2C.
    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(REG_MSRC_CONFIG_CONTROL, vl53l3cx_i2c_reg_read_8_ret(REG_MSRC_CONFIG_CONTROL) | 0x12); // Reads the current value, sets bits 1 and 4, and writes it back to disable signal rate limits.
    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    vl53l3cx_i2c_reg_write_16bit_val_8bit_addr(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32); // Writes 32 (0.25 in Q9.7 format) to the minimum count rate limit for final range measurements.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_SEQUENCE_CONFIG, 0xFF); // Sets the system sequence configuration to enable all measurement steps.
    getSpadInfo(&spad_count, &spad_type_is_aperture); // Retrieves SPAD (Single Photon Avalanche Diode) information.

    readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6); // Reads 6 bytes of SPAD enable data into ref_spad_map.
    writeRegList(ucSPAD); // Writes register/value pairs related to SPAD configuration.
    ucFirstSPAD = (spad_type_is_aperture) ? 12 : 0; // Determines the first enabled SPAD based on its type.
    ucSPADsEnabled = 0; // Initializes the count of enabled SPADs.

    // clear bits for unused SPADs
    for (i = 0; i < 48; i++)
    {
        if ((i < ucFirstSPAD) || (ucSPADsEnabled == spad_count))
        {
            ref_spad_map[i>>3] &= ~(1<<(i & 7)); // Clears bits in ref_spad_map for unused SPADs.
        }
        else if (ref_spad_map[i>>3] & (1<< (i & 7)))
        {
            ucSPADsEnabled++; // Increments the count of enabled SPADs.
        }
    }
    writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6); // Writes the modified SPAD enable data back to the device.

    // load default tuning settings
    writeRegList(ucDefTuning); // Writes a long list of register/value pairs to load default tuning settings.

    // change some settings for long range mode
    if (bLongRangeMode)
    {
        vl53l3cx_i2c_reg_write_16bit_val_8bit_addr(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 13); // Writes 13 (0.1 in Q9.7 format) for long-range mode.
        setVcselPulsePeriod(VcselPeriodPreRange, 18); // Sets the VCSEL pulse period for pre-range measurements.
        setVcselPulsePeriod(VcselPeriodFinalRange, 14); // Sets the VCSEL pulse period for final range measurements.
    }

    // set interrupt configuration to "new sample ready", already interrupt based
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04); // Configures the interrupt to trigger when a new sample is ready.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(GPIO_HV_MUX_ACTIVE_HIGH, vl53l3cx_i2c_reg_read_8_ret(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // Configures the interrupt pin to be active low.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_INTERRUPT_CLEAR, 0x01);     // Clears any pending interrupts.

    measurement_timing_budget_us = getMeasurementTimingBudget(); // Retrieves the measurement timing budget.

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_SEQUENCE_CONFIG, 0xe8); // Sets the system sequence configuration.
    setMeasurementTimingBudget(measurement_timing_budget_us); // Applies the measurement timing budget.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_SEQUENCE_CONFIG, 0x01); // Sets the system sequence configuration.

    if (!performSingleRefCalibration(0x40)) { return 0; } // Performs a single reference calibration.

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_SEQUENCE_CONFIG, 0x02); // Sets the system sequence configuration.

    if (!performSingleRefCalibration(0x00)) { return 0; } // Performs another single reference calibration.

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_SEQUENCE_CONFIG, 0xe8); // Sets the system sequence configuration.

    return 1;
}

uint16_t readRangeContinuousMillimeters(void)
{
    int iTimeout = 0;
    uint16_t range;

    // Polling loop: Wait for a new range measurement to be ready.
    while ((vl53l3cx_i2c_reg_read_8_ret(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        iTimeout++; // Increment timeout counter.

        k_msleep(5);  // Wait for 5 milliseconds.

        if (iTimeout > 50) // Check for timeout (250 milliseconds).
        {
            return -1; // Return -1 indicating a timeout error.
        }
    }

    // Read the range result from the device.
    // Assumptions: Linearity Corrective Gain is 1000 (default); fractional ranging is not enabled.
    range = readReg16(RESULT_RANGE_STATUS + 10); // Read 16-bit range data from RESULT_RANGE_STATUS + 10.

    // Clear the interrupt status.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSTEM_INTERRUPT_CLEAR, 0x01); // Write 0x01 to clear the interrupt.

    return range; // Return the read range value.
}

// Read the current distance in mm.
int tofReadDistance(void)
{
    // Start a single measurement sequence.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x80, 0x01); // Write 0x01 to register 0x80.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x01); // Write 0x01 to register 0xFF.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x00, 0x00); // Write 0x00 to register 0x00.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x91, stop_variable); // Write the previously read 'stop_variable' to register 0x91.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x00, 0x01); // Write 0x01 to register 0x00.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0xFF, 0x00); // Write 0x00 to register 0xFF.
    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(0x80, 0x00); // Write 0x00 to register 0x80.
    // These writes set up the device for a single ranging measurement.

    vl53l3cx_i2c_reg_write_8bit_addr_8bit_val(SYSRANGE_START, 0x01); // Start the ranging measurement by writing 0x01 to SYSRANGE_START.

    // Wait for the device to indicate that the ranging measurement is ready.
    if (vl53l3cx_is_ready())
    {
        // printf("Distance data ready\n");
        return readRangeContinuousMillimeters(); // Read and return the range result.
    }
    else
    {
        return -1;
    }

    // NOTE: reference for polling status and not using interrupt notification
    // Polling loop: Wait for the SYSRANGE_START bit to clear, indicating measurement completion.
    // int iTimeout = 0;

    // while (vl53l3cx_i2c_reg_read_8_ret(SYSRANGE_START) & 0x01)
    // {
    //     iTimeout++; // Increment timeout counter.

    //     usleep(5000); // Wait for 5 milliseconds.

    //     if (iTimeout > 50) // Check for timeout (250 milliseconds).
    //     {
    //         return -1; // Return -1 indicating a timeout error.
    //     }
    // }
}

static void initial_register_setup(void)
{
    int ret;
    int bLongRange;
    uint8_t who_am_i;

    // printf("initial_register_setup\n");

    // check who am i register
    ret = vl53l3cx_i2c_reg_read_8(VL53L3CX_REG_WHO_AM_I, &who_am_i);

    if (
        (ret < 0) ||
        (who_am_i != VL53L3CX_REG_WHO_AM_I_VAL)
    )
    {
        printf("VL53L3CX not detected\n");
        return;
    }

    bLongRange = 1;
    initSensor(bLongRange);
}

/**
 * @brief Initializes the VL53L3CX distance sensor.
 *
 * This function initializes the distance sensor by creating a new instance of the
 * VL53L3CX data structure, configuring the GPIO pin for the distance sensor XSHUT,
 * setting up the interrupt, and performing the initial register setup.
 *
 * @return N/A
 */
static void vl53l3cx_init(void)
{
    int ret_value = 0;

    // printf("vl53l3cx_init\n");

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

/**
 * @brief Checks if the VL53L3CX distance sensor is active.
 *
 * This function returns true if the distance sensor is active and false otherwise.
 *
 * @return true if the distance sensor is active, false otherwise.
 */
static bool vl53l3cx_is_active(void)
{
    // printf("vl53l3cx_is_active\n");

    return vl53l3cx_data->sensor_active;
}

/**
 * @brief Activates the VL53L3CX distance sensor.
 *
 * This function enables the VL53L3CX distance sensor by setting the XSHUT pin
 * high (active low). After enabling the sensor, it waits for the tBOOT (FWBOOT)
 * time before setting the sensor active state to true.
 */
static void vl53l3cx_activate(void)
{
    int ret = 0;

    // printf("vl53l3cx_activate\n");

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
    int ret = 1;

    // check interrupt and if there is interrupt, read the distance data
    // ret = k_sem_take(&vl53l3cx_data->distance_int_sem, K_FOREVER);
    ret = k_sem_take(&vl53l3cx_data->distance_int_sem, K_MSEC(100));

    if (ret == 0)
    {
        distance_int_processed_count++;
        vl53l3cx_data->sensor_ready = true; 
    }
    else
    {
        vl53l3cx_data->sensor_ready = false;
    }

    return vl53l3cx_data->sensor_ready;
}

/**
 * @brief Retrieves the last measured distance in millimeters.
 *
 * This function returns the distance measured by the VL53L3CX sensor.
 * After fetching the distance, it marks the sensor as not ready.
 *
 * @return The last measured distance in millimeters.
 */
static uint32_t vl53l3cx_get_distance_mm(void)
{
    // printf("vl53l3cx_get\n");

    vl53l3cx_data->distance_mm = tofReadDistance();

    if (vl53l3cx_data->distance_mm >= 4096)
    {
        printf("Distance not valid\n");
    }

    // after each distance read, set sensor ready state to false
    vl53l3cx_data->sensor_ready = false;

    // give 50 ms delay before next read
    k_msleep(50);

    return vl53l3cx_data->distance_mm;
}

/**
 * @brief Deactivates the VL53L3CX distance sensor.
 *
 * This function drives the XSHUT pin low to disable the VL53L3CX sensor,
 * effectively deactivating it. The function logs an error message if
 * it fails to disable the sensor, and updates the sensor_active state 
 * to false upon successful deactivation.
 */
static void vl53l3cx_deactivate(void)
{
    int ret = 0;
    // printf("vl53l3cx_deactivate\n");
    
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

/**
 * @brief Retrieves the distance sensor API.
 *
 * This function returns a pointer to the distance sensor API, which provides
 * function pointers for initializing, activating, deactivating, checking the 
 * active and ready status, and getting the distance measurement from the VL53L3CX 
 * distance sensor.
 *
 * @return A constant pointer to the distance_sensor_api_t structure containing
 *         the API functions for the distance sensor.
 */
const distance_sensor_api_t *distance_sensor_get_api(void)
{
    return &vl53l3cx_api;
}