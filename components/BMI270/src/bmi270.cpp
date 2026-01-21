#include "bmi270.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

// Use repo I2C abstraction
#include "lgfx.h"
#include "lgfx/v1/platforms/common.hpp"

using namespace lgfx;
using namespace lgfx::v1;

// Configurable I2C settings for CardPuter Adv
static const int BMI270_I2C_ADDR = 0x68;
static const int BMI270_I2C_PORT = 1; // repo commonly uses I2C_NUM_1 for m5stack
static const uint32_t BMI270_I2C_FREQ = 400000;

// NOTE: register constants below are typical but may require tuning
#define BMI270_REG_CHIP_ID     0x00
#define BMI270_WHOAMI_ID       0x24
#define BMI270_REG_CMD         0x7E
#define BMI270_CMD_SOFTRESET   0xB6

#define BMI270_REG_ACCEL_DATA  0x0A
#define BMI270_REG_GYRO_DATA   0x12

// Scale constants (tune after testing)
static const float ACCEL_LSB_PER_G = 16384.0f; // LSB per g for +/-2g (verify)
static const float GYRO_LSB_PER_DPS  = 131.0f; // LSB per deg/s for chosen range (verify)

static uint32_t last_ms = 0;
static float yaw_deg = 0.0f;
static float roll_offset = 0.0f;
static float yaw_offset = 0.0f;
static int inited = 0;

static bool i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (!lgfx::i2c::beginTransaction(BMI270_I2C_PORT, BMI270_I2C_ADDR, BMI270_I2C_FREQ, false).has_value()) return false;
    bool ok = !lgfx::i2c::writeBytes(BMI270_I2C_PORT, buf, 2).has_error();
    lgfx::i2c::endTransaction(BMI270_I2C_PORT).has_value();
    return ok;
}

static bool i2c_read_reg(uint8_t reg, uint8_t* dst, size_t len)
{
    if (!lgfx::i2c::beginTransaction(BMI270_I2C_PORT, BMI270_I2C_ADDR, BMI270_I2C_FREQ, false).has_value()) return false;
    uint8_t w = reg;
    bool ok = !lgfx::i2c::writeBytes(BMI270_I2C_PORT, &w, 1).has_error()
           && !lgfx::i2c::restart(BMI270_I2C_PORT, BMI270_I2C_ADDR, BMI270_I2C_FREQ, true).has_error()
           && !lgfx::i2c::readBytes(BMI270_I2C_PORT, dst, len).has_error();
    lgfx::i2c::endTransaction(BMI270_I2C_PORT).has_value();
    return ok;
}

int bmi270_init(void)
{
    uint8_t id = 0;
    if (!i2c_read_reg(BMI270_REG_CHIP_ID, &id, 1)) return 0;
    // Proceed even if WHOAMI differs; adjust constant if needed

    // Soft reset
    i2c_write_reg(BMI270_REG_CMD, BMI270_CMD_SOFTRESET);
    lgfx::delay(10);

    // TODO: full configuration (enable accel/gyro, set ODR/ranges) if required.

    last_ms = millis();
    yaw_deg = 0.0f;
    roll_offset = 0.0f;
    yaw_offset = 0.0f;
    inited = 1;
    return 1;
}

void bmi270_rezero(void)
{
    float r = 0.0f, y = 0.0f;
    if (bmi270_read_angles(&r, &y)) {
        roll_offset = r;
        yaw_offset = y;
    } else {
        roll_offset = 0.0f;
        yaw_offset = yaw_deg;
    }
}

int bmi270_read_angles(float *out_roll_deg, float *out_yaw_deg)
{
    if (!inited) return 0;

    uint8_t accbuf[6] = {0};
    uint8_t gyrbuf[6] = {0};

    bool ok_acc = i2c_read_reg(BMI270_REG_ACCEL_DATA, accbuf, sizeof(accbuf));
    bool ok_gyr = i2c_read_reg(BMI270_REG_GYRO_DATA, gyrbuf, sizeof(gyrbuf));

    if (!ok_acc && !ok_gyr) return 0;

    int16_t ax = (int16_t)((accbuf[1] << 8) | accbuf[0]);
    int16_t ay = (int16_t)((accbuf[3] << 8) | accbuf[2]);
    int16_t az = (int16_t)((accbuf[5] << 8) | accbuf[4]);

    int16_t gz = (int16_t)((gyrbuf[5] << 8) | gyrbuf[4]);

    float ax_g = (float)ax / ACCEL_LSB_PER_G;
    float az_g = (float)az / ACCEL_LSB_PER_G;

    float gz_dps = (float)gz / GYRO_LSB_PER_DPS;

    float roll_rad = atan2f(ax_g, az_g);
    float roll_deg = roll_rad * (180.0f / M_PI);

    uint32_t now = millis();
    float dt = (now > last_ms) ? ((now - last_ms) * 0.001f) : 0.0f;
    last_ms = now;

    yaw_deg += gz_dps * dt;

    float out_roll = roll_deg - roll_offset;
    float out_yaw = yaw_deg - yaw_offset;

    if (out_roll_deg) *out_roll_deg = out_roll;
    if (out_yaw_deg)  *out_yaw_deg  = out_yaw;

    return 1;
}