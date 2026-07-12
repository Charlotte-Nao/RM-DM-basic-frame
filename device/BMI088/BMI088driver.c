#include "BMI088driver.h"
#include "BMI088Middleware.h"

// 灵敏度系数，默认 3G 和 2000度/s
static fp32 accel_sen = BMI088_ACCEL_3G_SEN;
static fp32 gyro_sen = BMI088_GYRO_2000_SEN;

/* --- 内部 SPI 读写封装 --- */
static void accel_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
    BMI088_ACCEL_NS_H();
}

static void accel_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg | 0x80); // 读命令
    BMI088_read_write_byte(0x55);       // BMI088 加速度计读取第一个字节为 dummy byte
    while (len--) {
        *buf++ = BMI088_read_write_byte(0x55);
    }
    BMI088_ACCEL_NS_H();
}

static void gyro_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
    BMI088_GYRO_NS_H();
}

static void gyro_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(reg | 0x80);
    while (len--) {
        *buf++ = BMI088_read_write_byte(0x55);
    }
    BMI088_GYRO_NS_H();
}

/* --- 初始化流程 --- */
uint8_t BMI088_init(void) {
    uint8_t res;

    // 1. 硬件层初始化
    BMI088_GPIO_init();
    BMI088_com_init();

    // 2. 加速度计初始化
    accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(50);

    // 激活加速度计
    accel_write_single_reg(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE);
    accel_write_single_reg(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON);

    // 3. 陀螺仪初始化
    gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(50);
    gyro_write_single_reg(BMI088_GYRO_RANGE, BMI088_GYRO_2000); // 2000 deg/s
    gyro_write_single_reg(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | 0x80);

    return 0; // 简化版返回
}

/* --- 数据读取逻辑 --- */
void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate) {
    uint8_t buf[8];
    int16_t raw_data;

    // 读取加速度
    accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    accel[0] = ((int16_t)((buf[1] << 8) | buf[0])) * accel_sen;
    accel[1] = ((int16_t)((buf[3] << 8) | buf[2])) * accel_sen;
    accel[2] = ((int16_t)((buf[5] << 8) | buf[4])) * accel_sen;

    // 读取陀螺仪
    gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);
    gyro[0] = ((int16_t)((buf[1] << 8) | buf[0])) * gyro_sen;
    gyro[1] = ((int16_t)((buf[3] << 8) | buf[2])) * gyro_sen;
    gyro[2] = ((int16_t)((buf[5] << 8) | buf[4])) * gyro_sen;

    // 读取温度
    accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    raw_data = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw_data > 1023) raw_data -= 2048;
    *temperate = raw_data * 0.125f + 23.0f;
}