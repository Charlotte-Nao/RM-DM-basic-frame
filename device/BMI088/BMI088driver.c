#include "BMI088driver.h"
#include "BMI088Middleware.h"


static fp32 accel_sen = BMI088_ACCEL_3G_SEN;
static fp32 gyro_sen = BMI088_GYRO_2000_SEN;


/* 加速度计写一个寄存器 */
static void accel_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_ACCEL_NS_L();

    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);

    BMI088_ACCEL_NS_H();

    /*
     * BMI088 连续写寄存器之间需要留出间隔。
     */
    BMI088_delay_us(2U);
}


/* 加速度计连续读取 */
static void accel_read_muli_reg(uint8_t reg,
                                uint8_t *buf,
                                uint8_t len)
{
    BMI088_ACCEL_NS_L();

    BMI088_read_write_byte(reg | 0x80U);

    /*
     * 加速度计读取的第一个返回字节无效。
     */
    BMI088_read_write_byte(0x55U);

    while (len > 0U)
    {
        *buf = BMI088_read_write_byte(0x55U);

        buf++;
        len--;
    }

    BMI088_ACCEL_NS_H();

    BMI088_delay_us(2U);
}


/* 加速度计单寄存器读取 */
static uint8_t accel_read_single_reg(uint8_t reg)
{
    uint8_t data = 0U;

    accel_read_muli_reg(reg, &data, 1U);

    return data;
}


/* 陀螺仪写一个寄存器 */
static void gyro_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_GYRO_NS_L();

    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);

    BMI088_GYRO_NS_H();

    BMI088_delay_us(2U);
}


/* 陀螺仪连续读取 */
static void gyro_read_muli_reg(uint8_t reg,
                               uint8_t *buf,
                               uint8_t len)
{
    BMI088_GYRO_NS_L();

    BMI088_read_write_byte(reg | 0x80U);

    /*
     * 陀螺仪读取没有加速度计的额外 dummy byte。
     */
    while (len > 0U)
    {
        *buf = BMI088_read_write_byte(0x55U);

        buf++;
        len--;
    }

    BMI088_GYRO_NS_H();

    BMI088_delay_us(2U);
}


/* 陀螺仪单寄存器读取 */
static uint8_t gyro_read_single_reg(uint8_t reg)
{
    uint8_t data = 0U;

    gyro_read_muli_reg(reg, &data, 1U);

    return data;
}

/* --- 初始化流程 --- */
uint8_t BMI088_init(void)
{
    uint8_t accel_id;
    uint8_t gyro_id;

    BMI088_GPIO_init();
    BMI088_com_init();

    /*
     * 上电后先保持两个片选为高。
     */
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();

    BMI088_delay_ms(10U);

    /*
     * BMI088加速度计上电后需要通过一次SPI访问，
     * 从默认接口状态切换到SPI模式。
     *
     * 第一次读取结果丢弃。
     */
    (void)accel_read_single_reg(BMI088_ACC_CHIP_ID);

    BMI088_delay_ms(1U);

    /*
     * 加速度计软件复位。
     */
    accel_write_single_reg(BMI088_ACC_SOFTRESET,
                           BMI088_ACC_SOFTRESET_VALUE);

    BMI088_delay_ms(50U);

    /*
     * 软复位后重新进行一次SPI切换访问。
     */
    (void)accel_read_single_reg(BMI088_ACC_CHIP_ID);

    BMI088_delay_ms(1U);

    /*
     * 读取加速度计ID。
     * 正确值应该是0x1E。
     */
    accel_id =
        accel_read_single_reg(BMI088_ACC_CHIP_ID);

    if (accel_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return 1U;
    }

    /*
     * 打开加速度计。
     */
    accel_write_single_reg(BMI088_ACC_PWR_CTRL,
                           BMI088_ACC_ENABLE_ACC_ON);

    BMI088_delay_ms(1U);

    /*
     * 退出低功耗，进入工作模式。
     */
    accel_write_single_reg(BMI088_ACC_PWR_CONF,
                           BMI088_ACC_PWR_ACTIVE_MODE);

    BMI088_delay_ms(1U);

    /*
     * 加速度计配置：
     *
     * Normal滤波
     * 800Hz输出数据率
     */
    accel_write_single_reg(
        BMI088_ACC_CONF,
        BMI088_ACC_CONF_MUST_Set |
        BMI088_ACC_NORMAL |
        BMI088_ACC_800_HZ);

    BMI088_delay_ms(1U);

    /*
     * 设置为±3g。
     *
     * 必须和BMI088_ACCEL_3G_SEN对应。
     */
    accel_write_single_reg(BMI088_ACC_RANGE,
                           BMI088_ACC_RANGE_3G);

    BMI088_delay_ms(1U);

    /*
     * 陀螺仪软件复位。
     */
    gyro_write_single_reg(BMI088_GYRO_SOFTRESET,
                          BMI088_GYRO_SOFTRESET_VALUE);

    BMI088_delay_ms(50U);

    /*
     * 读取陀螺仪ID。
     * 正确值应该是0x0F。
     */
    gyro_id =
        gyro_read_single_reg(BMI088_GYRO_CHIP_ID);

    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return 2U;
    }

    /*
     * 设置陀螺仪量程为±2000°/s。
     */
    gyro_write_single_reg(BMI088_GYRO_RANGE,
                          BMI088_GYRO_2000);

    BMI088_delay_ms(1U);

    /*
     * 设置陀螺仪：
     *
     * ODR = 1000Hz
     * 带宽 = 116Hz
     */
    gyro_write_single_reg(
        BMI088_GYRO_BANDWIDTH,
        BMI088_GYRO_BANDWIDTH_MUST_Set |
        BMI088_GYRO_1000_116_HZ);

    BMI088_delay_ms(1U);

    return 0U;
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