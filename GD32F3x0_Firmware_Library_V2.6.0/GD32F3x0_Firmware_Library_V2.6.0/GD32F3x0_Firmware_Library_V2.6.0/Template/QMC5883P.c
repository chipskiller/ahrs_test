#include "QMC5883P.h"

HAL_StatusTypeDef QMC5883P_Transmit(struct QMC5883P_Data *QMC5883P_Data, uint8_t reg, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef res;
    res = HAL_I2C_Mem_Write(QMC5883P_Data->hi2c, QMC5883P_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
    return res;
}

HAL_StatusTypeDef QMC5883P_Receive(struct QMC5883P_Data *QMC5883P_Data, uint8_t reg, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef res;
    res = HAL_I2C_Mem_Read(QMC5883P_Data->hi2c, QMC5883P_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
    return res;
}

/*brief  QMC5883P初始化
 * @param  hi2c: I2C句柄指针
 * @retval 0:成功, 1:失败
 */
uint8_t QMC5883P_Init(struct QMC5883P_Data *QMC5883P_Data)
{
    uint8_t data;

    HAL_Delay(100);
    /*
     * Step 1 — 软复位（Software Reset）
     *   写 0x80 到 Control Register 2 (0x0A) 的 bit7=1
     *
     *   I2C 时序：
     *     START → 发 0x58 (0x2C<<1 | W) → ACK → 发 0x0A (CR2 地址) → ACK
     *          → 发 0x80 (软复位) → ACK → STOP
     *
     *   目的：清除芯片上电后的未知状态，使寄存器回到默认值。
     *   注意：软复位后芯片会短暂不可用，需要等待至少 100ms。
     */
    data = QMC5883P_CONTROL_2_SOFT_RESET;
    if (QMC5883P_Transmit(QMC5883P_Data, QMC5883P_REG_CONTROL_2, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;

    HAL_Delay(100);
    /*
     * Step 2 — 退出复位（Clear Reset Bit）
     *   写 0x00 到 Control Register 2 (0x0A)，清零所有 bit
     *
     *   I2C 时序：
     *     START → 发 0x58+W → ACK → 发 0x0A → ACK → 发 0x00 → ACK → STOP
     *
     *   目的：复位完成后必须把 bit7 写回 0，芯片才能进入正常工作状态。
     */
    data = 0x00;
    if (QMC5883P_Transmit(QMC5883P_Data, QMC5883P_REG_CONTROL_2, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;
    
     HAL_Delay(100);
    /*
     * Step 3 — 读 Chip ID（校验芯片型号与 I2C 通信是否正常）
     *   读寄存器 0x0D，期望值 0xFF
     *
     *   I2C 时序（写指针 + 重复起始 + 读）：
     *     START → 发 0x58+W → ACK → 发 0x0D (ID 地址) → ACK
     *          → REPEATED START → 发 0x59+R → ACK
     *          → 读 1 字节 → 主发 NACK → STOP
     *
     *   若不回 ACK 或返回值不是 0xFF，说明芯片不存在或型号不对。
     */
    if (QMC5883P_Receive(QMC5883P_Data, QMC5883P_REG_CHIP_ID, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;
    if (data != QMC5883P_CHIP_ID)
        return QMC5883P_ERROR_ID;
    /*
     * Step 4 — 配置量程
     *   写 0x10 到 Control Register 2 (0x0A) 的 bit4-5 = 01 → ±8G
     *
     *   I2C 时序：
     *     START → 发 0x58+W → ACK → 发 0x0A → ACK → 发 0x10 → ACK → STOP
     *
     *   默认量程是 ±2G，此处改为 ±8G。
     *   量程影响后续读数转换系数：±8G 下 1LSB = 1/2048 G。
     */
    data = QMC5883P_CONTROL_2_RNG;
    if (QMC5883P_Transmit(QMC5883P_Data, QMC5883P_REG_CONTROL_2, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;

    /*
     * Step 5 — 配置工作模式
     *   写 0x1D 到 Control Register 1 (0x09)：
     *     bit7-6 = 01 → 连续采样模式
     *     bit5-4 = 11 → 200Hz 输出速率
     *     bit3-2 = 10 → OSR = 512 (过采样率)
     *     bit1-0 = 01 → 正常模式（非待机）
     *
     *   I2C 时序：
     *     START → 发 0x58+W → ACK → 发 0x09 → ACK → 发 0x1D → ACK → STOP
     *
     *   配置完成后芯片开始以 200Hz 连续采集磁场数据，
     *   数据寄存器 0x00~0x05 每 5ms 更新一次。
     */
    data = QMC5883P_CONTROL_1_MODE_CONT | QMC5883P_CONTROL_1_ODR | QMC5883P_CONTROL_1_OSR1_2 | QMC5883P_CONTROL_1_OSR2_2;
    if (QMC5883P_Transmit(QMC5883P_Data, QMC5883P_REG_CONTROL_1, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;

    return QMC5883P_OK;
}

/*brief  读取磁力计数据
 * @param  hi2c: I2C句柄指针
 * @param  QMC5883P_Data: QMC5883P数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t QMC5883P_Read_Mag(struct QMC5883P_Data *QMC5883P_Data)
{
    uint8_t data[6] = {0};
    HAL_StatusTypeDef res;

    res = QMC5883P_Receive(QMC5883P_Data, QMC5883P_REG_XOUT_L, data, 6);
    if (res != HAL_OK)
        return QMC5883P_ERROR;


    // 将数据拼接为16位有符号整数
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    // 转换为高斯值
    QMC5883P_Data->mag_x = (float)raw_x / QMC5883_RNG_SENSITIVITY;
    QMC5883P_Data->mag_y = (float)raw_y / QMC5883_RNG_SENSITIVITY;
    QMC5883P_Data->mag_z = (float)raw_z / QMC5883_RNG_SENSITIVITY;

    QMC5883P_Data->mag_x = (QMC5883P_Data->mag_x - MAG_X_OFFSET) * MAG_X_SCALE;
    QMC5883P_Data->mag_y = (QMC5883P_Data->mag_y - MAG_Y_OFFSET) * MAG_Y_SCALE;
    QMC5883P_Data->mag_z = (QMC5883P_Data->mag_z - MAG_Z_OFFSET) * MAG_Z_SCALE;

    return QMC5883P_OK;
}

/*brief  读取中断状态
 * @param  hi2c: I2C句柄指针
 * @param  QMC5883P_Data: QMC5883P数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t QMC5883P_Read_INT_Status(struct QMC5883P_Data *QMC5883P_Data)
{
    HAL_StatusTypeDef res;

    res = QMC5883P_Receive(QMC5883P_Data, QMC5883P_REG_STATUS, &QMC5883P_Data->int_status, 1);
    if (res != HAL_OK)
        return QMC5883P_ERROR;

    return QMC5883P_OK;
}

/*brief  磁力计校准函数
 * @param  hi2c: I2C句柄指针
 * @param  QMC5883P_Data: QMC5883P数据结构体指针
 * @retval void
 */
void QMC5883P_Calibration(struct QMC5883P_Data *QMC5883P_Data)
{
    float max[3] = {0};
    float min[3] = {0};

    float mag_offset[3] = {0};
    float mag_scale[3] = {0};
    float avg_delta = 0;

    while(1)
    {
        QMC5883P_Read_Mag(QMC5883P_Data);

        if(QMC5883P_Data->mag_x >= max[0]) max[0] = QMC5883P_Data->mag_x;
        else if(QMC5883P_Data->mag_x <= min[0]) min[0] = QMC5883P_Data->mag_x;

        if(QMC5883P_Data->mag_y >= max[1]) max[1] = QMC5883P_Data->mag_y;
        else if(QMC5883P_Data->mag_y <= min[1]) min[1] = QMC5883P_Data->mag_y;

        if(QMC5883P_Data->mag_z >= max[2]) max[2] = QMC5883P_Data->mag_z;
        else if(QMC5883P_Data->mag_z <= min[2]) min[2] = QMC5883P_Data->mag_z;

        for(int i = 0; i < 3; i++)
        {
            mag_offset[i] = (max[i] + min[i]) / 2;
            mag_scale[i] = (max[i] - min[i]) / 2;
        }
        avg_delta = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3;
        for(int i = 0; i < 3; i++)
            mag_scale[i] = avg_delta / mag_scale[i];

        printf("%f, %f, %f, %f, %f, %f\n", mag_offset[0], mag_offset[1], mag_offset[2], mag_scale[0], mag_scale[1], mag_scale[2]);
    }
}
