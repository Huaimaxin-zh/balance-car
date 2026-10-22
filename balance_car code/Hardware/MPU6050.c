#include "stm32f10x.h"
#include "Delay.h"
#include "MyI2C.h"
#include "MPU6050.h"

/* AD0接低电平：0xD0；AD0接高电平：改为0xD2。这里是8位写地址。 */
#define ICM_ADDR             0xD0
#define ICM_WHO_AM_I         0x75
#define ICM_PWR_MGMT_1       0x6B
#define ICM_PWR_MGMT_2       0x6C
#define ICM_SMPLRT_DIV       0x19
#define ICM_CONFIG           0x1A
#define ICM_GYRO_CONFIG      0x1B
#define ICM_ACCEL_CONFIG     0x1C
#define ICM_ACCEL_CONFIG2    0x1D
#define ICM_ACCEL_XOUT_H     0x3B

static uint8_t WriteReg(uint8_t reg, uint8_t value)
{
    MyI2C_Start();
    MyI2C_SendByte(ICM_ADDR);
    if (MyI2C_ReceiveAck()) goto fail;
    MyI2C_SendByte(reg);
    if (MyI2C_ReceiveAck()) goto fail;
    MyI2C_SendByte(value);
    if (MyI2C_ReceiveAck()) goto fail;
    MyI2C_Stop();
    return 1;

fail:
    MyI2C_Stop();
    return 0;
}

static uint8_t ReadRegs(uint8_t reg, uint8_t *data, uint8_t count)
{
    uint8_t i;

    if (count == 0) return 0;
    MyI2C_Start();
    MyI2C_SendByte(ICM_ADDR);
    if (MyI2C_ReceiveAck()) goto fail;
    MyI2C_SendByte(reg);
    if (MyI2C_ReceiveAck()) goto fail;

    MyI2C_Start();
    MyI2C_SendByte(ICM_ADDR | 1);
    if (MyI2C_ReceiveAck()) goto fail;

    for (i = 0; i < count; i++) {
        data[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck((i == count - 1) ? 1 : 0);
    }
    MyI2C_Stop();
    return 1;

fail:
    MyI2C_Stop();
    return 0;
}

uint8_t MPU6050_GetID(void)
{
    uint8_t id = 0xFF;
    if (!ReadRegs(ICM_WHO_AM_I, &id, 1)) return 0xFF;
    return id;
}

uint8_t MPU6050_Init(void)
{
    MyI2C_Init();
    Delay_ms(100);

    /* ICM20602的WHO_AM_I通常应为0x12；先确认读到的是目标芯片。 */
    if (MPU6050_GetID() != 0x12) return 0;

    if (!WriteReg(ICM_PWR_MGMT_1, 0x01)) return 0;
    Delay_ms(10);
    if (!WriteReg(ICM_PWR_MGMT_2, 0x00)) return 0;
    if (!WriteReg(ICM_CONFIG, 0x03)) return 0;
    if (!WriteReg(ICM_SMPLRT_DIV, 0x04)) return 0;
    if (!WriteReg(ICM_GYRO_CONFIG, 0x18)) return 0;   /* ±2000°/s */
    if (!WriteReg(ICM_ACCEL_CONFIG, 0x18)) return 0;  /* ±16g */
    if (!WriteReg(ICM_ACCEL_CONFIG2, 0x03)) return 0;
    return 1;
}

uint8_t MPU6050_GetData(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t d[14];

    if (!ReadRegs(ICM_ACCEL_XOUT_H, d, 14)) return 0;
    *ax = (int16_t)((uint16_t)d[0]  << 8 | d[1]);
    *ay = (int16_t)((uint16_t)d[2]  << 8 | d[3]);
    *az = (int16_t)((uint16_t)d[4]  << 8 | d[5]);
    *gx = (int16_t)((uint16_t)d[8]  << 8 | d[9]);
    *gy = (int16_t)((uint16_t)d[10] << 8 | d[11]);
    *gz = (int16_t)((uint16_t)d[12] << 8 | d[13]);
    return 1;
}
