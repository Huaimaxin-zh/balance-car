#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

uint8_t MPU6050_Init(void);       /* 成功返回1，失败返回0 */
uint8_t MPU6050_GetID(void);
uint8_t MPU6050_GetData(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz);

#endif
