/*
 * imu.h
 *
 *  Created on: Mar 7, 2026
 *      Author: Ekemini Udofia
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include <stdint.h>

#define MPU_ADDR        (0x68 << 1)

#define WHO_AM_I_REG    0x75
#define PWR_MGMT_1      0x6B

#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43

// Sensitivity constants (for default ±2g and ±250dps)
#define ACCEL_SCALE     16384.0f
#define GYRO_SCALE      131.0f

typedef struct
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

} IMU_Data;


// Initialization
void IMU_Init(void);
void IMU_Wake(void);

// Raw sensor reads
void IMU_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az);
void IMU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);

// Combined raw read (faster because burst read)
void IMU_ReadRaw(IMU_Data *data);

// Scaled data (physical units)
void IMU_Read(IMU_Data *data);

// Calibration
void IMU_CalibrateGyro(void);

// Status / verification
uint8_t IMU_Check(void);

#endif /* INC_IMU_H_ */
