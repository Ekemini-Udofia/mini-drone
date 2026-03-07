/*
 * imu.c
 *
 *  Created on: Mar 7, 2026
 *      Author: Ekemini Udofia
 */


#include "imu.h"

extern I2C_HandleTypeDef hi2c1;

static float gyro_offset_x = 0;
static float gyro_offset_y = 0;
static float gyro_offset_z = 0;


static void imu_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, reg, 1, buffer, length, HAL_MAX_DELAY);
}


static void imu_write_byte(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);
}


void IMU_Init(void)
{
    IMU_Wake();
}


void IMU_Wake(void)
{
    uint8_t data = 0x00;
    imu_write_byte(PWR_MGMT_1, data);
}


uint8_t IMU_Check(void)
{
    uint8_t who = 0;
    imu_read_bytes(WHO_AM_I_REG, &who, 1);
    return who;
}


void IMU_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];

    imu_read_bytes(ACCEL_XOUT_H, buffer, 6);

    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);
}


void IMU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buffer[6];

    imu_read_bytes(GYRO_XOUT_H, buffer, 6);

    *gx = (int16_t)(buffer[0] << 8 | buffer[1]);
    *gy = (int16_t)(buffer[2] << 8 | buffer[3]);
    *gz = (int16_t)(buffer[4] << 8 | buffer[5]);
}


void IMU_ReadRaw(IMU_Data *data)
{
    uint8_t buffer[14];

    imu_read_bytes(ACCEL_XOUT_H, buffer, 14);

    int16_t ax = (buffer[0] << 8 | buffer[1]);
    int16_t ay = (buffer[2] << 8 | buffer[3]);
    int16_t az = (buffer[4] << 8 | buffer[5]);

    int16_t gx = (buffer[8] << 8 | buffer[9]);
    int16_t gy = (buffer[10] << 8 | buffer[11]);
    int16_t gz = (buffer[12] << 8 | buffer[13]);

    data->ax = ax;
    data->ay = ay;
    data->az = az;

    data->gx = gx;
    data->gy = gy;
    data->gz = gz;
}


void IMU_Read(IMU_Data *data)
{
    IMU_ReadRaw(data);

    data->ax /= ACCEL_SCALE;
    data->ay /= ACCEL_SCALE;
    data->az /= ACCEL_SCALE;

    data->gx = (data->gx / GYRO_SCALE) - gyro_offset_x;
    data->gy = (data->gy / GYRO_SCALE) - gyro_offset_y;
    data->gz = (data->gz / GYRO_SCALE) - gyro_offset_z;
}


void IMU_CalibrateGyro(void)
{
    const int samples = 500;

    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    IMU_Data data;

    for(int i = 0; i < samples; i++)
    {
        IMU_ReadRaw(&data);

        sum_x += data.gx;
        sum_y += data.gy;
        sum_z += data.gz;

        HAL_Delay(2);
    }

    gyro_offset_x = (sum_x / samples) / GYRO_SCALE;
    gyro_offset_y = (sum_y / samples) / GYRO_SCALE;
    gyro_offset_z = (sum_z / samples) / GYRO_SCALE;
}
