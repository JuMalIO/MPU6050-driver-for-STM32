#include "mpu6050.h"
#include "main.h"

#define MPU6050_ADDR 0xD0

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

#define ACCELELEROMETER_ROW_TO_G 16384.0
#define ACCELELEROMETER_Z_CORRECTOR 14418.0

static MPU6050_Position Position;

extern I2C_HandleTypeDef hi2c1;

void MPU6050_Init(void)
{
    uint8_t data;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &data, 1, 100);

    if (data == 0x68)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
			
        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> 2g
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> 250 /s
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);
    }
}

MPU6050_Position* MPU6050_ReadAccelerometer(void)
{
	uint8_t data[6];
	
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 6, 10);
	
	Position.X = (int16_t)(data[0] << 8 | data[1]) / ACCELELEROMETER_ROW_TO_G;
	Position.Y = (int16_t)(data[2] << 8 | data[3]) / ACCELELEROMETER_ROW_TO_G;
	Position.Z = (int16_t)(data[4] << 8 | data[5]) / ACCELELEROMETER_Z_CORRECTOR;

	return &Position;
}

void MPU6050_SetSleep(_Bool enabled)
{
	uint8_t data = enabled ? 0 : 0x40;
	
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
}
