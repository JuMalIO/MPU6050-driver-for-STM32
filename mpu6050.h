#ifndef __mpu6050_H
#define __mpu6050_H


#include <stdint.h>

#define MPU6050_GYRO_SCALE_FACTOR_FS_SEL_250 131.0
#define MPU6050_GYRO_SCALE_FACTOR_FS_SEL_500  65.5
#define MPU6050_GYRO_SCALE_FACTOR_FS_SEL_1000 32.8
#define MPU6050_GYRO_SCALE_FACTOR_FS_SEL_2000 16.4

#define MPU6050_TEMP_SCALE_FACTOR 340.0
#define MPU6050_TEMP_OFFSET        36.53

#define MPU6050_ACCEL_SCALE_FACTOR_AFS_SEL_2 16384.0
#define MPU6050_ACCEL_SCALE_FACTOR_AFS_SEL_4  8192.0
#define MPU6050_ACCEL_SCALE_FACTOR_AFS_SEL_8  4096.0
#define MPU6050_ACCEL_SCALE_FACTOR_AFS_SEL_16 2048.0

_Bool MPU6050_Init(void);

void MPU6050_GetAccelerometerRaw(int16_t* x, int16_t* y, int16_t* z);
void MPU6050_GetTemperatureRaw(int16_t* temp);
void MPU6050_GetGyroscopeRaw(int16_t* x, int16_t* y, int16_t* z);
void MPU6050_GetAllRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* temp, int16_t* gx, int16_t* gy, int16_t* gz);

int16_t MPU6050_GetXAccelOffset(void);
int16_t MPU6050_GetYAccelOffset(void);
int16_t MPU6050_GetZAccelOffset(void);
int16_t MPU6050_GetXGyroOffset(void);
int16_t MPU6050_GetYGyroOffset(void);
int16_t MPU6050_GetZGyroOffset(void);

void MPU6050_SetXAccelOffset(int16_t offset);
void MPU6050_SetYAccelOffset(int16_t offset);
void MPU6050_SetZAccelOffset(int16_t offset);
void MPU6050_SetXGyroOffset(int16_t offset);
void MPU6050_SetYGyroOffset(int16_t offset);
void MPU6050_SetZGyroOffset(int16_t offset);

void MPU6050_CalibrateGyro(uint8_t loops);
void MPU6050_CalibrateAccel(uint8_t loops);

void MPU6050_SetSleepEnabled(_Bool enabled);

typedef struct __MPU6050_Quaternion
{
	float w;
	float x;
	float y;
	float z;
} MPU6050_Quaternion;

typedef struct __MPU6050_VectorInt16
{
	int16_t x;
	int16_t y;
	int16_t z;
} MPU6050_VectorInt16;

typedef struct __MPU6050_VectorFloat
{
	float x;
	float y;
	float z;
} MPU6050_VectorFloat;

#define M_PI 3.14159265358979323846f
#define MPU6050_DMP_PACKET_SIZE 28

_Bool MPU6050_DmpInit(void);

void MPU6050_SetDMPEnabled(_Bool enabled);

void MPU6050_DmpSetSleepEnabled(_Bool enabled);

void MPU6050_DmpOnIntGetCurrentFIFOPacket(uint8_t* fifoBuffer);

void MPU6050_DmpGetQuaternion(MPU6050_Quaternion *q, const uint8_t* packet);
void MPU6050_DmpGetAccel(MPU6050_VectorInt16 *v, const uint8_t* packet);
void MPU6050_DmpGetGravity(MPU6050_VectorFloat* v, MPU6050_Quaternion* q);
void MPU6050_DmpGetEuler(float* data, MPU6050_Quaternion* q);
void MPU6050_DmpGetYawPitchRoll(float* data, MPU6050_Quaternion* q, MPU6050_VectorFloat* gravity);
void MPU6050_DmpGetLinearAccel(MPU6050_VectorInt16 *v, MPU6050_VectorInt16 *vRaw, MPU6050_VectorFloat *gravity);
void MPU6050_DmpGetLinearAccelInWorld(MPU6050_VectorInt16 *v, MPU6050_VectorInt16 *vReal, MPU6050_Quaternion *q);


#endif /*__ mpu6050_H */
