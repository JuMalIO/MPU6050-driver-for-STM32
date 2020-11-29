#ifndef __mpu6050_H
#define __mpu6050_H


typedef struct __MPU6050_Position
{
    float X;
    float Y;
    float Z;
} MPU6050_Position;

void MPU6050_Init(void);
MPU6050_Position* MPU6050_ReadAccelerometer(void);
void MPU6050_SetSleep(_Bool enabled);


#endif /*__ mpu6050_H */
