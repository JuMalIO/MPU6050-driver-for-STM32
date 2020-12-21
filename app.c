#include "app.h"
#include "main.h"
#include "mpu6050.h"
#include <stdio.h>

// uncomment "OUTPUT_RAW_NON_DMP" if you want to see the actual
// raw values without using DMP
//#define OUTPUT_RAW_NON_DMP

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

#define TEST_SLEEP_EVETY_5_SEC

// MPU control/status vars
_Bool ready = 0;
uint8_t initStatus;                          // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[MPU6050_DMP_PACKET_SIZE]; // FIFO storage buffer

// orientation/motion vars
MPU6050_Quaternion q;          // [w, x, y, z]       quaternion container
MPU6050_VectorInt16 aa;        // [x, y, z]          accel sensor measurements
MPU6050_VectorInt16 aaReal;    // [x, y, z]          gravity-free accel sensor measurements
MPU6050_VectorInt16 aaWorld;   // [x, y, z]          world-frame accel sensor measurements
MPU6050_VectorFloat gravity;   // [x, y, z]          gravity vector
float euler[3];                // [psi, theta, phi]  Euler angle container
float ypr[3];                  // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
int16_t ax, ay, az, temp, gx, gy, gz;

void APP_Init(void)
{
#ifdef OUTPUT_RAW_NON_DMP
	printf("Initializing...\r\n");
	initStatus = MPU6050_Init();
#else
	printf("Initializing DMP...\r\n");
	initStatus = MPU6050_DmpInit();
#endif

	printf(!initStatus ? "MPU6050 connection successful\r\n" : "MPU6050 connection failed\r\n");
	
	// supply your own gyro offsets here, scaled for min sensitivity
//	MPU6050_SetXGyroOffset(51);
//	MPU6050_SetYGyroOffset(8);
//	MPU6050_SetZGyroOffset(21);
//	MPU6050_SetXAccelOffset(1150);
//	MPU6050_SetYAccelOffset(-50);
//	MPU6050_SetZAccelOffset(1060);
	
	if (initStatus == 0)
	{
		printf("Offsets:\r\n");
		printf("%d,	", MPU6050_GetXGyroOffset());
		printf("%d,	", MPU6050_GetYGyroOffset());
		printf("%d,	", MPU6050_GetZGyroOffset());
		printf("%d,	", MPU6050_GetXAccelOffset());
		printf("%d,	", MPU6050_GetYAccelOffset());
		printf("%d\r\n", MPU6050_GetZAccelOffset());
		
		MPU6050_CalibrateAccel(6);
		MPU6050_CalibrateGyro(6);
		
		printf("Offsets after calibration:\r\n");
		printf("%d,	", MPU6050_GetXGyroOffset());
		printf("%d,	", MPU6050_GetYGyroOffset());
		printf("%d,	", MPU6050_GetZGyroOffset());
		printf("%d,	", MPU6050_GetXAccelOffset());
		printf("%d,	", MPU6050_GetYAccelOffset());
		printf("%d\r\n", MPU6050_GetZAccelOffset());
		
#ifndef OUTPUT_RAW_NON_DMP
		// turn on the DMP, now that it's ready
		printf("Enabling DMP...\r\n");
		MPU6050_SetDMPEnabled(1);
#endif

		ready = 1;
	}
}

int sleepTimer = 0;
_Bool sleep = 0;

void APP_Run(void)
{
	if (!ready) return;
	
#ifdef OUTPUT_READABLE_QUATERNION
	// display quaternion values in easy matrix form: w x y z
	MPU6050_DmpGetQuaternion(&q, fifoBuffer);

	printf("quat\t");
	printf("%.2f", q.w);
	printf("\t");
	printf("%.2f", q.x);
	printf("\t");
	printf("%.2f", q.y);
	printf("\t");
	printf("%.2f", q.z);
	printf("\r\n");
#endif

#ifdef OUTPUT_READABLE_EULER
	// display Euler angles in degrees
	MPU6050_DmpGetQuaternion(&q, fifoBuffer);
	MPU6050_DmpGetEuler(euler, &q);
	printf("euler\t");
	printf("%.2f", euler[0] * 180 / M_PI);
	printf("\t");
	printf("%.2f", euler[1] * 180 / M_PI);
	printf("\t");
	printf("%.2f", euler[2] * 180 / M_PI);
	printf("\r\n");
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
	// display Euler angles in degrees
	MPU6050_DmpGetQuaternion(&q, fifoBuffer);
	MPU6050_DmpGetGravity(&gravity, &q);
	MPU6050_DmpGetYawPitchRoll(ypr, &q, &gravity);
	printf("ypr\t");
	printf("%.2f", ypr[0] * 180 / M_PI);
	printf("\t");
	printf("%.2f", ypr[1] * 180 / M_PI);
	printf("\t");
	printf("%.2f", ypr[2] * 180 / M_PI);
	printf("\r\n");
#endif

#ifdef OUTPUT_READABLE_REALACCEL
	// display real acceleration, adjusted to remove gravity
	MPU6050_DmpGetQuaternion(&q, fifoBuffer);
	MPU6050_DmpGetAccel(&aa, fifoBuffer);
	MPU6050_DmpGetGravity(&gravity, &q);
	MPU6050_DmpGetLinearAccel(&aaReal, &aa, &gravity);
	printf("areal\t");
	printf("%d", aaReal.x);
	printf("\t");
	printf("%d", aaReal.y);
	printf("\t");
	printf("%d", aaReal.z);
	printf("\r\n");
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
	// display initial world-frame acceleration, adjusted to remove gravity
	// and rotated based on known orientation from quaternion
	MPU6050_DmpGetQuaternion(&q, fifoBuffer);
	MPU6050_DmpGetAccel(&aa, fifoBuffer);
	MPU6050_DmpGetGravity(&gravity, &q);
	MPU6050_DmpGetLinearAccel(&aaReal, &aa, &gravity);
	MPU6050_DmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	printf("aworld\t");
	printf("%d", aaWorld.x);
	printf("\t");
	printf("%d", aaWorld.y);
	printf("\t");
	printf("%d", aaWorld.z);
	printf("\r\n");
#endif

#ifdef OUTPUT_RAW_NON_DMP
	MPU6050_GetAllRaw(&ax, &ay, &az, &temp, &gx, &gy, &gz);
	printf("raw\t");
	printf("%.2f", ax / MPU6050_ACCEL_SCALE_FACTOR_AFS_SEL_2);
	printf("\t");
	printf("%.2f", ay / MPU6050_ACCEL_SCALE_FACTOR_AFS_SEL_2);
	printf("\t");
	printf("%.2f", az / MPU6050_ACCEL_SCALE_FACTOR_AFS_SEL_2);
	printf("\t");
	printf("%.2f", temp / MPU6050_TEMP_SCALE_FACTOR + MPU6050_TEMP_OFFSET);
	printf("\t");
	printf("%.2f", gx / MPU6050_GYRO_SCALE_FACTOR_FS_SEL_250);
	printf("\t");
	printf("%.2f", gy / MPU6050_GYRO_SCALE_FACTOR_FS_SEL_250);
	printf("\t");
	printf("%.2f", gz / MPU6050_GYRO_SCALE_FACTOR_FS_SEL_250);
	printf("\r\n");
#endif
	
	HAL_Delay(20);
	
#ifdef TEST_SLEEP_EVETY_5_SEC
	if (sleepTimer > 250)
	{
		sleepTimer = 0;
		sleep = !sleep;
		printf("sleeping: %d\r\n", sleep);
#ifdef OUTPUT_RAW_NON_DMP
		MPU6050_SetSleepEnabled(sleep);
#else
		MPU6050_DmpSetSleepEnabled(sleep);
#endif
		HAL_Delay(1000);
	}
	else
	{
		sleepTimer++;
	}
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	MPU6050_DmpOnIntGetCurrentFIFOPacket(fifoBuffer);
}
