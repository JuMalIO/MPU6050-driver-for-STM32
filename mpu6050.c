#include "main.h"
#include "mpu6050.h"
#include "mpu6050_def.h"
#include <math.h>

#define MPU6050_ADDRESS (MPU6050_DEFAULT_ADDRESS << 1)
#define MPU6050_I2C_TIMEOUT 100

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef ReadByte(uint8_t address, uint8_t subAddress, uint8_t* dest)
{	
	return HAL_I2C_Mem_Read(&hi2c1, address, subAddress, 1, dest, 1, MPU6050_I2C_TIMEOUT);
}

HAL_StatusTypeDef ReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
{	
	return HAL_I2C_Mem_Read(&hi2c1, address, subAddress, 1, dest, count, MPU6050_I2C_TIMEOUT);
}

HAL_StatusTypeDef WriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	return HAL_I2C_Mem_Write(&hi2c1, address, subAddress, 1, &data, 1, MPU6050_I2C_TIMEOUT);
}

HAL_StatusTypeDef WriteBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* data)
{
	return HAL_I2C_Mem_Write(&hi2c1, address, subAddress, 1, data, count, MPU6050_I2C_TIMEOUT);
}

HAL_StatusTypeDef ReadWord(uint8_t devAddr, uint8_t regAddr, uint16_t* data)
{
	uint8_t buffer[2];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, devAddr, regAddr, 1, buffer, 2, MPU6050_I2C_TIMEOUT);
	*data = (buffer[0] << 8) | buffer[1];
	return status;
}

HAL_StatusTypeDef WriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
	uint8_t buffer[2];
	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)data;
	return HAL_I2C_Mem_Write(&hi2c1, devAddr, regAddr, 1, buffer, 2, MPU6050_I2C_TIMEOUT);
}

void WriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t byte;
	ReadByte(devAddr, regAddr, &byte);
	byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
	WriteByte(devAddr, regAddr, byte);
}

void WriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//      xxx args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t b;
	ReadByte(devAddr, regAddr, &b);
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	b &= ~(mask); // zero all important bits in existing byte
	b |= data; // combine data with existing byte
	WriteByte(devAddr, regAddr, b);
}

void ReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//      xxx args: bitStart=4, length=3
	//      010 masked
	//   -> 010 shifted
	uint8_t b;
	ReadByte(devAddr, regAddr, &b);
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	b &= mask;
	b >>= (bitStart - length + 1);
	*data = b;
}

void MPU6050_GetAccelerometerRaw(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t data[6];
	ReadBytes(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 6, data);
	*x = (((int16_t)data[0]) << 8) | data[1];
	*y = (((int16_t)data[2]) << 8) | data[3];
	*z = (((int16_t)data[4]) << 8) | data[5];
}

void MPU6050_GetTemperatureRaw(int16_t* temp)
{
	ReadWord(MPU6050_ADDRESS, MPU6050_RA_TEMP_OUT_H, (uint16_t*)temp);
}

void MPU6050_GetGyroscopeRaw(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t data[6];
	ReadBytes(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 6, data);
	*x = (((int16_t)data[0]) << 8) | data[1];
	*y = (((int16_t)data[2]) << 8) | data[3];
	*z = (((int16_t)data[4]) << 8) | data[5];
}

void MPU6050_GetAllRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* temp, int16_t* gx, int16_t* gy, int16_t* gz)
{
	uint8_t data[14];
	ReadBytes(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, data);
	*ax = (((int16_t)data[0]) << 8) | data[1];
	*ay = (((int16_t)data[2]) << 8) | data[3];
	*az = (((int16_t)data[4]) << 8) | data[5];
	*temp = ((int16_t)(data[6]) << 8) | data[7];
	*gx = (((int16_t)data[8]) << 8) | data[9];
	*gy = (((int16_t)data[10]) << 8) | data[11];
	*gz = (((int16_t)data[12]) << 8) | data[13];
}

uint8_t MPU6050_GetDeviceId(void)
{
	uint8_t data;
	ReadBits(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &data);
	return data;
}

int16_t MPU6050_GetXAccelOffset(void)
{
	uint16_t data;
	ReadWord(MPU6050_ADDRESS, MPU6050_RA_XA_OFFS_H, &data);
	return data;
}

int16_t MPU6050_GetYAccelOffset(void)
{
	uint16_t data;
	ReadWord(MPU6050_ADDRESS, MPU6050_RA_YA_OFFS_H, &data);
	return data;
}

int16_t MPU6050_GetZAccelOffset(void)
{
	uint16_t data;
	ReadWord(MPU6050_ADDRESS, MPU6050_RA_ZA_OFFS_H, &data);
	return data;
}

int16_t MPU6050_GetXGyroOffset(void)
{
	uint16_t data;
	ReadWord(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH, &data);
	return data;
}

int16_t MPU6050_GetYGyroOffset(void)
{
	uint16_t data;
	ReadWord(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH, &data);
	return data;
}

int16_t MPU6050_GetZGyroOffset(void)
{
	uint16_t data;
	ReadWord(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, &data);
	return data;
}

void MPU6050_SetXAccelOffset(int16_t offset)
{
	WriteWord(MPU6050_ADDRESS, MPU6050_RA_XA_OFFS_H, offset);
}

void MPU6050_SetYAccelOffset(int16_t offset)
{
	WriteWord(MPU6050_ADDRESS, MPU6050_RA_YA_OFFS_H, offset);
}

void MPU6050_SetZAccelOffset(int16_t offset)
{
	WriteWord(MPU6050_ADDRESS, MPU6050_RA_ZA_OFFS_H, offset);
}

void MPU6050_SetXGyroOffset(int16_t offset)
{
	WriteWord(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH, offset);
}

void MPU6050_SetYGyroOffset(int16_t offset)
{
	WriteWord(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH, offset);
}

void MPU6050_SetZGyroOffset(int16_t offset)
{
	WriteWord(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, offset);
}

void MPU6050_ResetDMP(void)
{
	WriteBit(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}

void MPU6050_SetDMPEnabled(_Bool enabled)
{
	WriteBit(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

void MPU6050_ResetFIFO(void)
{
	WriteBit(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

void MPU6050_SetSleepEnabled(_Bool enabled)
{
	WriteBit(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void MPU6050_DmpSetSleepEnabled(_Bool enabled)
{
	uint8_t data = enabled ? 0x41 : 0x01;
	if (WriteByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, data) == HAL_OK)
	{
		return;
	}
	
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_Init(&hi2c1);
	WriteByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, data);
}

void MPU6050_SetClockSource(uint8_t source)
{
	WriteBits(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
	WriteBits(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
	WriteBits(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050_SetMemoryBank(uint8_t bank)
{
	bank &= 0x1F;
	WriteByte(MPU6050_ADDRESS, MPU6050_RA_BANK_SEL, bank);
}

void MPU6050_SetMemoryStartAddress(uint8_t address)
{
	WriteByte(MPU6050_ADDRESS, MPU6050_RA_MEM_START_ADDR, address);
}

void MPU6050_WriteMemoryBlock(const uint8_t* data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
	MPU6050_SetMemoryBank(bank);
	MPU6050_SetMemoryStartAddress(address);

	uint8_t chunkSize;
	uint8_t* progBuffer;

	for (uint16_t i = 0; i < dataSize;)
	{
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize) chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address) chunkSize = 256 - address;
		
		// write the chunk of data as specified
		progBuffer = (uint8_t*)data + i;

		WriteBytes(MPU6050_ADDRESS, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize)
		{
			if (address == 0)
			{
				bank++;
			}
			MPU6050_SetMemoryBank(bank);
			MPU6050_SetMemoryStartAddress(address);
		}
	}
}

_Bool MPU6050_Init(void)
{
	if (MPU6050_GetDeviceId() != 0x34)
	{
		return 1;
	}

	MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	MPU6050_SetSleepEnabled(0);

	return 0;
}

_Bool MPU6050_DmpInit(void)
{
	if (MPU6050_GetDeviceId() != 0x34)
	{
		return 1;
	}
	
	// Reset procedure per instructions in the "MPU-6000/MPU-6050 Register Map and Descriptions" page 41
	WriteBit(MPU6050_ADDRESS, 0x6B, 7, 0x01); //PWR_MGMT_1: reset with 100ms delay
	
	HAL_Delay(100);

	WriteBits(MPU6050_ADDRESS, 0x6A, 2, 3, 0x07); // full SIGNAL_PATH_RESET: with another 100ms delay
	
	HAL_Delay(100);
	
	WriteByte(MPU6050_ADDRESS, 0x6B, 0x01); // 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro

	WriteByte(MPU6050_ADDRESS, 0x38, 0x00); // 0000 0000 INT_ENABLE: no Interrupt

	WriteByte(MPU6050_ADDRESS, 0x23, 0x00); // 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
	
	WriteByte(MPU6050_ADDRESS, 0x1C, 0x00); // 0000 0000 ACCEL_CONFIG: 0 =	Accel Full Scale Select: 2g
	
	WriteByte(MPU6050_ADDRESS, 0x37, 0x80); // 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
	
	WriteByte(MPU6050_ADDRESS, 0x6B, 0x01); // 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro

	WriteByte(MPU6050_ADDRESS, 0x19, 0x04); // 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))

	WriteByte(MPU6050_ADDRESS, 0x1A, 0x01); // 0000 0001 CONFIG: Digital Low Pass Filter (DLPF) Configuration 188HZ	//Im betting this will be the beat
	
	MPU6050_WriteMemoryBlock(DmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0);
	
	uint8_t dmpUpdate[2] = { 0x00, MPU6050_DMP_FIFO_RATE_DIVISOR };
	MPU6050_WriteMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16);
	
	WriteWord(MPU6050_ADDRESS, 0x70, 0x0400); // DMP Program Start Address
	
	WriteByte(MPU6050_ADDRESS, 0x1B, 0x18); // 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec

	WriteByte(MPU6050_ADDRESS, 0x6A, 0xC0); // 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
	
	WriteByte(MPU6050_ADDRESS, 0x38, 0x02); // 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
	
	WriteBit(MPU6050_ADDRESS, 0x6A, 2, 1);  // Reset FIFO one last time just for kicks. (MPUi2cWrite reads 0x6A first and only alters 1 bit and then saves the byte)

	MPU6050_SetDMPEnabled(0); // disable DMP for compatibility with the MPU6050 library
	
	return 0;
}

static volatile uint8_t SampleCount = MPU6050_DMP_FIFO_RATE_DIVISOR;

void MPU6050_DmpOnIntGetCurrentFIFOPacket(uint8_t* fifoBuffer)
{
	if (SampleCount >= MPU6050_DMP_FIFO_RATE_DIVISOR)
	{
		uint16_t count;
		if (ReadWord(MPU6050_ADDRESS, MPU6050_RA_FIFO_COUNTH, &count) == HAL_OK)
		{
			if (count == MPU6050_DMP_PACKET_SIZE)
			{
				HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 1, fifoBuffer, MPU6050_DMP_PACKET_SIZE);
				SampleCount = 0;
			}
			else if (count > MPU6050_DMP_PACKET_SIZE)
			{
				WriteByte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0xC4);
				SampleCount = 0;
			}
		}
		else
		{
			HAL_I2C_DeInit(&hi2c1);
			HAL_I2C_Init(&hi2c1);
			SampleCount = 0;
		}
	}
	else
	{
		SampleCount++;
	}
}

void PID(uint8_t readAddress, float kP, float kI, uint8_t loops)
{
	uint8_t saveAddress = (readAddress == 0x3B) ? 0x06 : 0x13;
	int16_t data;
	float reading;
	int16_t bitZero[3];
	float error, pTerm, iTerm[3];
	int16_t eSample;
	uint32_t eSum;
	
	for (uint8_t i = 0; i < 3; i++)
	{
		ReadWord(MPU6050_ADDRESS, saveAddress + (i * 2), (uint16_t *)&data); // reads 1 or more 16 bit integers (Word)
		reading = data;
		if (saveAddress != 0x13)
		{
			bitZero[i] = data & 1;	// Capture Bit Zero to properly handle Accelerometer calibration
			iTerm[i] = ((float)reading) * 8;
		}
		else
		{
			iTerm[i] = reading * 4;
		}
	}
	for (uint8_t l = 0; l < loops; l++)
	{
		eSample = 0;
		for (uint8_t c = 0; c < 100; c++) // 100 PI Calculations
		{
			eSum = 0;
			for (uint8_t i = 0; i < 3; i++)
			{
				ReadWord(MPU6050_ADDRESS, readAddress + (i * 2), (uint16_t*)&data); // reads 1 or more 16 bit integers (Word)
				reading = data;
				if ((readAddress == 0x3B) && (i == 2))
				{
					reading -= 16384;	//remove Gravity
				}
				error = -reading;
				eSum += fabs(reading);
				pTerm = kP * error;
				iTerm[i] += (error * 0.001f) * kI;				// Integral term 1000 Calculations a second = 0.001
				if (saveAddress != 0x13)
				{
					data = round((pTerm + iTerm[i]) / 8);		// Compute PID Output
					data = (data & 0xFFFE) | bitZero[i];		// Insert Bit0 Saved at beginning
				}
				else
				{
					data = round((pTerm + iTerm[i]) / 4);		// Compute PID Output
				}
				WriteWord(MPU6050_ADDRESS, saveAddress + (i * 2), (uint16_t)data);
			}
			if ((c == 99) && eSum > 1000)
			{
				c = 0;		// Error is still to great to continue 
			}
			if ((eSum * ((readAddress == 0x3B) ? .05 : 1)) < 5)
			{
				eSample++;	// Successfully found offsets prepare to advance
			}
			if ((eSum < 100) && (c > 10) && (eSample >= 10))
			{
				break;		// Advance to next Loop
			}
			HAL_Delay(1);
		}
		
		kP *= .75f;
		kI *= .75f;
		for (uint8_t i = 0; i < 3; i++)
		{
			if (saveAddress != 0x13)
			{
				data = round(iTerm[i] / 8);		//Compute PID Output
				data = (data & 0xFFFE) | bitZero[i];	// Insert Bit0 Saved at beginning
			}
			else
			{
				data = round(iTerm[i] / 4);
			}
			WriteWord(MPU6050_ADDRESS, saveAddress + (i * 2), (uint16_t)data);
		}
	}
	MPU6050_ResetFIFO();
	MPU6050_ResetDMP();
}

float Map(float val, float iMin, float iMax, float oMin, float oMax)
{
	return val / (iMax - iMin) * (oMax - oMin) + oMin;
}

void MPU6050_CalibrateGyro(uint8_t loops)
{
	float kP = 0.3;
	float kI = 90;
	float x;
	x = (100 - Map(loops, 1, 5, 20, 0)) * .01f;
	kP *= x;
	kI *= x;

	PID(0x43, kP, kI, loops);
}

void MPU6050_CalibrateAccel(uint8_t loops)
{
	float kP = 0.3;
	float kI = 20;
	float x;
	x = (100 - Map(loops, 1, 5, 20, 0)) * .01f;
	kP *= x;
	kI *= x;
	
	PID(0x3B, kP, kI, loops);
}

void MPU6050_DmpGetQuaternionInt16(int16_t* data, const uint8_t* packet)
{
	data[0] = ((packet[0] << 8) | packet[1]);
	data[1] = ((packet[4] << 8) | packet[5]);
	data[2] = ((packet[8] << 8) | packet[9]);
	data[3] = ((packet[12] << 8) | packet[13]);
}

void MPU6050_DmpGetQuaternion(MPU6050_Quaternion* q, const uint8_t* packet)
{
	int16_t qI[4];
	MPU6050_DmpGetQuaternionInt16(qI, packet);
	q->w = (float)qI[0] / 16384.0f;
	q->x = (float)qI[1] / 16384.0f;
	q->y = (float)qI[2] / 16384.0f;
	q->z = (float)qI[3] / 16384.0f;
}

void MPU6050_DmpGetAccel(MPU6050_VectorInt16* v, const uint8_t* packet)
{
	v->x = (packet[16] << 8) | packet[17];
	v->y = (packet[18] << 8) | packet[19];
	v->z = (packet[20] << 8) | packet[21];
}

void MPU6050_DmpGetGravity(MPU6050_VectorFloat* v, MPU6050_Quaternion* q)
{
	v->x = 2 * (q->x * q->z - q->w * q->y);
	v->y = 2 * (q->w * q->x + q->y * q->z);
	v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
}

void MPU6050_DmpGetEuler(float* data, MPU6050_Quaternion* q)
{
	data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);	 // psi
	data[1] = -asin(2 * q->x * q->z + 2 * q->w * q->y);																					// theta
	data[2] = atan2(2 * q->y * q->z - 2 * q->w * q->x, 2 * q->w * q->w + 2 * q->z * q->z - 1);	 // phi
}

void MPU6050_DmpGetYawPitchRoll(float* data, MPU6050_Quaternion* q, MPU6050_VectorFloat* gravity)
{
	// yaw: (about Z axis)
	data[0] = atan2(2* q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan2(gravity->x, sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan2(gravity->y, gravity->z);
	if (gravity -> z < 0)
	{
		if (data[1] > 0)
		{
			data[1] = M_PI - data[1]; 
		}
		else
		{
			data[1] = -M_PI - data[1];
		}
	}
}

void MPU6050_DmpGetLinearAccel(MPU6050_VectorInt16* v, MPU6050_VectorInt16* vRaw, MPU6050_VectorFloat* gravity)
{
	// get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
	v->x = vRaw->x - gravity->x * 8192;
	v->y = vRaw->y - gravity->y * 8192;
	v->z = vRaw->z - gravity->z * 8192;
}

void GetConjugate(MPU6050_Quaternion* q)
{
	q->w = q->w;
	q->x = -q->x;
	q->y = -q->y;
	q->z = -q->z;
}

void GetProduct(MPU6050_Quaternion* res, MPU6050_Quaternion* qo, MPU6050_Quaternion* q)
{
	res->w = qo->w * q->w - qo->x * q->x - qo->y * q->y - qo->z * q->z;
	res->x = qo->w * q->x + qo->x * q->w + qo->y * q->z - qo->z * q->y;
	res->y = qo->w * q->y - qo->x * q->z + qo->y * q->w + qo->z * q->x;
	res->z = qo->w * q->z + qo->x * q->y - qo->y * q->x + qo->z * q->w;
}

void Rotate(MPU6050_VectorInt16* v, MPU6050_Quaternion* q)
{
	MPU6050_Quaternion p = { 0, v->x, v->y, v->z };
	MPU6050_Quaternion pp;

	// quaternion multiplication: q * p, stored back in pp
	GetProduct(&pp, q, &p);

	// quaternion multiplication: pp * conj(q), stored back in p
	GetConjugate(q);
	GetProduct(&p, &pp, q);

	// p quaternion is now [0, x', y', z']
	v->x = p.x;
	v->y = p.y;
	v->z = p.z;
}

void MPU6050_DmpGetLinearAccelInWorld(MPU6050_VectorInt16* v, MPU6050_VectorInt16* vReal, MPU6050_Quaternion* q)
{
	// rotate measured 3D acceleration vector into original state
	// frame of reference based on orientation quaternion
	v->x = vReal->x;
	v->y = vReal->y;
	v->z = vReal->z;
	
	Rotate(v, q);
}
