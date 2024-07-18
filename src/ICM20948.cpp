////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021 Mateusz Malinowski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <cstdint>
#ifdef LOG
#include <cstdio>
#endif /* LOG */
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "ICM20948.h"
#include "MadgwickAHRS.h"
#include "SimpleAHRS.h"

/* define ICM-20948 Device I2C address*/
#define I2C_ADD_ICM20948            0x69
#define I2C_ADD_ICM20948_AK09916    0x0C
#define I2C_ADD_ICM20948_AK09916_READ  0x80
#define I2C_ADD_ICM20948_AK09916_WRITE 0x00

/* define ICM-20948 Register */
/* user bank 0 register */
#define REG_ADD_WIA             0x00
#define REG_VAL_WIA             0xEA
#define REG_ADD_USER_CTRL       0x03
#define REG_VAL_BIT_DMP_EN          0x80
#define REG_VAL_BIT_FIFO_EN         0x40
#define REG_VAL_BIT_I2C_MST_EN      0x20
#define REG_VAL_BIT_I2C_IF_DIS      0x10
#define REG_VAL_BIT_DMP_RST         0x08
#define REG_VAL_BIT_DIAMOND_DMP_RST 0x04
#define REG_ADD_PWR_MIGMT_1     0x06
#define REG_VAL_ALL_RGE_RESET   0x80
#define REG_VAL_RUN_MODE        0x01    //Non low-power mode
#define REG_ADD_LP_CONFIG       0x05
#define REG_ADD_PWR_MGMT_1      0x06
#define REG_ADD_PWR_MGMT_2      0x07
#define REG_VAL_SENSORS_ON      0x00
#define REG_VAL_DISABLE_GYRO    0x07
#define REG_VAL_DISABLE_ACC     0x38
#define REG_ADD_ACCEL_XOUT_H    0x2D
#define REG_ADD_ACCEL_XOUT_L    0x2E
#define REG_ADD_ACCEL_YOUT_H    0x2F
#define REG_ADD_ACCEL_YOUT_L    0x30
#define REG_ADD_ACCEL_ZOUT_H    0x31
#define REG_ADD_ACCEL_ZOUT_L    0x32
#define REG_ADD_GYRO_XOUT_H     0x33
#define REG_ADD_GYRO_XOUT_L     0x34
#define REG_ADD_GYRO_YOUT_H     0x35
#define REG_ADD_GYRO_YOUT_L     0x36
#define REG_ADD_GYRO_ZOUT_H     0x37
#define REG_ADD_GYRO_ZOUT_L     0x38
#define REG_ADD_TEMP_OUT_H      0x39
#define REG_ADD_TEMP_OUT_L      0x3A
#define REG_ADD_EXT_SENS_DATA_00 0x3B
#define REG_ADD_FIFO_EN_1		0x66
#define REG_ADD_FIFO_EN_2		0x67
#define REG_ADD_FIFO_RST		0x68
#define REG_ADD_FIFO_MODE		0x69
#define REG_ADD_FIFO_COUNTH		0x70
#define REG_ADD_FIFO_COUNTL		0x71
#define REG_ADD_FIFO_R_W		0x72
#define REG_ADD_REG_BANK_SEL    0x7F
#define REG_VAL_REG_BANK_0  0x00
#define REG_VAL_REG_BANK_1  0x10
#define REG_VAL_REG_BANK_2  0x20
#define REG_VAL_REG_BANK_3  0x30

/* user bank 1 register */
/* user bank 2 register */
#define REG_ADD_GYRO_SMPLRT_DIV 0x00
#define REG_ADD_GYRO_CONFIG_1   0x01
#define REG_ADD_GYRO_CONFIG_2   0x02
#define REG_ADD_XG_OFFS_USRH	0x03
#define REG_ADD_XG_OFFS_USRL	0x04
#define REG_ADD_YG_OFFS_USRH	0x05
#define REG_ADD_YG_OFFS_USRL	0x06
#define REG_ADD_ZG_OFFS_USRH	0x07
#define REG_ADD_ZG_OFFS_USRL	0x08
#define REG_ADD_XA_OFFS_H		0x14
#define REG_ADD_XA_OFFS_L		0x15
#define REG_ADD_YA_OFFS_H		0x17
#define REG_ADD_YA_OFFS_L		0x18
#define REG_ADD_ZA_OFFS_H		0x1A
#define REG_ADD_ZA_OFFS_L		0x1B
#define REG_VAL_BIT_GYRO_DLPCFG_2   0x10 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_4   0x20 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_6   0x30 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_FS_250DPS  0x00 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_500DPS  0x02 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_DLPF       0x01 /* bit[0]   */
#define REG_ADD_ACCEL_SMPLRT_DIV_1  0x10
#define REG_ADD_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ADD_ACCEL_CONFIG        0x14
#define REG_ADD_ACCEL_CONFIG_2      0x15
#define REG_VAL_BIT_ACCEL_DLPCFG_2  0x10 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_4  0x20 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_6  0x30 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_FS_2g     0x00 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_4g     0x02 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_8g     0x04 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_16g    0x06 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_DLPF      0x01 /* bit[0]   */
#define REG_ADD_TEMP_CONFIG			0x53

/* user bank 3 register */
#define REG_ADD_I2C_MST_CTRL	0x01
#define REG_VAL_I2C_MST_CTRL_CLK_400KHZ	0x07
#define REG_ADD_I2C_SLV0_ADDR   0x03
#define REG_ADD_I2C_SLV0_REG    0x04
#define REG_ADD_I2C_SLV0_CTRL   0x05
#define REG_VAL_BIT_SLV0_EN     0x80
#define REG_VAL_BIT_MASK_LEN    0x07
#define REG_ADD_I2C_SLV0_DO     0x06
#define REG_ADD_I2C_SLV1_ADDR   0x07
#define REG_ADD_I2C_SLV1_REG    0x08
#define REG_ADD_I2C_SLV1_CTRL   0x09
#define REG_ADD_I2C_SLV1_DO     0x0A

/* define ICM-20948 Register  end */

/* define ICM-20948 MAG Register  */
#define REG_ADD_MAG_WIA1    0x00
#define REG_VAL_MAG_WIA1    0x48
#define REG_ADD_MAG_WIA2    0x01
#define REG_VAL_MAG_WIA2    0x09
#define REG_ADD_MAG_ST1     0x10
#define REG_ADD_MAG_DATA    0x11
#define REG_ADD_MAG_CNTL2   0x31
#define REG_VAL_MAG_MODE_PD     0x00
#define REG_VAL_MAG_MODE_SM     0x01
#define REG_VAL_MAG_MODE_10HZ   0x02
#define REG_VAL_MAG_MODE_20HZ   0x04
#define REG_VAL_MAG_MODE_50HZ   0x05
#define REG_VAL_MAG_MODE_100HZ  0x08
#define REG_VAL_MAG_MODE_ST     0x10
#define REG_ADD_MAG_CNTL3   0x32
#define REG_VAL_MAG_RESET	0x01
/* define ICM-20948 MAG Register  end */

namespace
{
/** The length of all IMU data that is being read at once in bytes. */
const short IMU_DATA_LEN = 22;
/** The length of individual gyroscope and accelerator data in bytes. */
const short GYRO_AND_ACC_DATA_LEN = 6;
/** The length of individual magnetometer data with status check in bytes. */
const short MAG_DATA_LEN = 8;
/** Constant scale for magnetometer. */
const float MAG_SCALE = 0.15f;
/** Scale to convert angles from degrees to radians. */
const float DEG_TO_RAD = M_PI / 180.0f;

/**
 * Puts the thread to sleep for given milliseconds.
 *  @param miliseconds the duration in ms for which the thread should sleep.
 */
void sleepMS(const short miliseconds)
{
	struct timespec recquired;
	struct timespec remain;

	recquired.tv_sec  = 0;
	recquired.tv_nsec = static_cast<long int>(miliseconds) * 1000000;

	nanosleep(&recquired, &remain);
}
} /* End of anonymous namespace */

ICM20948::ICM20948() : mCurrentBank(BANK_UNDEFINED), mConfig(),
	mGyroScale(0.0f), mAccScale(0.0f)
{
}

ICM20948::~ICM20948()
{
}

bool ICM20948::initialise(const Config& config)
{
	uint8_t deviceID = 0;

	if (strcmp(config.mDevice, mConfig.mDevice) != 0)
	{
		mI2C.closeSerialPort();
	}

	mConfig = config;
	mData.mUpdatePeriod = 1.0f / mConfig.mFramerate;

	if (mI2C.openSerialPort(mConfig.mDevice))
	{
		setBank(BANK_0);
		deviceID = mI2C.readByte(I2C_ADD_ICM20948, REG_ADD_WIA);
		if (REG_VAL_WIA == deviceID)
		{
			/* Reset all IMU configuration. */
			reset();

			setBank(BANK_3);
			/* Reset I2C master clock. */
			mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_MST_CTRL, 0);

			configureMasterI2C();

			if (mConfig.mTemp.mEnabled)
			{
				configureTemp();
			}

			if (mConfig.mGyro.mEnabled)
			{
				configureGyro();
			}
			if (mConfig.mAcc.mEnabled)
			{
				configureAcc();
			}

			if (mConfig.mMagEnabled)
			{
				mConfig.mMagEnabled = configureMag();
			}
		}
#ifdef LOG
		else
		{
			printf("Connected device is not ICM-20948! ID: %d \n", deviceID);
		}
#endif /* LOG */
	}
#ifdef LOG
	else
	{
		printf("Failed to open device: %s \n", mConfig.mDevice);
	}
#endif /* LOG */
	return (REG_VAL_WIA == deviceID);
}

const IMUData& ICM20948::imuDataGet()
{
	bool magEnabled = mConfig.mMagEnabled;
	int16_t s16Gyro[3], s16Accel[3], s16Magn[3];
	int16_t temperature;

	magEnabled &= readAllRawDAta(s16Gyro, s16Accel, s16Magn, temperature);
	mData.mGyro[0] = static_cast<float>(s16Gyro[0]) * mGyroScale * DEG_TO_RAD;
	mData.mGyro[1] = static_cast<float>(s16Gyro[1]) * mGyroScale * DEG_TO_RAD;
	mData.mGyro[2] = static_cast<float>(s16Gyro[2]) * mGyroScale * DEG_TO_RAD;

	mData.mAcc[0]  = static_cast<float>(s16Accel[0]) * mAccScale; // in G's, NOT in m/s/s
	mData.mAcc[1]  = static_cast<float>(s16Accel[1]) * mAccScale;
	mData.mAcc[2]  = static_cast<float>(s16Accel[2]) * mAccScale;

	mData.mMag[0]  = static_cast<float>(s16Magn[0]) * MAG_SCALE;
	mData.mMag[1]  = static_cast<float>(s16Magn[1]) * MAG_SCALE;
	mData.mMag[2]  = static_cast<float>(s16Magn[2]) * MAG_SCALE;

	mData.mTemp = (static_cast<float>(temperature - 21) / 333.87f) + 21.0f;

	switch (mConfig.mAHRS)
	{
		case NONE:
			break;
		case SIMPLE:
			SimpleAHRSupdate(mData);
		  break;
		case MADGWICK:
			/* Fall-through to the default case. */
		default:
			if (magEnabled)
			{
				MadgwickAHRSupdate(mData);
			}
			else
			{
				MadgwickAHRSupdateIMU(mData);
			}
		  break;
	}

	return mData;
}

void ICM20948::calibrateGyro() const
{
	int16_t i;
	int16_t s16G[3];
	int32_t s32G[3] = {0, 0, 0};

#ifdef LOG
	puts("Calibrating IMU offsets, please wait approximately 11 seconds.");
#endif /* LOG */

	// Reset the offset so that we perform the fresh calibration
	setBank(BANK_2);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_XG_OFFS_USRH, 0);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_XG_OFFS_USRL, 0);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_YG_OFFS_USRH, 0);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_YG_OFFS_USRL, 0);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ZG_OFFS_USRH, 0);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ZG_OFFS_USRL, 0);

	// Read several gyro measurements and average them.
	for (i = 0; i < 1024; ++i)
	{
		readRawGyro(s16G);
		s32G[0] += static_cast<int32_t>(s16G[0]);
		s32G[1] += static_cast<int32_t>(s16G[1]);
		s32G[2] += static_cast<int32_t>(s16G[2]);
		sleepMS(10);
	}

	s16G[0] = -static_cast<int16_t>(s32G[0] >> (12 - mConfig.mGyro.mRange));
	s16G[1] = -static_cast<int16_t>(s32G[1] >> (12 - mConfig.mGyro.mRange));
	s16G[2] = -static_cast<int16_t>(s32G[2] >> (12 - mConfig.mGyro.mRange));

	// Push gyroscope biases to hardware registers
	setBank(BANK_2);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_XG_OFFS_USRH, static_cast<uint8_t>((s16G[0] >> 8) & 0xFF));
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_XG_OFFS_USRL, static_cast<uint8_t>( s16G[0]       & 0xFF));
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_YG_OFFS_USRH, static_cast<uint8_t>((s16G[1] >> 8) & 0xFF));
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_YG_OFFS_USRL, static_cast<uint8_t>( s16G[1]       & 0xFF));
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ZG_OFFS_USRH, static_cast<uint8_t>((s16G[2] >> 8) & 0xFF));
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ZG_OFFS_USRL, static_cast<uint8_t>( s16G[2]       & 0xFF));
}

void ICM20948::setBank(const ICM_BANK bank) const
{
	if (bank != mCurrentBank && bank != BANK_UNDEFINED)
	{
		mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, bank);
		mCurrentBank = bank;
	}
}

void ICM20948::reset() const
{
	uint8_t sensorsFlag = REG_VAL_SENSORS_ON;

	magI2CWrite(REG_ADD_MAG_CNTL2, 0x00);
	sleepMS(100);

	setBank(BANK_0);
	/* Reset all settings on master device  */
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_PWR_MGMT_1, REG_VAL_ALL_RGE_RESET);

	sleepMS(10);
	/* Enable optimal on-board timer and configure temperature sensor */
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_PWR_MGMT_1, REG_VAL_RUN_MODE | (static_cast<uint8_t>(!mConfig.mTemp.mEnabled) << 3));

	/* Enable both sensors */
	if (!mConfig.mGyro.mEnabled)
	{
		sensorsFlag |= REG_VAL_DISABLE_GYRO;
	}
	if (!mConfig.mAcc.mEnabled)
	{
		sensorsFlag |= REG_VAL_DISABLE_ACC;
	}
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_PWR_MGMT_2, sensorsFlag);
	sleepMS(10);

	/* Reset all settings on magnetometer.
	 * NOTE: this will log error as the value is immediately changed back to 0 by the sensor itself.  */
	magI2CWrite(REG_ADD_MAG_CNTL3, REG_VAL_MAG_RESET);
	sleepMS(100);
}

void ICM20948::configureGyro()
{
	uint8_t sampleRateDivisor = mConfig.mGyro.mSampleRateDivisor;
	mGyroScale = static_cast<float>((mConfig.mGyro.mRange + 1) * 250) / 32768;

	switch (mConfig.mGyro.mAveraging)
	{
		case GYRO_AVERAGING_1X:
			sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor,   1);
			break;
		case GYRO_AVERAGING_2X:
			sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor,   2);
			break;
		case GYRO_AVERAGING_4X:
			sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor,   3);
			break;
		case GYRO_AVERAGING_8X:
			sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor,   5);
			break;
		case GYRO_AVERAGING_16X:
			sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor,  10);
			break;
		case GYRO_AVERAGING_32X:
			sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor,  22);
			break;
		case GYRO_AVERAGING_64X:
			sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor,  63);
			break;
		case GYRO_AVERAGING_128X:
			sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 255);
			break;
		case GYRO_AVERAGING_NONE:
			break;
		default:
#ifdef LOG
			printf("configureGyro:: this enum shouldn't be processed: %d !!! \n", static_cast<int>(mConfig.mGyro.mAveraging));
#endif /* LOG */
			break;
	}

	setBank(BANK_2);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, sampleRateDivisor & 0xFF);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1, mConfig.mGyro.mDLPFBandwidth | (mConfig.mGyro.mRange << 1));

	if (mConfig.mGyro.mAveraging > GYRO_AVERAGING_NONE)
	{
		mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_2, mConfig.mGyro.mAveraging);
	}
}

void ICM20948::configureAcc()
{
	ACC_DLPF_BANDWIDTH bandwidth = mConfig.mAcc.mDLPFBandwidth;
	uint16_t sampleRateDivisor = mConfig.mAcc.mSampleRateDivisor;

	mAccScale = powf(2.0, mConfig.mAcc.mRange + 1) / 32768;

	if (ACC_AVERAGING_NONE < mConfig.mAcc.mAveraging)
	{
		bandwidth = ACC_DLPF_BANDWIDTH_473HZ;
		switch (mConfig.mAcc.mAveraging)
		{
			case ACC_AVERAGING_4X:
				sampleRateDivisor = std::max<uint16_t>(mConfig.mAcc.mSampleRateDivisor, 3);
				break;
			case ACC_AVERAGING_8X:
				sampleRateDivisor = std::max<uint16_t>(mConfig.mAcc.mSampleRateDivisor, 5);
				break;
			case ACC_AVERAGING_16X:
				sampleRateDivisor = std::max<uint16_t>(mConfig.mAcc.mSampleRateDivisor, 7);
				break;
			case ACC_AVERAGING_32X:
				sampleRateDivisor = std::max<uint16_t>(mConfig.mAcc.mSampleRateDivisor, 10);
				break;
			case ACC_AVERAGING_NONE:
				break;
			default:
#ifdef LOG
				printf("configureAcc:: this enum shouldn't be processed: %d !!! \n", static_cast<int>(mConfig.mAcc.mAveraging));
#endif /* LOG */
				break;
		}
	}

	setBank(BANK_2);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_1, static_cast<uint8_t>(sampleRateDivisor >> 8) & 0x0F);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2, static_cast<uint8_t>(sampleRateDivisor     ) & 0xFF);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG, bandwidth | (mConfig.mAcc.mRange << 1));

	if (ACC_AVERAGING_NONE < mConfig.mAcc.mAveraging)
	{
		mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG_2, mConfig.mAcc.mAveraging);
	}
}

bool ICM20948::configureMag() const
{
	uint8_t u8Data[MAG_DATA_LEN];
	int counter = 0;
	bool flag = checkMag();

	while (!flag && (++counter <= 10))
	{
		sleepMS(100);
		flag = checkMag();
	}

	if (flag)
	{
#ifdef LOG
		puts("Found Magnetometer!");
#endif /* LOG */
		sleepMS(1000);
		magI2CWrite(REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);

		sleepMS(10);
		magI2CRead(REG_ADD_MAG_DATA, MAG_DATA_LEN, u8Data);
	}
#ifdef LOG
	else
	{
		puts("Failed to detect the magnetometer!");
	}
#endif /* LOG */
	return flag;
}

void ICM20948::configureTemp() const
{
	setBank(BANK_2);
	/* configure temperature sensor */
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_TEMP_CONFIG, mConfig.mTemp.mDLPFBandwidth);
}

void ICM20948::configureMasterI2C() const
{
	uint8_t temp;
	setBank(BANK_0);
	/* Read the current user control and update it with new configuration to enable I2C master */
	temp = mI2C.readByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, temp | REG_VAL_BIT_I2C_MST_EN);

	/* Set I2C master clock to recommended value. */
	setBank(BANK_3);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_MST_CTRL, REG_VAL_I2C_MST_CTRL_CLK_400KHZ);

	sleepMS(10);
}

void ICM20948::readRawGyro(int16_t gyro[3]) const
{
    uint8_t u8Buf[GYRO_AND_ACC_DATA_LEN];
	setBank(BANK_0);
    mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_GYRO_XOUT_H, GYRO_AND_ACC_DATA_LEN, u8Buf);
    gyro[0] = (static_cast<int16_t>(u8Buf[0]) << 8) | static_cast<int16_t>(u8Buf[1]);
    gyro[1] = (static_cast<int16_t>(u8Buf[2]) << 8) | static_cast<int16_t>(u8Buf[3]);
    gyro[2] = (static_cast<int16_t>(u8Buf[4]) << 8) | static_cast<int16_t>(u8Buf[5]);
}

void ICM20948::readRawAcc(int16_t acc[3]) const
{
    uint8_t u8Buf[GYRO_AND_ACC_DATA_LEN];
	setBank(BANK_0);
    mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_ACCEL_XOUT_H, GYRO_AND_ACC_DATA_LEN, u8Buf);
    acc[0] = (static_cast<int16_t>(u8Buf[0]) << 8) | static_cast<int16_t>(u8Buf[1]);
    acc[1] = (static_cast<int16_t>(u8Buf[2]) << 8) | static_cast<int16_t>(u8Buf[3]);
    acc[2] = (static_cast<int16_t>(u8Buf[4]) << 8) | static_cast<int16_t>(u8Buf[5]);
}

bool ICM20948::readRawMag(int16_t mag[3]) const
{
    uint8_t u8Buf[MAG_DATA_LEN];
	setBank(BANK_0);
    mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00, MAG_DATA_LEN, u8Buf);
    mag[0] =  (static_cast<int16_t>(u8Buf[1]) << 8) | static_cast<int16_t>(u8Buf[0]);
    mag[1] =-((static_cast<int16_t>(u8Buf[3]) << 8) | static_cast<int16_t>(u8Buf[2]));
    mag[2] =-((static_cast<int16_t>(u8Buf[5]) << 8) | static_cast<int16_t>(u8Buf[4]));
    /* Check if there was an overflow. */
    return (u8Buf[7] & 0x08);
}

int16_t ICM20948::readRawTemp() const
{
    uint8_t u8Buf[2];
	setBank(BANK_0);
    mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_TEMP_OUT_H, 2, u8Buf);
    return (static_cast<int16_t>(u8Buf[0]) << 8) | static_cast<int16_t>(u8Buf[1]);
}

bool ICM20948::readAllRawDAta(int16_t gyro[3], int16_t acc[3], int16_t mag[3], int16_t& temp)
{
	setBank(BANK_0);
    mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_ACCEL_XOUT_H, IMU_DATA_LEN, mRawDataBuf);

    /* Parse accelerometer data */
    acc[0]  =  (static_cast<int16_t>(mRawDataBuf[ 0]) << 8) | static_cast<int16_t>(mRawDataBuf[ 1]);
    acc[1]  =  (static_cast<int16_t>(mRawDataBuf[ 2]) << 8) | static_cast<int16_t>(mRawDataBuf[ 3]);
    acc[2]  =  (static_cast<int16_t>(mRawDataBuf[ 4]) << 8) | static_cast<int16_t>(mRawDataBuf[ 5]);

    /* Parse gyroscope data */
    gyro[0] =  (static_cast<int16_t>(mRawDataBuf[ 6]) << 8) | static_cast<int16_t>(mRawDataBuf[ 7]);
    gyro[1] =  (static_cast<int16_t>(mRawDataBuf[ 8]) << 8) | static_cast<int16_t>(mRawDataBuf[ 9]);
    gyro[2] =  (static_cast<int16_t>(mRawDataBuf[10]) << 8) | static_cast<int16_t>(mRawDataBuf[11]);

    /* Parse temperature data */
    temp 	=  (static_cast<int16_t>(mRawDataBuf[12]) << 8) | static_cast<int16_t>(mRawDataBuf[13]);

    /* Parse magnetic data */
    mag[0]  =  (static_cast<int16_t>(mRawDataBuf[15]) << 8) | static_cast<int16_t>(mRawDataBuf[14]);
    mag[1]  =-((static_cast<int16_t>(mRawDataBuf[17]) << 8) | static_cast<int16_t>(mRawDataBuf[16]));
    mag[2]  =-((static_cast<int16_t>(mRawDataBuf[19]) << 8) | static_cast<int16_t>(mRawDataBuf[18]));

    return !(mRawDataBuf[21] & 0x08);
}

bool ICM20948::checkMag() const
{
    uint8_t u8Ret[2];
    magI2CRead(REG_ADD_MAG_WIA1, 2, u8Ret);
#ifdef LOG
    if ((u8Ret[0] != REG_VAL_MAG_WIA1) || (u8Ret[1] != REG_VAL_MAG_WIA2))
    {
    	printf("Failed to obtain magnetometer address. Expected: %02x%02x, but received: %02x%02x Current bank: %d \n",
    			REG_VAL_MAG_WIA1, REG_VAL_MAG_WIA2, u8Ret[0], u8Ret[1], mCurrentBank);
    }
#endif /* LOG */
    return ((u8Ret[0] == REG_VAL_MAG_WIA1) && (u8Ret[1] == REG_VAL_MAG_WIA2));
}

void ICM20948::magI2CRead(const uint8_t regAddr, const uint8_t length, uint8_t* data) const
{
    setBank(BANK_3);
    mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ);
    mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG,  regAddr);
    mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | length);

    setBank(BANK_0);
    mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00, length, data);
}

void ICM20948::magI2CWrite(const uint8_t regAddr, const uint8_t value) const
{
	uint8_t u8Temp;
	setBank(BANK_3);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_WRITE);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG,  regAddr);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_DO,   value);
	mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | 1);

	sleepMS(100);
	magI2CRead(regAddr, 1, &u8Temp);
#ifdef LOG
	if (value != u8Temp)
	{
		printf("Failed to write %d to magnetometer address: %d. Data received: %d. Current bank: %d \n",
				value, regAddr, u8Temp, mCurrentBank);
	}
#endif /* LOG */
}
