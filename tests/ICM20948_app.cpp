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

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <time.h>
#include <unistd.h>

#include "ICM20948.h"

/** Scale to convert angles from radians to degrees. */
static const float RAD_TO_DEG = 180.0f / 3.14159f;

/**
 * Prints help information about this application.
 *  @param name the name of the executable.
 */
void printHelp(const char* name)
{
	printf("Usage: %s [options] \n", name);
	printf("    -h, --help  -> prints this information \n");
	printf("    -l, --loop  -> sets the maximum number of loop iterations \n");
	printf("    -c, --calib -> if set to one triggers gyro calibration \n");
	printf("\nExample: %s -l 2000 -c 1 \n", name);
}

/**
 *  @return the time of the system in seconds.
 */
double getTime()
{
    struct timespec timeStruct;
    clock_gettime(CLOCK_REALTIME, &timeStruct);
    return static_cast<double>(timeStruct.tv_sec) + static_cast<double>(timeStruct.tv_nsec / 1000000) / 1000;
}

int main(int argc, char** argv)
{
	/** Current time in seconds to calculate time needed to sleep for the main loop. */
	double currentTime;
	/** Loop iterator */
	ulong i;
	/** The maximum number of loops. */
	ulong max = 200;
	/** Time to sleep in milliseconds for the main loop. */
	int sleepTime;
	/** Flag to indicate of gyro calibration should be performed. */
	bool calib = true;
	/** IMU implementation. */
	ICM20948 imu;
	/** Pointer for IMU data.  */
	const IMUData* data;

	/* Parse any inputs that may have been provided. */
	for (i = 1; i < static_cast<ulong>(argc); ++i)
	{
		if ((0 == strcmp(argv[i], "--loop")) || (0 == strcmp(argv[i], "-l")))
		{
			max = static_cast<ulong>(atol(argv[i + 1]));
		}
		else if ((0 == strcmp(argv[i], "--calib")) || (0 == strcmp(argv[i], "-c")))
		{
			calib = static_cast<bool>(atoi(argv[i + 1]));
		}
		else if ((0 == strcmp(argv[i], "--help")) || (0 == strcmp(argv[i], "-h")))
		{
			printHelp(argv[0]);
			return 0;
		}
		else
		{
			/* nothing to do in here */
		}
	}

	if (imu.initialise())
	{
		puts("ICM-20948 detected successfully");
		if (calib)
		{
			puts("Performing gyroscope calibration");
			imu.calibrateGyro();
		}
		for (i = 0; i < max; ++i)
		{
			currentTime = getTime();
			data = &imu.imuDataGet();
			printf("\n /------- counter: %lu -----/ \n", i);
			printf("\n \t\t\t\t\t\tAngle：Roll: %.2f     Pitch: %.2f     Yaw: %.2f \n", data->mAngles[0], data->mAngles[1], data->mAngles[2]);
			printf("\n Acceleration(g): X: %.3f     Y: %.3f     Z: %.3f \n", data->mAcc[0], data->mAcc[1], data->mAcc[2]);
			printf("\n Gyroscope(dps): X: %.3f     Y: %.3f     Z: %.3f \n", data->mGyro[0] * RAD_TO_DEG, data->mGyro[1] * RAD_TO_DEG, data->mGyro[2] * RAD_TO_DEG);
			printf("\n Magnetic(uT): X: %.3f     Y: %.3f     Z: %.3f \n", data->mMag[0], data->mMag[1], data->mMag[2]);
			printf("\n Temperature(C): %.3f\n", data->mTemp);
			/* Calculate sleep time by taking expected period between IMU updates and reducing it by a time needed to process data. */
			sleepTime = static_cast<int>(((1.0f / imu.getConfig().mFramerate) - (getTime() - currentTime)) * 1000000.0);
			/* We use signed type in case processing takes more than expected period between updates. */
			if (sleepTime > 0)
			{
				usleep(static_cast<uint>(sleepTime));
			}
		}
	}
	else
	{
		puts("Failed to detect ICM-20948");
	}

	puts("finished");
	return 0;
}
