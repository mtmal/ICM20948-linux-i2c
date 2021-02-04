# ICM20948-linux-i2c

## General Info
Linux driver for I2C communication with ICM20948. It has been tested with Waveshare's Stereo Camera with integrated IMU: https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera

The code has been tested on Jetson Nano with JetPack 4.4.1.

## Setup
Default build without internal logging
```
$ mkdir build
$ cd build
$ cmake ..
$ make -j 4
$ ./test_ICM20948 -l 2000 -c 1
```
To build with additional logging, pass -DLOG=1 to cmake like:
```
$ cmake .. -DLOG=1
```

## Credit
* This project is based on an example code provided by Waveshare.
* This project includes Madgwick AHRS algorithm from https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

## Discussion
There are few existing solutions of ICM20948 driver (e.g. original from Invensense or drivers for Arduino), but I decided to diverge from them with the following:
* Invensense code seems a little bit too messy to me to use it as-is. There are also some errors when compiling with `-pedantic -Wall -Wextra` flags so it would require detailed review.
* Sensor fusion algorithm is executed on CPU rather than on DMP. Unfortunately, I couldn't find any documentation regarding writing custom code for DMP and there is no information (or very scarce) regarding the existing DMP firmware which can be found on Invensense website. Is it doing any fusion? Which algorithm is implemented? Using Madgwick AHRS on CPU seems like a better option.
* Some implementations perform gyroscope calibration by configuring FIFO to accumulate several updates. However, I believe that gryroscope needs to run for few seconds to stabilise before uploading average value as bias, especially with DLPF enabled.
* I do not perform acceleration calibration. One needs to know the exact orientation of IMU to calibrate any existing acceleratometer bias in dynamic environment. Without knowing that orientation, calibration may do more harm than good. 
