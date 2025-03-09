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

#pragma once


#include <cstdint>
#include <i2c.h>
#include "imu_data.h"

class ICM20948
{
public:
    enum ACC_RANGE
    {
        ACC_RANGE_2G  = 0,
        ACC_RANGE_4G  = 1,
        ACC_RANGE_8G  = 2,
        ACC_RANGE_16G = 3
    };

    enum ACC_DLPF_BANDWIDTH
    {
        ACC_DLPF_NONE              = 0b00000000,
        ACC_DLPF_BANDWIDTH_246HZ = 0b00001001,
        ACC_DLPF_BANDWIDTH_111HZ = 0b00010001,
        ACC_DLPF_BANDWIDTH_50HZ     = 0b00011001,
        ACC_DLPF_BANDWIDTH_24HZ     = 0b00100001,
        ACC_DLPF_BANDWIDTH_12HZ     = 0b00101001,
        ACC_DLPF_BANDWIDTH_6HZ     = 0b00110001,
        ACC_DLPF_BANDWIDTH_473HZ = 0b00111001
    };

    enum ACC_AVERAGING
    {
        ACC_AVERAGING_NONE =-1,
        ACC_AVERAGING_4X   = 0,
        ACC_AVERAGING_8X   = 1,
        ACC_AVERAGING_16X  = 2,
        ACC_AVERAGING_32X  = 3
    };

    struct AccConfig
    {
        /** The flag that indicates if the accelerometer should be enabled. */
        bool mEnabled;
        /** The range in which accelerometer should operate. */
        ACC_RANGE mRange;
        /** The bandwidth of digital low-pass filter. */
        ACC_DLPF_BANDWIDTH mDLPFBandwidth;
        /** The averaging of accelerometer output. When set mDLPFBandwidth will be set to 7 and mSampleRateDivisor may be adjusted.*/
        ACC_AVERAGING mAveraging;
        /** Sample rate divisor. Sample rate is calculated as 1.128kHz / (1 + mSampleRateDivisor) */
        uint16_t mSampleRateDivisor;

        /**
         * Initialise configuration structure with default values.
         */
        AccConfig() : mEnabled(true), mRange(ACC_RANGE_2G), mDLPFBandwidth(ACC_DLPF_BANDWIDTH_6HZ),
                          mAveraging(ACC_AVERAGING_4X), mSampleRateDivisor(4) {};
    };

      enum GYRO_RANGE
    {
        GYRO_RANGE_250DPS  = 0,
        GYRO_RANGE_500DPS  = 1,
        GYRO_RANGE_1000DPS = 2,
        GYRO_RANGE_2000DPS = 3
    };

    enum GYRO_RANGE_DLPF_BANDWIDTH
    {
        GYRO_DLPF_NONE              = 0b00000000,
        GYRO_DLPF_BANDWIDTH_197HZ = 0b00000001,
        GYRO_DLPF_BANDWIDTH_152HZ = 0b00001001,
        GYRO_DLPF_BANDWIDTH_120HZ = 0b00010001,
        GYRO_DLPF_BANDWIDTH_51HZ  = 0b00011001,
        GYRO_DLPF_BANDWIDTH_24HZ  = 0b00100001,
        GYRO_DLPF_BANDWIDTH_12HZ  = 0b00101001,
        GYRO_DLPF_BANDWIDTH_6HZ      = 0b00110001,
        GYRO_DLPF_BANDWIDTH_361HZ = 0b00111001
    };

    enum GYRO_AVERAGING
    {
        GYRO_AVERAGING_NONE = -1,
        GYRO_AVERAGING_1X   =  0,
        GYRO_AVERAGING_2X   =  1,
        GYRO_AVERAGING_4X   =  2,
        GYRO_AVERAGING_8X   =  3,
        GYRO_AVERAGING_16X  =  4,
        GYRO_AVERAGING_32X  =  5,
        GYRO_AVERAGING_64X  =  6,
        GYRO_AVERAGING_128X =  7
    };

    struct GyroConfig
    {
        /** The flag that indicates if the gyroscope should be enabled. */
        bool mEnabled;
        /** The range in which gyroscope should operate. */
        GYRO_RANGE mRange;
        /** The bandwidth of digital low-pass filter. */
        GYRO_RANGE_DLPF_BANDWIDTH mDLPFBandwidth;
        /** The averaging of gyroscope output. DLPF needs to be enabled for this to be used. mSampleRateDivisor may be adjusted. */
        GYRO_AVERAGING mAveraging;
        /** Sample rate divisor. Sample rate is calculated as 1.128kHz / (1 + mSampleRateDivisor) */
        uint8_t mSampleRateDivisor;

        /**
         * Initialise configuration structure with default values.
         */
        GyroConfig() : mEnabled(true), mRange(GYRO_RANGE_250DPS), mDLPFBandwidth(GYRO_DLPF_BANDWIDTH_6HZ),
                           mAveraging(GYRO_AVERAGING_4X), mSampleRateDivisor(4) {};
    };

    enum TEMP_DLPF_BANDWIDTH
    {
        TEMP_DLPF_NONE              = 0,
        TEMP_DLPF_BANDWIDTH_218HZ = 1,
        TEMP_DLPF_BANDWIDTH_124HZ = 2,
        TEMP_DLPF_BANDWIDTH_66HZ  = 3,
        TEMP_DLPF_BANDWIDTH_34HZ  = 4,
        TEMP_DLPF_BANDWIDTH_17HZ  = 5,
        TEMP_DLPF_BANDWIDTH_9HZ      = 6
    };

    struct TempConfig
    {
        /** The flag that indicates if the temperature sensor should be enabled. */
        bool mEnabled;
        /** The bandwidth of digital low-pass filter. */
        TEMP_DLPF_BANDWIDTH mDLPFBandwidth;

        /**
         * Initialise configuration structure with default values.
         */
        TempConfig() : mEnabled(true), mDLPFBandwidth(TEMP_DLPF_BANDWIDTH_9HZ) {};
    };

    enum AHRS_ALGORITHM
    {
        NONE     = 0,
        SIMPLE      = 1,
        MADGWICK = 2
    };

    struct Config
    {
        /** The I2C device address. */
        char mDevice[1024];
        /** The gyroscope configuration data. */
        GyroConfig mGyro;
        /** The accelerometer configuration data. */
        AccConfig mAcc;
        /** The temperature sensor configuration data. */
        TempConfig mTemp;
        /** The flag that indicates if the compass should be enabled. */
        bool mMagEnabled;
        /** The AHRS algorithm to use for sensors fusion. */
        AHRS_ALGORITHM mAHRS;
        /** The frame rate in which data will be processed for AHRS. */
        float mFramerate;

        Config() : mDevice("/dev/i2c-0"), mGyro(), mAcc(), mTemp(), mMagEnabled(true), mAHRS(MADGWICK), mFramerate(100.0f) {};
    };

    /**
     * Basic constructor, initialises all variables.
     */
    ICM20948();

    /**
     * Destructor - closes the serial connection if it was opened.
     */
    virtual ~ICM20948();

    /**
     * Initialises the IMU with provided configuration data.
     *  @param config the IMU configuration.
     */
    bool initialise(const Config& config = Config());

    /**
     *  @return the current configuration.
     */
    constexpr const Config& getConfig() const
    {
        return mConfig;
    }

    /**
     * Pulls the data from the device and processes it.
     *  @return a reference to the latest IMU data (not thread-safe).
     */
    const IMUData& imuDataGet();

    /**
     * Calibrates the gyroscope by averaging 1024 samples of gyro and uploading them as a bias to the device.
     * @note make sure to be stationary!
     */
    void calibrateGyro() const;

private:
    enum ICM_BANK
    {
        BANK_0            = 0x00,
        BANK_1            = 0x10,
        BANK_2            = 0x20,
        BANK_3            = 0x30,
        BANK_UNDEFINED = 0xFF
    };

    /**
     * Switches to new memory bank if not already in it.
     *  @param bank the new memory bank.
     */
    void setBank(const ICM_BANK bank) const;

    /**
     * Resets the device and all its settings.
     */
    void reset() const;

    /**
     * Configures gyroscope with data from the internal configuration structure.
     */
    void configureGyro();

    /**
     * Configures accelerometer with data from the internal configuration structure.
     */
    void configureAcc();

    /**
     * Configures magnetometer with data from the internal configuration structure.
     */
    bool configureMag() const;

    /**
     * Configures temperature sensor with data from the internal configuration structure.
     */
    void configureTemp() const;

    /**
     * Configures the ICM device to master on I2C bus so that it may pull updates from magnetometer.
     */
    void configureMasterI2C() const;

    /**
     * Reads raw gyro data.
     *  @param[out] gyro the raw gyro data for all three axes.
     */
    void readRawGyro(int16_t gyro[3]) const;

    /**
     * Reads raw acceleration data.
     *  @param[out] acc the raw acceleration data for all three axes.
     */
    void readRawAcc(int16_t acc[3]) const;

    /**
     * Reads raw magnetometer data.
     *  @param[out] mag the raw magnetometer data for all three axes.
     *  @return the flag indicating magnetometer overflow.
     */
    bool readRawMag(int16_t mag[3]) const;

    /**
     *  @return the raw temperature data.
     */
    int16_t readRawTemp() const;

    /**
     * Reads all data from the device.
     *  @param[out] gyro the raw gyro data for all three axes.
     *  @param[out] acc the raw acceleration data for all three axes.
     *  @param[out] mag the raw magnetometer data for all three axes.
     *  @param[out] temp the raw temperature data.
     *  @return the flag indicating magnetometer overflow.
     */
    bool readAllRawDAta(int16_t gyro[3], int16_t acc[3], int16_t mag[3], int16_t& temp);

    /**
     *  @return true if magnetometer was detected.
     */
    bool checkMag() const;

    /**
     * Reads data from AK09916 sensor.
     *  @param regAddr the address of the register from which the read should begin.
     *  @param length the number of bytes to read (the length of @p data).
     *  @param[out] data preallocated buffer for read data.
     */
    void magI2CRead(const uint8_t regAddr, const uint8_t length, uint8_t* data) const;

    /**
     * Writes a single byte data to AK09916 sensor.
     *  @param regAddr the address of the register from which the read should begin.
     *  @param value a new data to upload to the device.
     */
    void magI2CWrite(const uint8_t regAddr, const uint8_t value) const;

    /** Current memory bank. */
    mutable ICM_BANK mCurrentBank;
    /** IMU configuration. */
    Config mConfig;
    /** Scale for gyroscope - depends on configuration. */
    float mGyroScale;
    /** Scale for accelerometer - depends on configuration. */
    float mAccScale;
    /** Processed IMU data. */
    IMUData mData;
    /** Preallocated buffer for raw IMU data. */
    uint8_t mRawDataBuf[22];
    /** Class for controlling I2C communication. */
    I2C mI2C;
};
