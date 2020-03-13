/*
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MPU9250_h
#define MPU9250_h

#include <Arduino.h>
#include <I2C.h>    // I2C library

class MPU9250 {
  public:
    enum GyroRange
    {
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G    
    };
    enum DlpfBandwidth
    {
      DLPF_BANDWIDTH_184HZ,
      DLPF_BANDWIDTH_92HZ,
      DLPF_BANDWIDTH_41HZ,
      DLPF_BANDWIDTH_20HZ,
      DLPF_BANDWIDTH_10HZ,
      DLPF_BANDWIDTH_5HZ
    };
    enum LpAccelOdr
    {
      LP_ACCEL_ODR_0_24HZ = 0,
      LP_ACCEL_ODR_0_49HZ = 1,
      LP_ACCEL_ODR_0_98HZ = 2,
      LP_ACCEL_ODR_1_95HZ = 3,
      LP_ACCEL_ODR_3_91HZ = 4,
      LP_ACCEL_ODR_7_81HZ = 5,
      LP_ACCEL_ODR_15_63HZ = 6,
      LP_ACCEL_ODR_31_25HZ = 7,
      LP_ACCEL_ODR_62_50HZ = 8,
      LP_ACCEL_ODR_125HZ = 9,
      LP_ACCEL_ODR_250HZ = 10,
      LP_ACCEL_ODR_500HZ = 11
    };
    MPU9250(I2C &bus,uint8_t address);
    int begin();
    int setAccelRange(AccelRange range);
    int setGyroRange(GyroRange range);
    int setDlpfBandwidth(DlpfBandwidth bandwidth);
    int setSrd(uint8_t srd);
    int enableDataReadyInterrupt();
    int disableDataReadyInterrupt();
    bool isDataReady();
    bool tryReadSensor();
    int readSensor();
    float getAccelX_g();
    float getAccelY_g();
    float getAccelZ_g();
    float getGyroX_rads();
    float getGyroY_rads();
    float getGyroZ_rads();
    float getMagX_uT();
    float getMagY_uT();
    float getMagZ_uT();
    float getTemperature_C();
    
    int calibrateGyro();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
    void setGyroBiasX_rads(float bias);
    void setGyroBiasY_rads(float bias);
    void setGyroBiasZ_rads(float bias);
    float getAccelBiasX_g();
    float getAccelBiasY_g();
    float getAccelBiasZ_g();
    void getAccelTM(float (&)[3][3]);
    void setAccelCalX(float bias);
    void setAccelCalY(float bias);
    void setAccelCalZ(float bias);
    void setAccelTM(const float (&&)[3][3]);
    float getMagBiasX_uT();
    float getMagBiasY_uT();
    float getMagBiasZ_uT();
    void getMagTM(float (&)[3][3]);
    void setMagCalX(float bias);
    void setMagCalY(float bias);
    void setMagCalZ(float bias);
    void setMagTM(const float (&&)[3][3]);
  protected:
    // i2c
    uint8_t _address;
    I2C *_i2c;
    size_t _numBytes; // number of bytes received from I2C
    // buffer for reading from sensor
    uint8_t _buffer[21];
    // data counts
    int16_t _axcounts,_aycounts,_azcounts;
    int16_t _gxcounts,_gycounts,_gzcounts;
    int16_t _hxcounts,_hycounts,_hzcounts;
    int16_t _tcounts;
    // data buffer
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _hx, _hy, _hz;
    float _t;
    // scale factors
    float _accelScale;
    float _gyroScale;
    float _magScaleX, _magScaleY, _magScaleZ;
    static const float _tempScale = 333.87f;
    static const float _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    DlpfBandwidth _bandwidth;
    uint8_t _srd;
    // gyro bias estimation
    static const size_t _numSamples = 400;
    double _gxbD, _gybD, _gzbD;
    float _gxb, _gyb, _gzb;
    // accel bias estimation and transformation matrix
    float _axb, _ayb, _azb;
    float _aTM[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    // magnetometer bias estimation and transformation matrix
    float _hxb, _hyb, _hzb;
    float _hTM[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    // constants
    static const float _d2r = PI/180.0f;
    // MPU9250 registers
    static const uint8_t ACCEL_OUT = 0x3B;
    static const uint8_t GYRO_OUT = 0x43;
    static const uint8_t TEMP_OUT = 0x41;
    static const uint8_t EXT_SENS_DATA_00 = 0x49;
    static const uint8_t ACCEL_CONFIG = 0x1C;
    static const uint8_t ACCEL_FS_SEL_2G = 0x00;
    static const uint8_t ACCEL_FS_SEL_4G = 0x08;
    static const uint8_t ACCEL_FS_SEL_8G = 0x10;
    static const uint8_t ACCEL_FS_SEL_16G = 0x18;
    static const uint8_t GYRO_CONFIG = 0x1B;
    static const uint8_t GYRO_FS_SEL_250DPS = 0x00;
    static const uint8_t GYRO_FS_SEL_500DPS = 0x08;
    static const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
    static const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
    static const uint8_t ACCEL_CONFIG2 = 0x1D;
    static const uint8_t ACCEL_DLPF_184 = 0x01;
    static const uint8_t ACCEL_DLPF_92 = 0x02;
    static const uint8_t ACCEL_DLPF_41 = 0x03;
    static const uint8_t ACCEL_DLPF_20 = 0x04;
    static const uint8_t ACCEL_DLPF_10 = 0x05;
    static const uint8_t ACCEL_DLPF_5 = 0x06;
    static const uint8_t CONFIG = 0x1A;
    static const uint8_t GYRO_DLPF_184 = 0x01;
    static const uint8_t GYRO_DLPF_92 = 0x02;
    static const uint8_t GYRO_DLPF_41 = 0x03;
    static const uint8_t GYRO_DLPF_20 = 0x04;
    static const uint8_t GYRO_DLPF_10 = 0x05;
    static const uint8_t GYRO_DLPF_5 = 0x06;
    static const uint8_t SMPDIV = 0x19;
    static const uint8_t INT_PIN_CFG = 0x37;
    static const uint8_t INT_ENABLE = 0x38;
    static const uint8_t INT_STATUS = 0x3A;
    static const uint8_t RAW_DATA_RDY_INT = 0x01;
    static const uint8_t INT_DISABLE = 0x00;
    static const uint8_t INT_PULSE_50US = 0x00;
    static const uint8_t INT_WOM_EN = 0x40;
    static const uint8_t INT_RAW_RDY_EN = 0x01;
    static const uint8_t PWR_MGMNT_1 = 0x6B;
    static const uint8_t PWR_CYCLE = 0x20;
    static const uint8_t PWR_RESET = 0x80;
    static const uint8_t CLOCK_SEL_PLL = 0x01;
    static const uint8_t PWR_MGMNT_2 = 0x6C;
    static const uint8_t SEN_ENABLE = 0x00;
    static const uint8_t DIS_GYRO = 0x07;
    static const uint8_t USER_CTRL = 0x6A;
    static const uint8_t I2C_MST_EN = 0x20;
    static const uint8_t I2C_MST_CLK = 0x0D;
    static const uint8_t I2C_MST_CTRL = 0x24;
    static const uint8_t I2C_SLV0_ADDR = 0x25;
    static const uint8_t I2C_SLV0_REG = 0x26;
    static const uint8_t I2C_SLV0_DO = 0x63;
    static const uint8_t I2C_SLV0_CTRL = 0x27;
    static const uint8_t I2C_SLV0_EN = 0x80;
    static const uint8_t I2C_READ_FLAG = 0x80;
    static const uint8_t MOT_DETECT_CTRL = 0x69;
    static const uint8_t ACCEL_INTEL_EN = 0x80;
    static const uint8_t ACCEL_INTEL_MODE = 0x40;
    static const uint8_t LP_ACCEL_ODR = 0x1E;
    static const uint8_t WOM_THR = 0x1F;
    static const uint8_t WHO_AM_I = 0x75;
    static const uint8_t FIFO_EN = 0x23;
    static const uint8_t FIFO_TEMP = 0x80;
    static const uint8_t FIFO_GYRO = 0x70;
    static const uint8_t FIFO_ACCEL = 0x08;
    static const uint8_t FIFO_MAG = 0x01;
    static const uint8_t FIFO_COUNT = 0x72;
    static const uint8_t FIFO_READ = 0x74;
    // AK8963 registers
    static const uint8_t AK8963_I2C_ADDR = 0x0C;
    static const uint8_t AK8963_HXL = 0x03; 
    static const uint8_t AK8963_CNTL1 = 0x0A;
    static const uint8_t AK8963_PWR_DOWN = 0x00;
    static const uint8_t AK8963_CNT_MEAS1 = 0x12;
    static const uint8_t AK8963_CNT_MEAS2 = 0x16;
    static const uint8_t AK8963_FUSE_ROM = 0x0F;
    static const uint8_t AK8963_CNTL2 = 0x0B;
    static const uint8_t AK8963_RESET = 0x01;
    static const uint8_t AK8963_ASA = 0x10;
    static const uint8_t AK8963_WHO_AM_I = 0x00;
    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int writeAK8963Register(uint8_t subAddress, uint8_t data);
    int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
    int whoAmIAK8963();
};

#endif
