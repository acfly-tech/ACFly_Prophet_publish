#pragma once

#include "IMUDriverBase.hpp"

class SPI_CLASS;

class imuDrv_bmi088 : public IMUDriverBase
{
  public:
    imuDrv_bmi088(SPI_CLASS *spi, GPIO_TypeDef *gyro_cs_gpio, uint8_t gyro_cs_pin, GPIO_TypeDef *accel_cs_gpio, uint8_t accel_cs_pin, const int8_t axis[3]);

    IMU_DRIVER_TYPE init();
    bool sample(uint8_t sId, vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, double *temperature, IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2,
                IMU_DATA_STATUS *status3);

    IMU_DRIVER_TYPE get_driverType() const
    {
        return driver_type;
    }

    void get_freq(uint16_t freq[]) const
    {
        freq[0] = freqGyro;
        freq[1] = freqAccel;
    }

    SName get_name() const
    {
        return name;
    }

    double get_param1() const
    {
        return gyro_sensitivity;
    }
    double get_param2() const
    {
        return accel_sensitivity;
    }
    double get_param3() const
    {
        return 0;
    }

  private:
    SPI_CLASS *spi;
    GPIO_TypeDef *gyroCs_gpio;
    uint8_t gyroCs_pin;
    GPIO_TypeDef *accelCs_gpio;
    uint8_t accelCs_pin;
    int8_t axis_index[3];

    IMU_DRIVER_TYPE driver_type;
    uint16_t freqGyro, freqAccel;
    SName name;

    double gyro_sensitivity;
    double accel_sensitivity;

    void pu_Gyro_CS()
    {
        gyroCs_gpio->BSRR = 1 << (gyroCs_pin + 0);
    }
    void pd_Gyro_CS()
    {
        gyroCs_gpio->BSRR = 1 << (gyroCs_pin + 16);
    }
    void pu_Accel_CS()
    {
        accelCs_gpio->BSRR = 1 << (accelCs_pin + 0);
    }
    void pd_Accel_CS()
    {
        accelCs_gpio->BSRR = 1 << (accelCs_pin + 16);
    }
};