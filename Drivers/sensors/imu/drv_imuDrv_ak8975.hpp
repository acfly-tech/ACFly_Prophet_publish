#pragma once

#include "IMUDriverBase.hpp"

class SPI_CLASS;

class imuDrv_ak8975 : public IMUDriverBase
{
  public:
    imuDrv_ak8975(SPI_CLASS *spi, GPIO_TypeDef *cs_gpio, uint8_t cs_pin, const int8_t axis[3]);

    IMU_DRIVER_TYPE init();
    bool sample(uint8_t sId, vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, double *temperature, IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2,
                IMU_DATA_STATUS *status3);

    IMU_DRIVER_TYPE get_driverType() const
    {
        return driver_type;
    }

    void get_freq(uint16_t freq[]) const
    {
        freq[0] = freqMag;
    }

    SName get_name() const
    {
        return "ak8975";
    }

    double get_param1() const
    {
        return 0;
    }
    double get_param2() const
    {
        return 0;
    }
    double get_param3() const
    {
        // AK8975 标称灵敏度 0.3µT/LSB
        return 0.003;
    }

  private:
    SPI_CLASS *spi;
    GPIO_TypeDef *cs_gpio;
    uint8_t cs_pin;
    int8_t axis_index[3];

    IMU_DRIVER_TYPE driver_type;
    uint16_t freqMag;

    // ASA 灵敏度调整系数（init 时从 Fuse ROM 读取）
    double asa[3];

    void pu_CS()
    {
        cs_gpio->BSRR = 1u << (cs_pin + 0);
    }
    void pd_CS()
    {
        cs_gpio->BSRR = 1u << (cs_pin + 16);
    }
};
