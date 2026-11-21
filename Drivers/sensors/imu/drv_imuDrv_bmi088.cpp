#include "drv_imuDrv_bmi088.hpp"
#include "drv_spi.hpp"

#include "basic.hpp"

imuDrv_bmi088::imuDrv_bmi088(SPI_CLASS *spi, GPIO_TypeDef *gyro_cs_gpio, uint8_t gyro_cs_pin, GPIO_TypeDef *accel_cs_gpio, uint8_t accel_cs_pin, const int8_t axis[3])
    : spi(spi)
    , gyroCs_gpio(gyro_cs_gpio)
    , gyroCs_pin(gyro_cs_pin)
    , accelCs_gpio(accel_cs_gpio)
    , accelCs_pin(accel_cs_pin)
    , driver_type(IMU_DRIVER_TYPE_NULL)
    , freqGyro(2000)
    , freqAccel(1000)
    , name("bmi088")
    , gyro_sensitivity(0.00106526443603169529841533860381)
    , accel_sensitivity(GravityAcc / 1365.0)
{
    axis_index[0] = axis[0];
    axis_index[1] = axis[1];
    axis_index[2] = axis[2];

    // gyro cs
    set_register(gyroCs_gpio->MODER, 0b01, 2 * gyroCs_pin, 2);
    set_register(gyroCs_gpio->OTYPER, 0, 1 * gyroCs_pin, 1);
    set_register(gyroCs_gpio->OSPEEDR, 2, 2 * gyroCs_pin, 2);
    pu_Gyro_CS();

    // accel cs
    set_register(accelCs_gpio->MODER, 0b01, 2 * accelCs_pin, 2);
    set_register(accelCs_gpio->OTYPER, 0, 1 * accelCs_pin, 1);
    set_register(accelCs_gpio->OSPEEDR, 2, 2 * accelCs_pin, 2);
    pu_Accel_CS();
}

IMU_DRIVER_TYPE imuDrv_bmi088::init()
{
    // gyro cs
    set_register(gyroCs_gpio->MODER, 0b01, 2 * gyroCs_pin, 2);
    set_register(gyroCs_gpio->OTYPER, 0, 1 * gyroCs_pin, 1);
    set_register(gyroCs_gpio->OSPEEDR, 2, 2 * gyroCs_pin, 2);
    pu_Gyro_CS();

    // accel cs
    set_register(accelCs_gpio->MODER, 0b01, 2 * accelCs_pin, 2);
    set_register(accelCs_gpio->OTYPER, 0, 1 * accelCs_pin, 1);
    set_register(accelCs_gpio->OSPEEDR, 2, 2 * accelCs_pin, 2);
    pu_Accel_CS();

    os_delay(0.01);

#ifdef DCACHE_SIZE
#define TX_BUF_SIZE DCACHE_SIZE
#define RX_BUF_SIZE TX_BUF_SIZE
    Aligned_DMABuf uint8_t tx_buf[TX_BUF_SIZE];
    uint8_t *rx_buf = tx_buf;
#else
#define TX_BUF_SIZE 32
#define RX_BUF_SIZE TX_BUF_SIZE
    uint8_t tx_buf[TX_BUF_SIZE];
    uint8_t *rx_buf = tx_buf;
#endif

    // 完整初始化整体重试 3 次
    bool initOK = false;
    for (uint8_t retryCnt = 0; retryCnt < 3 && !initOK; ++retryCnt)
    {
        /*加速度计初始化*/
        // 使加速度计进入spi模式
        pd_Accel_CS();
        tx_buf[0] = (1 << 7) | 0;
        spi->writeNread(tx_buf, 3, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_Accel_CS();
        os_delay(0.01);

        // 复位
        pd_Accel_CS();
        tx_buf[0] = (0 << 7) | 0x7e;
        tx_buf[1] = 0xb6;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Accel_CS();
        os_delay(0.05);

        // 使加速度计进入spi模式
        pd_Accel_CS();
        tx_buf[0] = (1 << 7) | 0;
        spi->writeNread(tx_buf, 3, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_Accel_CS();
        os_delay(0.01);

        // 读取序列号
        pd_Accel_CS();
        tx_buf[0] = (1 << 7) | 0;
        spi->writeNread(tx_buf, 3, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_Accel_CS();
        if (rx_buf[2] != 0x1e)
        {
            os_delay(0.05);
            continue; // 本轮失败重试
        }
        os_delay(0.01);

        // 打开加速度计
        pd_Accel_CS();
        tx_buf[0] = (0 << 7) | 0x7d;
        tx_buf[1] = 0x04;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Accel_CS();
        os_delay(0.01);

        // 加速度计ODR 800hz
        pd_Accel_CS();
        tx_buf[0] = (0 << 7) | 0x40;
        tx_buf[1] = (0x8 << 4) | (0xB << 0);
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Accel_CS();
        os_delay(0.01);

        // 加速度计量程24g
        pd_Accel_CS();
        tx_buf[0] = (0 << 7) | 0x41;
        tx_buf[1] = 0x03;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Accel_CS();
        os_delay(0.01);

        // 进入Active模式
        pd_Accel_CS();
        tx_buf[0] = (0 << 7) | 0x7c;
        tx_buf[1] = 0x00;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Accel_CS();
        os_delay(0.01);
        /*加速度计初始化*/

        /*陀螺仪初始化*/
        // 复位
        pd_Gyro_CS();
        tx_buf[0] = (0 << 7) | 0x14;
        tx_buf[1] = 0xb6;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Gyro_CS();
        os_delay(0.05);

        // 读取序列号
        pd_Gyro_CS();
        tx_buf[0] = (1 << 7) | 0;
        spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_Gyro_CS();
        if (rx_buf[1] != 0x0f)
        {
            os_delay(0.05);
            continue; // 本轮失败重试
        }
        os_delay(0.01);

        // GYRO_LPM1(0x11): normal mode
        pd_Gyro_CS();
        tx_buf[0] = (0 << 7) | 0x11;
        tx_buf[1] = 0x00;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Gyro_CS();
        os_delay(0.01);

        // GYRO_RANGE(0x0f): 2000deg/s
        pd_Gyro_CS();
        tx_buf[0] = (0 << 7) | 0x0f;
        tx_buf[1] = 0x00;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Gyro_CS();
        os_delay(0.01);

        // GYRO_BANDWIDTH(0x10): ODR=2000hz bandwidth=230hz
        pd_Gyro_CS();
        tx_buf[0] = (0 << 7) | 0x10;
        tx_buf[1] = 0x02;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_Gyro_CS();
        os_delay(0.01);
        /*陀螺仪初始化*/

        initOK = true;
    } // for retryCnt

    if (!initOK)
        return IMU_DRIVER_TYPE_NULL;

    driver_type = IMU_DRIVER_TYPE_GYRO_ACCEL;
    return driver_type;
}

bool imuDrv_bmi088::sample(uint8_t sId, vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, double *temperature, IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2,
                           IMU_DATA_STATUS *status3)
{
#ifdef DCACHE_SIZE
#define TX_BUF_SIZE DCACHE_SIZE
#define RX_BUF_SIZE TX_BUF_SIZE
    Aligned_DMABuf uint8_t tx_buf[TX_BUF_SIZE];
    uint8_t *rx_buf = tx_buf;
#else
#define TX_BUF_SIZE 32
#define RX_BUF_SIZE TX_BUF_SIZE
    uint8_t tx_buf[TX_BUF_SIZE];
    uint8_t *rx_buf = tx_buf;
#endif

    if (sId == 0)
    { // 陀螺
        struct __BMI088_GyroData
        {
            uint8_t rsv1;
            int16_t gyro[3]; // 0x02-0x07
        } __AC_PACKED;

        pd_Gyro_CS();
        tx_buf[0] = (1 << 7) | 0x02;
        spi->writeNread(tx_buf, sizeof(__BMI088_GyroData), TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_Gyro_CS();

        __BMI088_GyroData *rData = (__BMI088_GyroData *) rx_buf;
        if (axis_index[0] > 0)
            rx_data1->x = rData->gyro[axis_index[0] - 1];
        else
            rx_data1->x = -rData->gyro[-axis_index[0] - 1];
        if (axis_index[1] > 0)
            rx_data1->y = rData->gyro[axis_index[1] - 1];
        else
            rx_data1->y = -rData->gyro[-axis_index[1] - 1];
        if (axis_index[2] > 0)
            rx_data1->z = rData->gyro[axis_index[2] - 1];
        else
            rx_data1->z = -rData->gyro[-axis_index[2] - 1];
        *temperature = -1000;
        *status1 = IMU_DATA_STATUS_HEALTHY;

        return true;
    }
    else if (sId == 1)
    { // 加速度
        struct __BMI088_AccelData
        {
            uint8_t rsv1[2];
            int16_t acc[3];            // 0x12-0x17
            uint32_t sensor_time : 24; // 0x18-0x1a
            uint8_t rsv2[2];           // 0x1b-0x1c
            uint8_t acc_int_stat;      // 0x1d
            uint8_t rsv3[4];           // 0x1e-0x21
            uint16_t temperature;      // 0x22-0x23
        } __AC_PACKED;

        pd_Accel_CS();
        tx_buf[0] = (1 << 7) | 0x12;
        spi->writeNread(tx_buf, sizeof(__BMI088_AccelData), TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_Accel_CS();

        __BMI088_AccelData *rData = (__BMI088_AccelData *) rx_buf;
        if (axis_index[0] > 0)
            rx_data2->x = rData->acc[axis_index[0] - 1];
        else
            rx_data2->x = -rData->acc[-axis_index[0] - 1];
        if (axis_index[1] > 0)
            rx_data2->y = rData->acc[axis_index[1] - 1];
        else
            rx_data2->y = -rData->acc[-axis_index[1] - 1];
        if (axis_index[2] > 0)
            rx_data2->z = rData->acc[axis_index[2] - 1];
        else
            rx_data2->z = -rData->acc[-axis_index[2] - 1];
        *temperature = -1000;
        *status2 = IMU_DATA_STATUS_HEALTHY;
        return true;
    }

    return false;
}