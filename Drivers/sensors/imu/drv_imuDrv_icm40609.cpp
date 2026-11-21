#include "drv_imuDrv_icm40609.hpp"
#include "drv_spi.hpp"

#include "basic.hpp"

imuDrv_icm40609::imuDrv_icm40609(SPI_CLASS *spi, GPIO_TypeDef *cs_gpio, uint8_t cs_pin, const int8_t axis[3])
    : spi(spi)
    , csGpio(cs_gpio)
    , csPin(cs_pin)
    , driver_type(IMU_DRIVER_TYPE_NULL)
    , freqGyro(1000)
    , freqAccel(1000)
    , name("icm40609")
    , gyro_sensitivity(0.001064225153655)
    , accel_sensitivity(GravityAcc / 1024.0)
{
    axis_index[0] = axis[0];
    axis_index[1] = axis[1];
    axis_index[2] = axis[2];

    // cs
    set_register(csGpio->MODER, 0b01, 2 * csPin, 2);
    set_register(csGpio->OTYPER, 0, 1 * csPin, 1);
    set_register(csGpio->OSPEEDR, 2, 2 * csPin, 2);
    pu_CS();
}

IMU_DRIVER_TYPE imuDrv_icm40609::init()
{
    // cs
    set_register(csGpio->MODER, 0b01, 2 * csPin, 2);
    set_register(csGpio->OTYPER, 0, 1 * csPin, 1);
    set_register(csGpio->OSPEEDR, 2, 2 * csPin, 2);
    pu_CS();

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
    // 防止 SPI 总线一开始异常导致 ID 读取不到直接初始化失败
    bool initOK = false;
    for (uint8_t retryCnt = 0; retryCnt < 3 && !initOK; ++retryCnt)
    {
        // 先复位，让芯片从确定状态启动
        tx_buf[0] = (0 << 7) | 17;
        tx_buf[1] = (1 << 0);
        pd_CS();
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.3);

        // 校验Chip ID
        tx_buf[0] = (1 << 7) | 117;
        pd_CS();
        spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);
        if (rx_buf[1] != 0x3b)
        {
            os_delay(0.05);
            continue; // 本轮失败重试
        }

        // 设置spi引脚速度
        tx_buf[0] = (0 << 7) | 19;
        tx_buf[1] = (0b011 << 0);
        pd_CS();
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.3);

        // 设置 ICM40609 陀螺量程:±2000dps 2000hz
        tx_buf[0] = (0 << 7) | 79;
        tx_buf[1] = (0b000 << 5) | (0b0101 << 0);
        pd_CS();
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);

        // 设置 ICM40609 加计量程: ±32G 2000hz
        tx_buf[0] = (0 << 7) | 80;
        tx_buf[1] = (0b000 << 5) | (0b0101 << 0);
        pd_CS();
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);

        // 设置 ICM40609  陀螺LPF： 温度20hz ui滤波器二阶 m2滤波器三阶
        tx_buf[0] = (0 << 7) | 81;
        tx_buf[1] = (0b100 << 5) | (0b01 << 2) | (0b10 << 0);
        pd_CS();
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);

        // 设置 ICM40609 LPF：加速度BW=ODR/16  陀螺BW=ODR/10
        tx_buf[0] = (0 << 7) | 82;
        tx_buf[1] = (5 << 4) | (4 << 0);
        pd_CS();
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);

        // 设置 ICM40609  加速度LPF： ui滤波器二阶 m2滤波器三阶
        tx_buf[0] = (0 << 7) | 83;
        tx_buf[1] = (0b01 << 3) | (0b10 << 1);
        pd_CS();
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);

        // 使能 ICM40609 加计和陀螺
        tx_buf[0] = (0 << 7) | 78;
        tx_buf[1] = (0b11 << 2) | (0b11 << 0);
        pd_CS();
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);

        initOK = true;
    } // for retryCnt

    if (!initOK)
        return IMU_DRIVER_TYPE_NULL;

    driver_type = IMU_DRIVER_TYPE_GYROACCEL;
    return driver_type;
}

bool imuDrv_icm40609::sample(uint8_t sId, vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, double *temperature, IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2,
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

    struct __SENSOR_Data
    {
        uint8_t rsv1;
        int16_t temperature;
        int16_t acc[3];
        int16_t gyro[3];
    } __AC_PACKED;
    if (sId == 0)
    { // 陀螺+加速度
        pd_CS();
        tx_buf[0] = (1 << 7) | 29;
        spi->writeNread(tx_buf, sizeof(__SENSOR_Data), TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_CS();

        __SENSOR_Data *rData = (__SENSOR_Data *) rx_buf;
        rData->gyro[0] = __REV16(rData->gyro[0]);
        rData->gyro[1] = __REV16(rData->gyro[1]);
        rData->gyro[2] = __REV16(rData->gyro[2]);
        rData->acc[0] = __REV16(rData->acc[0]);
        rData->acc[1] = __REV16(rData->acc[1]);
        rData->acc[2] = __REV16(rData->acc[2]);

        *temperature = -1000;

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
        *status1 = IMU_DATA_STATUS_HEALTHY;

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

        *status2 = IMU_DATA_STATUS_HEALTHY;

        return true;
    }

    return false;
}
