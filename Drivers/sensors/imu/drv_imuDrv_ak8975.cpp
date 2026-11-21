#include "drv_imuDrv_ak8975.hpp"
#include "basic.hpp"
#include "drv_spi.hpp"

imuDrv_ak8975::imuDrv_ak8975(SPI_CLASS *spi, GPIO_TypeDef *cs_gpio, uint8_t cs_pin, const int8_t axis[3])
    : spi(spi)
    , cs_gpio(cs_gpio)
    , cs_pin(cs_pin)
    , driver_type(IMU_DRIVER_TYPE_NULL)
    , freqMag(100)
{
    axis_index[0] = axis[0];
    axis_index[1] = axis[1];
    axis_index[2] = axis[2];
    asa[0] = asa[1] = asa[2] = 1.0;

    // CS 初始化为推挽输出，拉高
    set_register(cs_gpio->MODER, 0b01u, 2 * cs_pin, 2);
    set_register(cs_gpio->OTYPER, 0u, 1 * cs_pin, 1);
    set_register(cs_gpio->OSPEEDR, 2u, 2 * cs_pin, 2);
    pu_CS();
}

IMU_DRIVER_TYPE imuDrv_ak8975::init()
{
    set_register(cs_gpio->MODER, 0b01u, 2 * cs_pin, 2);
    set_register(cs_gpio->OTYPER, 0u, 1 * cs_pin, 1);
    set_register(cs_gpio->OSPEEDR, 2u, 2 * cs_pin, 2);
    pu_CS();
    os_delay(0.01);

#ifdef DCACHE_SIZE
#define AK_TX_BUF_SIZE DCACHE_SIZE
    Aligned_DMABuf uint8_t tx_buf[AK_TX_BUF_SIZE];
    uint8_t *rx_buf = tx_buf;
#else
#define AK_TX_BUF_SIZE 32
    uint8_t tx_buf[AK_TX_BUF_SIZE];
    uint8_t *rx_buf = tx_buf;
#endif

    // 验证 Chip ID（WIA 寄存器 0x00，期望值 0x48）
    tx_buf[0] = (1u << 7) | 0x00u;
    tx_buf[1] = 0;
    tx_buf[2] = 0;
    pd_CS();
    spi->writeNread(tx_buf, 3, AK_TX_BUF_SIZE, rx_buf, AK_TX_BUF_SIZE);
    pu_CS();
    if (rx_buf[1] != 0x48)
        return IMU_DRIVER_TYPE_NULL;
    os_delay(0.01);

    // 禁用 I2C 接口（I2CDIS 寄存器 0x0F = 0x1B）
    tx_buf[0] = (0u << 7) | 0x0Fu;
    tx_buf[1] = 0x1Bu;
    pd_CS();
    spi->write(tx_buf, 2, AK_TX_BUF_SIZE);
    pu_CS();
    os_delay(0.01);

    // 进入 Fuse 模式读取 ASA 校准数据（CNTL = 0x0F）
    tx_buf[0] = (0u << 7) | 0x0Au;
    tx_buf[1] = 0x0Fu;
    pd_CS();
    spi->write(tx_buf, 2, AK_TX_BUF_SIZE);
    pu_CS();
    os_delay(0.01);

    // 读取 ASA X/Y/Z（0x10~0x12）
    tx_buf[0] = (1u << 7) | 0x10u;
    tx_buf[1] = tx_buf[2] = tx_buf[3] = 0;
    pd_CS();
    spi->writeNread(tx_buf, 4, AK_TX_BUF_SIZE, rx_buf, AK_TX_BUF_SIZE);
    pu_CS();
    asa[0] = (rx_buf[1] - 128) * 0.5 / 128.0 + 1.0;
    asa[1] = (rx_buf[2] - 128) * 0.5 / 128.0 + 1.0;
    asa[2] = (rx_buf[3] - 128) * 0.5 / 128.0 + 1.0;
    os_delay(0.01);

    // 回到 Power-down 模式
    tx_buf[0] = (0u << 7) | 0x0Au;
    tx_buf[1] = 0x00u;
    pd_CS();
    spi->write(tx_buf, 2, AK_TX_BUF_SIZE);
    pu_CS();
    os_delay(0.01);

    // 发送第一次 Single Measurement 指令，之后每次 sample() 末尾触发下次采样
    tx_buf[0] = (0u << 7) | 0x0Au;
    tx_buf[1] = 0x01u;
    pd_CS();
    spi->write(tx_buf, 2, AK_TX_BUF_SIZE);
    pu_CS();
    os_delay(0.01);

    driver_type = IMU_DRIVER_TYPE_MAG;
    return driver_type;
}

bool imuDrv_ak8975::sample(uint8_t sId, vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, double *temperature, IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2,
                           IMU_DATA_STATUS *status3)
{
#ifdef DCACHE_SIZE
#define AK_TX_BUF_SIZE DCACHE_SIZE
    Aligned_DMABuf uint8_t tx_buf[AK_TX_BUF_SIZE];
    uint8_t *rx_buf = tx_buf;
#else
#define AK_TX_BUF_SIZE 32
    uint8_t tx_buf[AK_TX_BUF_SIZE];
    uint8_t *rx_buf = tx_buf;
#endif

    // 读取测量结果：ST1(0x02) + HX/HY/HZ(0x03-0x08) + ST2(0x09)，共 8 字节（+1 地址字节）
    // 寄存器布局：reg[0x03]=HXL, [0x04]=HXH, [0x05]=HYL, [0x06]=HYH, [0x07]=HZL, [0x08]=HZH
    struct __AK8975_Data
    {
        uint8_t st1;    // 0x02 状态
        int16_t mag[3]; // 0x03-0x08  HX HY HZ
        uint8_t st2;    // 0x09 状态2
    } __AC_PACKED;

    tx_buf[0] = (1u << 7) | 0x02u;
    for (uint8_t i = 1; i < sizeof(__AK8975_Data) + 1; ++i)
        tx_buf[i] = 0;

    pd_CS();
    spi->writeNread(tx_buf, sizeof(__AK8975_Data) + 1, AK_TX_BUF_SIZE, rx_buf, AK_TX_BUF_SIZE);
    pu_CS();

    __AK8975_Data *d = (__AK8975_Data *) (rx_buf + 1);

    // HOFL（溢出）位在 ST2 bit3
    bool overflow = (d->st2 & (1u << 3)) != 0;

    // 应用 ASA 灵敏度校准（按原始轴）
    double raw[3];
    raw[0] = d->mag[0] * asa[0];
    raw[1] = d->mag[1] * asa[1];
    raw[2] = d->mag[2] * asa[2];

    // 轴向映射
    if (axis_index[0] > 0)
        rx_data1->x = (int32_t) raw[axis_index[0] - 1];
    else
        rx_data1->x = (int32_t) (-raw[-axis_index[0] - 1]);
    if (axis_index[1] > 0)
        rx_data1->y = (int32_t) raw[axis_index[1] - 1];
    else
        rx_data1->y = (int32_t) (-raw[-axis_index[1] - 1]);
    if (axis_index[2] > 0)
        rx_data1->z = (int32_t) raw[axis_index[2] - 1];
    else
        rx_data1->z = (int32_t) (-raw[-axis_index[2] - 1]);

    *temperature = -1000;
    *status1 = overflow ? IMU_DATA_STATUS_ERROR : IMU_DATA_STATUS_HEALTHY;

    // 触发下一次 Single Measurement
    tx_buf[0] = (0u << 7) | 0x0Au;
    tx_buf[1] = 0x01u;
    pd_CS();
    spi->write(tx_buf, 2, AK_TX_BUF_SIZE);
    pu_CS();

    return true;
}
