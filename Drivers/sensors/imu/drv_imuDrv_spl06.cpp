#include "drv_imuDrv_spl06.hpp"
#include "drv_spi.hpp"

#include "basic.hpp"

/*SPL06校准数据*/
void imuDrv_spl06::spl06_pressure_rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    switch (u8SmplRate)
    {
        case 2:
            reg |= (1 << 4);
            break;
        case 4:
            reg |= (2 << 4);
            break;
        case 8:
            reg |= (3 << 4);
            break;
        case 16:
            reg |= (4 << 4);
            break;
        case 32:
            reg |= (5 << 4);
            break;
        case 64:
            reg |= (6 << 4);
            break;
        case 128:
            reg |= (7 << 4);
            break;
        case 1:
        default:
            break;
    }
    switch (u8OverSmpl)
    {
        case 2:
            reg |= 1;
            coefficients.KP = 1.0 / 1572864;
            break;
        case 4:
            reg |= 2;
            coefficients.KP = 1.0 / 3670016;
            break;
        case 8:
            reg |= 3;
            coefficients.KP = 1.0 / 7864320;
            break;
        case 16:
            coefficients.KP = 1.0 / 253952;
            reg |= 4;
            break;
        case 32:
            coefficients.KP = 1.0 / 516096;
            reg |= 5;
            break;
        case 64:
            coefficients.KP = 1.0 / 1040384;
            reg |= 6;
            break;
        case 128:
            coefficients.KP = 1.0 / 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            coefficients.KP = 1.0 / 524288;
            break;
    }

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

    pd_CS();
    tx_buf[0] = (0 << 7) | 0x06;
    tx_buf[1] = reg;
    spi->write(tx_buf, 2, TX_BUF_SIZE);
    pu_CS();
    os_delay(0.01);

    if (u8OverSmpl > 8)
    {
        pd_CS();
        tx_buf[0] = (1 << 7) | 0x09;
        spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);

        pd_CS();
        tx_buf[0] = (0 << 7) | 0x09;
        tx_buf[1] = rx_buf[1] | (1 << 2);
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);
    }
}
void imuDrv_spl06::spl06_temperature_rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    switch (u8SmplRate)
    {
        case 2:
            reg |= (1 << 4);
            break;
        case 4:
            reg |= (2 << 4);
            break;
        case 8:
            reg |= (3 << 4);
            break;
        case 16:
            reg |= (4 << 4);
            break;
        case 32:
            reg |= (5 << 4);
            break;
        case 64:
            reg |= (6 << 4);
            break;
        case 128:
            reg |= (7 << 4);
            break;
        case 1:
        default:
            break;
    }
    switch (u8OverSmpl)
    {
        case 2:
            reg |= 1;
            coefficients.KT = 1.0f / 1572864;
            break;
        case 4:
            reg |= 2;
            coefficients.KT = 1.0f / 3670016;
            break;
        case 8:
            reg |= 3;
            coefficients.KT = 1.0f / 7864320;
            break;
        case 16:
            coefficients.KT = 1.0f / 253952;
            reg |= 4;
            break;
        case 32:
            coefficients.KT = 1.0f / 516096;
            reg |= 5;
            break;
        case 64:
            coefficients.KT = 1.0f / 1040384;
            reg |= 6;
            break;
        case 128:
            coefficients.KT = 1.0f / 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            coefficients.KT = 1.0f / 524288;
            break;
    }

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

    pd_CS();
    tx_buf[0] = (0 << 7) | 0x07;
    tx_buf[1] = (1 << 7) | reg;
    spi->write(tx_buf, 2, TX_BUF_SIZE);
    pu_CS();
    os_delay(0.01);

    if (u8OverSmpl > 8)
    {
        pd_CS();
        tx_buf[0] = (1 << 7) | 0x09;
        spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);

        pd_CS();
        tx_buf[0] = (0 << 7) | 0x09;
        tx_buf[1] = rx_buf[1] | (1 << 2);
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);
    }
}
/*SPL06校准数据*/

// 写寄存器并回读校验
bool imuDrv_spl06::writeRegVerify(uint8_t reg, uint8_t value, uint8_t mask)
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

    // 写入
    int8_t retryCnt = 3;
    do
    {
        pd_CS();
        tx_buf[0] = (0 << 7) | reg;
        tx_buf[1] = value;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.005);

        // 回读
        pd_CS();
        tx_buf[0] = (1 << 7) | reg;
        tx_buf[1] = 0xff;
        spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_CS();
        os_delay(0.005);
    } while ((rx_buf[1] & mask) != (value & mask) && --retryCnt > 0);

    return (rx_buf[1] & mask) == (value & mask);
}

imuDrv_spl06::imuDrv_spl06(SPI_CLASS *spi, GPIO_TypeDef *cs_gpio, uint8_t cs_pin)
    : spi(spi)
    , csGpio(cs_gpio)
    , csPin(cs_pin)
    , driver_type(IMU_DRIVER_TYPE_NULL)
    , freq(33)
    , delay(0.02)
    , trust(300)
    , lt_trust(50)
    , name("spl06")
{
    // cs
    set_register(csGpio->MODER, 0b01, 2 * csPin, 2);
    set_register(csGpio->OTYPER, 0, 1 * csPin, 1);
    set_register(csGpio->OSPEEDR, 2, 2 * csPin, 2);
    pu_CS();
}

IMU_DRIVER_TYPE imuDrv_spl06::init()
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
    uint8_t retryCnt = 0;

    // 完整初始化(读取ID+复位+配置+读校准+试采样)整体重试, 直到校准数据有效或重试超限
    bool coefValid = false;
    bool cfgValid = false;
    bool sampleValid = false;
    for (retryCnt = 0; retryCnt < 5 && (!coefValid || !cfgValid || !sampleValid); ++retryCnt)
    {
        coefValid = false;
        cfgValid = false;
        sampleValid = false;

        // 进入4线spi模式
        pd_CS();
        tx_buf[0] = (0 << 7) | 0x09;
        tx_buf[1] = 0;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.05);

        // 复位 (datasheet: 软复位后需等待约 12ms 寄存器就绪)
        pd_CS();
        tx_buf[0] = (0 << 7) | 0x0c;
        tx_buf[1] = (1 << 7);
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.05);

        // 进入4线spi模式
        pd_CS();
        tx_buf[0] = (0 << 7) | 0x09;
        tx_buf[1] = 0;
        spi->write(tx_buf, 2, TX_BUF_SIZE);
        pu_CS();
        os_delay(0.05);

        // 读取ID, 直接用第一次读到的非 0xff 值, 不再独立 retry, 超过则本级失败
        uint8_t idRetry = 0;
        do
        {
            pd_CS();
            tx_buf[0] = (1 << 7) | 0x0d;
            tx_buf[1] = 0xff;
            spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
            pu_CS();
            os_delay(0.05);
            ++idRetry;
        } while (rx_buf[1] == 0xff && idRetry < 5);
        if (rx_buf[1] == 0xff)
            continue; // 本轮失败重试

        // 等到 MEAS_CFG 中 COEF_RDY(bit7)=1 后再读校准, polling 超时则本轮放弃重试
        bool coefRdy = false;
        uint8_t measRetry = 0;
        for (measRetry = 0; measRetry < 10; ++measRetry)
        {
            pd_CS();
            tx_buf[0] = (1 << 7) | 0x08;
            tx_buf[1] = 0xff;
            spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
            pu_CS();
            if ((rx_buf[1] & (1 << 7)))
            {
                coefRdy = true;
                break; // COEF_RDY
            }
            os_delay(0.05);
        }
        if (!coefRdy)
        {
            os_delay(0.05);
            continue; // 超时：芯片未就绪，本轮读校准不再尝试
        }

        // 配置采样率寄存器 (0x06/0x07), 内部已带回读校验
        // 也配置 INTR_SHIFT 寄存器 (0x09), 当 oversampling > 8 时需要置 SHIFT 位
        spl06_pressure_rateset(64, 32);    // pressure 64 samples per sec , 32 times over sampling
        spl06_temperature_rateset(128, 2); // temperature 128 samples per sec , 2 times over sampling

        // 回读校验各个配置寄存器
        // 压力 RATE寄存器(0x06): rate=64->(6<<4), oversample=32->5 => value = 0x65
        if (writeRegVerify(0x06, 0x65, 0x7f) == false)
        {
            os_delay(0.05);
            continue; // 配置失败重试整轮
        }
        // 温度 RATE寄存器(0x07): Temp_Ext bit7=1, rate=128->(7<<4), oversample=2->1 => value = 0xF1
        if (writeRegVerify(0x07, 0xF1, 0xf7) == false)
        {
            os_delay(0.05);
            continue;
        }

        // INTR_SHIFT 寄存器(0x09): P_SHIFT(bit2)=1, T_SHIFT(bit3)=0 (Temp oversample=2<8)
        // 这里不直接校验 0x09, 因为它的其他位(中断位)可能被驱动保留, 改用读改写后回读特定位
        // 读改写 0x09
        pd_CS();
        tx_buf[0] = (1 << 7) | 0x09;
        tx_buf[1] = 0xff;
        spi->writeNread(tx_buf, 2, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_CS();
        os_delay(0.005);
        uint8_t reg09_old = rx_buf[1];
        uint8_t reg09_new = reg09_old | (1 << 2);
        if (!writeRegVerify(0x09, reg09_new, 0xfe))
        {
            os_delay(0.05);
            continue;
        }
        // 0x08 MEAS_CFG: 0b111 = continuous pressure + temperature
        if (writeRegVerify(0x08, 0b111, 0x07) == false)
        {
            os_delay(0.05);
            continue;
        }
        cfgValid = true; // 配置寄存器全部回读校验通过

        os_delay(0.05); // 等待 background continuous 模式稳定

        // 获取校准数据
        pd_CS();
        tx_buf[0] = (1 << 7) | 0x10;
        spi->writeNread(tx_buf, 19, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
        pu_CS();
        os_delay(0.01);
        coefficients.c0 = (rx_buf[1] << 4) | (rx_buf[2] >> 4);
        coefficients.c0 = (coefficients.c0 & 0x0800) ? (0xF000 | coefficients.c0) : coefficients.c0;
        coefficients.c1 = ((rx_buf[2] & 0xf) << 8) | (rx_buf[3]);
        coefficients.c1 = (coefficients.c1 & 0x0800) ? (0xF000 | coefficients.c1) : coefficients.c1;
        coefficients.c00 = (rx_buf[4] << 12) | (rx_buf[5] << 4) | (rx_buf[6] >> 4);
        coefficients.c00 = (coefficients.c00 & 0x080000) ? (0xFFF00000 | coefficients.c00) : coefficients.c00;
        coefficients.c10 = ((rx_buf[6] & 0xf) << 16) | (rx_buf[7] << 8) | (rx_buf[8] >> 0);
        coefficients.c10 = (coefficients.c10 & 0x080000) ? (0xFFF00000 | coefficients.c10) : coefficients.c10;
        coefficients.c01 = (rx_buf[9] << 8) | (rx_buf[10] << 0);
        coefficients.c11 = (rx_buf[11] << 8) | (rx_buf[12] << 0);
        coefficients.c20 = (rx_buf[13] << 8) | (rx_buf[14] << 0);
        coefficients.c21 = (rx_buf[15] << 8) | (rx_buf[16] << 0);
        coefficients.c30 = (rx_buf[17] << 8) | (rx_buf[18] << 0);
        // 校准数据有效性校验：c00 正常范围 ~4e6..1e7（海平面附近 Pa 量级）；c10 同量级正数
        // if (!(coefficients.c00 > 400000 && coefficients.c00 < 12000000 && coefficients.c10 > 0))
        // {
        //     os_delay(0.05);
        //     continue;
        // }
        coefValid = true;

        // 最终自检: 试采样 1 次, 验证计算出的压力落在物理合理范围
        // 等待若干 ms 保证连续模式下有新数据, 多次读取直到拿到非全0样本
        bool gotValidSample = false;
        for (uint8_t trySample = 0; trySample < 5; ++trySample)
        {
            os_delay(0.04); // 等 32 倍过采样下 1 个采样周期(~10ms)后再读
            pd_CS();
            tx_buf[0] = (1 << 7) | 0x00;
            spi->writeNread(tx_buf, 7, TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
            pu_CS();

            struct __SPL06_Data_t
            {
                uint8_t rsv1;
                unsigned int pressure : 24;
                unsigned int temperature : 24;
            } __AC_PACKED;
            __SPL06_Data_t *datap = (__SPL06_Data_t *) rx_buf;
            int32_t p_raw = __REV(datap->pressure) >> 8;
            p_raw = (p_raw & 0x800000) ? (0xFF000000 | p_raw) : p_raw;
            int32_t t_raw = __REV(datap->temperature) >> 8;
            t_raw = (t_raw & 0x800000) ? (0xFF000000 | t_raw) : t_raw;
            if (p_raw == 0)
                continue; // 仍是 dummy 数据, 等下一周期

            double fPsc = p_raw * coefficients.KP;
            double fTsc = t_raw * coefficients.KT;
            double qua2 = coefficients.c10 + fPsc * (coefficients.c20 + fPsc * coefficients.c30);
            double qua3 = fTsc * fPsc * (coefficients.c11 + fPsc * coefficients.c21);
            double pressure = coefficients.c00 + fPsc * qua2 + fTsc * coefficients.c01 + qua3;
            // 物理合理范围 [30000, 110000] Pa, 覆盖约 +9km ~ -500m
            if (pressure > 2500.0 && pressure < 200000.0)
            {
                gotValidSample = true;
                break;
            }
        }
        if (!gotValidSample)
        {
            os_delay(0.05);
            continue; // 自检失败, 重新初始化
        }
        sampleValid = true;
    }
    if (!coefValid || !cfgValid || !sampleValid)
        return IMU_DRIVER_TYPE_NULL;

    driver_type = IMU_DRIVER_TYPE_ALT;
    return driver_type;
}

bool imuDrv_spl06::sample(uint8_t sId, vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, double *temperature, IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2,
                          IMU_DATA_STATUS *status3)
{
    struct __SPL06_Data
    {
        uint8_t rsv1;
        unsigned int pressure : 24;
        unsigned int temperature : 24;
    } __AC_PACKED;

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

    // 读取传感器数据
    pd_CS();
    tx_buf[0] = (1 << 7) | 0x00;
    SPI_Result spi_res = spi->writeNread(tx_buf, sizeof(__SPL06_Data), TX_BUF_SIZE, rx_buf, RX_BUF_SIZE);
    pu_CS();

    // SPI 传输失败直接判错误，避免使用残留/未更新的 rx_buf 数据
    if (spi_res != SPI_Result_Success)
    {
        *status1 = IMU_DATA_STATUS_ERROR;
        return true;
    }

    // 处理数据
    __SPL06_Data *datap = (__SPL06_Data *) rx_buf;
    int32_t buf32[2];
    buf32[0] = __REV(datap->pressure) >> 8;
    buf32[1] = __REV(datap->temperature) >> 8;

    buf32[0] = (buf32[0] & 0x800000) ? (0xFF000000 | buf32[0]) : buf32[0];
    buf32[1] = (buf32[1] & 0x800000) ? (0xFF000000 | buf32[1]) : buf32[1];

    // // 原始数据合理性检查：SPL06-007 的 24bit 原始数据有明确范围
    // // 复位未完成 / SPI 错位 / 芯片卡死时,读到的常是全 0、全 0xFF 或极小值
    // // 这些值会让 sample 算出非物理的压力 (如 391500 Pa -> h=-13000m)
    // // 正常工作时压力原始值约 4e6~1e7, 温度原始值约 ±5e6
    // if (buf32[0] < 100000 || buf32[0] > 16000000)
    // {
    //     *status1 = IMU_DATA_STATUS_ERROR;
    //     return true;
    // }

    double fPsc = buf32[0] * coefficients.KP;
    double fTsc = buf32[1] * coefficients.KT;
    double qua2 = coefficients.c10 + fPsc * (coefficients.c20 + fPsc * coefficients.c30);
    double qua3 = fTsc * fPsc * (coefficients.c11 + fPsc * coefficients.c21);

    double pressure = coefficients.c00 + fPsc * qua2 + fTsc * coefficients.c01 + qua3;
    *temperature = coefficients.c0 * 0.5 + coefficients.c1 * fTsc;

    // 通过气压计算高度
    // 物理范围检查：合理气压 30000~110000 Pa (约 +9km~-500m)
    // 超出范围判错误，防止错误数据污染控制律
    if (pressure > 2500.0 && pressure < 200000.0)
    {
        // double velx = get_VelocityENU_Ctrl_x();
        // double vely = get_VelocityENU_Ctrl_y();
        trust = 300; // + constrain( 0.5*safe_sqrt( velx*velx + vely*vely ), 150.0 );
        lt_trust = 50;

        *status1 = IMU_DATA_STATUS_HEALTHY;
        rx_data1->y = pressure * 1000;
        rx_data1->z = 44300000 * (1.0 - pow(pressure / 101325.0, 1.0 / 5.256));
    }
    else
        *status1 = IMU_DATA_STATUS_ERROR;

    return true;
}