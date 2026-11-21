#include "drv_avoidance_TFMini_I.hpp"

#include "Avoidance.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "basic.hpp"
#include "drv_i2c.hpp"

struct _TfMini
{
    uint16_t header;
    uint16_t Dist;     // Dist距离（30-1200cm）
    uint16_t Strength; // Strength信号强度（20-2000可信）
    uint16_t temp;
    uint8_t checksum;
} __AC_PACKED;

static void avoidance_TFMini_I_Server(void *pvParameters)
{
    I2C_CLASS *i2c_port = (I2C_CLASS *) pvParameters;
    // 缓冲区
    Aligned_DMABuf uint8_t tx_buf[DCACHE_SIZE];
    Aligned_DMABuf uint8_t rx_buf[DCACHE_SIZE];
    uint16_t tx_buf_size = sizeof(tx_buf);
    uint16_t rx_buf_size = sizeof(rx_buf);

    uint32_t sensor_key1 = registere_AvTarget();
    uint32_t sensor_key2 = registere_AvTarget();
    uint32_t sensor_key3 = registere_AvTarget();
    uint32_t sensor_key4 = registere_AvTarget();

    tx_buf[0] = 0x5a;
    tx_buf[1] = 0x05;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x01;
    tx_buf[4] = 0x60;

    // 获取避障参数
    const AvoidanceCfg *avCfg = getAvCfg();

    I2C_Result res;
    _TfMini *data;
reTry:
    while (1)
    {
        i2c_port->write(0x10, tx_buf, 5, tx_buf_size, I2C_SPEED_100k);
        i2c_port->write(0x11, tx_buf, 5, tx_buf_size, I2C_SPEED_100k);
        i2c_port->write(0x12, tx_buf, 5, tx_buf_size, I2C_SPEED_100k);
        i2c_port->write(0x13, tx_buf, 5, tx_buf_size, I2C_SPEED_100k);

        res = i2c_port->read(0x10, rx_buf, 9, rx_buf_size, I2C_SPEED_100k);
        if (res == I2C_Result_Success)
        {
            data = (_TfMini *) rx_buf;
            vector2<double> dis;
            dis.x = data->Dist;
            if (data->Dist > avCfg->wheelbase[0] && data->Dist < 65500)
                set_AvTargetXYStraightLine_RelativeFlu(sensor_key1, dis);
            else
                set_AvTargetInavailable(sensor_key1);
        }
        else
            set_AvTargetInavailable(sensor_key1);

        res = i2c_port->read(0x11, rx_buf, 9, rx_buf_size, I2C_SPEED_100k);
        if (res == I2C_Result_Success)
        {
            data = (_TfMini *) rx_buf;
            vector2<double> dis;
            dis.y = data->Dist;
            if (data->Dist > avCfg->wheelbase[0] && data->Dist < 65500)
                set_AvTargetXYStraightLine_RelativeFlu(sensor_key2, dis);
            else
                set_AvTargetInavailable(sensor_key2);
        }
        else
            set_AvTargetInavailable(sensor_key2);

        res = i2c_port->read(0x12, rx_buf, 9, rx_buf_size, I2C_SPEED_100k);
        if (res == I2C_Result_Success)
        {
            data = (_TfMini *) rx_buf;
            vector2<double> dis;
            dis.x = -data->Dist;
            if (data->Dist > avCfg->wheelbase[0] && data->Dist < 65500)
                set_AvTargetXYStraightLine_RelativeFlu(sensor_key3, dis);
            else
                set_AvTargetInavailable(sensor_key3);
        }
        else
            set_AvTargetInavailable(sensor_key3);

        res = i2c_port->read(0x13, rx_buf, 9, rx_buf_size, I2C_SPEED_100k);
        if (res == I2C_Result_Success)
        {
            data = (_TfMini *) rx_buf;
            vector2<double> dis;
            dis.y = -data->Dist;
            if (data->Dist > avCfg->wheelbase[0] && data->Dist < 65500)
                set_AvTargetXYStraightLine_RelativeFlu(sensor_key4, dis);
            else
                set_AvTargetInavailable(sensor_key4);
        }
        else
            set_AvTargetInavailable(sensor_key4);

        os_delay(0.05);
    }
}

static bool avoidance_TFMini_I_DriverInit(I2C_CLASS *i2c)
{
    return true;
}
static bool avoidance_TFMini_I_DriverRun(I2C_CLASS *i2c)
{
    xTaskCreate(avoidance_TFMini_I_Server, "IICTFMiniAv", 800, (void *) i2c, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_avoidance_TFMini_I()
{
    I2CFunc_Register(96, avoidance_TFMini_I_DriverInit, avoidance_TFMini_I_DriverRun);
}
