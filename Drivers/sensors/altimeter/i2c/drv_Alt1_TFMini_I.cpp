#include "drv_Alt1_TFMini_I.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "MeasurementSystem.hpp"
#include "SensorsBackend.hpp"
#include "drv_i2c.hpp"

#define SensorInd 2

struct DriverInfo
{
    uint32_t sensor_key;
    I2C_CLASS *i2c_port;
};

typedef struct
{
    uint16_t header;
    uint16_t Dist;     // Dist距离（30-1200cm）
    uint16_t Strength; // Strength信号强度（20-2000可信）
    uint16_t temp;
    uint8_t checksum;
} __PACKED _TfMini;

static void TFMini_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    I2C_CLASS *i2c_port = driver_info.i2c_port;

    Aligned_DMABuf uint8_t tx_buf[DCACHE_SIZE];
    Aligned_DMABuf uint8_t rx_buf[DCACHE_SIZE];

    tx_buf[0] = 0x5a;
    tx_buf[1] = 0x05;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x01;
    tx_buf[4] = 0x60;

    I2C_Result res;
    _TfMini *data;
    while (1)
    {
        os_delay(0.1);

        i2c_port->write(0x10, tx_buf, 5, sizeof(tx_buf), I2C_SPEED_100k);
        res = i2c_port->read(0x10, rx_buf, 9, sizeof(rx_buf), I2C_SPEED_100k);
        if (res == I2C_Result_Success)
        {
            data = (_TfMini *) rx_buf;

            if (data->Strength > 20 && data->Dist > 1 && data->Dist < 25000)
            {
                vector3<double> position;
                position.z = data->Dist;
                // 获取倾角
                Quaternion quat;
                get_Airframe_quat(&quat);
                double lean_cosin = quat.get_lean_angle_cosin();
                // 更新
                position.z *= lean_cosin;
                PositionSensorUpdatePosition(SensorInd, driver_info.sensor_key, position, true);
            }
            else
                PositionSensorSetInavailable(SensorInd, driver_info.sensor_key);
        }
        else
            PositionSensorSetInavailable(SensorInd, driver_info.sensor_key);
    }
}

static bool I2C_TFMini_DriverInit(I2C_CLASS *i2c)
{
    return true;
}
static bool I2C_TFMini_DriverRun(I2C_CLASS *i2c)
{
    uint32_t sensor_key = PositionSensorRegister(SensorInd, "TFMini", Position_Sensor_Type_RangePositioning, Position_Sensor_DataType_s_z, Position_Sensor_frame_ENU, 0.01, // 延时
                                                 0,                                                                                                                         // xy信任度
                                                 0                                                                                                                          // z信任度
    );
    if (sensor_key == 0)
        return false;
    DriverInfo *driver_info = new DriverInfo;
    driver_info->sensor_key = sensor_key;
    driver_info->i2c_port = i2c;
    xTaskCreate(TFMini_Server, "TFMini", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_Alt1_TFMini_I()
{
    I2CFunc_Register(33, I2C_TFMini_DriverInit, I2C_TFMini_DriverRun);
}
