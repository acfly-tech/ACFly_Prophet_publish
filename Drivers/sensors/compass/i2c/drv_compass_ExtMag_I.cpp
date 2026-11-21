#include "Basic.hpp"
#include "Commulink.hpp"
#include "HardWareConfig.hpp"
#include "SensorsBackend.hpp"
#include "drv_Compass.hpp"
#include "drv_i2c.hpp"
static void ExtMag_Server(void *pvParameters)
{
    I2C_CompassInfo *info = (I2C_CompassInfo *) pvParameters;
    I2C_CLASS *i2c_port = info->i2c;
    const I2C_MagnetoMeter *MagnetoMeters = info->meters;
    const uint8_t MagnetoMeter_Count = info->count;
    delete info;
ScanExtMag:
    // 当前使用的外置罗盘型号
    int8_t current_ExtMag = -1;
    // 缓冲区
    Aligned_DMABuf uint8_t tx_buf[32];
    Aligned_DMABuf uint8_t rx_buf[32];
    // 前期采样延时
    uint16_t sample_delay = 0;
    // 传感器key
    uint32_t key = 0;

    // 操作结果
    I2C_Result res;
    while (1)
    {
        for (int8_t i = 0; i < MagnetoMeter_Count; ++i)
        {
            vTaskDelay(0.1 * configTICK_RATE_HZ);
            const I2C_MAG_PROTOCOL *sensor = MagnetoMeters[i].protocol;
            const uint8_t dev_addr = MagnetoMeters[i].addr_override ? MagnetoMeters[i].addr_override : sensor->device_address;

            // 判断传感器ID是否正确
            tx_buf[0] = sensor->ID_address;
            res = i2c_port->writeNread(dev_addr, tx_buf, 1, sizeof(tx_buf), rx_buf, 1, sizeof(rx_buf), sensor->iic_speed);
            if (res != I2C_Result_Success)
                continue;
            if (rx_buf[0] != sensor->ID)
                continue;

            // 发送传感器配置
            for (uint8_t k = 0; k < sensor->configuration_count; ++k)
            {
                tx_buf[0] = sensor->configurations_addresses[k];
                tx_buf[1] = sensor->configurations[k];
                res = i2c_port->write(dev_addr, tx_buf, 2, sizeof(tx_buf), sensor->iic_speed);
                if (res != I2C_Result_Success)
                    break;
            }
            if (res != I2C_Result_Success)
                continue;

            // 检查传感器配置是否成功
            for (uint8_t k = 0; k < sensor->configuration_count; ++k)
            {
                tx_buf[0] = sensor->configurations_addresses[k];
                res = i2c_port->writeNread(dev_addr, tx_buf, 1, sizeof(tx_buf), rx_buf, 1, sizeof(rx_buf), sensor->iic_speed);
                if (res != I2C_Result_Success)
                    break;
                if (rx_buf[0] != sensor->configurations[k])
                { // 配置校验错误
                    res = I2C_Result_ERROR;
                    break;
                }
            }
            if (res != I2C_Result_Success)
                continue;

            // 已成功识别到传感器
            key = IMUMagnetometerRegister(External_Magnetometer_Index, SName("e") + SName(sensor->name), sensor->sensitivity);
            if (key)
            {
                current_ExtMag = i;
                goto ExtMagDetected;
            }
        }
    }

ExtMagDetected:
    sample_delay = 0;
    const I2C_MagnetoMeter *meter = &MagnetoMeters[current_ExtMag];
    const I2C_MAG_PROTOCOL *sensor = meter->protocol;
    const uint8_t dev_addr = meter->addr_override ? meter->addr_override : sensor->device_address;
    int32_t raw[3] = {0};
    while (1)
    {
        // 周期对传感器采样
        // vTaskDelay(sensor->sample_mseconds * 1e-3 * configTICK_RATE_HZ + 1);

        // 采样
        uint8_t rt = 0;
        do
        {
            // 周期对传感器采样
            // 改变delay位置，避免do_while里面读取过快,导致某些磁罗盘未就绪就读取的问题
            vTaskDelay(sensor->sample_mseconds * 1e-3 * configTICK_RATE_HZ + 1);

            if (++rt > 3)
            {
                IMUMagnetometerUnRegister(External_Magnetometer_Index, key);
                goto ScanExtMag;
            }

            raw[0] = raw[1] = raw[2] = 0;

            if (sensor->need_status_check)
            {
                // 先检查状态寄存器
                tx_buf[0] = sensor->status_address;
                res = i2c_port->writeNread(dev_addr, tx_buf, 1, sizeof(tx_buf), rx_buf, 1, sizeof(rx_buf), sensor->iic_speed);

                if (res != I2C_Result_Success || (rx_buf[0] & sensor->status_ready_mask) != sensor->status_ready_value)
                {
                    // 未就绪退出循环
                    continue;
                }
            }

            // 传感器读取
            tx_buf[0] = sensor->data_address;
            res = i2c_port->writeNread(dev_addr, tx_buf, 1, sizeof(tx_buf), rx_buf, sensor->data_bytes, sizeof(rx_buf), sensor->iic_speed);

            // ====================== 统一数据转换 ======================
            if (res == I2C_Result_Success)
            {
                if (sensor->data_bytes == 9) // 24-bit (RM3100)
                {
                    uint32_t mag_data_x = ((uint32_t) rx_buf[0] << 16) | ((uint32_t) rx_buf[1] << 8) | rx_buf[2];
                    if (mag_data_x >= 0x00800000)
                        mag_data_x |= 0xff000000;

                    uint32_t mag_data_y = ((uint32_t) rx_buf[3] << 16) | ((uint32_t) rx_buf[4] << 8) | rx_buf[5];
                    if (mag_data_y >= 0x00800000)
                        mag_data_y |= 0xff000000;

                    uint32_t mag_data_z = ((uint32_t) rx_buf[6] << 16) | ((uint32_t) rx_buf[7] << 8) | rx_buf[8];
                    if (mag_data_z >= 0x00800000)
                        mag_data_z |= 0xff000000;

                    raw[0] = (int32_t) mag_data_x * 0.1;
                    raw[1] = (int32_t) mag_data_y * 0.1;
                    raw[2] = (int32_t) mag_data_z * 0.1;
                }
                else // 16-bit
                {
                    if (sensor->MSB)
                    {
                        ((uint16_t *) rx_buf)[0] = __REV16(((uint16_t *) rx_buf)[0]);
                        ((uint16_t *) rx_buf)[1] = __REV16(((uint16_t *) rx_buf)[1]);
                        ((uint16_t *) rx_buf)[2] = __REV16(((uint16_t *) rx_buf)[2]);
                    }

                    raw[0] = (signed short) ((uint16_t *) rx_buf)[0];
                    raw[1] = (signed short) ((uint16_t *) rx_buf)[1];
                    raw[2] = (signed short) ((uint16_t *) rx_buf)[2];
                }
            }

        } while (res != I2C_Result_Success);

        // ====================== 统一触发一次采样（POLL 或 手动采样） ======================
        if (sensor->need_set_sample)
        {
            tx_buf[0] = sensor->sample_address;
            tx_buf[1] = sensor->sample_cfg;
            I2C_Result r2 = i2c_port->write(dev_addr, tx_buf, 2, sizeof(tx_buf), sensor->iic_speed);
            if (r2 != I2C_Result_Success)
                res = r2;
        }
        // ====================== 统一轴向转换 ======================
        vector3<int32_t> data;

        if (meter->axis_index[0] > 0)
            data.x = raw[meter->axis_index[0] - 1];
        else
            data.x = -raw[(-meter->axis_index[0]) - 1];

        if (meter->axis_index[1] > 0)
            data.y = raw[meter->axis_index[1] - 1];
        else
            data.y = -raw[(-meter->axis_index[1]) - 1];

        if (meter->axis_index[2] > 0)
            data.z = raw[meter->axis_index[2] - 1];
        else
            data.z = -raw[(-meter->axis_index[2]) - 1];
        // ====================== 更新数据 ======================
        if (sample_delay >= 50)
        {
            IMUMagnetometerUpdate(External_Magnetometer_Index, key, data, false);
        }
        else
        {
            ++sample_delay;
        }
    }
}

static bool I2C_ExtMag_DriverInit(I2C_CLASS *i2c)
{
    return true;
}
static bool I2C_ExtMag_DriverRun(I2C_CLASS *i2c)
{
    // 遍历所有协议, 查找配置文件中定义了轴向的罗盘
    uint8_t count = 0;
    for (uint8_t i = 0; i < I2C_Mag_Protocol_Count; i++)
    {
        HW_ConfigEntry entry;
        if (HW_ReadConfigEntry(HW_DEV_I2C_COMPASS_EXTERNAL, I2C_Mag_Protocols[i].device, &entry))
            count++;
    }
    if (count == 0)
        return false;

    I2C_CompassInfo *info = new I2C_CompassInfo;
    info->i2c = i2c;
    info->count = count;
    info->meters = new I2C_MagnetoMeter[count];
    uint8_t idx = 0;
    for (uint8_t i = 0; i < I2C_Mag_Protocol_Count; i++)
    {
        HW_ConfigEntry entry;
        if (HW_ReadConfigEntry(HW_DEV_I2C_COMPASS_EXTERNAL, I2C_Mag_Protocols[i].device, &entry))
        {
            info->meters[idx].protocol = &I2C_Mag_Protocols[i];
            HW_DecodeAxis(entry.value_low, info->meters[idx].axis_index);
            info->meters[idx].addr_override = (uint8_t) entry.value_high;
            idx++;
        }
    }

    xTaskCreate(ExtMag_Server, "extMag", 800, (void *) info, SysPriority_UserTask, NULL);
    return true;
}

void init_drv_compass_ExtMag_I()
{
    I2CFunc_Register(9, I2C_ExtMag_DriverInit, I2C_ExtMag_DriverRun);
}