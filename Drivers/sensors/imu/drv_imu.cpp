#include "drv_imu.hpp"
#include "FreeRTOS.h"
#include "MS_Main.hpp"
#include "SensorsBackend.hpp"
#include "basic.hpp"
#include "queue.h"
#include "timers.h"

#include "HardWareConfig.hpp"
#include "IMUDriverBase.hpp"
#include "drv_spi.hpp"

#include "drv_imuDrv_ak8975.hpp"
#include "drv_imuDrv_bmi088.hpp"
#include "drv_imuDrv_icm20689.hpp"
#include "drv_imuDrv_icm40609.hpp"

#include "drv_imuDrv_spl06.hpp"

#include "Sensors.hpp"

// alt传感器起始
#define ALT_SENSOR_IND 4
// 最大alt传感器个数
#define MAX_ALT_DRIVERS 1

float INTERNAL_PRESSURE = 1e5;
float INTERNAL_TEMPERATURE = 0;

/*imu线程*/
static QueueHandle_t IMUDaemon_Queue;
struct IMUDaemonTaskInf
{
    void (*task)(void *pvParameter1, uint32_t ulParameter2);
    void *pvParameter1;
    uint32_t ulParameter2;
};

static BaseType_t xIMUDaemonTaskCallFromISR(void (*task)(void *pvParameter1, uint32_t ulParameter2), void *pvParameter1, uint32_t ulParameter2, BaseType_t *xHigherPriorityTaskWoken)
{
    IMUDaemonTaskInf task_inf;
    task_inf.task = task;
    task_inf.pvParameter1 = pvParameter1;
    task_inf.ulParameter2 = ulParameter2;
    return xQueueSendFromISR(IMUDaemon_Queue, &task_inf, xHigherPriorityTaskWoken);
}
static BaseType_t xIMUDaemonTaskCall(void (*task)(void *pvParameter1, uint32_t ulParameter2), void *pvParameter1, uint32_t ulParameter2, TickType_t xTicksToWait)
{
    IMUDaemonTaskInf task_inf;
    task_inf.task = task;
    task_inf.pvParameter1 = pvParameter1;
    task_inf.ulParameter2 = ulParameter2;
    return xQueueSend(IMUDaemon_Queue, &task_inf, xTicksToWait);
}

static void IMUDaemonTask(void *pvParameters)
{
    while (1)
    {
        IMUDaemonTaskInf task_inf;
        xQueueReceive(IMUDaemon_Queue, &task_inf, portMAX_DELAY);
        (*task_inf.task)(task_inf.pvParameter1, task_inf.ulParameter2);
    }
}
/*imu线程*/

// 定时器频率
static uint32_t tmr_freq = 0;
#define CNT_MAX 0xffdf

// 传感器阵列
static uint8_t imu_drivers_count = 0;
static uint8_t alt_drivers_count = 0;
static uint16_t imu_drivers_cmpD[IMU_Sensors_Count + MAX_ALT_DRIVERS][3] = {0};
static uint32_t imu_drivers_cmpR[IMU_Sensors_Count + MAX_ALT_DRIVERS][3] = {0};
static uint32_t imu_drivers_cmp[IMU_Sensors_Count + MAX_ALT_DRIVERS][3] = {0};
static IMUDriverBase *imu_drivers[IMU_Sensors_Count + MAX_ALT_DRIVERS] = {0};
static uint32_t sensor_keys[IMU_Sensors_Count + MAX_ALT_DRIVERS][3] = {0};
static inline bool add_driver(IMUDriverBase *driver)
{
    // 初始化传感器
    if (!driver->init())
        return false;

    // 获取传感器频率
    uint16_t freqs[3];
    driver->get_freq(freqs);

    uint8_t chan;
    uint8_t tmsCount;
    uint8_t *drivers_count_ptr;
    uint32_t regMask = 0; // 0-gyro 1-accel 2-mag >=100-alt
    switch (driver->get_driverType())
    {
        case IMU_DRIVER_TYPE_GYRO_ACCEL:
        {
            // 通道号
            if (imu_drivers_count >= IMU_Sensors_Count)
                return false;
            chan = imu_drivers_count;
            drivers_count_ptr = &imu_drivers_count;
            regMask = (1 << 0) | (1 << 1);
            // 配置解算系统运行频率
            set_IMU_Gyroscope_UpdateFreq(chan, freqs[0]);
            set_IMU_Accelerometer_UpdateFreq(chan, freqs[1]);
            // 通道数量
            tmsCount = 2;
            break;
        }
        case IMU_DRIVER_TYPE_GYROACCEL:
        {
            // 通道号
            if (imu_drivers_count >= IMU_Sensors_Count)
                return false;
            chan = imu_drivers_count;
            drivers_count_ptr = &imu_drivers_count;
            regMask = (1 << 0) | (1 << 1);
            // 配置解算系统运行频率
            set_IMU_Gyroscope_UpdateFreq(chan, freqs[0]);
            set_IMU_Accelerometer_UpdateFreq(chan, freqs[0]);
            // 通道数量
            tmsCount = 1;
            break;
        }

        case IMU_DRIVER_TYPE_MAG:
        {
            // 通道号
            if (imu_drivers_count >= IMU_Sensors_Count)
                return false;
            chan = imu_drivers_count;
            drivers_count_ptr = &imu_drivers_count;
            regMask = (1 << 2);
            // 通道数量
            tmsCount = 1;
            break;
        }

        case IMU_DRIVER_TYPE_ALT:
        {
            // 通道号
            if (alt_drivers_count >= MAX_ALT_DRIVERS)
                return false;
            chan = IMU_Sensors_Count + alt_drivers_count;
            drivers_count_ptr = &alt_drivers_count;
            regMask = 100 + ALT_SENSOR_IND + alt_drivers_count;
            // 通道数量
            tmsCount = 1;
            break;
        }

        default:
        {
            return false;
        }
    }

    // 配置定时器
    HRTIM1->sTimerxRegs[chan].REPxR = 0;
    HRTIM1->sTimerxRegs[chan].PERxR = CNT_MAX;
    for (uint8_t i = 0; i < tmsCount; ++i)
    {
        __IO uint32_t *cmp;
        if (i == 0)
            cmp = &(HRTIM1->sTimerxRegs[chan].CMP1xR);
        else
            cmp = &(HRTIM1->sTimerxRegs[chan].CMP2xR) + (i - 1);
        imu_drivers_cmp[chan][i] = tmr_freq / freqs[i];
        uint32_t next_cmp = imu_drivers_cmp[chan][i];
        if (next_cmp > CNT_MAX)
        { // 需要多个周期触发
            // 计算第一个周期的cmp变化
            // 使每两次中断间隔不要过小
            uint32_t cmpD = imu_drivers_cmp[chan][i] % CNT_MAX;
            if (imu_drivers_cmpD[chan][i] == 0)
                imu_drivers_cmpD[chan][i] = CNT_MAX;
            if (cmpD != 0 && cmpD < CNT_MAX / 2)
                imu_drivers_cmpD[chan][i] = cmpD > CNT_MAX / 2 - cmpD ? cmpD : CNT_MAX / 2 - cmpD;
            next_cmp = imu_drivers_cmpD[chan][i];
        }
        imu_drivers_cmpR[chan][i] = imu_drivers_cmp[chan][i] - next_cmp;
        *cmp = next_cmp;
        // 使能比较中断（仅使能，事件是否产生还取决于 MCR 是否启动计数器）
        HRTIM1->sTimerxRegs[chan].TIMxDIER |= 1 << (0 + i);
    }

    // 使能 NVIC（计数器未启动时不会产生中断）
    // HRTIM1->sTimerxRegs[chan].TIMxDIER |= 1 << 4;
    NVIC_EnableIRQ((IRQn_Type) (HRTIM1_TIMA_IRQn + chan));

    // 注册传感器
    imu_drivers[chan] = driver;
    if (regMask & (1 << 0))
        sensor_keys[chan][0] = IMUGyroscopeRegister(imu_drivers_count, driver->get_name(), driver->get_param1());
    if (regMask & (1 << 1))
        sensor_keys[chan][1] = IMUAccelerometerRegister(imu_drivers_count, driver->get_name(), driver->get_param2());
    if (regMask & (1 << 2))
        sensor_keys[chan][2] = IMUMagnetometerRegister(Internal_Magnetometer_Index, driver->get_name(), driver->get_param3());
    if (regMask >= 100)
    {
        sensor_keys[chan][0] = PositionSensorRegister(regMask - 100, driver->get_name(), Position_Sensor_Type_RelativePositioning, Position_Sensor_DataType_s_z, Position_Sensor_frame_ENU,
                                                      driver->get_param1(), // 延时
                                                      0,                    // xy信任度
                                                      driver->get_param2()  // z信任度
        );
    }
    // 增加传感器数量
    ++(*drivers_count_ptr);

    // 开启计数器
    // HRTIM1->sMasterRegs.MCR |= 1 << (17+chan);

    return true;
}

static void imuDriverTask(void *pvParameter1, uint32_t ulParameter2)
{
    uint8_t chan = (ulParameter2 >> 8) & 0xff;
    uint8_t drv_chan = (ulParameter2 >> 0) & 0xff;
    IMUDriverBase *driver = imu_drivers[chan];
    if (driver == 0)
        return;

    vector3<int32_t> s_data1, s_data2, s_data3;
    double temperature = -1000;
    IMU_DATA_STATUS status1 = IMU_DATA_STATUS_ERROR;
    IMU_DATA_STATUS status2 = IMU_DATA_STATUS_ERROR;
    IMU_DATA_STATUS status3 = IMU_DATA_STATUS_ERROR;
    driver->sample(drv_chan, &s_data1, &s_data2, &s_data3, &temperature, &status1, &status2, &status3);

    switch (driver->get_driverType())
    {
        case IMU_DRIVER_TYPE_GYRO_ACCEL:
        {
            // 通道号
            switch (drv_chan)
            {
                case 0:
                { // gyro
                    bool res;
                    if (temperature > -273)
                        res = IMUGyroscopeUpdateTC(chan, sensor_keys[chan][0], s_data1, false, temperature);
                    else
                        res = IMUGyroscopeUpdate(chan, sensor_keys[chan][0], s_data1, false);
                    if (res)
                        MS_Notify_IMUGyroUpdate(chan);
                    break;
                }
                case 1:
                { // accel
                    bool res;
                    if (temperature > -273)
                        res = IMUAccelerometerUpdateTC(chan, sensor_keys[chan][1], s_data2, false, temperature);
                    else
                        res = IMUAccelerometerUpdate(chan, sensor_keys[chan][1], s_data2, false);
                    if (res)
                        MS_Notify_IMUAccelUpdate(chan);
                    break;
                }
            }
            break;
        }
        case IMU_DRIVER_TYPE_GYROACCEL:
        {
            // 通道号
            switch (drv_chan)
            {
                case 0:
                { // gyro+accel
                    bool res;

                    if (temperature > -273)
                        res = IMUGyroscopeUpdateTC(chan, sensor_keys[chan][0], s_data1, false, temperature);
                    else
                        res = IMUGyroscopeUpdate(chan, sensor_keys[chan][0], s_data1, false);
                    if (res)
                        MS_Notify_IMUGyroUpdate(chan);

                    if (temperature > -273)
                        res = IMUAccelerometerUpdateTC(chan, sensor_keys[chan][1], s_data2, false, temperature);
                    else
                        res = IMUAccelerometerUpdate(chan, sensor_keys[chan][1], s_data2, false);
                    if (res)
                        MS_Notify_IMUAccelUpdate(chan);
                    break;
                }
            }

            break;
        }

        case IMU_DRIVER_TYPE_MAG:
        {
            IMUMagnetometerUpdate(Internal_Magnetometer_Index, sensor_keys[chan][2], s_data1, status1 != IMU_DATA_STATUS_HEALTHY);
            break;
        }

        case IMU_DRIVER_TYPE_ALT:
        {
            if (chan >= IMU_Sensors_Count && status1 == IMU_DATA_STATUS_HEALTHY)
            {
                static int8_t current_internal_state_chan = -1;

                vector3<double> position;
                position.z = s_data1.z * 0.1;
                PositionSensorUpdatePosition(chan - IMU_Sensors_Count + ALT_SENSOR_IND, sensor_keys[chan][0], position,
                                             status1 == IMU_DATA_STATUS_HEALTHY ? true : false, // available
                                             driver->get_param1(),                              // delay
                                             -1,                                                // xy trust
                                             driver->get_param2(),                              // z trust
                                             0,                                                 // addition inf
                                             -1,                                                // xy LTtrust
                                             driver->get_param3()                               // z LTtrust
                );

                if (status1 != IMU_DATA_STATUS_HEALTHY)
                    current_internal_state_chan = -1;
                else if (current_internal_state_chan < 0)
                    current_internal_state_chan = chan;

                if (current_internal_state_chan == chan)
                {
                    INTERNAL_PRESSURE = s_data1.y * 1e-3;
                    INTERNAL_TEMPERATURE = temperature;
                }
            }
            break;
        }
        default:
            break;
    }
}

//-- 传感器定时中断
extern "C" void HRTIM1_TIMA_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    const uint8_t chan = 0;

    uint32_t ISR = HRTIM1->sTimerxRegs[chan].TIMxISR;
    uint32_t DIER = HRTIM1->sTimerxRegs[chan].TIMxDIER;

    for (uint8_t i = 0; i < 3; ++i)
    { // 处理cmp通道中断
        if ((DIER & (1 << (0 + i))) && (ISR & (1 << (0 + i))))
        {
            // 清空中断
            HRTIM1->sTimerxRegs[chan].TIMxICR = 1 << i;

            // 获取cmp
            __IO uint32_t *cmp;
            if (i == 0)
                cmp = &(HRTIM1->sTimerxRegs[chan].CMP1xR);
            else
                cmp = &(HRTIM1->sTimerxRegs[chan].CMP2xR) + (i - 1);

            if (imu_drivers_cmpR[chan][i] == 0)
            { // 已达到触发条件
                // 发送到线程处理
                uint32_t chanInfo = (chan << 8) | (i << 0);
                BaseType_t res = xIMUDaemonTaskCallFromISR(imuDriverTask, 0, chanInfo, &xHigherPriorityTaskWoken);
                // 计算下一次cmp值
                uint32_t next_cmp;
                if (imu_drivers_cmpD[chan][i] > 0)
                {
                    next_cmp = *cmp + imu_drivers_cmpD[chan][i];
                    imu_drivers_cmpR[chan][i] = imu_drivers_cmp[chan][i] - imu_drivers_cmpD[chan][i];
                }
                else
                    next_cmp = *cmp + imu_drivers_cmp[chan][i];
                if (next_cmp > CNT_MAX)
                    next_cmp -= CNT_MAX;
                *cmp = next_cmp;
            }
            else
            { // 需要多次触发
                if (imu_drivers_cmpR[chan][i] >= CNT_MAX)
                    imu_drivers_cmpR[chan][i] -= CNT_MAX;
                else
                {
                    uint32_t next_cmp = *cmp + imu_drivers_cmpR[chan][i];
                    if (next_cmp > CNT_MAX)
                        next_cmp -= CNT_MAX;
                    imu_drivers_cmpR[chan][i] = 0;
                    *cmp = next_cmp;
                }
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
extern "C" void HRTIM1_TIMB_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    const uint8_t chan = 1;

    uint32_t ISR = HRTIM1->sTimerxRegs[chan].TIMxISR;
    uint32_t DIER = HRTIM1->sTimerxRegs[chan].TIMxDIER;

    for (uint8_t i = 0; i < 3; ++i)
    { // 处理cmp通道中断
        if ((DIER & (1 << (0 + i))) && (ISR & (1 << (0 + i))))
        {
            // 清空中断
            HRTIM1->sTimerxRegs[chan].TIMxICR = 1 << i;

            // 获取cmp
            __IO uint32_t *cmp;
            if (i == 0)
                cmp = &(HRTIM1->sTimerxRegs[chan].CMP1xR);
            else
                cmp = &(HRTIM1->sTimerxRegs[chan].CMP2xR) + (i - 1);

            if (imu_drivers_cmpR[chan][i] == 0)
            { // 已达到触发条件
                // 发送到线程处理
                uint32_t chanInfo = (chan << 8) | (i << 0);
                BaseType_t res = xIMUDaemonTaskCallFromISR(imuDriverTask, 0, chanInfo, &xHigherPriorityTaskWoken);
                // 计算下一次cmp值
                uint32_t next_cmp;
                if (imu_drivers_cmpD[chan][i] > 0)
                {
                    next_cmp = *cmp + imu_drivers_cmpD[chan][i];
                    imu_drivers_cmpR[chan][i] = imu_drivers_cmp[chan][i] - imu_drivers_cmpD[chan][i];
                }
                else
                    next_cmp = *cmp + imu_drivers_cmp[chan][i];
                if (next_cmp > CNT_MAX)
                    next_cmp -= CNT_MAX;
                *cmp = next_cmp;
            }
            else
            { // 需要多次触发
                if (imu_drivers_cmpR[chan][i] >= CNT_MAX)
                    imu_drivers_cmpR[chan][i] -= CNT_MAX;
                else
                {
                    uint32_t next_cmp = *cmp + imu_drivers_cmpR[chan][i];
                    if (next_cmp > CNT_MAX)
                        next_cmp -= CNT_MAX;
                    imu_drivers_cmpR[chan][i] = 0;
                    *cmp = next_cmp;
                }
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
extern "C" void HRTIM1_TIMC_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    const uint8_t chan = 2;

    uint32_t ISR = HRTIM1->sTimerxRegs[chan].TIMxISR;
    uint32_t DIER = HRTIM1->sTimerxRegs[chan].TIMxDIER;

    for (uint8_t i = 0; i < 3; ++i)
    { // 处理cmp通道中断
        if ((DIER & (1 << (0 + i))) && (ISR & (1 << (0 + i))))
        {
            // 清空中断
            HRTIM1->sTimerxRegs[chan].TIMxICR = 1 << i;

            // 获取cmp
            __IO uint32_t *cmp;
            if (i == 0)
                cmp = &(HRTIM1->sTimerxRegs[chan].CMP1xR);
            else
                cmp = &(HRTIM1->sTimerxRegs[chan].CMP2xR) + (i - 1);

            if (imu_drivers_cmpR[chan][i] == 0)
            { // 已达到触发条件
                // 发送到线程处理
                uint32_t chanInfo = (chan << 8) | (i << 0);
                BaseType_t res = xIMUDaemonTaskCallFromISR(imuDriverTask, 0, chanInfo, &xHigherPriorityTaskWoken);
                // 计算下一次cmp值
                uint32_t next_cmp;
                if (imu_drivers_cmpD[chan][i] > 0)
                {
                    next_cmp = *cmp + imu_drivers_cmpD[chan][i];
                    imu_drivers_cmpR[chan][i] = imu_drivers_cmp[chan][i] - imu_drivers_cmpD[chan][i];
                }
                else
                    next_cmp = *cmp + imu_drivers_cmp[chan][i];
                if (next_cmp > CNT_MAX)
                    next_cmp -= CNT_MAX;
                *cmp = next_cmp;
            }
            else
            { // 需要多次触发
                if (imu_drivers_cmpR[chan][i] >= CNT_MAX)
                    imu_drivers_cmpR[chan][i] -= CNT_MAX;
                else
                {
                    uint32_t next_cmp = *cmp + imu_drivers_cmpR[chan][i];
                    if (next_cmp > CNT_MAX)
                        next_cmp -= CNT_MAX;
                    imu_drivers_cmpR[chan][i] = 0;
                    *cmp = next_cmp;
                }
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
extern "C" void HRTIM1_TIMD_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    const uint8_t chan = 3;

    uint32_t ISR = HRTIM1->sTimerxRegs[chan].TIMxISR;
    uint32_t DIER = HRTIM1->sTimerxRegs[chan].TIMxDIER;

    for (uint8_t i = 0; i < 3; ++i)
    { // 处理cmp通道中断
        if ((DIER & (1 << (0 + i))) && (ISR & (1 << (0 + i))))
        {
            // 清空中断
            HRTIM1->sTimerxRegs[chan].TIMxICR = 1 << i;

            // 获取cmp
            __IO uint32_t *cmp;
            if (i == 0)
                cmp = &(HRTIM1->sTimerxRegs[chan].CMP1xR);
            else
                cmp = &(HRTIM1->sTimerxRegs[chan].CMP2xR) + (i - 1);

            if (imu_drivers_cmpR[chan][i] == 0)
            { // 已达到触发条件
                // 发送到线程处理
                uint32_t chanInfo = (chan << 8) | (i << 0);
                BaseType_t res = xIMUDaemonTaskCallFromISR(imuDriverTask, 0, chanInfo, &xHigherPriorityTaskWoken);
                // 计算下一次cmp值
                uint32_t next_cmp;
                if (imu_drivers_cmpD[chan][i] > 0)
                {
                    next_cmp = *cmp + imu_drivers_cmpD[chan][i];
                    imu_drivers_cmpR[chan][i] = imu_drivers_cmp[chan][i] - imu_drivers_cmpD[chan][i];
                }
                else
                    next_cmp = *cmp + imu_drivers_cmp[chan][i];
                if (next_cmp > CNT_MAX)
                    next_cmp -= CNT_MAX;
                *cmp = next_cmp;
            }
            else
            { // 需要多次触发
                if (imu_drivers_cmpR[chan][i] >= CNT_MAX)
                    imu_drivers_cmpR[chan][i] -= CNT_MAX;
                else
                {
                    uint32_t next_cmp = *cmp + imu_drivers_cmpR[chan][i];
                    if (next_cmp > CNT_MAX)
                        next_cmp -= CNT_MAX;
                    imu_drivers_cmpR[chan][i] = 0;
                    *cmp = next_cmp;
                }
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#define IMUTask_StackSize 1200
Static_AXIDMABuf StackType_t IMUTask_Stack[IMUTask_StackSize];
Static_AXIDMABuf StaticTask_t IMUTask_TaskBuffer;
// 从HW_CFG_AXIS的value_low解码轴向到int8_t axis[3]
// nibble编码: 1=+sX, 2=+sY, 3=+sZ, 0x9=-sX, 0xA=-sY, 0xB=-sZ
static void decodeAxis(uint16_t value_low, int8_t axis[3])
{
    for (int i = 0; i < 3; i++)
    {
        uint8_t nibble = (value_low >> (i * 4)) & 0xF;
        bool neg = (nibble & 0x8) != 0;
        uint8_t sAxis = nibble & 0x7; // 1=X, 2=Y, 3=Z
        axis[i] = neg ? -(int8_t) sAxis : (int8_t) sAxis;
    }
}

// spi imu设备枚举
using SPI_IMUCreator = IMUDriverBase *(*) (SPI_CLASS * spi, GPIO_TypeDef *cs1_port, uint16_t cs1_pin, // PIN1 (通常 gyro CS)
                                           GPIO_TypeDef *cs2_port, uint16_t cs2_pin,                  // PIN2 (通常 accel CS 或忽略)
                                           const int8_t axis[3]);
struct SPI_IMU_DRIVER_DEF
{
    HW_Device device;
    uint8_t pins_used; // 0-不使用PIN1和PIN2 1-使用PIN1 2-使用PIN1和PIN2
    SPI_IMUCreator creator;
};
static const SPI_IMU_DRIVER_DEF spi_imu_driver_defs[] = {
    {HW_DEV_IMU_BMI088, // device
     2,                 // pins_used
     [](SPI_CLASS *spi, GPIO_TypeDef *cs1_port, uint16_t cs1_pin, GPIO_TypeDef *cs2_port, uint16_t cs2_pin, const int8_t axis[3]) -> IMUDriverBase *
     { return new imuDrv_bmi088(spi, cs1_port, cs1_pin, cs2_port, cs2_pin, axis); }},
    {HW_DEV_IMU_ICM20689, // device
     1,                   // pins_used
     [](SPI_CLASS *spi, GPIO_TypeDef *cs1_port, uint16_t cs1_pin, GPIO_TypeDef *cs2_port, uint16_t cs2_pin, const int8_t axis[3]) -> IMUDriverBase *
     { return new imuDrv_icm20689(spi, cs1_port, cs1_pin, axis); }},
    {HW_DEV_IMU_ICM40609, // device
     1,                   // pins_used
     [](SPI_CLASS *spi, GPIO_TypeDef *cs1_port, uint16_t cs1_pin, GPIO_TypeDef *cs2_port, uint16_t cs2_pin, const int8_t axis[3]) -> IMUDriverBase *
     { return new imuDrv_icm40609(spi, cs1_port, cs1_pin, axis); }},
    {HW_DEV_IMUBARO_SPL06, // device
     1,                    // pins_used
     [](SPI_CLASS *spi, GPIO_TypeDef *cs1_port, uint16_t cs1_pin, GPIO_TypeDef *cs2_port, uint16_t cs2_pin, const int8_t axis[3]) -> IMUDriverBase *
     { return new imuDrv_spl06(spi, cs1_port, cs1_pin); }},
    {HW_DEV_IMU_AK8975, // device
     1,                 // pins_used (PIN1 = CS)
     [](SPI_CLASS *spi, GPIO_TypeDef *cs1_port, uint16_t cs1_pin, GPIO_TypeDef *cs2_port, uint16_t cs2_pin, const int8_t axis[3]) -> IMUDriverBase *
     { return new imuDrv_ak8975(spi, cs1_port, cs1_pin, axis); }},
};

void init_drv_imu()
{
    // 创建imu守护线程
    IMUDaemon_Queue = xQueueCreate(30, sizeof(IMUDaemonTaskInf));
    xTaskCreateStatic(IMUDaemonTask, "IMUDaemonTask", IMUTask_StackSize, NULL, configTIMER_TASK_PRIORITY, IMUTask_Stack, &IMUTask_TaskBuffer);

    // 打开HRTIM时钟
    RCC->APB2ENR |= (1 << 29);
    // 设置中断优先级
    NVIC_SetPriority(HRTIM1_TIMA_IRQn, IRQPrioriy_ImuSensor);
    NVIC_SetPriority(HRTIM1_TIMB_IRQn, IRQPrioriy_ImuSensor);
    NVIC_SetPriority(HRTIM1_TIMC_IRQn, IRQPrioriy_ImuSensor);
    NVIC_SetPriority(HRTIM1_TIMD_IRQn, IRQPrioriy_ImuSensor);

    // 获取定时器频率
    HRTIM1->sTimerxRegs[0].TIMxCR = HRTIM1->sTimerxRegs[1].TIMxCR = HRTIM1->sTimerxRegs[2].TIMxCR = HRTIM1->sTimerxRegs[3].TIMxCR = (0 << 27) | (1 << 18) | (1 << 3) | (0b111 << 0);
    tmr_freq = APB2TIMERCLK / 4;

    HW_ConfigEntry entry;

// 遍历所有SPI设备，初始化IMU和气压计
#define MAX_IMU_DRIVERS 9
    struct PRE_ADD_DRIVER
    {
        IMUDriverBase *driver;
        uint8_t priority; // 优先级，数值越大优先级越高
    };
    PRE_ADD_DRIVER drivers_to_add[MAX_IMU_DRIVERS] = {0};
    uint8_t drivers_to_add_count = 0;
    for (uint8_t i = 0; i < sizeof(spi_imu_driver_defs) / sizeof(spi_imu_driver_defs[0]); ++i)
    {
        if (HW_ReadConfigEntry(spi_imu_driver_defs[i].device, HW_CFG_SPI_REF, &entry))
        {
            SPI_CLASS *spi_inst = get_spi_instance(entry.value_low);
            if (spi_inst)
            {
                HW_ConfigEntry pin1_entry, pin2_entry, axis_entry;
                if (spi_imu_driver_defs[i].pins_used >= 1 && !HW_ReadConfigEntry(spi_imu_driver_defs[i].device, HW_CFG_PIN1, &pin1_entry))
                    continue;
                if (spi_imu_driver_defs[i].pins_used >= 2 && !HW_ReadConfigEntry(spi_imu_driver_defs[i].device, HW_CFG_PIN2, &pin2_entry))
                    continue;

                int8_t axis[3] = {1, 2, 3}; // 默认 +x,+y,+z
                if (HW_ReadConfigEntry(spi_imu_driver_defs[i].device, HW_CFG_AXIS, &axis_entry))
                    decodeAxis(axis_entry.value_low, axis);

                // PIN1=gyro_cs_pin, PIN2=accel_cs_pin
                bool has_pin2 = spi_imu_driver_defs[i].pins_used >= 2;
                IMUDriverBase *drv = spi_imu_driver_defs[i].creator(spi_inst, HW_GpioTable[pin1_entry.value_high], pin1_entry.value_low, has_pin2 ? HW_GpioTable[pin2_entry.value_high] : nullptr,
                                                                    has_pin2 ? pin2_entry.value_low : 0, axis);
                drivers_to_add[drivers_to_add_count].driver = drv;
                drivers_to_add[drivers_to_add_count].priority = 0; // 默认优先级为0
                if (HW_ReadConfigEntry(spi_imu_driver_defs[i].device, HW_CFG_PRIO, &axis_entry))
                    drivers_to_add[drivers_to_add_count].priority = (uint8_t) axis_entry.value_low;
                drivers_to_add_count++;

                if (drivers_to_add_count >= MAX_IMU_DRIVERS)
                    break;
            }
        }
    }

    os_delay(0.1);

    // 根据优先级排序驱动
    for (uint8_t i = 0; i < drivers_to_add_count - 1; ++i)
    {
        for (uint8_t j = 0; j < drivers_to_add_count - 1 - i; ++j)
        {
            if (drivers_to_add[j].driver && drivers_to_add[j + 1].driver && drivers_to_add[j].priority < drivers_to_add[j + 1].priority)
            {
                PRE_ADD_DRIVER tmp = drivers_to_add[j];
                drivers_to_add[j] = drivers_to_add[j + 1];
                drivers_to_add[j + 1] = tmp;
            }
        }
    }

    // 添加驱动
    for (uint8_t i = 0; i < drivers_to_add_count; ++i)
    {
        if (drivers_to_add[i].driver)
            add_driver(drivers_to_add[i].driver);
    }

    // 开启计数器（此时所有传感器 init 都已走完, 才会真正产生采样中断）
    // 注: 上面 add_driver 中虽然提前使能了 TIMxDIER/NVIC, 但 MCR 未启动计数器时不会产生任何中断,
    // 所以这里统一启动 MCR 即为采样启动的同步点
    for (uint8_t i = 0; i < IMU_Sensors_Count + MAX_ALT_DRIVERS; ++i)
    {
        if (imu_drivers[i])
        {
            // 开启计数器
            HRTIM1->sMasterRegs.MCR |= 1 << (17 + i);
        }
    }
}