#include "MavlinkSendFuncs.hpp"
#include "mavlink.h"
extern "C" float getCPULoad();

#include "AC_Math.hpp"
#include "AuxFuncs.hpp"
#include "Avoidance.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "Missions.hpp"
#include "Modes.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "Sensors.hpp"
#include "StorageSystem.hpp"
#include "drv_ADC.hpp"
#include "drv_PWMOut.hpp"

bool Msg206_ACFlyPosSensor_INFO(uint8_t port, mavlink_message_t *msg_sd, uint8_t ind)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    Position_Sensor sensor;
    if (GetPositionSensor(ind, &sensor))
    {
        char name_ch[17];
        sensor.inf.name.get_CharStr(name_ch);

        uint8_t additionInfo1[4] = {0};
        float additionInfo2[8] = {0};

        mavlink_msg_acflypossensor_info_pack_chan(get_CommulinkSysId(),  // system id
                                                  get_CommulinkCompId(), // component id
                                                  port,                  // chan
                                                  msg_sd,
                                                  TIME::get_System_Run_Time() * 1e6, // boot us
                                                  sensor.data.position.x * 0.01, sensor.data.position.y * 0.01, sensor.data.position.z * 0.01, sensor.data.velocity.x * 0.01,
                                                  sensor.data.velocity.y * 0.01, sensor.data.velocity.z * 0.01, sensor.data.position_Global.x * 1e7, sensor.data.position_Global.y * 1e7,
                                                  sensor.data.delay, sensor.data.xy_trustD * 0.01, sensor.data.z_trustD * 0.01, name_ch, ind, sensor.data.sensor_type, sensor.data.data_frame,
                                                  sensor.data.sensor_DataType, additionInfo1, additionInfo2);
        return true;
    }
    return false;
}

// ACFly 系统状态补充数据(由 Msg200 SYS_STATUS_ACFLY 承载,不再塞入标准 SYS_STATUS)
struct AcflySysStatusData
{
    uint32_t statusFlag;  // 原 SYS_STATUS.present_extended 语义
    uint32_t statusFlag2; // 原 SYS_STATUS.health_extended 语义
    uint32_t posCflags;
    uint32_t posAflags;
    uint32_t imuCflags;
    uint32_t imuAflags;
};

static void getAcflySysStatusData(AcflySysStatusData &d)
{
    // 位置传感器状态
    d.posCflags = 0;
    d.posAflags = 0;
    for (uint8_t i = 0; i < Position_Sensors_Count; ++i)
    {
        Position_Sensor_Data sensor;
        if (GetPositionSensorData(i, &sensor))
        {
            d.posCflags |= (1 << i);
            if (sensor.available)
                d.posAflags |= (1 << i);
        }
    }

    // IMU 传感器状态
    d.imuCflags = 0;
    d.imuAflags = 0;
    for (uint8_t i = 0; i < IMU_Sensors_Count; ++i)
    {
        IMU_Sensor sensor;
        if (GetAccelerometer(i, &sensor))
        {
            d.imuCflags |= (1 << (i + 0));
            if (sensor.calibrated)
                d.imuAflags |= (1 << (i + 0));
        }
    }
    for (uint8_t i = 0; i < IMU_Sensors_Count; ++i)
    {
        IMU_Sensor sensor;
        if (GetGyroscope(i, &sensor))
        {
            d.imuCflags |= (1 << (i + 4));
            if (sensor.calibrated)
                d.imuAflags |= (1 << (i + 4));
        }
    }
    for (uint8_t i = 0; i < IMU_Sensors_Count; ++i)
    {
        IMU_Sensor sensor;
        if (GetMagnetometer(i, &sensor))
        {
            d.imuCflags |= (1 << (i + 8));
            if (sensor.calibrated)
                d.imuAflags |= (1 << (i + 8));
        }
    }
    // bit12: 航向异常标志
    // bit13: 姿态异常标志
    // bit14: 高度异常标志
    // bit15: 位置异常标志
    if (get_YawHealthEst() < 0)
    {
        // 航向异常标志位
        d.imuAflags |= (1 << 12);
    }
    if (get_Altitude_MSStatus() != MS_Ready)
    {
        // 高度异常标志位
        d.imuAflags |= (1 << 14);
    }
    if (get_Position_MSStatus() != MS_Ready)
    {
        // 位置异常标志位
        d.imuAflags |= (1 << 15);
    }

    // 风力标志位
    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight)
    {
        vector3<double> windDisturbance;
        float maxLean[2] = {0};
        get_WindDisturbance(&windDisturbance);
        ReadParam("AC_maxLean", 0, 0, (uint64_t *) &maxLean[0], 0);
        float AntiWindAngle = atan2(safe_sqrt(windDisturbance.get_square()), GravityAcc);
        if (AntiWindAngle > maxLean[0])
            d.imuCflags |= (1 << 13) | (1 << 12); // 完全超出承载能力
        else if (AntiWindAngle > maxLean[0] * 0.7f)
            d.imuCflags |= (1 << 13); // 强风
        else if (AntiWindAngle > maxLean[0] * 0.4f)
            d.imuCflags |= (1 << 12); // 中级风
    }

    // 系统状态标志(对齐 nanwang: sysStatusFlag=激活/DAO/风力/异常/伞, sysStatusFlag2=功能位)
    d.statusFlag = 0;
    uint8_t regStatus[8] = {0};
    if (ReadParam("ID_regStatus", 0, 0, (uint64_t *) regStatus, 0) == PR_OK)
    { // 获取是否已激活. [0: 无需激活(生产模式)], [1: 未激活(出货模式), [2: 已激活(出货模式)]
        d.statusFlag |= regStatus[0] & 0x03;
    }
    // bit2: 测向传感器固定解
    if (isDAOSensorFixed())
        d.statusFlag |= (1 << 2);
    // bit3-4: 风力等级, 映射自 imu_cflags 风力位 12-13 (0-无, 1-中级风, 2-强风, 3-超强风)
    d.statusFlag |= ((d.imuCflags >> 12) & 0x3) << 3;
    // bit5: 姿态异常, 姿态解算系统未就绪
    if (get_Attitude_MSStatus() != MS_Ready)
        d.statusFlag |= (1 << 5);
    // bit6: 位置异常, 映射自 imu_aflags bit15
    if (d.imuAflags & (1 << 15))
        d.statusFlag |= (1 << 6);
    // bit7: 航向异常, 映射自 imu_aflags bit12
    if (d.imuAflags & (1 << 12))
        d.statusFlag |= (1 << 7);
    // bit8: 高度异常, 映射自 imu_aflags bit14
    if (d.imuAflags & (1 << 14))
        d.statusFlag |= (1 << 8);
    // bit10-11: 伞1状态, 本工程无降落伞驱动保持0(与nanwang未装伞时一致)

    // 功能位(原 SYS_STATUS present_extended 语义)
    d.statusFlag2 = 0;
    if (GLB_FlightModeFlags & FMF_TerrainFollowingEna)
        d.statusFlag2 |= (1 << 0);
    if (GLB_FlightModeFlags & FMF_AvoidanceEna)
        d.statusFlag2 |= (1 << 1);
    if (GLB_FlightModeFlags & FMF_HeadlessEna)
        d.statusFlag2 |= (1 << 2);
    // bit3: 绕圈模式进行中
    if (getCurrentFlyMode() == AFunc_Circle)
        d.statusFlag2 |= (1 << 3);
    // bit4: 指令飞行(Offboard)进行中
    if (getCurrentFlyMode() == AFunc_Offboard || getCurrentFlyMode() == AFunc_OffboardOrbit)
        d.statusFlag2 |= (1 << 4);
    d.statusFlag2 |= GLB_FlightModeFlags & FMF_RecordTrajectory_Mask;
}

// 测向传感器方位角(单位 0.1deg; -10000: 无测向传感器, 1000: 有传感器但不可用, 0-3600: 正常)
static int16_t getDaoYawCentideg(uint8_t ind)
{
    DAO_Sensor daoSensor;
    if (!GetDAOSensor(ind, &daoSensor))
        return -10000;
    if (!daoSensor.available)
        return 1000;
    double daoYaw = 90.0 - rad2degree(atan2(daoSensor.relPos.y, daoSensor.relPos.x) - atan2(daoSensor.st_relPos.y, daoSensor.st_relPos.x));
    if (daoYaw < 0)
        daoYaw += 360;
    if (daoYaw > 360)
        daoYaw -= 360;
    return (int16_t) (daoYaw * 10.0);
}

// 上云API mode_code (飞行器状态)
// 方言(mavlink.h)已定义同名枚举时使用方言定义，本地定义仅作兜底
#ifndef HAVE_ENUM_MODE_CODE
enum MODE_CODE
{
    // 待机
    MODE_CODE_STANDBY = 0,
    // 起飞准备
    MODE_CODE_TAKEOFF_PREPARING = 1,
    // 起飞准备完毕
    MODE_CODE_TAKEOFF_READY = 2,
    // 手动飞行
    MODE_CODE_MANUAL_FLIGHT = 3,
    // 自动起飞
    MODE_CODE_AUTO_TAKEOFF = 4,
    // 航线飞行
    MODE_CODE_MISSION_FLIGHT = 5,
    // 智能跟随
    MODE_CODE_SMART_FOLLOW = 7,
    // 自动返航
    MODE_CODE_AUTO_RTL = 9,
    // 自动降落
    MODE_CODE_AUTO_LAND = 10,
    // 强制降落
    MODE_CODE_FORCE_LAND = 11,
};
#endif

// 计算上云API mode_code
// preparing_reason: 非空时返回待机/起飞准备状态的具体原因 (100-104)
static uint16_t get_mode_code(uint16_t *preparing_reason = 0)
{
    if (preparing_reason)
        *preparing_reason = MODE_CODE_REASON_NONE;

    bool inFlight;
    get_is_inFlight(&inFlight);
    if (!inFlight)
    { // 地面状态由定位系统决定
        if (get_Attitude_MSStatus() != MS_Ready)
        { // 姿态解锁系统未初始化完成 - 待机
            if (preparing_reason)
                *preparing_reason = MODE_CODE_REASON_ATTITUDE_NOT_READY;
            return MODE_CODE_STANDBY;
        }
        if (get_Altitude_MSStatus() != MS_Ready)
        { // 高度解算系统未初始化 - 待机
            if (preparing_reason)
                *preparing_reason = MODE_CODE_REASON_ALTITUDE_NOT_READY;
            return MODE_CODE_STANDBY;
        }

        // 获取GPS卫星数与搜星精度
        uint8_t gps_sat = 0;
        double gps_hacc = -1; // <0表示无GPS数据
        {
            Position_Sensor gps_sensor;
            for (uint8_t i = 0; i < default_max_gps_sensors; ++i)
            {
                if (GetPositionSensor(default_gps1_sensor_index + i, &gps_sensor))
                {
                    gps_sat = gps_sensor.inf.addition_inf[0];
                    gps_hacc = gps_sensor.inf.addition_inf[4];
                    break;
                }
            }
        }
        bool posReady = (get_Position_MSStatus() == MS_Ready);

        // 卫星数不足10颗且搜星精度差(>=5m)且位置解算未初始化 -> 飞机在机巢内, 待机
        if (!posReady && (gps_sat < 12 || gps_hacc >= 6))
        {
            if (preparing_reason)
                *preparing_reason = MODE_CODE_REASON_NO_POSITION;
            return MODE_CODE_STANDBY;
        }

        // 位置解算未初始化但卫星数足够(>10)且精度好(<5m) -> 起飞准备
        if (!posReady)
        {
            if (preparing_reason)
                *preparing_reason = MODE_CODE_REASON_NO_POSITION;
            return MODE_CODE_TAKEOFF_PREPARING;
        }

        // 定位未固定解 (使能MCfg_NoUnlockWithouFixed_Bit情况下) - 起飞准备
        bool gpsFixedReady = true;
        uint32_t cfgBits[2] = {0};
        if (ReadParam("MFunc_Configs", 0, 0, (uint64_t *) cfgBits, 0) == PR_OK && (cfgBits[0] & MCfg_NoUnlockWithouFixed_Bit))
            gpsFixedReady = isGPSFixedSolution();

        // 测向传感器存在但无固定解 - 起飞准备
        bool daoFixedReady = true;
        {
            DAO_Sensor daoSensor;
            if ((GetDAOSensor(0, &daoSensor) || GetDAOSensor(1, &daoSensor)) && !isDAOSensorFixed())
                daoFixedReady = false;
        }

        // 高度位置解算全部就绪且定位测向固定解 -> 起飞准备完毕
        if (gpsFixedReady && daoFixedReady)
            return MODE_CODE_TAKEOFF_READY;

        // 起飞准备-反馈未就绪原因 (优先级: GPS固定解 > 测向固定解)
        if (preparing_reason)
        {
            if (!gpsFixedReady)
                *preparing_reason = MODE_CODE_REASON_GPS_NOT_FIXED;
            else
                *preparing_reason = MODE_CODE_REASON_DAO_NOT_FIXED;
        }
        return MODE_CODE_TAKEOFF_PREPARING;
    }

    switch (getCurrentFlyMode())
    {
        case AFunc_TakeOff:
            return MODE_CODE_AUTO_TAKEOFF;
        case AFunc_Mission:
            return MODE_CODE_MISSION_FLIGHT;
        case AFunc_Follow:
        case AFunc_Loiter:
            return MODE_CODE_SMART_FOLLOW;
        case AFunc_RTL:
        case AFunc_RTL2Point:
            return MODE_CODE_AUTO_RTL;
        case AFunc_Land:
        { // 不可中断降落(低电压/失控)为强制降落
            uint16_t reason = getCurrentFlyModeReason();
            if (reason == MODE_CODE_REASON_LVOLT_RTL || reason == MODE_CODE_REASON_LVOLT_LAND || reason == MODE_CODE_REASON_RC_LOST || reason == MODE_CODE_REASON_LVOLT_RTL_LAND)
                return MODE_CODE_FORCE_LAND;
            return MODE_CODE_AUTO_LAND;
        }
        default:
            return MODE_CODE_MANUAL_FLIGHT;
    }
}

// ACFly 系统状态补充报文(SYS_STATUS_ACFLY, msgid 200, CRC 127)
// 承载原非标 SYS_STATUS 扩展字段的全部补充数据 (字段语义与 nanwang 对齐)
static bool Msg200_SYS_STATUS_ACFLY(uint8_t port, mavlink_message_t *msg_sd)
{
    AcflySysStatusData status;
    getAcflySysStatusData(status);

    uint16_t sdTotGB = getSdTotalSizeGB() * 100;
    uint16_t sdFreGB = getSdFreeSizeGB() * 100;

    // Calculate ADC voltage and current readings
    // adc_voltage[0]/adc_current[0]: battery 8, adc_voltage[1]/adc_current[1]: battery 9
    uint16_t adcVoltage[2] = {0};
    uint16_t adcCurrent[2] = {0};
    for (uint8_t i = 0; i < 2; ++i)
    {
        BatteryInfo batInfo;
        if (getBatteryInfo(8 + i, &batInfo))
        {
            // Field unit is 0.1V, values below 5V are treated as no battery
            if (batInfo.totalVoltRaw >= 5)
                adcVoltage[i] = batInfo.totalVoltRaw * 10;

            // Sent in 0.01A, clip to the 0.01A field range
            if (batInfo.available >= 0 && batInfo.totalCurrent >= 0)
            {
                float cur = batInfo.totalCurrent * 100;
                if (cur > 65535)
                    cur = 65535;
                adcCurrent[i] = cur;
            }
        }
    }

    int16_t daoYaw[2] = {getDaoYawCentideg(0), getDaoYawCentideg(1)};

    // mode_code及reason: 待机(0)/起飞准备(1)时以未就绪原因(100-104)覆盖reason, 其余状态用模式切换原因
    uint16_t preparing_reason = MODE_CODE_REASON_NONE;
    uint16_t mode_code = get_mode_code(&preparing_reason);
    uint16_t mode_code_reason = (mode_code <= MODE_CODE_TAKEOFF_PREPARING && preparing_reason != MODE_CODE_REASON_NONE) ? preparing_reason : getCurrentFlyModeReason();

    mavlink_msg_sys_status_acfly_pack_chan(get_CommulinkSysId(),  // system id
                                           get_CommulinkCompId(), // component id
                                           port,                  // chan
                                           msg_sd,
                                           (uint16_t) (getCPULoad() * 10),      // cpu_usage [0.1%]
                                           (uint32_t) (xPortGetFreeHeapSize()), // remaining_memory (与 nanwang 一致)
                                           daoYaw,                              // daoYaw [0.1deg]
                                           adcVoltage,                          // adc_voltage [0.1V]
                                           adcCurrent,                          // adc_current [0.01A]
                                           sdTotGB,                             // sd_card_capacity [10MB]
                                           sdFreGB,                             // sd_card_remain [10MB]
                                           status.statusFlag,                   // sysStatusFlag
                                           status.statusFlag2,                  // sysStatusFlag2
                                           status.posCflags,                    // pos_cflags
                                           status.posAflags,                    // pos_aflags
                                           (uint16_t) status.imuCflags,         // imu_cflags
                                           (uint16_t) status.imuAflags,         // imu_aflags
                                           mode_code,                           // mode_code (上云API)
                                           mode_code_reason);                   // mode_code_reason (上云API)
    return true;
}

static bool Msg01_SYS_STATUS(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    // 计算电压发送值
    BatteryInfo batInfo = {0};

    //	getCurrentBatteryInfo(&batInfo);
    getBatteryInfo(8, &batInfo);
    float sd_volt = 0;
    if (batInfo.totalVoltRaw < 5)
        sd_volt = 0;
    else if (batInfo.totalVoltRaw < 65.5)
        sd_volt = batInfo.totalVoltRaw * 1000;
    else
        sd_volt = batInfo.totalVoltRaw * 10;

    // 发送电池信息
    const Port *p = get_CommuPort(port);
    if (p && p->write)
    {
        static uint8_t battery_send_ind[MAVLINK_COMM_NUM_BUFFERS] = {0};
        static uint8_t batteryCell_sendCouter[MAVLINK_COMM_NUM_BUFFERS] = {0};
        if (port < MAVLINK_COMM_NUM_BUFFERS)
        {
            for (uint8_t k = 0; k < 3; ++k)
            {
                // 必须初始化: getBatteryInfo获取失败时不会写入batInfo
                BatteryInfo batInfo = {0};
                bool batAvailable = false;
                for (uint8_t i = 0; i < MAX_BATTERYS; ++i)
                {
                    ++battery_send_ind[port];
                    if (battery_send_ind[port] >= MAX_BATTERYS)
                        battery_send_ind[port] = 0;
                    if (getBatteryInfo(battery_send_ind[port], &batInfo) && batInfo.available > 0)
                    {
                        batAvailable = true;
                        break;
                    }
                }
                if (batAvailable == false)
                { // 无可用电池(上电初期电池尚未注册),不发送未初始化数据
                    break;
                }
                { // 电池可用
                    uint8_t batIndSd = battery_send_ind[port];
                    batIndSd = (batIndSd >= MAX_BATTERYS - INT_BATTERYS ? batIndSd + INT_BATTERYS - MAX_BATTERYS : batIndSd + INT_BATTERYS);

                    uint8_t voltages[48];
                    memset(voltages, 0, 48);
                    static const char serialNoEmpty[30] = {0};
                    if (batInfo.cellVoltAvailable == false /*|| batInfo.cells>14*/)
                    { // 发送电池总压
                        mavlink_msg_battery_status_acfly_pack_chan(get_CommulinkSysId(), get_CommulinkCompId(), port, msg_sd, batIndSd,
                                                                   batInfo.health,                                                    // health
                                                                   batInfo.totalPercent,                                              // remaining_percentage
                                                                   batInfo.cells,                                                     // cell_count
                                                                   batInfo.errorFlags,                                                // fault_bitmask
                                                                   batInfo.temperature < -273 ? INT16_MAX : batInfo.temperature * 10, // 电池温度
                                                                   batInfo.cycle_count,                                               // cycle_count
                                                                   batInfo.totalCurrent * 10,                                         // current
                                                                   batInfo.totalVoltRaw * 10,                                         // voltage
                                                                   batInfo.capacity_design,                                           // capacity_actual
                                                                   batInfo.sequence_num,                                              // sequence_num
                                                                   voltages,                                                          // voltages
                                                                   serialNoEmpty                                                      // serial_number
                        );

                        mavlink_send_msg(p, msg_sd);
                    }
                    else
                    { // 发送单独电池电压
                        memset(voltages, 0, 48);
                        int cellsCnt = batInfo.cells <= 48 ? batInfo.cells : 48;
                        for (uint8_t i = 0; i < cellsCnt; ++i)
                            voltages[i] = batInfo.cellVolts[i] * 50;

                        mavlink_msg_battery_status_acfly_pack_chan(get_CommulinkSysId(), get_CommulinkCompId(), port, msg_sd, batIndSd,
                                                                   batInfo.health,                                                    // health
                                                                   batInfo.totalPercent,                                              // remaining_percentage
                                                                   batInfo.cells,                                                     // cell_count
                                                                   batInfo.errorFlags,                                                // fault_bitmask
                                                                   batInfo.temperature < -273 ? INT16_MAX : batInfo.temperature * 10, // 电池温度
                                                                   batInfo.cycle_count,                                               // cycle_count
                                                                   batInfo.totalCurrent * 10,                                         // current
                                                                   batInfo.totalVoltRaw * 10,                                         // voltage
                                                                   batInfo.capacity_design,                                           // capacity_actual
                                                                   batInfo.sequence_num,                                              // sequence_num
                                                                   voltages,                                                          // voltages
                                                                   serialNoEmpty                                                      // serial_number
                        );

                        mavlink_send_msg(p, msg_sd);
                    }
                }
            }
        }
    }

    // 发送包(SYS_STATUS 仅承载标准字段;ACFly 补充数据由 Msg200 SYS_STATUS_ACFLY 承载)
    mavlink_msg_sys_status_pack_chan(get_CommulinkSysId(),  // system id
                                     get_CommulinkCompId(), // component id
                                     port,                  // chan
                                     msg_sd, 0, 0, 0, getCPULoad() * 10, sd_volt, batInfo.totalCurrent * 100, batInfo.totalPercent, 0, 0, 0, 0, 0, 0,
                                     0,  // onboard_control_sensors_present_extended
                                     0,  // onboard_control_sensors_enabled_extended
                                     0); // onboard_control_sensors_health_extended
    return true;
}
static bool Msg245_EXTENDED_SYS_STATE(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    bool inFlight;
    get_is_inFlight(&inFlight);
    mavlink_msg_extended_sys_state_pack_chan(get_CommulinkSysId(),  // system id
                                             get_CommulinkCompId(), // component id
                                             port,                  // chan
                                             msg_sd, MAV_VTOL_STATE_MC, inFlight ? MAV_LANDED_STATE_IN_AIR : MAV_LANDED_STATE_ON_GROUND);
    return true;
}

int8_t TimeZone = 8;
static bool Msg02_SYSTEM_TIME(uint8_t port, mavlink_message_t *msg_sd)
{
    RTC_TimeStruct rtc;
    rtc = Get_RTC_Time();
    uint32_t unixT = LocalTime2Unix(&rtc, TimeZone);

    // 发送包
    mavlink_msg_system_time_pack_chan(get_CommulinkSysId(),  // system id
                                      get_CommulinkCompId(), // component id
                                      port,                  // chan
                                      msg_sd, unixT * 1e6,
                                      TIME::get_System_Run_Time() * 1e3 // boot ms
    );
    return true;
}

// static bool Msg147_BATTERY_STATUS( uint8_t Port_index , mavlink_message_t* msg )
//{
//	const Port* port = get_CommuPort( Port_index );
//	if( port->write )
//	{
//		BatteryInfo batInfo;
//		if( getCurrentBatteryInfo(&batInfo) && batInfo.available>=0 )
//		{	//电池可用
//			uint16_t voltage[10];
//			mavlink_msg_battery_status_pack_chan(
//				get_CommulinkSysId() ,	//system id
//				get_CommulinkCompId() ,	//component id
//				Port_index ,	//chan
//				msg,
//				0 ,	// Battery ID
//				MAV_BATTERY_FUNCTION_UNKNOWN ,	// Function of the battery
//				MAV_BATTERY_TYPE_UNKNOWN ,	// Type (chemistry) of the battery
//				INT16_MAX , // Temperature of the battery
//				voltage , // voltages [mV]
//				-1 , // current_battery [cA]
//				current*1000 , // current_consumed [mAh]
//				-1 , // energy_consumed [hJ]
//				-1 , // battery_remaining [%]
//				-1 , // time_remaining [s]
//				MAV_BATTERY_CHARGE_STATE_OK    // charge_state
//			);
//			return true;
//		}
//		return false;
//	}
//	for( uint8_t i=0; i<MAX_BATTERYS; ++i )
//	{
//
//
//	}
//	return true;
//
//	float volt, current, RMPercent;
//	get_MainBatteryInf( &volt, &current, 0, 0, &RMPercent );
//	float sd_volt;
//	if( volt < 5 )
//		sd_volt = 0;
//	else if( volt < 65.5 )
//		sd_volt = volt*1000;
//	else
//		sd_volt = volt*10;
//
//   float vol_all = sd_volt;
//
//
//	for(int i =0; i<10; i++ )
//	{
//		if( vol_all > 4200 )
//		{
//			voltage[i] = 4200;
//			vol_all-=4200;
//		}
//		else
//		{
//		  voltage[i] = vol_all;
//			vol_all=0;
//		}
//	}
//
//
//	return false;
// }

static bool Msg26_SCALED_IMU(uint8_t port, mavlink_message_t *msg_sd)
{
    IMU_Sensor sensor;
    uint8_t s_count = 0;
    vector3<float> acc, gyro, mag;
    float temp = 0;
    if (GetAccelerometer(0, &sensor))
    {
        ++s_count;
        acc.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
    }
    if (GetGyroscope(0, &sensor))
    {
        ++s_count;
        gyro.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
        temp = sensor.temperature;
    }
    if (GetMagnetometer(0, &sensor))
    {
        ++s_count;
        mag.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
    }
    DAO_Sensor daoSensor;
    double daoYaw = -1000;
    if (GetDAOSensor(0, &daoSensor) || GetDAOSensor(1, &daoSensor))
    {
        ++s_count;
        if (daoSensor.available)
        {
            daoYaw = 90.0 - rad2degree(atan2(daoSensor.relPos.y, daoSensor.relPos.x) - atan2(daoSensor.st_relPos.y, daoSensor.st_relPos.x));
            if (daoYaw < 0)
                daoYaw += 360;
            if (daoYaw > 360)
                daoYaw -= 360;
        }
        else
            daoYaw = 1000;
    }
    if (s_count == 0)
        return false;

    mavlink_msg_scaled_imu_pack_chan(get_CommulinkSysId(),  // system id
                                     get_CommulinkCompId(), // component id
                                     port,                  // chan
                                     msg_sd,
                                     TIME::get_System_Run_Time() * 1e3, // boot ms
                                     acc.x * (1000 / GravityAcc), acc.y * (1000 / GravityAcc), acc.z * (1000 / GravityAcc), gyro.x * 1000, gyro.y * 1000, gyro.z * 1000, mag.x * 1000, mag.y * 1000,
                                     mag.z * 1000, temp * 100);
    return true;
}
static bool Msg29_SCALED_PRESSURE(uint8_t port, mavlink_message_t *msg_sd)
{
    // 内部气压
    extern float INTERNAL_PRESSURE;
    // 内部温度
    extern float INTERNAL_TEMPERATURE;

    if (HTL_isEnabled())
    {
        Position_Sensor baroSensor;
        if (GetPositionSensor(internal_baro_sensor_index, &baroSensor) && baroSensor.data.available)
        {
            double altM = baroSensor.data.position.z * 0.01;
            double isaTempK = SSL_AIR_TEMPERATURE - ISA_LAPSE_RATE * altM;
            if (isaTempK < 0.1)
                isaTempK = 0.1;
            double pressure = 101325.0 * pow(1.0 - (altM > 44290.0 ? 44290.0 : altM) / 44300.0, 5.256);
            double temperature = isaTempK - C_TO_KELVIN;

            mavlink_msg_scaled_pressure_pack_chan(get_CommulinkSysId(),  // system id
                                                  get_CommulinkCompId(), // component id
                                                  port,                  // chan
                                                  msg_sd,
                                                  TIME::get_System_Run_Time() * 1e3, // boot ms
                                                  pressure * 1e-2, 0, temperature * 100, 0);
            return true;
        }
    }

    mavlink_msg_scaled_pressure_pack_chan(get_CommulinkSysId(),  // system id
                                          get_CommulinkCompId(), // component id
                                          port,                  // chan
                                          msg_sd,
                                          TIME::get_System_Run_Time() * 1e3, // boot ms
                                          INTERNAL_PRESSURE * 1e-2, 0, INTERNAL_TEMPERATURE * 100, 0);
    return true;
}
static bool Msg116_SCALED_IMU2(uint8_t port, mavlink_message_t *msg_sd)
{
    IMU_Sensor sensor;
    uint8_t s_count = 0;
    vector3<float> acc, gyro, mag;
    float temp = 0;
    if (GetAccelerometer(1, &sensor))
    {
        ++s_count;
        acc.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
    }
    if (GetGyroscope(1, &sensor))
    {
        ++s_count;
        gyro.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
        temp = sensor.temperature;
    }
    if (GetMagnetometer(1, &sensor))
    {
        ++s_count;
        mag.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
    }
    if (s_count == 0)
        return false;

    mavlink_msg_scaled_imu2_pack_chan(get_CommulinkSysId(),  // system id
                                      get_CommulinkCompId(), // component id
                                      port,                  // chan
                                      msg_sd,
                                      TIME::get_System_Run_Time() * 1e6, // time usec
                                      acc.x * (1000 / GravityAcc), acc.y * (1000 / GravityAcc), acc.z * (1000 / GravityAcc), gyro.x * 1000, gyro.y * 1000, gyro.z * 1000, mag.x * 1000, mag.y * 1000,
                                      mag.z * 1000, temp * 100);
    return true;
}
static bool Msg129_SCALED_IMU3(uint8_t port, mavlink_message_t *msg_sd)
{
    IMU_Sensor sensor;
    uint8_t s_count = 0;
    vector3<float> acc, gyro, mag;
    float temp = 0;
    if (GetAccelerometer(2, &sensor))
    {
        ++s_count;
        acc.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
    }
    if (GetGyroscope(2, &sensor))
    {
        ++s_count;
        gyro.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
        temp = sensor.temperature;
    }
    if (GetMagnetometer(2, &sensor))
    {
        ++s_count;
        mag.set_vector(sensor.data.x, sensor.data.y, sensor.data.z);
    }
    if (s_count == 0)
        return false;

    mavlink_msg_scaled_imu3_pack_chan(get_CommulinkSysId(),  // system id
                                      get_CommulinkCompId(), // component id
                                      port,                  // chan
                                      msg_sd,
                                      TIME::get_System_Run_Time() * 1e6, // time usec
                                      acc.x * (1000 / GravityAcc), acc.y * (1000 / GravityAcc), acc.z * (1000 / GravityAcc), gyro.x * 1000, gyro.y * 1000, gyro.z * 1000, mag.x * 1000, mag.y * 1000,
                                      mag.z * 1000, temp * 100);
    return true;
}

static bool Msg210_MISSION_REPORT(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    bool inFlight;
    get_is_inFlight(&inFlight);
    uint8_t flyMode = getCurrentFlyMode();

    static bool inFlightLast = inFlight;
    static uint8_t last_flyMode = flyMode;
    static uint8_t sendCntConfirm[MAVLINK_COMM_NUM_BUFFERS] = {0};
    if ((inFlightLast != inFlight && !inFlight) || (last_flyMode != flyMode && last_flyMode == AFunc_Mission && flyMode != AFunc_Mission && flyMode != AFunc_Land && flyMode != AFunc_RTL))
    { // disarm or mode switch, send 6 time
        if (port < MAVLINK_COMM_NUM_BUFFERS)
            sendCntConfirm[port] = 6;
    }
    last_flyMode = flyMode;
    inFlightLast = inFlight;

    extern bool inMissionFlight;
    if ((port < MAVLINK_COMM_NUM_BUFFERS && sendCntConfirm[port] == 0) && (!(flyMode == AFunc_Mission || flyMode == AFunc_RTL || flyMode == AFunc_Land) || !inFlight || !inMissionFlight))
        return false;

    // Read from global mission item report (thread-safe)
    MissionItemReport rpt = get_missionItemReport();
    rpt.mission_count = sendCntConfirm[port];
    mavlink_msg_mission_report_pack_chan(get_CommulinkSysId(),  // system id
                                         get_CommulinkCompId(), // component id
                                         port,                  // chan
                                         msg_sd,
                                         TIME::get_System_Run_Time() * 1e6, // time_usec
                                         rpt.seq,                           // seq
                                         rpt.cmd,                           // cmd
                                         rpt.mission_count,                 // mission_count
                                         rpt.progress,                      // progress
                                         rpt.result                         // result
    );
    if (port < MAVLINK_COMM_NUM_BUFFERS)
    {
        if (sendCntConfirm[port] > 0)
            sendCntConfirm[port]--;
    }
    return true;
}

static bool Msg330_OBSTACLE_DISTANCE(uint8_t port, mavlink_message_t *msg_sd)
{
    uint16_t distances[72];
    memset(distances, 0xff, sizeof(distances));
    // 获取避障数据
    const AvoidanceCfg *avCfg = getAvCfg();
    //	for( uint8_t i=0; i<16; ++i )
    //	{
    //		const int8_t x[] = { 1, 0, -1, 0 };
    //		const int8_t y[] = { 0, 1, 0, -1 };
    //		double angle = i*2*Pi/16;
    //		double sinA, cosA;
    //		fast_sin_cos( angle, &sinA, &cosA );
    //		vector3<double> tVelVec( cosA, sinA, 0);
    //		double AvDistance = -1;
    //		get_AvLineDistanceFlu( &AvDistance, tVelVec, avCfg->wheelbase[0] );
    //		if( AvDistance>=0 )
    //		{
    //			AvDistance -= 0.5f*avCfg->wheelbase[0];
    //			if( AvDistance < 0 )
    //				AvDistance = 0;
    //			distances[i] = AvDistance;
    //		}
    //	}

    double AvDistance[5];
    vector3<double> posOffsets[4];
    // 前向
    vector3<double> tVelVec(1, 0, 0);
    posOffsets[0].set_vector(0, -200, 0);
    posOffsets[1].set_vector(0, -500, 0);
    posOffsets[2].set_vector(0, +500, 0);
    posOffsets[3].set_vector(0, +200, 0);
    get_AvLineDistanceFlu(AvDistance, tVelVec, avCfg->wheelbase[0], posOffsets, 4);
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (AvDistance[i] >= 10 && AvDistance[0] <= 10000)
        {
            if (AvDistance[i] < 0)
                AvDistance[i] = 0;
            distances[i + 0] = AvDistance[i];
        }
    }
    for (uint8_t i = 3; i < 5; ++i)
    {
        if (AvDistance[i] >= 10 && AvDistance[0] <= 10000)
        {
            if (AvDistance[i] < 0)
                AvDistance[i] = 0;
            distances[i - 3 + 14] = AvDistance[i];
        }
    }

    // 后向
    tVelVec.set_vector(-1, 0, 0);
    posOffsets[0].set_vector(0, -500, 0);
    posOffsets[1].set_vector(0, -200, 0);
    posOffsets[2].set_vector(0, +200, 0);
    posOffsets[3].set_vector(0, +500, 0);
    get_AvLineDistanceFlu(AvDistance, tVelVec, avCfg->wheelbase[0], posOffsets, 4);
    if (AvDistance[0] >= 10 && AvDistance[0] <= 10000)
    {
        if (AvDistance[0] < 0)
            AvDistance[0] = 0;
        distances[8] = AvDistance[0];
    }
    for (uint8_t i = 1; i < 3; ++i)
    {
        if (AvDistance[i] >= 10 && AvDistance[i] <= 10000)
        {
            if (AvDistance[i] < 0)
                AvDistance[i] = 0;
            distances[i + 5] = AvDistance[i];
        }
    }
    for (uint8_t i = 3; i < 5; ++i)
    {
        if (AvDistance[i] >= 10 && AvDistance[0] <= 10000)
        {
            if (AvDistance[i] < 0)
                AvDistance[i] = 0;
            distances[i + 6] = AvDistance[i];
        }
    }

    // 右向
    tVelVec.set_vector(0, -1, 0);
    posOffsets[0].set_vector(0, +200, 0);
    posOffsets[1].set_vector(0, -200, 0);
    get_AvLineDistanceFlu(AvDistance, tVelVec, avCfg->wheelbase[0], posOffsets, 2);
    if (AvDistance[0] >= 10 && AvDistance[0] <= 10000)
    {
        if (AvDistance[0] < 0)
            AvDistance[0] = 0;
        distances[4] = AvDistance[0];
    }
    if (AvDistance[1] >= 10 && AvDistance[1] <= 10000)
    {
        if (AvDistance[1] < 0)
            AvDistance[1] = 0;
        distances[3] = AvDistance[1];
    }
    if (AvDistance[2] >= 10 && AvDistance[2] <= 10000)
    {
        if (AvDistance[2] < 0)
            AvDistance[2] = 0;
        distances[5] = AvDistance[2];
    }

    // 左向
    tVelVec.set_vector(0, 1, 0);
    posOffsets[0].set_vector(0, -200, 0);
    posOffsets[1].set_vector(0, +200, 0);
    get_AvLineDistanceFlu(AvDistance, tVelVec, avCfg->wheelbase[0], posOffsets, 2);
    if (AvDistance[0] >= 10 && AvDistance[0] <= 10000)
    {
        if (AvDistance[0] < 0)
            AvDistance[0] = 0;
        distances[12] = AvDistance[0];
    }
    if (AvDistance[1] >= 10 && AvDistance[1] <= 10000)
    {
        if (AvDistance[1] < 0)
            AvDistance[1] = 0;
        distances[11] = AvDistance[1];
    }
    if (AvDistance[2] >= 10 && AvDistance[2] <= 10000)
    {
        if (AvDistance[2] < 0)
            AvDistance[2] = 0;
        distances[13] = AvDistance[2];
    }

    // 下
    PosSensorHealthInf1 pos_inf;
    get_OptimalRange_Z(&pos_inf);
    Position_Sensor_Data sensor;
    if (GetPositionSensorData(pos_inf.sensor_ind, &sensor))
    {
        distances[32] = sensor.position.z * 0.1;
    }

    mavlink_msg_obstacle_distance_pack_chan(get_CommulinkSysId(),  // system id
                                            get_CommulinkCompId(), // component id
                                            port,                  // chan
                                            msg_sd, TIME::get_System_Run_Time() * 1e6, MAV_DISTANCE_SENSOR_RADAR, distances, 23, 0, 10000, -90, 0, MAV_FRAME_BODY_FLU);
    return true;
}

static bool Msg30_ATTITUDE(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    Quaternion airframe_quat;
    get_AirframeY_quat(&airframe_quat);
    airframe_quat.Enu2Ned();
    vector3<double> angular_rate;
    get_AngularRate_Ctrl(&angular_rate);

    double heading = airframe_quat.getYaw();
    if (heading < 0)
        heading = 2 * Pi + heading;
    mavlink_msg_attitude_pack_chan(get_CommulinkSysId(),  // system id
                                   get_CommulinkCompId(), // component id
                                   port,                  // chan
                                   msg_sd,
                                   TIME::get_System_Run_Time() * 1000, // boot ms
                                   airframe_quat.getRoll(), airframe_quat.getPitch(), heading, angular_rate.x, angular_rate.y, angular_rate.z);
    return true;
}

static bool Msg31_ATTITUDE_QUATERNION(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    Quaternion airframe_quat;
    get_AirframeY_quat(&airframe_quat);
    airframe_quat.Enu2Ned();
    vector3<double> angular_rate;
    get_AngularRate_Ctrl(&angular_rate);

    mavlink_msg_attitude_quaternion_pack_chan(get_CommulinkSysId(),  // system id
                                              get_CommulinkCompId(), // component id
                                              port,                  // chan
                                              msg_sd,
                                              TIME::get_System_Run_Time() * 1000, // boot ms
                                              airframe_quat.get_qw(), airframe_quat.get_qx(), airframe_quat.get_qy(), airframe_quat.get_qz(), angular_rate.x, angular_rate.y, angular_rate.z, 0);

    return true;
}

static bool Msg24_GPS_RAW_INT(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Altitude_MSStatus() != MS_Ready)
        return false;

    bool gps_exist = false;
    Position_Sensor gps_sensor;
    uint8_t checked_gps = 0;
    for (uint8_t i = 0; i < default_max_gps_sensors; ++i)
    {
        if (GetPositionSensor(default_gps1_sensor_index + i, &gps_sensor))
        {
            ++checked_gps;
            if (checked_gps == 1)
            {
                gps_exist = true;
                break;
            }
        }
    }

    if (gps_exist)
    {
        double vel = safe_sqrt(sq(gps_sensor.data.velocity.x) + sq(gps_sensor.data.velocity.y));
        mavlink_msg_gps_raw_int_pack_chan(get_CommulinkSysId(),  // system id
                                          get_CommulinkCompId(), // component id
                                          port,                  // chan
                                          msg_sd,
                                          TIME::get_System_Run_Time() * 1e6,                                         // usec
                                          (gps_sensor.inf.addition_inf[1] > 1) ? gps_sensor.inf.addition_inf[1] : 1, // fixtype
                                          gps_sensor.data.position_Global.x * 1e+7,                                  // lat
                                          gps_sensor.data.position_Global.y * 1e+7,                                  // lon
                                          gps_sensor.data.position_Global.z * 10,                                    // alt
                                          gps_sensor.inf.addition_inf[4],                                            // eph
                                          gps_sensor.inf.addition_inf[5],                                            // epv
                                          vel,                                                                       // vel
                                          0,                                                                         // cog
                                          gps_sensor.inf.addition_inf[0],                                            // satellites_visible
                                          0,                                                                         // alt_ellipsoid
                                          gps_sensor.inf.addition_inf[4] * 10,                                       // h_acc
                                          gps_sensor.inf.addition_inf[5] * 10,                                       // v_acc
                                          gps_sensor.inf.addition_inf[6] * 10,                                       // vel_acc
                                          0,                                                                         // hdg_acc
                                          0                                                                          // yaw
        );
        return true;
    }
    else
    {
        mavlink_msg_gps_raw_int_pack_chan(get_CommulinkSysId(),  // system id
                                          get_CommulinkCompId(), // component id
                                          port,                  // chan
                                          msg_sd,
                                          TIME::get_System_Run_Time() * 1e6, // usec
                                          0,                                 // fixtype
                                          0,                                 // lat
                                          0,                                 // lon
                                          0,                                 // alt
                                          0,                                 // eph
                                          0,                                 // epv
                                          0,                                 // vel
                                          0,                                 // cog
                                          0,                                 // satellites_visible
                                          0,                                 // alt_ellipsoid
                                          0,                                 // h_acc
                                          0,                                 // v_acc
                                          0,                                 // vel_acc
                                          0,                                 // hdg_acc
                                          0                                  // yaw
        );
        return true;
    }
    return false;
}

static bool Msg124_GPS2_RAW(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Altitude_MSStatus() != MS_Ready)
        return false;

    bool gps_exist = false;
    Position_Sensor gps_sensor;
    uint8_t checked_gps = 0;
    for (uint8_t i = 0; i < default_max_gps_sensors; ++i)
    {
        if (GetPositionSensor(default_gps1_sensor_index + i, &gps_sensor))
        {
            ++checked_gps;
            if (checked_gps == 2)
            {
                gps_exist = true;
                break;
            }
        }
    }

    if (gps_exist)
    {
        mavlink_msg_gps2_raw_pack_chan(get_CommulinkSysId(),  // system id
                                       get_CommulinkCompId(), // component id
                                       port,                  // chan
                                       msg_sd,
                                       TIME::get_System_Run_Time() * 1e6,                                         // usec
                                       (gps_sensor.inf.addition_inf[1] > 1) ? gps_sensor.inf.addition_inf[1] : 1, // fixtype
                                       gps_sensor.data.position_Global.x * 1e+7,                                  // lat
                                       gps_sensor.data.position_Global.y * 1e+7,                                  // lon
                                       gps_sensor.data.position_Global.z * 10,                                    // alt
                                       gps_sensor.inf.addition_inf[4],                                            // h_acc
                                       gps_sensor.inf.addition_inf[5],                                            // v_acc
                                       0,                                                                         // vel
                                       0,                                                                         // cog
                                       gps_sensor.inf.addition_inf[0],                                            // satellites_visible
                                       0,                                                                         // dgps_numch
                                       0,                                                                         // dgps_age
                                       0,                                                                         // yaw
                                       0,                                                                         // alt_ellipsoid
                                       gps_sensor.inf.addition_inf[4],                                            // h_acc
                                       gps_sensor.inf.addition_inf[5],                                            // v_acc
                                       0,                                                                         // vel_acc
                                       0                                                                          // hdg_acc
        );
        return true;
    }
    else
    {
        mavlink_msg_gps2_raw_pack_chan(get_CommulinkSysId(),  // system id
                                       get_CommulinkCompId(), // component id
                                       port,                  // chan
                                       msg_sd,
                                       TIME::get_System_Run_Time() * 1e6, // usec
                                       0,                                 // fixtype
                                       0,                                 // lat
                                       0,                                 // lon
                                       0,                                 // alt
                                       0,                                 // eph
                                       0,                                 // epv
                                       0,                                 // vel
                                       0,                                 // cog
                                       0,                                 // satellites_visible
                                       0,                                 // dgps_numch
                                       0,                                 // dgps_age
                                       0,                                 // yaw
                                       0,                                 // alt_ellipsoid
                                       0,                                 // h_acc
                                       0,                                 // v_acc
                                       0,                                 // vel_acc
                                       0                                  // hdg_acc
        );
        return true;
    }
    return false;
}

static bool Msg33_GLOBAL_POSITION_INT(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    double lat = 0, lon = 0, alt = 0;

    PosSensorHealthInf2 global_posInf;
    if (get_OptimalGlobal_XY(&global_posInf))
    {
        map_projection_reproject(&global_posInf.mp, global_posInf.PositionENU.x + global_posInf.HOffset.x, global_posInf.PositionENU.y + global_posInf.HOffset.y, &lat, &lon);
    }

    PosSensorHealthInf1 z_posInf;
    if (get_OptimalGlobal_Z(&z_posInf))
        alt = z_posInf.PositionENU.z + z_posInf.HOffset;

    static bool last_inFlight = false;
    static double homeZ = 0;
    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight != last_inFlight)
    {
        getHomeLocalZ(&homeZ, 0, 0.01);
        last_inFlight = inFlight;
    }
    //	double homeZ = 0;
    //	getHomeLocalZ( &homeZ, 0, 0.01 );
    vector3<double> position;
    get_Position_Ctrl(&position);
    double heightAboveGround = position.z - homeZ;

    vector3<double> vel;
    get_VelocityENU_Ctrl(&vel);

    Quaternion airframe_quat;
    get_AirframeY_quat(&airframe_quat);
    airframe_quat.Enu2Ned();
    double heading = airframe_quat.getYaw();
    if (heading < 0)
        heading = 2 * Pi + heading;

    mavlink_msg_global_position_int_pack_chan(get_CommulinkSysId(),  // system id
                                              get_CommulinkCompId(), // component id
                                              port,                  // chan
                                              msg_sd,
                                              TIME::get_System_Run_Time() * 1e3, // usec
                                              lat * 1e7,                         // lat
                                              lon * 1e7,                         // lon
                                              alt * 10,                          // alt
                                              heightAboveGround * 10,            // Altitude above ground
                                              vel.y,                             // vel north
                                              vel.x,                             // vel east
                                              -vel.z,                            // vel down
                                              rad2degree(heading) * 100          // Vehicle heading
    );
    return true;
}

static bool Msg32_LOCAL_POSITION_NED(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Altitude_MSStatus() != MS_Ready)
        return false;

    vector3<double> Position;
    get_Position_Ctrl(&Position);
    vector3<double> VelocityENU;
    get_VelocityENU_Ctrl(&VelocityENU);

    mavlink_msg_local_position_ned_pack_chan(get_CommulinkSysId(),  // system id
                                             get_CommulinkCompId(), // component id
                                             port,                  // chan
                                             msg_sd,
                                             TIME::get_System_Run_Time() * 1000, // boot ms
                                             Position.y * 0.01f, Position.x * 0.01f, Position.z * -0.01f, VelocityENU.y * 0.01f, VelocityENU.x * 0.01f, VelocityENU.z * -0.01f);
    return true;
}

static bool Msg34_RC_CHANNELS_SCALED(uint8_t port, mavlink_message_t *msg_sd)
{
    Receiver rc;
    if (getReceiver(&rc))
    {
        if (rc.available)
        {
            mavlink_msg_rc_channels_scaled_pack_chan(get_CommulinkSysId(),  // system id
                                                     get_CommulinkCompId(), // component id
                                                     port,                  // chan
                                                     msg_sd,
                                                     TIME::get_System_Run_Time() * 1000, // boot ms
                                                     0,                                  // port
                                                     rc.available_channels > 0 ? rc.data[0] * 200 - 10000 : INT16_MAX, rc.available_channels > 1 ? rc.data[1] * 200 - 10000 : INT16_MAX,
                                                     rc.available_channels > 2 ? rc.data[2] * 200 - 10000 : INT16_MAX, rc.available_channels > 3 ? rc.data[3] * 200 - 10000 : INT16_MAX,
                                                     rc.available_channels > 4 ? rc.data[4] * 200 - 10000 : INT16_MAX, rc.available_channels > 5 ? rc.data[5] * 200 - 10000 : INT16_MAX,
                                                     rc.available_channels > 6 ? rc.data[6] * 200 - 10000 : INT16_MAX, rc.available_channels > 7 ? rc.data[7] * 200 - 10000 : INT16_MAX,
                                                     255 // rssi
            );
            return true;
        }
    }
    mavlink_msg_rc_channels_scaled_pack_chan(get_CommulinkSysId(),  // system id
                                             get_CommulinkCompId(), // component id
                                             port,                  // chan
                                             msg_sd,
                                             TIME::get_System_Run_Time() * 1000, // boot ms
                                             0,                                  // port
                                             INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX,
                                             255 // rssi
    );
    return true;
}

static bool Msg36_SERVO_OUTPUT_RAW(uint8_t port, mavlink_message_t *msg_sd)
{
    uint8_t pwm_count = get_PWMChannelsCount();
    extern volatile float PWM_CHANS[MAX_PWM_COUNT];
    uint16_t servo[16] = {0};
    for (uint8_t i = 0; i < 16 && i < pwm_count; ++i)
        servo[i] = (uint16_t) (PWM_CHANS[i] * 10.0f + 1000.0f);

    mavlink_msg_servo_output_raw_pack_chan(get_CommulinkSysId(),  // system id
                                           get_CommulinkCompId(), // component id
                                           port,                  // chan
                                           msg_sd,
                                           TIME::get_System_Run_Time() * 1e6, // boot us
                                           0,                                 // port (0=MAIN)
                                           servo[0], servo[1], servo[2], servo[3], servo[4], servo[5], servo[6], servo[7], servo[8], servo[9], servo[10], servo[11], servo[12], servo[13], servo[14],
                                           servo[15]);
    return true;
}

static bool Msg65_RC_CHANNELS(uint8_t port, mavlink_message_t *msg_sd)
{
    Receiver rc;
    if (getReceiver(&rc))
    {
        if (rc.connected)
        {
            mavlink_msg_rc_channels_pack_chan(get_CommulinkSysId(),  // system id
                                              get_CommulinkCompId(), // component id
                                              port,                  // chan
                                              msg_sd,
                                              TIME::get_System_Run_Time() * 1000, // boot ms
                                              rc.raw_available_channels,          // chan count
                                              rc.raw_available_channels > 0 ? rc.raw_data[0] * 10 + 1000 : UINT16_MAX, rc.raw_available_channels > 1 ? rc.raw_data[1] * 10 + 1000 : UINT16_MAX,
                                              rc.raw_available_channels > 2 ? rc.raw_data[2] * 10 + 1000 : UINT16_MAX, rc.raw_available_channels > 3 ? rc.raw_data[3] * 10 + 1000 : UINT16_MAX,
                                              rc.raw_available_channels > 4 ? rc.raw_data[4] * 10 + 1000 : UINT16_MAX, rc.raw_available_channels > 5 ? rc.raw_data[5] * 10 + 1000 : UINT16_MAX,
                                              rc.raw_available_channels > 6 ? rc.raw_data[6] * 10 + 1000 : UINT16_MAX, rc.raw_available_channels > 7 ? rc.raw_data[7] * 10 + 1000 : UINT16_MAX,
                                              rc.raw_available_channels > 8 ? rc.raw_data[8] * 10 + 1000 : UINT16_MAX, rc.raw_available_channels > 9 ? rc.raw_data[9] * 10 + 1000 : UINT16_MAX,
                                              rc.raw_available_channels > 10 ? rc.raw_data[10] * 10 + 1000 : UINT16_MAX, rc.raw_available_channels > 11 ? rc.raw_data[11] * 10 + 1000 : UINT16_MAX,
                                              rc.raw_available_channels > 12 ? rc.raw_data[12] * 10 + 1000 : UINT16_MAX, rc.raw_available_channels > 13 ? rc.raw_data[13] * 10 + 1000 : UINT16_MAX,
                                              rc.raw_available_channels > 14 ? rc.raw_data[14] * 10 + 1000 : UINT16_MAX, rc.raw_available_channels > 15 ? rc.raw_data[15] * 10 + 1000 : UINT16_MAX,
                                              UINT16_MAX, UINT16_MAX,
                                              255 // rssi
            );
            return true;
        }
    }
    mavlink_msg_rc_channels_pack_chan(get_CommulinkSysId(),  // system id
                                      get_CommulinkCompId(), // component id
                                      port,                  // chan
                                      msg_sd,
                                      TIME::get_System_Run_Time() * 1000, // boot ms
                                      0,                                  // chan count
                                      UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                                      UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                                      255 // rssi
    );
    return true;
}

static bool Msg69_MANUAL_CONTROL(uint8_t port, mavlink_message_t *msg_sd)
{
    Receiver rc;
    if (getReceiver(&rc))
    {
        if (rc.available)
        {
            mavlink_msg_manual_control_pack_chan(get_CommulinkSysId(),  // system id
                                                 get_CommulinkCompId(), // component id
                                                 port,                  // chan
                                                 msg_sd,
                                                 get_CommulinkSysId(),                  // target
                                                 (int16_t) ((rc.data[2] - 50.0f) * 20), // x: 俯仰(前为正)
                                                 (int16_t) ((rc.data[3] - 50.0f) * 20), // y: 横滚(右为正)
                                                 (int16_t) (rc.data[0] * 10.0f),        // z: 油门(0~1000)
                                                 (int16_t) ((rc.data[1] - 50.0f) * 20), // r: 偏航(右为正)
                                                 0,                                     // buttons
                                                 0,                                     // buttons2
                                                 0,                                     // enabled_extensions
                                                 0, 0,                                  // s, t
                                                 0, 0, 0, 0, 0, 0);                     // aux1~6
            return true;
        }
    }
    mavlink_msg_manual_control_pack_chan(get_CommulinkSysId(),  // system id
                                         get_CommulinkCompId(), // component id
                                         port,                  // chan
                                         msg_sd,
                                         get_CommulinkSysId(),                                              // target
                                         INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX,                        // x, y, z, r (无效)
                                         0,                                                                 // buttons
                                         0,                                                                 // buttons2
                                         0,                                                                 // enabled_extensions
                                         0, 0,                                                              // s, t
                                         INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX); // aux1~6
    return true;
}

static bool Msg74_VFR_HUD(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Altitude_MSStatus() != MS_Ready)
        return false;

    vector3<double> Position;
    get_Position_Ctrl(&Position);
    vector3<double> VelocityENU;
    get_VelocityENU_Ctrl(&VelocityENU);

    double speed = safe_sqrt(VelocityENU.x * VelocityENU.x + VelocityENU.y * VelocityENU.y);
    double throttle = 0;
    double stThr = getAttCtrlCfg()->STThrottle[0];
    get_OutputThrottle(&throttle);
    if (throttle < 1)
        throttle = 0;
    else
        throttle = (throttle * (100 - stThr) / 100) + stThr;

    double homeZ;
    double heightAboveGround = 0;
    if (getHomeLocalZ(&homeZ, 0, 0.01))
        heightAboveGround = Position.z - homeZ;

    mavlink_msg_vfr_hud_pack_chan(get_CommulinkSysId(),  // system id
                                  get_CommulinkCompId(), // component id
                                  port,                  // chan
                                  msg_sd,
                                  speed * 0.01,                                     // airspeed
                                  speed * 0.01,                                     // groundspeed
                                  rad2degree(atan2f(VelocityENU.y, VelocityENU.x)), // heading
                                  throttle,                                         // throttle
                                  heightAboveGround * 0.01f,                        // alt
                                  VelocityENU.z * 0.01f                             // climb
    );
    return true;
}

static bool Msg42_MISSION_CURRENT(uint8_t port, mavlink_message_t *msg_sd)
{
    uint16_t flag = 0;

    bool inFlight;
    get_is_inFlight(&inFlight);

    extern bool inMissionFlight;
    if (getCurrentFlyMode() == AFunc_Mission && inFlight && inMissionFlight)
        flag |= MAV_MISSION_CURRENT_IS_IN_MISSION_FLIGHT;

    RestoreWpInf restoreWpInf;
    uint16_t resumeSeq = 0;
    vector3<float> vec;
    float line_fs = 0;
    float trigInt = 0;
    float speed = 0;
    float radius = 0;
    if (ReadParamGroup("RestoreWpInf", (uint64_t *) &restoreWpInf, 0) == PR_OK)
    {
        if (restoreWpInf.CurrentWp[0] > 0)
        {
            flag |= MAV_MISSION_CURRENT_IS_RESUME_WAYPOINT_EXIST;

            resumeSeq = restoreWpInf.CurrentWp[0];
            vec.x = restoreWpInf.line_y * 0.01;
            vec.y = restoreWpInf.line_x * 0.01;
            vec.z = -restoreWpInf.line_z * 0.01;
            line_fs = restoreWpInf.line_fs * 0.01;
            trigInt = restoreWpInf.CamTrigDist * 0.01;
            speed = restoreWpInf.vel * 0.01;
            radius = restoreWpInf.wpR * 0.01;
        }
    }

    mavlink_msg_mission_current_pack_chan(get_CommulinkSysId(),  // system id
                                          get_CommulinkCompId(), // component id
                                          port,                  // chan
                                          msg_sd,
                                          getCurrentMissionInd(), // current mission ind
                                          resumeSeq,              // resume ind
                                          flag,                   // flag
                                          0,                      // heading
                                          vec.x, vec.y, vec.z);
    return true;
}
static bool Msg62_NAV_CONTROLLER_OUTPUT(uint8_t port, mavlink_message_t *msg_sd)
{
    // 获取目标roll pitch角度
    double rol, pit;
    Attitude_Control_get_Target_RollPitch(&rol, &pit);

    // 获取目标偏航角
    double yaw;
    Attitude_Control_get_TargetYaw(&yaw);
    double bearing = 0;
    vector3<double> AB;
    if (Position_Control_get_LineFlightABDistance(&AB, 0))
        bearing = atan2(-AB.y, -AB.x);

    // 计算航点距离
    MissionInf current_mission;
    uint16_t current_mission_ind = 0;
    float wp_distance = 0;
    if (ReadCurrentMission(&current_mission, &current_mission_ind))
    {
        if (current_mission.cmd == MAV_CMD_NAV_WAYPOINT)
        {
            switch (current_mission.frame)
            {
                case MAV_FRAME_GLOBAL:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                case MAV_FRAME_GLOBAL_INT:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                { // 全球定位
                    // 获取最优全球定位传感器信息
                    PosSensorHealthInf2 global_inf;
                    if (get_OptimalGlobal_XY(&global_inf))
                    {
                        // 获取指定经纬度平面坐标
                        double x, y;
                        map_projection_project(&global_inf.mp, current_mission.params[4], current_mission.params[5], &x, &y);
                        x -= global_inf.HOffset.x;
                        y -= global_inf.HOffset.y;
                        wp_distance = safe_sqrt(sq(x - global_inf.PositionENU.x) + sq(y - global_inf.PositionENU.y));
                    }
                    break;
                }

                case MAV_FRAME_LOCAL_NED:
                {
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    wp_distance = safe_sqrt(sq(current_mission.params[5] * 100 - position.x) + sq(current_mission.params[4] * 100 - position.y));
                    break;
                }

                case MAV_FRAME_LOCAL_ENU:
                {
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    wp_distance = safe_sqrt(sq(current_mission.params[4] * 100 - position.x) + sq(current_mission.params[5] * 100 - position.y));
                    break;
                }

                case MAV_FRAME_LOCAL_OFFSET_NED:
                case MAV_FRAME_BODY_NED:
                case MAV_FRAME_BODY_FRD:
                case MAV_FRAME_BODY_OFFSET_NED:
                case MAV_FRAME_BODY_FLU:
                {
                    wp_distance = safe_sqrt(sq(current_mission.params[4] * 100) + sq(current_mission.params[5] * 100));
                    break;
                }

                default:
                    wp_distance = 0;
            }
        }
    }

    mavlink_msg_nav_controller_output_pack_chan(get_CommulinkSysId(),  // system id
                                                get_CommulinkCompId(), // component id
                                                port,                  // chan
                                                msg_sd,
                                                rad2degree(rol),            // nav roll
                                                rad2degree(pit),            // nav pitch
                                                fmod(rad2degree(yaw), 360), // nav yaw
                                                rad2degree(bearing),        // nav bearing
                                                wp_distance * 0.01f,        // wp_dist
                                                0,                          // current alt error
                                                0,                          // current airspeed error
                                                0                           // current crosstrack error on x-y plane
    );
    return true;
}

static bool Msg234_HIGH_LATENCY(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    bool inFlight;
    get_is_inFlight(&inFlight);

    extern bool GCS_is_MP[MAVLINK_COMM_NUM_BUFFERS];
    uint16_t mav_mode, mav_main_mode, mav_sub_mode;
    get_mav_modes(mav_mode, mav_main_mode, mav_sub_mode);

    px4_custom_mode custom_mode;
    custom_mode.main_mode = mav_main_mode;
    if ((mav_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) && port < MAVLINK_COMM_NUM_BUFFERS && GCS_is_MP[port] && mav_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO)
        custom_mode.main_mode = 0;
    custom_mode.sub_mode = mav_sub_mode;

    Quaternion airframe_quat;
    get_Attitude_quat(&airframe_quat);
    airframe_quat.Enu2Ned();
    vector3<double> angular_rate;
    get_AngularRate_Ctrl(&angular_rate);
    double heading = airframe_quat.getYaw();
    if (heading < 0)
        heading = 2 * Pi + heading;

    double throttle = 0;
    double stThr = getAttCtrlCfg()->STThrottle[0];
    get_OutputThrottle(&throttle);
    if (throttle < 1)
        throttle = 0;
    else
        throttle = (throttle * (100 - stThr) / 100) + stThr;

    uint8_t gps_sat = 0;
    uint8_t gps_fix = 0;
    int32_t gps_lat = 0;
    int32_t gps_lon = 0;
    int32_t gps_alt = 0;
    Position_Sensor gps_sensor;
    if (GetPositionSensor(default_gps2_sensor_index, &gps_sensor))
    {
        gps_sat = gps_sensor.inf.addition_inf[0];
        gps_fix = (gps_sensor.inf.addition_inf[1] > 1) ? gps_sensor.inf.addition_inf[1] : 1;
        gps_lat = gps_sensor.data.position_Global.x * 1e+7;
        gps_lon = gps_sensor.data.position_Global.y * 1e+7;
        gps_alt = gps_sensor.data.position_Global.z * 10;
    }
    else if (GetPositionSensor(default_gps1_sensor_index, &gps_sensor))
    {
        gps_sat = gps_sensor.inf.addition_inf[0];
        gps_fix = (gps_sensor.inf.addition_inf[1] > 1) ? gps_sensor.inf.addition_inf[1] : 1;
        gps_lat = gps_sensor.data.position_Global.x * 1e+7;
        gps_lon = gps_sensor.data.position_Global.y * 1e+7;
        gps_alt = gps_sensor.data.position_Global.z * 10;
    }

    vector3<double> Position;
    get_Position_Ctrl(&Position);
    double homeZ;
    double heightAboveGround = 0;
    if (getHomeLocalZ(&homeZ, 0, 0.01))
        heightAboveGround = Position.z - homeZ;

    vector3<double> vel;
    get_VelocityENU_Ctrl(&vel);
    double Gs = safe_sqrt(vel.x * vel.x + vel.y * vel.y);

    BatteryInfo batInfo = {0};
    getCurrentBatteryInfo(&batInfo);

    MissionInf current_mission;
    uint16_t current_mission_ind = 0;
    float wp_distance = 0;
    if (ReadCurrentMission(&current_mission, &current_mission_ind))
    {
        if (current_mission.cmd == MAV_CMD_NAV_WAYPOINT)
        {
            switch (current_mission.frame)
            {
                case MAV_FRAME_GLOBAL:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                case MAV_FRAME_GLOBAL_INT:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                { // 全球定位
                    // 获取最优全球定位传感器信息
                    PosSensorHealthInf2 global_inf;
                    if (get_OptimalGlobal_XY(&global_inf))
                    {
                        // 获取指定经纬度平面坐标
                        double x, y;
                        map_projection_project(&global_inf.mp, current_mission.params[4], current_mission.params[5], &x, &y);
                        x -= global_inf.HOffset.x;
                        y -= global_inf.HOffset.y;
                        wp_distance = safe_sqrt(sq(x - global_inf.PositionENU.x) + sq(y - global_inf.PositionENU.y));
                    }
                    break;
                }

                case MAV_FRAME_LOCAL_NED:
                {
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    wp_distance = safe_sqrt(sq(current_mission.params[5] * 100 - position.x) + sq(current_mission.params[4] * 100 - position.y));
                    break;
                }

                case MAV_FRAME_LOCAL_ENU:
                {
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    wp_distance = safe_sqrt(sq(current_mission.params[4] * 100 - position.x) + sq(current_mission.params[5] * 100 - position.y));
                    break;
                }

                case MAV_FRAME_LOCAL_OFFSET_NED:
                case MAV_FRAME_BODY_NED:
                case MAV_FRAME_BODY_FRD:
                case MAV_FRAME_BODY_OFFSET_NED:
                case MAV_FRAME_BODY_FLU:
                {
                    wp_distance = safe_sqrt(sq(current_mission.params[4] * 100) + sq(current_mission.params[5] * 100));
                    break;
                }

                default:
                    wp_distance = 0;
            }
        }
    }

    mavlink_msg_high_latency_pack_chan(get_CommulinkSysId(),  // system id
                                       get_CommulinkCompId(), // component id
                                       port,                  // chan
                                       msg_sd,
                                       mav_mode > 0 ? MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mav_mode : 0, // base mode
                                       custom_mode.data,                                                // custom mode
                                       inFlight ? MAV_LANDED_STATE_IN_AIR : MAV_LANDED_STATE_ON_GROUND, // landed state
                                       rad2degree(airframe_quat.getRoll()) * 100,                       // roll
                                       rad2degree(airframe_quat.getPitch()) * 100,                      // pitch
                                       rad2degree(heading) * 100,                                       // heading
                                       throttle,                                                        // throttle
                                       rad2degree(angular_rate.z) * 100,                                // heading sp
                                       gps_lat,                                                         // lat
                                       gps_lon,                                                         // lon
                                       heightAboveGround * 0.01,                                        // alt_amsl
                                       heightAboveGround * 0.01,                                        // alt setpoint
                                       Gs * 0.01,                                                       // air speed
                                       0,                                                               // air speed setpoint
                                       Gs * 0.01,                                                       // ground speed
                                       vel.z * 0.01,                                                    // climb rate
                                       gps_sat,                                                         // gps nsat
                                       gps_fix,                                                         // gps fixtype
                                       batInfo.totalPercent,                                            // battery remaining
                                       0,                                                               // temperature
                                       0,                                                               // temperature air
                                       0,                                                               // failsafe
                                       current_mission_ind,                                             // wp_num
                                       wp_distance * 0.01                                               // wp_distance
    );

    return true;
}

static bool Msg241_VIBRATION(uint8_t Port_index, mavlink_message_t *msg)
{
    vector3<float> vibration;
    if (get_Vibration(&vibration))
    {
        const Port *port = get_CommuPort(Port_index);
        if (port && port->write)
        {
            mavlink_msg_vibration_pack_chan(get_CommulinkSysId(),  // system id
                                            get_CommulinkCompId(), // component id
                                            Port_index,            // chan
                                            msg,
                                            TIME::get_System_Run_Time() * 1e6, // usec
                                            vibration.x * 0.01, vibration.y * 0.01, vibration.z * 0.01, 0, 0, 0);

            return true;
        }
    }
    return false;
}

static bool Msg242_HOME_POSITION(uint8_t Port_index, mavlink_message_t *msg)
{
    static int32_t alt = 0;
    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight == false)
    {
        PosSensorHealthInf1 z_posInf;
        if (get_OptimalGlobal_Z(&z_posInf))
            alt = z_posInf.PositionENU.z + z_posInf.HOffset;
    }

    double Altitude_Local = 0;
    vector2<double> homePLatLon, homePoint;
    if (getHomeLatLon(&homePLatLon) && getHomeLocalZ(&Altitude_Local) && getHomePoint(&homePoint))
    {
        const Port *port = get_CommuPort(Port_index);
        if (port && port->write)
        {
            // 将存储的 int32 经纬度(degE7)按位模式重解释为 float 发送
            // approach 字段是 float 类型，数值按位传递
            uint32_t AL_lat = 0, AL_lon = 0;
            MSafeCfg safeCfg;
            if (ReadParamGroup("MSafe", (uint64_t *) &safeCfg, 0) == PR_OK)
            {
                // 先转 int32 再比较：无符号数与负边界比较永远不会失败
                // （南纬/西经坐标存为补码，按无符号解释会超出 90e7/180e7 被误判无效）
                int32_t lat_i = (int32_t) safeCfg.AL1_lat[0];
                int32_t lon_i = (int32_t) safeCfg.AL1_lon[0];
                if (lat_i != 0 && lon_i != 0 && lat_i >= -90e7 && lat_i <= 90e7 && lon_i >= -180e7 && lon_i <= 180e7)
                {
                    AL_lat = safeCfg.AL1_lat[0];
                    AL_lon = safeCfg.AL1_lon[0];
                }
            }
            mavlink_msg_home_position_pack_chan(get_CommulinkSysId(),  // system id
                                                get_CommulinkCompId(), // component id
                                                Port_index,            // chan
                                                msg,
                                                homePLatLon.x * 1e7,   // latitude
                                                homePLatLon.y * 1e7,   // longitude
                                                alt * 10,              // altitude,mm
                                                homePoint.x * 0.01,    // Local X position
                                                homePoint.y * 0.01,    // Local Y position
                                                Altitude_Local * 0.01, // Local Z position
                                                0,                     // World to surface normal and heading transformation of the takeoff position
                                                bitcast_u2f(AL_lat),   // 进近向量末端 X（按位携带备降点1纬度 degE7）
                                                bitcast_u2f(AL_lon),   // 进近向量末端 Y（按位携带备降点1经度 degE7）
                                                0,                     // Local Z position of the end of the approach vector
                                                TIME::get_System_Run_Time() * 1e6);

            return true;
        }
    }
    return false;
}

// msg331 ODOMETRY：发送 SLAM 传感器（槽位11, default_vslam_pos_index）的原始位置/速度读数
// SLAM 输出 q = FLU body → ENU world；MAVLink ODOMETRY 要求 FRD body → NED world
// q_331 = R_ned←enu · q_slam · R_flu←frd
static bool Msg331_ODOMETRY(uint8_t port, mavlink_message_t *msg_sd)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    // SLAM 传感器固定槽位 11（由 Msg102/Msg331 接收时注册到该槽位）
    const uint8_t slamSensorInd = 11;

    Position_Sensor_Data sensor;
    if (!GetPositionSensorData(slamSensorInd, &sensor))
        return false;
    if (!sensor.available)
        return false;

    // 协方差填 NaN（飞控不输出协方差）
    float pose_cov[21], vel_cov[21];
    for (uint8_t i = 0; i < 21; ++i)
    {
        pose_cov[i] = NAN;
        vel_cov[i] = NAN;
    }

    // 四元数 FLU→ENU 转 FRD→NED
    // R_ned←enu: 180° 绕 [1,1,0]/√2 轴 → (0, √2/2, √2/2, 0)
    // R_flu←frd: 180° 绕 x 轴 → (0, 1, 0, 0)
    Quaternion q_ned_enu(0, 0.7071067811865476, 0.7071067811865476, 0);
    Quaternion q_flu_frd(0, 1, 0, 0);
    Quaternion q_out = q_ned_enu * sensor.data_quat * q_flu_frd;
    float q[4] = {(float) q_out.get_qw(), (float) q_out.get_qx(), (float) q_out.get_qy(), (float) q_out.get_qz()};

    // 速度可用性判断（DataType 去掉 nAC 偏移后 >=8 表示含速度数据）
    // ENU→NED：x(North)=ENU.y, y(East)=ENU.x, z(Down)=-ENU.z
    uint8_t dt = sensor.sensor_DataType % 100;
    float vx = (dt >= 8) ? sensor.velocity.y * 0.01f : NAN;
    float vy = (dt >= 8) ? sensor.velocity.x * 0.01f : NAN;
    float vz = (dt >= 8) ? -sensor.velocity.z * 0.01f : NAN;

    mavlink_msg_odometry_pack_chan(get_CommulinkSysId(),  // system id
                                   get_CommulinkCompId(), // component id
                                   port,                  // chan
                                   msg_sd,
                                   TIME::get_System_Run_Time() * 1e6, // time_usec
                                   MAV_FRAME_VISION_NED,              // frame_id (NED)
                                   MAV_FRAME_BODY_FRD,                // child_frame_id
                                   sensor.position.y * 0.01f,         // x (North=ENU.y, cm→m)
                                   sensor.position.x * 0.01f,         // y (East=ENU.x, cm→m)
                                   -sensor.position.z * 0.01f,        // z (Down=-ENU.z, cm→m)
                                   q,                                 // quaternion (FRD→NED)
                                   vx, vy, vz,                        // velocity (ENU→NED)
                                   0, 0, 0,                           // rollspeed/pitchspeed/yawspeed
                                   pose_cov, vel_cov,                 // covariance (NaN)
                                   0,                                 // reset_counter
                                   MAV_ESTIMATOR_TYPE_VISION,         // estimator_type
                                   0);                                // quality (0=unknown/unset)
    return true;
}

static bool Msg162_FENCE_STATUS(uint8_t port, mavlink_message_t *msg_sd)
{
    FenceBreachStatus fenceStatus;
    if (!getFenceStatus(&fenceStatus, 1))
        return false;
    mavlink_msg_fence_status_pack_chan(get_CommulinkSysId(),  // system id
                                       get_CommulinkCompId(), // component id
                                       port,                  // chan
                                       msg_sd,
                                       fenceStatus.active ? 1 : 0, // breach_status (0 inside, 1 outside)
                                       fenceStatus.count,          // breach_count
                                       fenceStatus.type,           // breach_type
                                       fenceStatus.time,           // breach_time
                                       fenceStatus.mitigation      // breach_mitigation
    );
    if (port == 0)
        tryResetFenceBreachStatus();
    return true;
}

bool (*const Mavlink_Send_Funcs[])(uint8_t port, mavlink_message_t *msg_sd) = {
    /*000-*/ 0,
    /*001-*/ Msg01_SYS_STATUS,
    /*002-*/ Msg02_SYSTEM_TIME,
    /*003-*/ 0,
    /*004-*/ 0,
    /*005-*/ 0,
    /*006-*/ 0,
    /*007-*/ 0,
    /*008-*/ 0,
    /*009-*/ 0,
    /*010-*/ 0,
    /*011-*/ 0,
    /*012-*/ 0,
    /*013-*/ 0,
    /*014-*/ 0,
    /*015-*/ 0,
    /*016-*/ 0,
    /*017-*/ 0,
    /*018-*/ 0,
    /*019-*/ 0,
    /*020-*/ 0,
    /*021-*/ 0,
    /*022-*/ 0,
    /*023-*/ 0,
    /*024-*/ Msg24_GPS_RAW_INT,
    /*025-*/ 0,
    /*026-*/ Msg26_SCALED_IMU,
    /*027-*/ 0,
    /*028-*/ 0,
    /*029-*/ Msg29_SCALED_PRESSURE,
    /*030-*/ Msg30_ATTITUDE,
    /*031-*/ Msg31_ATTITUDE_QUATERNION,
    /*032-*/ Msg32_LOCAL_POSITION_NED,
    /*033-*/ Msg33_GLOBAL_POSITION_INT,
    /*034-*/ Msg34_RC_CHANNELS_SCALED,
    /*035-*/ 0,
    /*036-*/ Msg36_SERVO_OUTPUT_RAW,
    /*037-*/ 0,
    /*038-*/ 0,
    /*039-*/ 0,
    /*040-*/ 0,
    /*041-*/ 0,
    /*042-*/ Msg42_MISSION_CURRENT,
    /*043-*/ 0,
    /*044-*/ 0,
    /*045-*/ 0,
    /*046-*/ 0,
    /*047-*/ 0,
    /*048-*/ 0,
    /*049-*/ 0,
    /*050-*/ 0,
    /*051-*/ 0,
    /*052-*/ 0,
    /*053-*/ 0,
    /*054-*/ 0,
    /*055-*/ 0,
    /*056-*/ 0,
    /*057-*/ 0,
    /*058-*/ 0,
    /*059-*/ 0,
    /*060-*/ 0,
    /*061-*/ 0,
    /*062-*/ Msg62_NAV_CONTROLLER_OUTPUT,
    /*063-*/ 0,
    /*064-*/ 0,
    /*065-*/ Msg65_RC_CHANNELS,
    /*066-*/ 0,
    /*067-*/ 0,
    /*068-*/ 0,
    /*069-*/ Msg69_MANUAL_CONTROL,
    /*070-*/ 0,
    /*071-*/ 0,
    /*072-*/ 0,
    /*073-*/ 0,
    /*074-*/ Msg74_VFR_HUD,
    /*075-*/ 0,
    /*076-*/ 0,
    /*077-*/ 0,
    /*078-*/ 0,
    /*079-*/ 0,
    /*080-*/ 0,
    /*081-*/ 0,
    /*082-*/ 0,
    /*083-*/ 0,
    /*084-*/ 0,
    /*085-*/ 0,
    /*086-*/ 0,
    /*087-*/ 0,
    /*088-*/ 0,
    /*089-*/ 0,
    /*090-*/ 0,
    /*091-*/ 0,
    /*092-*/ 0,
    /*093-*/ 0,
    /*094-*/ 0,
    /*095-*/ 0,
    /*096-*/ 0,
    /*097-*/ 0,
    /*098-*/ 0,
    /*099-*/ 0,
    /*100-*/ 0,
    /*101-*/ 0,
    /*102-*/ 0,
    /*103-*/ 0,
    /*104-*/ 0,
    /*105-*/ 0,
    /*106-*/ 0,
    /*107-*/ 0,
    /*108-*/ 0,
    /*109-*/ 0,
    /*110-*/ 0,
    /*111-*/ 0,
    /*112-*/ 0,
    /*113-*/ 0,
    /*114-*/ 0,
    /*115-*/ 0,
    /*116-*/ Msg116_SCALED_IMU2,
    /*117-*/ 0,
    /*118-*/ 0,
    /*119-*/ 0,
    /*120-*/ 0,
    /*121-*/ 0,
    /*122-*/ 0,
    /*123-*/ 0,
    /*124-*/ Msg124_GPS2_RAW,
    /*125-*/ 0,
    /*126-*/ 0,
    /*127-*/ 0,
    /*128-*/ 0,
    /*129-*/ Msg129_SCALED_IMU3,
    /*130-*/ 0,
    /*131-*/ 0,
    /*132-*/ 0,
    /*133-*/ 0,
    /*134-*/ 0,
    /*135-*/ 0,
    /*136-*/ 0,
    /*137-*/ 0,
    /*138-*/ 0,
    /*139-*/ 0,
    /*140-*/ 0,
    /*141-*/ 0,
    /*142-*/ 0,
    /*143-*/ 0,
    /*144-*/ 0,
    /*145-*/ 0,
    /*146-*/ 0,
    /*147-*/ 0,
    /*148-*/ 0,
    /*149-*/ 0,
    /*150-*/ 0,
    /*151-*/ 0,
    /*152-*/ 0,
    /*153-*/ 0,
    /*154-*/ 0,
    /*155-*/ 0,
    /*156-*/ 0,
    /*157-*/ 0,
    /*158-*/ 0,
    /*159-*/ 0,
    /*160-*/ 0,
    /*161-*/ 0,
    /*162-*/ Msg162_FENCE_STATUS,
    /*163-*/ 0,
    /*164-*/ 0,
    /*165-*/ 0,
    /*166-*/ 0,
    /*167-*/ 0,
    /*168-*/ 0,
    /*169-*/ 0,
    /*170-*/ 0,
    /*171-*/ 0,
    /*172-*/ 0,
    /*173-*/ 0,
    /*174-*/ 0,
    /*175-*/ 0,
    /*176-*/ 0,
    /*177-*/ 0,
    /*178-*/ 0,
    /*179-*/ 0,
    /*180-*/ 0,
    /*181-*/ 0,
    /*182-*/ 0,
    /*183-*/ 0,
    /*184-*/ 0,
    /*185-*/ 0,
    /*186-*/ 0,
    /*187-*/ 0,
    /*188-*/ 0,
    /*189-*/ 0,
    /*190-*/ 0,
    /*191-*/ 0,
    /*192-*/ 0,
    /*193-*/ 0,
    /*194-*/ 0,
    /*195-*/ 0,
    /*196-*/ 0,
    /*197-*/ 0,
    /*198-*/ 0,
    /*199-*/ 0,
    /*200-*/ Msg200_SYS_STATUS_ACFLY,
    /*201-*/ 0,
    /*202-*/ 0,
    /*203-*/ 0,
    /*204-*/ 0,
    /*205-*/ 0,
    /*206-*/ 0,
    /*207-*/ 0,
    /*208-*/ 0,
    /*209-*/ 0,
    /*210-*/ Msg210_MISSION_REPORT,
    /*211-*/ 0,
    /*212-*/ 0,
    /*213-*/ 0,
    /*214-*/ 0,
    /*215-*/ 0,
    /*216-*/ 0,
    /*217-*/ 0,
    /*218-*/ 0,
    /*219-*/ 0,
    /*220-*/ 0,
    /*221-*/ 0,
    /*222-*/ 0,
    /*223-*/ 0,
    /*224-*/ 0,
    /*225-*/ 0,
    /*226-*/ 0,
    /*227-*/ 0,
    /*228-*/ 0,
    /*229-*/ 0,
    /*230-*/ 0,
    /*231-*/ 0,
    /*232-*/ 0,
    /*233-*/ 0,
    /*234-*/ Msg234_HIGH_LATENCY,
    /*235-*/ 0,
    /*236-*/ 0,
    /*237-*/ 0,
    /*238-*/ 0,
    /*239-*/ 0,
    /*240-*/ 0,
    /*241-*/ Msg241_VIBRATION,
    /*242-*/ Msg242_HOME_POSITION,
    /*243-*/ 0,
    /*244-*/ 0,
    /*245-*/ Msg245_EXTENDED_SYS_STATE,
    /*246-*/ 0,
    /*247-*/ 0,
    /*248-*/ 0,
    /*249-*/ 0,
    /*250-*/ 0,
    /*251-*/ 0,
    /*252-*/ 0,
    /*253-*/ 0,
    /*254-*/ 0,
    /*255-*/ 0,
    /*256-*/ 0,
    /*257-*/ 0,
    /*258-*/ 0,
    /*259-*/ 0,
    /*260-*/ 0,
    /*261-*/ 0,
    /*262-*/ 0,
    /*263-*/ 0,
    /*264-*/ 0,
    /*265-*/ 0,
    /*266-*/ 0,
    /*267-*/ 0,
    /*268-*/ 0,
    /*269-*/ 0,
    /*270-*/ 0,
    /*271-*/ 0,
    /*272-*/ 0,
    /*273-*/ 0,
    /*274-*/ 0,
    /*275-*/ 0,
    /*276-*/ 0,
    /*277-*/ 0,
    /*278-*/ 0,
    /*279-*/ 0,
    /*280-*/ 0,
    /*281-*/ 0,
    /*282-*/ 0,
    /*283-*/ 0,
    /*284-*/ 0,
    /*285-*/ 0,
    /*286-*/ 0,
    /*287-*/ 0,
    /*288-*/ 0,
    /*289-*/ 0,
    /*290-*/ 0,
    /*291-*/ 0,
    /*292-*/ 0,
    /*293-*/ 0,
    /*294-*/ 0,
    /*295-*/ 0,
    /*296-*/ 0,
    /*297-*/ 0,
    /*298-*/ 0,
    /*299-*/ 0,
    /*300-*/ 0,
    /*301-*/ 0,
    /*302-*/ 0,
    /*303-*/ 0,
    /*304-*/ 0,
    /*305-*/ 0,
    /*306-*/ 0,
    /*307-*/ 0,
    /*308-*/ 0,
    /*309-*/ 0,
    /*310-*/ 0,
    /*311-*/ 0,
    /*312-*/ 0,
    /*313-*/ 0,
    /*314-*/ 0,
    /*315-*/ 0,
    /*316-*/ 0,
    /*317-*/ 0,
    /*318-*/ 0,
    /*319-*/ 0,
    /*320-*/ 0,
    /*321-*/ 0,
    /*322-*/ 0,
    /*323-*/ 0,
    /*324-*/ 0,
    /*325-*/ 0,
    /*326-*/ 0,
    /*327-*/ 0,
    /*328-*/ 0,
    /*329-*/ 0,
    /*330-*/ Msg330_OBSTACLE_DISTANCE,
    /*331-*/ Msg331_ODOMETRY,
    /*332-*/ 0,
    /*333-*/ 0,
    /*334-*/ 0,
    /*335-*/ 0,
    /*336-*/ 0,
    /*337-*/ 0,
    /*338-*/ 0,
    /*339-*/ 0};
const uint16_t Mavlink_Send_Funcs_Count = sizeof(Mavlink_Send_Funcs) / sizeof(void *);