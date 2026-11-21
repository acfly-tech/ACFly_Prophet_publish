#include "Modes.hpp"
#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "MavlinkRCProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "StorageSystem.hpp"
#include "drv_PWMOut.hpp"
#include "followTarget.hpp"
#include "mavlink.h"
#include "precLand.hpp"
#include "queue.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include <atomic>

#include "M10_RCCalib.hpp"
#include "M11_TempCalib.hpp"
#include "M12_AccCalib.hpp"
#include "M13_MagCalib.hpp"
#include "M14_HorizonCalib.hpp"

#include "M30_AttCtrl.hpp"
#include "M32_PosCtrl.hpp"
// #include "M35_Auto1.hpp"

// 当前航点记录（用于断点续飞）
RestoreWpInf restoreWpInf;
static SemaphoreHandle_t restoreWpInfSemphr = xSemaphoreCreateMutex();

static bool RestoreWpInf_needStore = false;
void set_RestoreWpInf_needStore()
{
    RestoreWpInf_needStore = true;
}

bool read_restoreWpInf(double TIMEOUT)
{
    return (ReadParamGroup("RestoreWpInf", (uint64_t *) &restoreWpInf, 0) == PR_OK);
}
bool storeRAM_restoreWpInf(double TIMEOUT)
{
    RestoreWpInf_needStore = true;
    return (UpdateParamGroup("RestoreWpInf", (uint64_t *) &restoreWpInf, 0, sizeof(restoreWpInf) / 8, false, TIMEOUT) == PR_OK);
}
bool store_restoreWpInf(double TIMEOUT)
{
    return (UpdateParamGroup("RestoreWpInf", (uint64_t *) &restoreWpInf, 0, sizeof(restoreWpInf) / 8, true, TIMEOUT) == PR_OK);
}

// 飞行统计信息
FlightStats flightStats;
static bool FlightStats_needStore = false;
void set_FlightStats_needStore()
{
    FlightStats_needStore = true;
}
bool read_flightStats(double TIMEOUT)
{
    return (ReadParamGroup("Init", (uint64_t *) &flightStats, 0, FLIGHTSTATS_PARAM_OFFSET, FLIGHTSTATS_PARAM_COUNT, TIMEOUT) == PR_OK);
}
bool storeRAM_flightStats(double TIMEOUT)
{
    FlightStats_needStore = true;
    return (UpdateParamGroup("Init", (uint64_t *) &flightStats, FLIGHTSTATS_PARAM_OFFSET, FLIGHTSTATS_PARAM_COUNT, false, TIMEOUT) == PR_OK);
}
bool store_flightStats(double TIMEOUT)
{
    return (UpdateParamGroup("Init", (uint64_t *) &flightStats, FLIGHTSTATS_PARAM_OFFSET, FLIGHTSTATS_PARAM_COUNT, true, TIMEOUT) == PR_OK);
}

// 是否在正常航线飞行
bool inMissionFlight = false;

// Current mission item execution report (protected by mutex)
static MissionItemReport missionItemReport = {0, 0, 0, 0, ACFLY_MISSION_REPORT_EXECUTING};
static SemaphoreHandle_t missionReportMutex = xSemaphoreCreateMutex();

MissionItemReport get_missionItemReport(double TIMEOUT)
{
    MissionItemReport copy = {0, 0, 0, 0, ACFLY_MISSION_REPORT_EXECUTING};
    TickType_t ticks = (TIMEOUT < 0) ? portMAX_DELAY : pdMS_TO_TICKS(TIMEOUT * 1000);
    if (xSemaphoreTake(missionReportMutex, ticks) == pdTRUE)
    {
        copy = missionItemReport;
        xSemaphoreGive(missionReportMutex);
    }
    return copy;
}

void set_missionItemReport(const MissionItemReport &report)
{
    if (xSemaphoreTake(missionReportMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        missionItemReport = report;
        xSemaphoreGive(missionReportMutex);
    }
}

// 模式状态存储
static AFunc currentFlyMode = AFunc_Stabilize;
// 模式进入原因 (上云API mode_code_reason) - 多任务读写, 原子操作避免撕裂
static std::atomic<uint16_t> currentFlyModeReason{MODE_CODE_REASON_NONE};
// 挂起的模式切换原因 (由M32主循环在模式切换时设置, 模式self-report不带reason时采用)
static std::atomic<uint16_t> pendingFlyModeReason{MODE_CODE_REASON_NONE};
bool setCurrentFlyMode(AFunc cMode, uint16_t reason)
{
    extern TaskHandle_t MSafeTaskHandle;
    bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
    if (!isMSafe && is_MSafeCtrl())
    { // 屏蔽用户控制
        return false;
    }
    // 模式改变时: 优先采用显式reason, 无显式reason时采用挂起原因, 模式未变时保留原reason
    if (cMode != currentFlyMode)
    {
        if (reason != MODE_CODE_REASON_NONE)
            currentFlyModeReason = reason;
        else
            currentFlyModeReason = pendingFlyModeReason.load();
        pendingFlyModeReason = MODE_CODE_REASON_NONE;
    }
    else if (reason != MODE_CODE_REASON_NONE)
        currentFlyModeReason = reason;
    currentFlyMode = cMode;
    return true;
}
// 设置挂起的模式切换原因 (M32主循环使用)
void setPendingFlyModeReason(uint16_t reason)
{
    pendingFlyModeReason = reason;
}
AFunc getCurrentFlyMode()
{
    return currentFlyMode;
}
uint16_t getCurrentFlyModeReason()
{
    return currentFlyModeReason;
}
// 判断是否有GPS固定解 (任意GPS传感器fixtype==6)
bool isGPSFixedSolution()
{
    Position_Sensor gps_sensor;
    for (uint8_t i = default_gps1_sensor_index; i < default_gps1_sensor_index + default_max_gps_sensors; ++i)
    {
        if (GetPositionSensor(i, &gps_sensor) && gps_sensor.inf.addition_inf[1] == 6)
            return true;
    }
    return false;
}

#define MAX_MODES_COUNT 80
Mode_Base *modes[MAX_MODES_COUNT] = {0};
static QueueHandle_t message_queue = xQueueCreate(20, sizeof(ModeMsg));
bool SendMsgToMode(ModeMsg msg, double TIMEOUT)
{
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xQueueSend(message_queue, &msg, TIMEOUT_Ticks) == pdTRUE)
        return true;
    else
        return false;
}
bool ModeReceiveMsg(ModeMsg *msg, double TIMEOUT)
{
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xQueueReceive(message_queue, msg, TIMEOUT_Ticks) == pdTRUE)
        return true;
    else
        return false;
}
void ModeMsgAck(const ModeMsg &msg, bool accepted)
{
    uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
    const Port *port = get_CommuPort(port_index);
    if ((msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK && port && port->write)
    {
        mavlink_message_t msg_sd;
        if (mavlink_lock_chan(port_index, 0.01))
        {
            mavlink_msg_command_ack_pack_chan(get_CommulinkSysId(), get_CommulinkCompId(), port_index, &msg_sd, msg.cmd, accepted ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED, 100, 0, msg.sd_sysid,
                                              msg.sd_compid);
            mavlink_send_msg(port, &msg_sd);
            mavlink_unlock_chan(port_index);
        }
    }
}

// changeMode 同步调用 main_func，原消息在调用期间有效；仅 Modes 任务使用
static const ModeMsg *modeEntryMsg = nullptr;
void ModeEntryAck(bool accepted)
{
    if (modeEntryMsg)
    {
        const ModeMsg *msg = modeEntryMsg;
        modeEntryMsg = nullptr;
        ModeMsgAck(*msg, accepted);
    }
}

static bool changeMode(uint16_t mode_index, void *param1, uint32_t param2, ModeResult *result, bool *msg_available, bool *msg_handled, ModeMsg *msg)
{
    const bool mode_valid = mode_index < MAX_MODES_COUNT && modes[mode_index] != 0;
    if (*msg_available)
    {
        modeEntryMsg = msg;
        // 位置控制模式的指令回执延后到入口检查和控制器启用完成
        if (!(mode_valid && mode_index == 32 && *msg_handled))
            ModeEntryAck(mode_valid && *msg_handled);
        *msg_available = false;
    }

    if (mode_valid)
    {
        xQueueReset(message_queue);
        sendLedSignal(LEDSignal_Start1);
        if (result != 0)
            *result = modes[mode_index]->main_func(param1, param2);
        else
            modes[mode_index]->main_func(param1, param2);
        // 入口提前失败时补发拒绝；成功入口已经清空待回复消息
        ModeEntryAck(false);
        xQueueReset(message_queue);
        setLedMode(LEDMode_Normal1);
        return true;
    }
    return false;
}

static bool runTakeoffAltCheck(double takeoffAlt, bool isRelative, bool sendFailInfo = true)
{
    // Invalid takeoff altitude - let the takeoff handler reject it
    if (!isvalid(takeoffAlt))
        return true;

    // Read avoidance/fence parameters
    AvoidanceCfg avCfg;
    ReadParamGroup("Avoidance", (uint64_t *) &avCfg, 0, 0);

    // Only check when fence max altitude is enabled
    if (((avCfg.fenceEnable[0] & FenceEnable_SplFenceFlag) || (avCfg.fenceEnable[0] & FenceEnable_CpxFenceFlag)) && avCfg.fenceMaxAlt[0] > 0.5)
    {
        // Fence max altitude in cm (relative to home)
        double fenceMaxAltCm = (double) avCfg.fenceMaxAlt[0] * 100.0;

        // Home position Z (local frame)
        double homeZ = 0;
        getHomeLocalZ(&homeZ, 0, 0.01);

        // Calculate target height above home
        double targetHeightAboveHome;
        if (isRelative)
        { // Relative takeoff: climb takeoffAlt above current position
            vector3<double> pos;
            get_Position_Ctrl(&pos);
            targetHeightAboveHome = (pos.z - homeZ) + takeoffAlt;
        }
        else
        { // Global (AMSL) takeoff: convert global altitude to local Z
            PosSensorHealthInf1 global_inf;
            if (!get_OptimalGlobal_Z(&global_inf))
            { // No global positioning sensor, cannot convert - skip check
                // (preflight check already rejects takeoff without positioning)
                return true;
            }
            double targetLocalZ = takeoffAlt - global_inf.HOffset;
            targetHeightAboveHome = targetLocalZ - homeZ;
        }

        if (targetHeightAboveHome > fenceMaxAltCm)
        { // Takeoff altitude exceeds fence max altitude
            if (sendFailInfo)
            {
                sendLedSignal(LEDSignal_Err1);
                sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to takeoff! Altitude exceeds fence!");
            }
            return false;
        }
    }

    return true;
}

static bool runPreflightCheck(const ModeFuncCfg &MFunc_cfg, bool sendFailInfo = true, bool requirePosition = false)
{
    /*电压检测*/
    BatteryInfo batInf;
    getCurrentBatteryInfo(&batInf);
    // 电压大于或小于标准电压的30%不允许解锁
    if (((MFunc_cfg.configs[0] & MCfg_CanUnlockVoltageAbnormal_Bit) == 0) && batInf.stVolt > 7 && batInf.totalVoltRawFilted > 7 &&
        (batInf.totalVoltFilted > batInf.stVolt * 1.3f || batInf.totalVoltRawFilted < batInf.stVolt * 0.7f))
    { // 电压异常
        if (sendFailInfo)
        { // 发送异常信息
            sendLedSignal(LEDSignal_Err1);
            sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! Check battery voltage!");
        }
        return false;
    }
    /*电压检测*/

    /*定位异常*/
    float msHealthXY = get_MSHealthXY();
    if (((MFunc_cfg.configs[0] & MCfg_CanUnlockPositioningAbnormal_Bit) == 0) && msHealthXY > 0 && msHealthXY < 25)
    { // 定位异常
        if (sendFailInfo)
        { // 发送异常信息
            sendLedSignal(LEDSignal_Err1);
            sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! Positioning abnormal!");
        }
        return false;
    }
    /*定位异常*/

    /*无位置信息(水平+高度)*/
    if (requirePosition && (get_Position_MSStatus() != MS_Ready || get_Altitude_MSStatus() != MS_Ready))
    { // 自动任务模式需要位置信息(定点模式同款: 水平+高度)
        if (sendFailInfo)
        { // 发送异常信息
            sendLedSignal(LEDSignal_Err1);
            for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
            {
                const Port *port = get_CommuPort(i);
                if (port && port->write)
                {
                    mavlink_message_t msg_sd;
                    if (mavlink_lock_chan(i, 0.01))
                    {
                        mavlink_msg_statustext_pack_chan(get_CommulinkSysId(),  // system id
                                                         get_CommulinkCompId(), // component id
                                                         i,                     // chan
                                                         &msg_sd, MAV_SEVERITY_ERROR, "Refused to unlock! No position information!", 0, 0);
                        mavlink_send_msg(port, &msg_sd, 0, 0.01);
                        mavlink_unlock_chan(i);
                    }
                }
            }
        }
        return false;
    }
    /*无位置信息*/
    /*固定解检查*/
    if ((MFunc_cfg.configs[0] & MCfg_NoUnlockWithouFixed_Bit) && !isGPSFixedSolution())
    { // 无固定解不允许解锁起飞
        if (sendFailInfo)
        { // 发送异常信息
            sendLedSignal(LEDSignal_Err1);
            sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! No GPS fixed solution!");
        }
        return false;
    }
    /*固定解检查*/

    /*航向异常*/
    bool daoAbnormal = false;
    DAO_Sensor daoSensor;
    if ((GetDAOSensor(0, &daoSensor) || GetDAOSensor(1, &daoSensor)) && isDAOSensorFixed() == false)
        daoAbnormal = true;
    bool magDiff = false;
    double iniMagAng = -10000000;
    double magErrMin = 0, magErrMax = 0;
    for (uint8_t i = 0; i < IMU_Sensors_Count; ++i)
    {
        vector3<double> mag;
        bool magSlam = false;
        if (get_MagEnuFilted(i, &mag, &magSlam) && magSlam == false)
        {
            if (iniMagAng < -10000)
            {
                iniMagAng = atan2(mag.x, mag.y);
            }
            else
            {
                double ang = atan2(mag.x, mag.y);
                double angErr = ang - iniMagAng;
                if (angErr > Pi)
                    angErr -= 2 * Pi;
                else if (angErr < -Pi)
                    angErr += 2 * Pi;

                if (angErr < magErrMin)
                    magErrMin = angErr;
                else if (angErr > magErrMax)
                    magErrMax = angErr;
            }
        }
    }
    if (magErrMax - magErrMin > degree2rad(50.0))
        magDiff = true;
    if ((MFunc_cfg.configs[0] & MCfg_NoUnlockHeadingAbnormal_Bit) && (get_YawHealthEst() < 0 || daoAbnormal || magDiff))
    {
        if (sendFailInfo)
        { // 发送异常信息
            sendLedSignal(LEDSignal_Err1);
            sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! Heading abnormal!");
        }
        return false;
    }
    /*航向异常*/

    /*SD容量不足*/
    if ((MFunc_cfg.configs[0] & MCfg_NoUnlockSdInsufficient_Bit) && getSdFreeSizeGB() < 1)
    {
        if (sendFailInfo)
        { // 发送异常信息
            sendLedSignal(LEDSignal_Err1);
            sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! Insufficient space on sd card!");
        }
        return false;
    }
    /*SD容量不足*/

    /*无定位*/
    if ((MFunc_cfg.configs[0] & MCfg_NoUnlockNoPositioning_Bit) && get_Position_MSStatus() != MS_Ready)
    {
        if (sendFailInfo)
        { // 发送异常信息
            sendLedSignal(LEDSignal_Err1);
            sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! No positioning!");
        }
        return false;
    }
    /*无定位*/

    /*检测围栏*/
    // 获取避参数
    AvoidanceCfg avCfg;
    ReadParamGroup("Avoidance", (uint64_t *) &avCfg, 0, 0);
    if (avCfg.fenceEnable[0] & FenceEnable_CpxFenceFlag)
    { // 检测复杂围栏
        vector3<double> pos;
        get_Position_Ctrl(&pos);
        vector3<double> tVelVec;
        bool res = is_insideFence(pos, tVelVec, 0);
        if (!res)
        {
            if (sendFailInfo)
            { // 发送异常信息
                sendLedSignal(LEDSignal_Err1);
                sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! Outside fence!");
            }
            return false;
        }
    }
    /*检测复杂围栏*/

    // 禁飞区围栏使能时, 需向地面站请求解锁/起飞授权
    // 发送MAV_CMD_ARM_AUTHORIZATION_REQUEST后, 5秒内收到允许的COMMAND_ACK才可解锁/起飞
    if (avCfg.fenceEnable[0] & FenceEnable_LimitFenceFlag)
    {
        sendArmAuthorizationRequest(2.0);
        if (!waitArmAuthorizationResult(2.0))
        {
            if (sendFailInfo)
            { // 发送异常信息
                sendLedSignal(LEDSignal_Err1);
                sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! Arm authorization denied!");
            }
            return false;
        }
    }
    /*检测禁飞区围栏*/
    /*检测围栏*/

    /*检测激活*/
    uint8_t regStatus[8] = {0};
    if (ReadParam("ID_regStatus", 0, 0, (uint64_t *) regStatus, 0) == PR_OK)
    { // 获取是否已激活. [0: 未激活(生产模式)], [1: 未激活], [2: 已激活], [3: 已注销]
        if (regStatus[0] == 1 || regStatus[0] == 3)
        {
            sendLedSignal(LEDSignal_Err1);
            sendStatusText_toAll(MAV_SEVERITY_ERROR, "Refused to unlock! Vehicle not actived!");
            return false;
        }
    }
    /*检测激活*/

    return true;
}

static void Modes_Server(void *pvParameters)
{
    const double h = 0.02;
    const double freq = 1.0 / h;
    // 初始化Aux处理
    init_process_AuxFuncs();

    // 等待驱动初始化完成
    while (getInitializationCompleted() == false)
    {
        Receiver rc;
        getReceiver(&rc);
        // 处理Aux通道
        process_AuxFuncs(&rc, h);
        os_delay(h);
    }
    setLedManualCtrl(0, 0, 30, true, 800);
    os_delay(0.2);
    setLedMode(LEDMode_Processing1);

    // 等待姿态解算系统准备完成
    while (get_Attitude_MSStatus() != MS_Ready)
    {
        Receiver rc;
        getReceiver(&rc);
        // 处理Aux通道
        process_AuxFuncs(&rc, h);
        os_delay(h);
    }
    setLedManualCtrl(0, 0, 30, true, 1000);
    os_delay(0.2);
    setLedMode(LEDMode_Processing1);

    // 等待位置解算系统准备完成
    while (get_Altitude_MSStatus() != MS_Ready)
    {
        Receiver rc;
        getReceiver(&rc);
        // 处理Aux通道
        process_AuxFuncs(&rc, h);
        os_delay(h);
    }
    setLedManualCtrl(0, 0, 30, true, 1200);
    os_delay(0.2);
    setLedMode(LEDMode_Processing1);
    sendLedSignal(LEDSignal_Start2);

    volatile uint32_t *ARM_CM_DHCSR = ((volatile uint32_t *) 0xE000EDF0UL); /* Cortex M CoreDebug->DHCSR */
    if (((*ARM_CM_DHCSR) & 1UL) == 0)
    { // 不在仿真模式
      //  关闭调试功能
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = 0;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    // 进入地面模式
    xQueueReset(message_queue);
    setLedMode(LEDMode_Normal1);
    uint16_t pre_enter_mode_counter = 0;
    uint8_t last_pre_enter_mode = 0;
    bool rcUnlockErrorFlag = false;

    // 状态
    AFunc MF_mode = AFunc_PosHold;
    uint8_t btn_zones[4];
    btn_zones[0] = btn_zones[1] = btn_zones[2] = btn_zones[3] = 255;
    uint8_t ModeButtonZone = 255;

#define reset_States                                                                                                                                                                                   \
    {                                                                                                                                                                                                  \
        last_pre_enter_mode = 0;                                                                                                                                                                       \
        btn_zones[0] = btn_zones[1] = btn_zones[2] = btn_zones[3] = 255;                                                                                                                               \
        ModeButtonZone = 255;                                                                                                                                                                          \
        MF_mode = getCurrentFlyMode();                                                                                                                                                                 \
    }

    while (1)
    {
        os_delay(h);

        // 关闭控制器
        Attitude_Control_Disable();
        set_mav_state(MAV_STATE_STANDBY);
        // 关闭水泵切换到手动控制
        setPump1(PumpOffManualPercent);

        // 保存断点续飞信息
        if (RestoreWpInf_needStore)
        {
            UpdateParamGroup("RestoreWpInf", (uint64_t *) &restoreWpInf, 0, sizeof(restoreWpInf) / 8);
            RestoreWpInf_needStore = false;
        }

        // 保存飞行统计信息
        if (FlightStats_needStore)
        {
            UpdateParamGroup("Init", (uint64_t *) &flightStats, FLIGHTSTATS_PARAM_OFFSET, FLIGHTSTATS_PARAM_COUNT);
            FlightStats_needStore = false;
        }

        // 获取接收机
        Receiver rc;
        getReceiver(&rc, 0, 0.02);

        if (rc.available)
        { // 接收机可用更新模式按钮状态
            uint8_t new_ModeButtonZone;
            if (rc.available_channels >= 5)
                new_ModeButtonZone = get_RcButtonZone(rc.data[4], ModeButtonZone);
            else
                new_ModeButtonZone = 255;
            if (new_ModeButtonZone <= 5 && new_ModeButtonZone != ModeButtonZone)
            { // 模式按钮改变重置锁定状态
            }
            ModeButtonZone = new_ModeButtonZone;
        }
        else
        { // 接收机不可用重置遥控状态
            ModeButtonZone = 255;
        }

        // 处理Aux通道
        process_AuxFuncs(&rc, h);

        // 获取消息
        bool msg_available;
        bool msg_handled = false;
        ModeMsg msg;
        msg_available = ModeReceiveMsg(&msg, 0);

        // 读取模式配置
        ModeFuncCfg MFunc_cfg;
        ReadParamGroup("MFunc", (uint64_t *) &MFunc_cfg, 0);

        // 获取定位状态
        if (get_Position_MSStatus() == MS_Ready)
            setLedMode(LEDMode_Normal2);
        else
            setLedMode(LEDMode_Normal1);

        /*模式回传显示*/
        uint8_t p_mode = 0;
        if (ModeButtonZone <= 5)
            p_mode = MFunc_cfg.Bt1PAFunc[ModeButtonZone][0];
        else
            p_mode = MFunc_cfg.Bt1PAFunc[5][0];

        if (p_mode < MAX_MODES_COUNT && modes[p_mode] != 0)
        {
            modes[p_mode]->get_MavlinkMode(MFunc_cfg, rc, btn_zones, &MF_mode);
            setCurrentFlyMode(MF_mode);
        }
        /*模式回传显示*/

        if (rc.available)
        {
            uint8_t pre_enter_mode = 0;

            if ((rc.data[0] < 10.0f) && (rc.data[1] < 10.0f) && (rc.data[2] < 10.0f) && (rc.data[3] < 10.0f))
                pre_enter_mode = 12; // 加速度校准
            else if ((rc.data[0] < 10.0f) && (rc.data[1] < 10.0f) && (rc.data[2] > 90.0f) && (rc.data[3] < 10.0f))
                pre_enter_mode = 13; // 磁力计校准
            else if ((rc.data[0] < 10.0f) && (rc.data[1] > 90.0f) && (rc.data[2] > 45.0f) && (rc.data[2] < 55.0f) && (rc.data[3] > 90.0f))
                pre_enter_mode = 11; // 温度系数校准
            // 一键解锁(功能码1003)
            bool switchBtUnlock = false;
            for (uint8_t i = 0; i < 3; ++i)
            {
                if (MFunc_cfg.Bt234Func[i][0] == 1003 && rc.available_channels >= 6 + i && rc.data[5 + i] < 10.0f && rc.data[0] < 2.0f)
                {
                    switchBtUnlock = true;
                    break;
                }
            }

            if ((rc.data[0] < 10.0f) && (rc.data[1] > 90.0f) && (rc.data[2] < 10.0f) && (rc.data[3] < 10.0f) || switchBtUnlock)
            { // 解锁动作
                if (rcUnlockErrorFlag == false)
                {
                    if (runPreflightCheck(MFunc_cfg, true, is_AFunc_auto(MF_mode)))
                    {
                        if (ModeButtonZone <= 5)
                            pre_enter_mode = MFunc_cfg.Bt1PAFunc[ModeButtonZone][0];
                        else
                            pre_enter_mode = MFunc_cfg.Bt1PAFunc[5][0];
                    }
                }
                else
                    pre_enter_mode = 0;
            }
            else
                rcUnlockErrorFlag = false;

            // 计数进入模式
            if (pre_enter_mode == 0 || pre_enter_mode != last_pre_enter_mode)
                pre_enter_mode_counter = 0;
            else
            {
                if (++pre_enter_mode_counter >= 50)
                {
                    if (pre_enter_mode < MAX_MODES_COUNT && modes[pre_enter_mode] != 0)
                    {
                        if (runPreflightCheck(MFunc_cfg))
                        {
                            changeMode(pre_enter_mode, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                            reset_States;
                            continue;
                        }
                        else
                            rcUnlockErrorFlag = true;
                    }
                }
            }
            last_pre_enter_mode = pre_enter_mode;
        }

        // 处理消息
        if (msg_available)
        {
            switch (msg.cmd)
            {
                case 241:
                { // 进入校准
                    if (msg.params[0] == 1)
                    { // 陀螺校准
                        changeMode(11, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                        reset_States;
                    }
                    else if (msg.params[1] == 1)
                    { // 罗盘校准
                        changeMode(13, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                        reset_States;
                    }
                    else if (msg.params[4] == 1)
                    { // 加速度校准
                        changeMode(12, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                        reset_States;
                    }
                    else if (msg.params[4] == 2)
                    { // 水平校准
                        changeMode(14, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                        reset_States;
                    }
                    else if (msg.params[3] == 1)
                    { // 遥控器校准
                        changeMode(10, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                        reset_States;
                    }
                    break;
                }
                case 176:
                { // do set mode
                    if (msg.params[0] == 0)
                    {
                        bool res = true;
                        if (msg.params[1] >= 30)
                            res = runPreflightCheck(MFunc_cfg);
                        if (res)
                        {
                            changeMode(msg.params[1], 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                            reset_States;
                        }
                    }
                    else if ((int) msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                    { // mavlink定义模式
                        uint32_t main_mode = msg.params[1];
                        uint32_t sub_mode = msg.params[2];
                        if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_MISSION)
                        { // 自动模式
                            msg_handled = true;
                            MF_mode = AFunc_Mission;
                            if ((int) msg.params[0] & MAV_MODE_FLAG_SAFETY_ARMED)
                            {
                                if (runPreflightCheck(MFunc_cfg, true, true))
                                {
                                    changeMode(32, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                                    reset_States;
                                }
                            }
                        }
                        else if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == AFunc_AutoMission)
                        { // 预选二次开发自动任务模式(解锁后直接运行任务)
                            msg_handled = true;
                            MF_mode = AFunc_AutoMission;
                            if ((int) msg.params[0] & MAV_MODE_FLAG_SAFETY_ARMED)
                            {
                                if (runPreflightCheck(MFunc_cfg, true, true))
                                {
                                    changeMode(32, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                                    reset_States;
                                }
                            }
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode == PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL)
                        { // 指令进入手动
                            msg_handled = true;
                            MF_mode = AFunc_PosHold;
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL)
                        { // 指令进入手动
                            msg_handled = true;
                            MF_mode = AFunc_Manual;
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED)
                        { // 指令进入增稳
                            msg_handled = true;
                            MF_mode = AFunc_Stabilize;
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_RATTITUDE)
                        {
                            msg_handled = true;
                            MF_mode = AFunc_Rattitude;
                        }
                        // else if (main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode == PX4_CUSTOM_SUB_MODE_POSCTL_Headless)
                        // {
                        //     msg_handled = true;
                        //     MF_mode = AFunc_PosHoldNH;
                        // }
                        // else if (main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode == PX4_CUSTOM_SUB_MODE_POSCTL_HeadlessAv)
                        // {
                        //     msg_handled = true;
                        //     MF_mode = AFunc_PosHoldNHAv;
                        // }
                        // else if (main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode == PX4_CUSTOM_SUB_MODE_POSCTL_Circle)
                        //{
                        //    msg_handled = true;
                        //    MF_mode = AFunc_Circle;
                        //}
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL)
                        { // 定高模式
                            msg_handled = true;
                            MF_mode = AFunc_AltHold;
                        }
                        if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_RTL)
                        { // 返航
                            msg_handled = true;
                            MF_mode = AFunc_RTL;
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                        { // OFFBOARD
                            msg_handled = true;
                            MF_mode = AFunc_Offboard;
                        }
                        else if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
                        { // 跟踪
                            msg_handled = true;
                            MF_mode = AFunc_Loiter;
                        }
                    }
                    break;
                }

                case 20:
                { // 返航到指定地点
                    if (msg.params[0] == 1)
                    {
                        if (isvalid(msg.params[4]) && isvalid(msg.params[5]) && msg.params[4] >= -90 && msg.params[4] <= 90 && msg.params[5] >= -180 && msg.params[5] <= 180)
                        {
                            if (runPreflightCheck(MFunc_cfg))
                            {
                                msg_handled = true;
                                MF_mode = AFunc_RTL2Point;
                                double rtl_params[4];
                                rtl_params[0] = msg.params[4];
                                rtl_params[1] = msg.params[5];
                                rtl_params[2] = msg.params[6];
                                rtl_params[3] = msg.params[1];
                                changeMode(32, rtl_params, MF_mode, 0, &msg_available, &msg_handled, &msg);
                                reset_States;
                            }
                        }
                    }
                    break;
                }
                case 22:
                { // takeoff起飞
                    if (runPreflightCheck(MFunc_cfg))
                    {
                        if (runTakeoffAltCheck(msg.params[6] * 100, false))
                        {
                            msg_handled = true;
                            MF_mode = AFunc_TakeOff;
                            float takeoff_params[2];
                            takeoff_params[0] = 0;
                            takeoff_params[1] = msg.params[6] * 100;
                            changeMode(32, takeoff_params, MF_mode, 0, &msg_available, &msg_handled, &msg);
                            reset_States;
                        }
                    }
                    break;
                }

                case 24:
                { // takeoff local起飞
                    if (runPreflightCheck(MFunc_cfg))
                    {
                        if (runTakeoffAltCheck(msg.params[6] * 100, true))
                        {
                            msg_handled = true;
                            MF_mode = AFunc_TakeOff;
                            float takeoff_params[2];
                            takeoff_params[0] = 1;
                            takeoff_params[1] = msg.params[6] * 100;
                            changeMode(32, takeoff_params, MF_mode, 0, &msg_available, &msg_handled, &msg);
                            reset_States;
                        }
                    }
                    break;
                }

                case MAV_CMD_COMPONENT_ARM_DISARM:
                { // 解锁
                    if (msg.params[0] == 1)
                    {
                        if (runPreflightCheck(MFunc_cfg))
                        {
                            msg_handled = true;
                            changeMode(32, 0, MF_mode, 0, &msg_available, &msg_handled, &msg);
                            reset_States;
                        }
                    }
                    break;
                }

                case 248:
                { // 上云API功能开关 (MAV_CMD_SET_ACFLY_FUNCTION)
                    switch ((int) msg.params[0])
                    {
                        case ACFLY_FUNCTION_HEADLESS_MODE:
                        { // 无头
                            if ((int) msg.params[1] == 1)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_HeadlessEna);
                                msg_handled = true;
                            }
                            else if ((int) msg.params[1] == 0)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_HeadlessEna);
                                msg_handled = true;
                            }
                            break;
                        }
                        case ACFLY_FUNCTION_OBSTACLE_AVOIDANCE:
                        { // 避障
                            if ((int) msg.params[1] == 1)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_AvoidanceEna);
                                msg_handled = true;
                            }
                            else if ((int) msg.params[1] == 0)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_AvoidanceEna);
                                msg_handled = true;
                            }
                            break;
                        }
                        case ACFLY_FUNCTION_TERRAIN_FOLLOWING:
                        { // 仿地
                            if ((int) msg.params[1] == 1)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_TerrainFollowingEna);
                                msg_handled = true;
                            }
                            else if ((int) msg.params[1] == 0)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_TerrainFollowingEna);
                                msg_handled = true;
                            }
                            break;
                        }
                        default:
                            break;
                    }
                    break;
                }
            }
        }

        /*返回消息处理结果*/
        if (msg_available)
        {
            uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
            const Port *port = get_CommuPort(port_index);
            if ((msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK && port && port->write)
            {
                mavlink_message_t msg_sd;
                if (mavlink_lock_chan(port_index, 0.01))
                {
                    mavlink_msg_command_ack_pack_chan(get_CommulinkSysId(),  // system id
                                                      get_CommulinkCompId(), // component id
                                                      port_index, &msg_sd,
                                                      msg.cmd,                                                    // command
                                                      msg_handled == 1 ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED, // result
                                                      0,                                                          // progress
                                                      0,                                                          // param2
                                                      msg.sd_sysid,                                               // target system
                                                      msg.sd_compid                                               // target component
                    );
                    mavlink_send_msg(port, &msg_sd);
                    mavlink_unlock_chan(port_index);
                }
            }
        }
        /*返回消息处理结果*/
    }
}

void ModeRegister(Mode_Base *mode, uint8_t id)
{
    if (id < MAX_MODES_COUNT && modes[id] == 0)
        modes[id] = mode;
}

#define Modes_StackSize 4200
Static_SRAM3Buf __attribute__((aligned(8))) StackType_t Modes_Stack[Modes_StackSize];
Static_SRAM1Buf StaticTask_t Modes_TaskBuffer;
void init_Modes()
{
    // 注册模式
    new M10_RCCalib();
    new M11_TempCalib();
    new M12_AccCalib();
    new M13_MagCalib();
    new M14_HorizonCalib();

    new M30_AttCtrl();
    new M32_PosCtrl();
    // new M35_Auto1();

    // 注册参数
    ModeFuncCfg initial_cfg;
    // 按钮1解锁前功能（模式序号）
    initial_cfg.Bt1PAFunc[0][0] = 32;
    initial_cfg.Bt1PAFunc[1][0] = 32;
    initial_cfg.Bt1PAFunc[2][0] = 32;
    initial_cfg.Bt1PAFunc[3][0] = 32;
    initial_cfg.Bt1PAFunc[4][0] = 32;
    initial_cfg.Bt1PAFunc[5][0] = 32;
    // 按钮1解锁后功能
    initial_cfg.Bt1AFunc[0][0] = 1;
    initial_cfg.Bt1AFunc[1][0] = 1;
    initial_cfg.Bt1AFunc[2][0] = 22;
    initial_cfg.Bt1AFunc[3][0] = 22;
    initial_cfg.Bt1AFunc[4][0] = 2;
    initial_cfg.Bt1AFunc[5][0] = 2;
    // 按钮2-4功能
    initial_cfg.Bt234Func[0][0] = 0;
    initial_cfg.Bt234Func[1][0] = 0;
    initial_cfg.Bt234Func[2][0] = 0;
    // 安全按钮
    initial_cfg.SafeBt[0] = 0;
    // 中位死区
    initial_cfg.NeutralZone[0] = 5.0;
    // 位置速度响应曲线系数
    initial_cfg.PosVelAlpha[0] = 1.6;
    // 姿态响应曲线系数
    initial_cfg.RolRateE[0] = 1.0;
    initial_cfg.PitRateE[0] = 1.0;
    initial_cfg.YawRateE[0] = 1.0;
    // 半自稳模式摇杆区域
    initial_cfg.ratitudeZone[0] = 5.0;
    // 失控返航时间
    initial_cfg.nRcT[0] = 2.0;
    // 配置选项开关
    initial_cfg.configs[0] = MCfg_NoUnlockHeadingAbnormal_Bit;

    MAV_PARAM_TYPE param_types[] = {
        // 按钮1解锁前功能（模式序号）
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        // 按钮1解锁后功能
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        MAV_PARAM_TYPE_UINT8,
        // 按钮2-4功能
        MAV_PARAM_TYPE_UINT32,
        MAV_PARAM_TYPE_UINT32,
        MAV_PARAM_TYPE_UINT32,
        // 安全按钮
        MAV_PARAM_TYPE_UINT8,
        // 中位死区
        MAV_PARAM_TYPE_REAL32,
        // 位置速度响应曲线系数
        MAV_PARAM_TYPE_REAL32,
        // 姿态响应曲线系数
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        // 半自稳模式摇杆区域
        MAV_PARAM_TYPE_REAL32,
        // 失控返航时间
        MAV_PARAM_TYPE_REAL32,
        // 配置选项开关
        MAV_PARAM_TYPE_UINT32,
    };
    SName param_names[] = {// 按钮1解锁前功能（模式序号）
                           "MFunc_Bt1PAF1", "MFunc_Bt1PAF2", "MFunc_Bt1PAF3", "MFunc_Bt1PAF4", "MFunc_Bt1PAF5", "MFunc_Bt1PAF6",
                           // 按钮1解锁后功能
                           "MFunc_Bt1AF1", "MFunc_Bt1AF2", "MFunc_Bt1AF3", "MFunc_Bt1AF4", "MFunc_Bt1AF5", "MFunc_Bt1AF6",
                           // 按钮2-4功能
                           "MFunc_Bt2Func", "MFunc_Bt3Func", "MFunc_Bt4Func",
                           // 安全按钮
                           "MFunc_SafeBt",
                           // 中位死区
                           "MFunc_NeutralZ",
                           // 位置速度响应曲线系数
                           "MFunc_PVAlpha",
                           // 姿态响应曲线系数
                           "MFunc_RolRateE", "MFunc_PitRateE", "MFunc_YawRateE",
                           // 半自稳模式摇杆区域
                           "MFunc_RatitudeZ",
                           // 失控返航时间
                           "MFunc_nRcT",
                           // 配置选项开关
                           "MFunc_Configs"};
    InitialParametersInit((uint64_t *) &initial_cfg, param_types, param_names, sizeof(ModeFuncCfg) / 8);
    ParamGroupRegister("MFunc", 3, sizeof(ModeFuncCfg) / 8, param_types, param_names, (uint64_t *) &initial_cfg);

    // 注册当前航点信息
    RestoreWpInf initial_RestoreWpInf;
    reset_RestoreWpInf(initial_RestoreWpInf);
    ParamGroupRegister("RestoreWpInf", 1, sizeof(RestoreWpInf) / 8, 0, 0, (uint64_t *) &initial_RestoreWpInf);

    // 注册精准降落
    PrecLandInf initial_PrecLandInf;
    initial_PrecLandInf.ctrlKp[0] = 8;
    initial_PrecLandInf.ctrlKi[0] = 0.3;
    initial_PrecLandInf.maxDynamicErrRaw[0] = 20;
    initial_PrecLandInf.maxDynamicErrCtrl[0] = 12;
    initial_PrecLandInf.maxStaticErrRaw[0] = 15;
    initial_PrecLandInf.maxStaticErrCtrl[0] = 10;
    initial_PrecLandInf.pitch_offset[0] = 0;
    initial_PrecLandInf.roll_offset[0] = 0;
    initial_PrecLandInf.sensorInfs[0].delay[0] = 0.1;
    initial_PrecLandInf.sensorInfs[0].ofsType[0] = 0;
    initial_PrecLandInf.sensorInfs[0].ofs_x[0] = 0;
    initial_PrecLandInf.sensorInfs[0].ofs_y[0] = 0;
    initial_PrecLandInf.sensorInfs[0].ofs_z[0] = 0;
    initial_PrecLandInf.sensorInfs[0].pofs_x[0] = 0;
    initial_PrecLandInf.sensorInfs[0].pofs_y[0] = 0;
    initial_PrecLandInf.sensorInfs[0].pofs_z[0] = 0;
    initial_PrecLandInf.sensorInfs[1].delay[0] = 0.1;
    initial_PrecLandInf.sensorInfs[1].ofsType[0] = 0;
    initial_PrecLandInf.sensorInfs[1].ofs_x[0] = 0;
    initial_PrecLandInf.sensorInfs[1].ofs_y[0] = 0;
    initial_PrecLandInf.sensorInfs[1].ofs_z[0] = 0;
    initial_PrecLandInf.sensorInfs[1].pofs_x[0] = 0;
    initial_PrecLandInf.sensorInfs[1].pofs_y[0] = 0;
    initial_PrecLandInf.sensorInfs[1].pofs_z[0] = 0;
    initial_PrecLandInf.sensorInfs[2].delay[0] = 0.1;
    initial_PrecLandInf.sensorInfs[2].ofsType[0] = 0;
    initial_PrecLandInf.sensorInfs[2].ofs_x[0] = 0;
    initial_PrecLandInf.sensorInfs[2].ofs_y[0] = 0;
    initial_PrecLandInf.sensorInfs[2].ofs_z[0] = 0;
    initial_PrecLandInf.sensorInfs[2].pofs_x[0] = 0;
    initial_PrecLandInf.sensorInfs[2].pofs_y[0] = 0;
    initial_PrecLandInf.sensorInfs[2].pofs_z[0] = 0;
    MAV_PARAM_TYPE precLand_param_types[] = {
        // 控制参数
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,

        // 最大动态误差
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        // 最大静态误差
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        // 安装倾角补偿
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,

        //  传感器1延迟
        MAV_PARAM_TYPE_REAL32,
        // 传感器1降落偏移坐标系
        MAV_PARAM_TYPE_UINT32,
        // 传感器1降落偏移
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        // 传感器1传感器偏移
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,

        //  传感器2延迟
        MAV_PARAM_TYPE_REAL32,
        // 传感器2降落偏移坐标系
        MAV_PARAM_TYPE_UINT32,
        // 传感器2降落偏移
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        // 传感器2传感器偏移
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,

        //  传感器3延迟
        MAV_PARAM_TYPE_REAL32,
        // 传感器3降落偏移坐标系
        MAV_PARAM_TYPE_UINT32,
        // 传感器3降落偏移
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        // 传感器3传感器偏移
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
    };
    SName precLand_param_names[] = {
        // 控制参数
        "PrecLand_ctrlKp",
        "PrecLand_ctrlKi",

        // 最大动态误差
        "PrecLand_maxDER",
        "PrecLand_maxDEC",
        // 最大静态误差
        "PrecLand_maxSER",
        "PrecLand_maxSEC",
        // 安装倾角补偿
        "PrecLand_PitOff",
        "PrecLand_RollOff",

        // 传感器1延时
        "PrecLand_S1Delay",
        // 传感器1降落偏移坐标系
        "PrecLand_S1OfsTp",
        // 传感器1降落偏移
        "PrecLand_S1OfsX",
        "PrecLand_S1OfsY",
        "PrecLand_S1OfsZ",
        // 传感器1传感器偏移
        "PrecLand_S1POfsX",
        "PrecLand_S1POfsY",
        "PrecLand_S1POfsZ",

        // 延时
        "PrecLand_S2Delay",
        // 传感器2降落偏移坐标系
        "PrecLand_S2OfsTp",
        // 传感器2降落偏移
        "PrecLand_S2OfsX",
        "PrecLand_S2OfsY",
        "PrecLand_S2OfsZ",
        // 传感器2传感器偏移
        "PrecLand_S2POfsX",
        "PrecLand_S2POfsY",
        "PrecLand_S2POfsZ",

        // 传感器3延时
        "PrecLand_S3Delay",
        // 传感器3降落偏移坐标系
        "PrecLand_S3OfsTp",
        // 传感器3降落偏移
        "PrecLand_S3OfsX",
        "PrecLand_S3OfsY",
        "PrecLand_S3OfsZ",
        // 传感器3传感器偏移
        "PrecLand_S3POfsX",
        "PrecLand_S3POfsY",
        "PrecLand_S3POfsZ",
    };
    InitialParametersInit((uint64_t *) &initial_PrecLandInf, precLand_param_types, precLand_param_names, sizeof(PrecLandInf) / 8);
    ParamGroupRegister("precLand", 2, sizeof(PrecLandInf) / 8, precLand_param_types, precLand_param_names, (uint64_t *) &initial_PrecLandInf);

    // 注册跟随
    FollowInf initial_FollowInf;
    initial_FollowInf.ofsType[0] = 0;
    initial_FollowInf.ofs_x[0] = 0;
    initial_FollowInf.ofs_y[0] = 0;
    initial_FollowInf.ofs_z[0] = 0;
    MAV_PARAM_TYPE followInf_param_types[] = {
        // 跟随偏移坐标系
        MAV_PARAM_TYPE_UINT32,
        // 跟随偏移
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
    };
    SName followInf_param_names[] = {
        // 跟随偏移坐标系
        "Follow_OfsType",
        // 跟随偏移
        "Follow_OfsX",
        "Follow_OfsY",
        "Follow_OfsZ",
    };
    InitialParametersInit((uint64_t *) &initial_FollowInf, followInf_param_types, followInf_param_names, sizeof(FollowInf) / 8);
    ParamGroupRegister("followInf", 1, sizeof(FollowInf) / 8, followInf_param_types, followInf_param_names, (uint64_t *) &initial_FollowInf);

    init_AuxFuncs();

    // xTaskCreate( Modes_Server, "Modes", 4200, NULL, SysPriority_UserTask, NULL);
    xTaskCreateStatic(Modes_Server, "Modes", Modes_StackSize, NULL, SysPriority_UserTask, Modes_Stack, &Modes_TaskBuffer);
}