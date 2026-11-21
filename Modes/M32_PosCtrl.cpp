#include "M32_PosCtrl.hpp"
#include "AC_Math.hpp"
#include "AuxFuncs.hpp"
#include "Avoidance.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "Filters_LP.hpp"
#include "FlightModes.hpp"
#include "FlightModules.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Missions.hpp"
#include "NavCmdProcess.hpp"
#include "Parachute.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "Sensors.hpp"
#include "StorageSystem.hpp"
#include "ctrl_Main.hpp"
#include "drv_PWMOut.hpp"
#include "followTarget.hpp"
#include "vector2.hpp"
#include "vector3.hpp"

M32_PosCtrl::M32_PosCtrl()
    : Mode_Base("PosCtrl", 32)
{
}

void M32_PosCtrl::get_MavlinkMode(ModeFuncCfg cfg, Receiver rc, uint8_t btn_zones[4], AFunc *mode)
{ // 获取飞行模式
    if (rc.available)
    { // 接收机可用更新模式按钮状态
        uint8_t MF_mode = *mode;

        uint8_t new_btn_zones[4];
        for (uint8_t i = 0; i < 4; ++i)
        { // 更新按钮状态
            if (rc.available_channels >= 5 + i)
                new_btn_zones[i] = get_RcButtonZone(rc.data[4 + i], btn_zones[i]);
            else
                new_btn_zones[i] = 255;
        }

        if (new_btn_zones[0] <= 5 && btn_zones[0] <= 5 && new_btn_zones[0] != btn_zones[0])
        { // 模式按钮改变更改模式
            std::unique_ptr<FlightMode_Base> new_req_mode = FlightModeFactory::createMode(cfg.Bt1AFunc[new_btn_zones[0]][0]);
            if (new_req_mode != nullptr)
            {
                MF_mode = cfg.Bt1AFunc[new_btn_zones[0]][0];
            }
        }
        btn_zones[0] = new_btn_zones[0];

        // 检测模式更改按钮
        for (uint8_t i = 0; i < 3; ++i)
        {
            uint32_t btn_func = cfg.Bt234Func[i][0];
            if (btn_func > 0)
            {
                // 检测模式开关更改飞行模式
                if (new_btn_zones[i + 1] <= 5 && btn_zones[i + 1] <= 5 && new_btn_zones[i + 1] != btn_zones[i + 1])
                { // 模式按钮改变更改模式
                    if (btn_func < 1000)
                    { // 切换到指定模式编号
                        std::unique_ptr<FlightMode_Base> new_req_mode = FlightModeFactory::createMode(btn_func);
                        if (new_req_mode != nullptr)
                        {
                            if (MF_mode == new_req_mode->get_modeId())
                            { // 模式一致尝试切换到其他模式
                                if (btn_zones[0] <= 5)
                                {
                                    new_req_mode.reset();
                                    new_req_mode = FlightModeFactory::createMode(cfg.Bt1AFunc[btn_zones[0]][0]);
                                    if (new_req_mode != nullptr)
                                    {
                                        MF_mode = cfg.Bt1AFunc[btn_zones[0]][0];
                                        new_req_mode.reset();
                                    }
                                }
                            }
                            else
                            { // 切换到选定模式
                                MF_mode = btn_func;
                                new_req_mode.reset();
                            }
                        }
                    }
                    else
                    {
                        switch (btn_func)
                        {
                            case 1000:
                            { // 切换无头状态
                                if (GLB_FlightModeFlags & FMF_HeadlessEna)
                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_HeadlessEna);
                                else
                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_HeadlessEna);
                                break;
                            }
                            case 1001:
                            { // 切换避障状态
                                if (GLB_FlightModeFlags & FMF_AvoidanceEna)
                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_AvoidanceEna);
                                else
                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_AvoidanceEna);
                                break;
                            }
                            case 1002:
                            { // 切换仿地状态
                                if (GLB_FlightModeFlags & FMF_TerrainFollowingEna)
                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_TerrainFollowingEna);
                                else
                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_TerrainFollowingEna);
                                break;
                            }
                        }
                    }
                }
                btn_zones[i + 1] = new_btn_zones[i + 1];
            }
            else
            {
                btn_zones[i + 1] = 255;
            }
        }

        /*判断返航*/
        if (MF_mode == 0 && new_btn_zones[0] <= 5)
            MF_mode = cfg.Bt1AFunc[new_btn_zones[0]][0];

        *mode = (AFunc) MF_mode;
    }
    else
    {
        for (uint8_t i = 0; i < 4; ++i)
        {
            btn_zones[i] = 255;
        }
        if (*mode == 0)
            *mode = AFunc_RTL;
    }
}

ModeResult M32_PosCtrl::main_func(void *param1, uint32_t param2)
{
    double freq = 50;
    double h = 1.0 / freq;
    // 按钮消抖时间(秒)，RC短时间跳动脉冲过滤
    const double button_debounce_time = 0.1;
    const uint8_t button_debounce_frames = (uint8_t) (button_debounce_time * freq + 0.5);
    setLedMode(LEDMode_FlyingManualAlt);

    // 读取模式配置
    ModeFuncCfg cfg;
    ReadParamGroup("MFunc", (uint64_t *) &cfg, 0);
    // 获取避障设置
    const AvoidanceCfg *avCfg = getAvCfg();

    // 读取是否已提前启动
    bool att_enabled = false;
    is_Attitude_Control_Enabled(&att_enabled);

    // 模式
    std::unique_ptr<FlightMode_Base> currentFlightMode = nullptr;
    // 模式通用变量
    FlightMode_CommonVars comVars;

    // 初始化无头
    Quaternion quat;
    get_AirframeY_quat(&quat);
    comVars.set_headlessYaw(quat.getYaw());

    // 初始化模式
    if (param2)
    {
        currentFlightMode = FlightModeFactory::createMode(param2);

        // 模式检查
        if (currentFlightMode)
        {
            // 起飞模式特例
            if (currentFlightMode->get_modeId() == AFunc_TakeOff)
            {
                if (param1)
                {
                    float *takeoff_params = (float *) param1;
                    double params[2];
                    params[0] = takeoff_params[0] + 1;
                    params[1] = takeoff_params[1];
                    currentFlightMode->set_params(params, 2);
                }
            }
            // 返航到指定点特例
            if (currentFlightMode->get_modeId() == AFunc_RTL2Point)
            {
                if (param1)
                {
                    double *rtl_params = (double *) param1;
                    // 计算返航高度(相对地面)
                    double req_alt_local;
                    if (get_localZ_fromOptimalGlobal(rtl_params[2] * 100, &req_alt_local))
                    {
                        // 获取起飞点local z
                        // 相对地面高 = local_z - homeZ
                        double homeZ;
                        getHomeLocalZ(&homeZ);
                        req_alt_local -= homeZ;

                        // 创建飞行模式
                        double params[4];
                        params[0] = rtl_params[0];       // lat
                        params[1] = rtl_params[1];       // lon
                        params[2] = req_alt_local;       // alt
                        params[3] = rtl_params[3] * 100; // rtl height
                        currentFlightMode->set_params(params, 4);
                    }
                }
            }

            // 模式检查
            Receiver rc;
            getReceiver(&rc, 0, 0.02);
            FlightModeCheckResult init_fmcr = currentFlightMode->check(rc, GLB_FlightModeFlags, comVars, &cfg, avCfg);
            if (init_fmcr != FMCR_OK)
            {
                if (att_enabled)
                { // 控制器已提前打开(异常路径)：保持原行为，清空模式由主循环安全模式接管
                    currentFlightMode.reset();
                }
                else
                { /* 未解锁：明确拒绝解锁并报警，不再打开姿态控制器。
                   * 修复：原实现仅清空模式，随后仍会执行 Attitude_Control_Enable()，
                   * 电机进入怠速后主循环发现模式为空又静默停桨，地面站无任何提示 */
                    sendLedSignal(LEDSignal_Err1);
                    const char *text;
                    switch (init_fmcr)
                    {
                        case FMCR_Err_NoPositioning:
                            text = "Unable to arm in this mode without positioning.";
                            break;
                        case FMCR_Err_TakenOff:
                            text = "Unable to arm: mode unavailable after takeoff.";
                            break;
                        case FMCR_Err_NotTakeOff:
                            text = "Unable to arm: mode requires takeoff first.";
                            break;
                        case FMCR_Err_ParamInvalid:
                            text = "Unable to arm: invalid mode parameters.";
                            break;
                        default:
                            text = "Unable to arm: flight mode check failed.";
                            break;
                    }
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
                                                                 &msg_sd, MAV_SEVERITY_ALERT, text, 0, 0);
                                mavlink_send_msg(port, &msg_sd);
                                mavlink_unlock_chan(i);
                            }
                        }
                    }
                    {
                        double logbuf[3] = {9, (double) init_fmcr, (double) currentFlightMode->get_modeId()};
                        SDLog_Msg_DebugVect("exitmode", logbuf, 3);
                    }
                    return MR_Err;
                }
            }
        }
    }

    // 解锁异常判断
    if (!att_enabled && currentFlightMode != nullptr)
    {
        // 获取接收机
        Receiver rc;
        getReceiver(&rc, 0, 0.02);
        // 检查模式运行设置
        FlightModeCheckResult fmcr = currentFlightMode->check(rc, GLB_FlightModeFlags, comVars, &cfg, avCfg);

        if (fmcr == FMCR_Err_NoPositioning)
        { // 切换到未定位模式错误
            sendLedSignal(LEDSignal_Err1);
            const char text[50] = {"Unable to arm in this mode without positioning."};
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
                                                         &msg_sd, MAV_SEVERITY_ALERT, text, 0, 0);
                        mavlink_send_msg(port, &msg_sd);
                        mavlink_unlock_chan(i);
                    }
                }
            }
            {
                double logbuf[3] = {1, (double) (att_enabled ? 1 : 0), 0};
                SDLog_Msg_DebugVect("exitmode", logbuf, 3);
            }
            return MR_Err;
        }

        if (0)
        { // 无避障传感器不允许在避障模式解锁
            sendLedSignal(LEDSignal_Err1);
            const char text[50] = {"Unable to arm in this mode without OASYS."};
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
                                                         &msg_sd, MAV_SEVERITY_ALERT, text, 0, 0);
                        mavlink_send_msg(port, &msg_sd);
                        mavlink_unlock_chan(i);
                    }
                }
            }
            {
                double logbuf[3] = {2, 0, 0};
                SDLog_Msg_DebugVect("exitmode", logbuf, 3);
            }
            return MR_Err;
        }
    }

    // 读取初始航线信息
    read_restoreWpInf();

    // 读取飞行统计信息
    read_flightStats();

    // 需要解锁初始化航点为0
    if (cfg.configs[0] & MCfg_RstWp0_Bit)
        setCurrentMission(0);

    // 打开控制器
    bool arm_started = Attitude_Control_Enable();
    // 入口检查与控制器启用均完成，回复延迟的入口指令回执（仅首次成功时发送）
    ModeEntryAck(att_enabled || arm_started);

    // 低电量状态
    LPW_STATE lpw_state = LPW_STATE::NORMAL;

    // 恢复模式
    int16_t rcvMode = -1;
    TIME rcvTIME(false);

    // 退出模式计数器
    uint16_t exit_mode_Gcounter = 0;
    // 上次的接收机名称
    SName last_rcName("");
    // 模式按钮状态
    uint8_t ModeButtonZone = 255;
    // 模式按钮防抖
    uint8_t ModeButtonPendingZone = 255;
    uint8_t ModeButtonDebounceCounter = 0;
    // 按钮234状态
    uint8_t ButtonsZone[3] = {255, 255, 255};
    // 按钮234防抖
    uint8_t ButtonsPendingZone[3] = {255, 255, 255};
    uint8_t ButtonsDebounceCounter[3] = {0, 0, 0};
    // 按钮234切换时间(防止过快切换)
    TIME ButtonsSwTIME[3];
    for (uint8_t i = 0; i < 3; ++i)
        ButtonsSwTIME[i].set_invalid();
    // 摇杆不回中防抖计数器(过滤RC跳动脉冲)
    uint8_t SticksOutDebounceCounter = 0;
    // 低电量接管手动防抖计数器(过滤RC跳动脉冲)
    uint8_t NDSticksOutDebounceCounter = 0;
    // 1003一键上锁边沿检测：需要开关先拨到低位再拨到高位才触发上锁
    bool switchBt1003_seenLow = false;

    // 轨迹复演
    std::unique_ptr<FlightModule_RecordTrajectory> fmodule_recordTrajectory = nullptr;

    // 初始化Aux处理
    init_process_AuxFuncs();

    // 飞行统计累计变量
    bool last_inFlight = false;
    double stats_time_accum = 0.0;
    double stats_dist_accum = 0.0;
    vector3<double> stats_last_pos;
    bool stats_pos_valid = false;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, h * configTICK_RATE_HZ);

        // 获取接收机
        Receiver rc;
        SName rcName;
        getReceiver(&rc, &rcName, 0.02);

        // 处理Aux通道
        process_AuxFuncs(&rc, h);

        // 获取消息
        bool msg_available;
        ModeMsg msg;
        msg_available = ModeReceiveMsg(&msg, 0);
        FlightModeMsgResult msg_handled = FMMR_Unprocessed;

        // 获取起飞状态
        bool inFlight;
        get_is_inFlight(&inFlight);
        if (inFlight)
            comVars.taken_off = true;

        // 统计飞行架次、时间和距离
        if (inFlight && !last_inFlight)
        { // inFlight上升沿：新架次
            flightStats.flight_count[0]++;
            stats_pos_valid = false;
        }
        if (inFlight)
        {
            stats_time_accum += h;
            vector3<double> cur_pos;
            if (get_Position_Ctrl(&cur_pos))
            {
                if (stats_pos_valid)
                {
                    double dx = cur_pos.x - stats_last_pos.x;
                    double dy = cur_pos.y - stats_last_pos.y;
                    double dist_cm = safe_sqrt(dx * dx + dy * dy);
                    if (dist_cm >= 50.0) // 移动超过0.5m才记录，避免悬停抖动累积
                    {
                        stats_dist_accum += dist_cm * 0.01; // cm → m
                        stats_last_pos = cur_pos;
                    }
                }
                else
                {
                    stats_last_pos = cur_pos;
                    stats_pos_valid = true;
                }
            }
        }
        else
        {
            stats_pos_valid = false;
        }
        last_inFlight = inFlight;

        // 获取电量状态
        PWR_STATE pwr_state = get_pwr_state();

        // 判断进入低电量状态
        bool noLpw = false;
        if (currentFlightMode && currentFlightMode->get_functionCode() == AFunc_Loiter)
        { // 跟踪模式且传感器需求低电量跟随主机时不进入低电量状态
            followSensor sensor;
            if (get_followSensor(&sensor) && sensor.available && sensor.followInLpw)
                noLpw = true;
        }
        if (!noLpw)
        {
            switch (pwr_state)
            {
                case PWR_STATE::NORMAL:
                {
                    if (!inFlight)
                        lpw_state = LPW_STATE::NORMAL;
                    break;
                }
                case PWR_STATE::LVOLT_REQ_RTL:
                {
                    if (lpw_state < LPW_STATE::ND_RTL)
                        lpw_state = LPW_STATE::ND_RTL;
                    break;
                }
                case PWR_STATE::LVOLT_REQ_LAND:
                {
                    if (lpw_state < LPW_STATE::ND_LAND)
                        lpw_state = LPW_STATE::ND_LAND;
                    break;
                }
            }
        }

        // 地面站加锁
        if (msg_available && msg.cmd == MAV_CMD_COMPONENT_ARM_DISARM)
        {
            if (msg.params[0] == 0 && !inFlight)
            {
                if ((msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK)
                {
                    Attitude_Control_Disable();
                    os_delay(1.0);

                    uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
                    const Port *port = get_CommuPort(port_index);
                    if (port && port->write)
                    {
                        mavlink_message_t msg_sd;
                        if (mavlink_lock_chan(port_index, 0.01))
                        {
                            mavlink_msg_command_ack_pack_chan(get_CommulinkSysId(),  // system id
                                                              get_CommulinkCompId(), // component id
                                                              port_index, &msg_sd,
                                                              msg.cmd,             // command
                                                              MAV_RESULT_ACCEPTED, // result
                                                              100,                 // progress
                                                              0,                   // param2
                                                              msg.sd_sysid,        // target system
                                                              msg.sd_compid        // target component
                            );
                            mavlink_send_msg(port, &msg_sd);
                            mavlink_unlock_chan(port_index);
                        }
                    }
                }
                {
                    double logbuf[3] = {3, (double) msg.params[0], (double) (inFlight ? 1 : 0)};
                    SDLog_Msg_DebugVect("exitmode", logbuf, 3);
                }
                return MR_OK;
            }
            msg_handled = FMMR_Success;
        }
        // 侧翻加锁
        if (get_CrashedState())
        {
            {
                double logbuf[3] = {4, 0, 0};
                SDLog_Msg_DebugVect("exitmode", logbuf, 3);
            }
            Attitude_Control_Disable();
            // 触发开伞
            Parachute_release();
            return MR_Err;
        }

        // 请求进入的模式
        std::unique_ptr<FlightMode_Base> req_mode = nullptr;
        // 请求的模式标志
        FlightModeFlag req_fmFlags = GLB_FlightModeFlags;
        // 模式切换来源 (上云API mode_code_reason)
        uint16_t req_reason = MODE_CODE_REASON_NONE;

        // 本层负责的模式指令：在case内与req_mode创建一体置位，未实际采用前不能回复成功
        bool mode_cmd = false;
        // 当前req_mode是否仍为消息请求的模式（RC按钮等来源覆盖req_mode时置false）
        bool req_mode_from_msg = false;

        // 是否在安全模式
        bool is_msafe = is_MSafeCtrl();

        // 根据指令更改飞行模式
        if (msg_available)
        {
            // 消息来源: mavlink=App, 遥控Aux=遥控按键, 机巢/集群=外部设备
            uint8_t msgSrc = msg.cmd_type & CMD_TYPE_MASK;
            if (msgSrc == CMD_TYPE_MAVLINK)
                req_reason = MODE_CODE_REASON_APP;
            else if (msgSrc == CMD_TYPE_RC_AUX)
                req_reason = MODE_CODE_REASON_RC_BUTTON;
            else if (msgSrc == CMD_TYPE_EXTERNAL_DEV)
                req_reason = MODE_CODE_REASON_EXTERNAL;
            else
                req_reason = MODE_CODE_REASON_NONE;
            switch (msg.cmd)
            {
                case 176:
                { // 切换模式（本层认领，未匹配到支持的模式时由延迟回执拒绝）
                    mode_cmd = true;
                    if ((int) msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                    { // mavlink定义模式
                        uint32_t main_mode = msg.params[1];
                        uint32_t sub_mode = msg.params[2];
                        if (main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode == PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL)
                        { // 指令进入定点模式
                            req_mode = FlightModeFactory::createMode(AFunc_PosHold);
                            {
                                double logbuf[3] = {1, AFunc_PosHold, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        // else if (main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode == PX4_CUSTOM_SUB_MODE_POSCTL_Circle)
                        //{
                        //     reqMode = AFunc_ManualCircle;
                        //     msg_handled = FMMR_Success;
                        // }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL)
                        { // 指令进入定高
                            req_mode = FlightModeFactory::createMode(AFunc_AltHold);
                            {
                                double logbuf[3] = {1, AFunc_AltHold, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_MISSION)
                        { // 指令进入任务模式
                            req_mode = FlightModeFactory::createMode(AFunc_Mission);
                            {
                                double logbuf[3] = {1, AFunc_Mission, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == AFunc_AutoMission)
                        { // 指令进入二次开发自动任务模式
                            req_mode = FlightModeFactory::createMode(AFunc_AutoMission);
                            {
                                double logbuf[3] = {1, AFunc_AutoMission, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_RTL)
                        { // 指令进入返航模式
                            req_mode = FlightModeFactory::createMode(AFunc_RTL);
                            {
                                double logbuf[3] = {1, AFunc_RTL, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LAND)
                        { // 指令进入降落模式
                            req_mode = FlightModeFactory::createMode(AFunc_Land);
                            {
                                double logbuf[3] = {1, AFunc_Land, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if ((main_mode == PX4_CUSTOM_MAIN_MODE_AUTO || main_mode == 0) && sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
                        { // 指令进入跟踪模式
                            req_mode = FlightModeFactory::createMode(AFunc_Loiter);
                            {
                                double logbuf[3] = {1, AFunc_Loiter, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_RATTITUDE)
                        { // 指令进入半自稳
                            req_mode = FlightModeFactory::createMode(AFunc_Rattitude);
                            {
                                double logbuf[3] = {1, AFunc_Rattitude, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL)
                        { // 指令进入手动
                            req_mode = FlightModeFactory::createMode(AFunc_Manual);
                            {
                                double logbuf[3] = {1, AFunc_Manual, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED)
                        { // 指令进入增稳
                            req_mode = FlightModeFactory::createMode(AFunc_Stabilize);
                            {
                                double logbuf[3] = {1, AFunc_Stabilize, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
                        }
                        else if (main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                        { // 指令进入OFFBOARD模式
                            req_mode = FlightModeFactory::createMode(AFunc_Offboard);
                            {
                                double logbuf[3] = {1, AFunc_Offboard, 0};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            msg_handled = FMMR_Success;
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
                                msg_handled = FMMR_Success;
                            }
                            else if ((int) msg.params[1] == 0)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_HeadlessEna);
                                msg_handled = FMMR_Success;
                            }
                            else
                                msg_handled = FMMR_Failed;
                            break;
                        }
                        case ACFLY_FUNCTION_OBSTACLE_AVOIDANCE:
                        { // 避障
                            if ((int) msg.params[1] == 1)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_AvoidanceEna);
                                msg_handled = FMMR_Success;
                            }
                            else if ((int) msg.params[1] == 0)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_AvoidanceEna);
                                msg_handled = FMMR_Success;
                            }
                            else
                                msg_handled = FMMR_Failed;
                            break;
                        }
                        case ACFLY_FUNCTION_TERRAIN_FOLLOWING:
                        { // 仿地
                            if ((int) msg.params[1] == 1)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_TerrainFollowingEna);
                                msg_handled = FMMR_Success;
                            }
                            else if ((int) msg.params[1] == 0)
                            {
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_TerrainFollowingEna);
                                msg_handled = FMMR_Success;
                            }
                            else
                                msg_handled = FMMR_Failed;
                            break;
                        }
                        case ACFLY_FUNCTION_TRAJECTORY_RECORD:
                        { // 轨迹记录
                            int rec_ind = (int) msg.params[1];
                            if (rec_ind >= 1 && rec_ind <= 15 && fmodule_recordTrajectory == nullptr)
                            {
                                fmodule_recordTrajectory = std::make_unique<FlightModule_RecordTrajectory>(rec_ind, round(0.1 / h), 0.1);
                                GLB_FlightModeFlags = (FlightModeFlag) (((int) GLB_FlightModeFlags & ~(int) FMF_RecordTrajectory_Mask) | (rec_ind << 12));
                                msg_handled = FMMR_Success;
                            }
                            else if (rec_ind == 0 && fmodule_recordTrajectory)
                            {
                                fmodule_recordTrajectory->stop_record();
                                fmodule_recordTrajectory.reset();
                                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_RecordTrajectory_Mask);
                                msg_handled = FMMR_Success;
                            }
                            else
                                msg_handled = FMMR_Failed;
                            break;
                        }
                        case ACFLY_FUNCTION_TRAJECTORY_REPLAY:
                        { // 轨迹复演: params[1]=轨迹编号(0=停止), params[2]=起点模式, params[3]=方向
                            mode_cmd = true;
                            if ((int) msg.params[1] == 0)
                            { // 停止复演: 当前在复演则切定点悬停
                                if (currentFlightMode && currentFlightMode->get_modeId() == AFunc_TrajectoryReplay)
                                {
                                    req_mode = FlightModeFactory::createMode(AFunc_PosHold);
                                    {
                                        double logbuf[3] = {2, AFunc_PosHold, 0};
                                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                                    }
                                    msg_handled = req_mode ? FMMR_Success : FMMR_Failed;
                                }
                                else
                                    msg_handled = FMMR_Failed;
                            }
                            else
                            { // 启动复演
                                req_mode = FlightModeFactory::createMode(AFunc_TrajectoryReplay);
                                if (req_mode)
                                {
                                    double params[3];
                                    params[0] = msg.params[1];
                                    params[1] = msg.params[2];
                                    params[2] = msg.params[3];
                                    if (req_mode->set_params(params, 3))
                                    {
                                        double logbuf[3] = {2, AFunc_TrajectoryReplay, 0};
                                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                                        msg_handled = FMMR_Success;
                                    }
                                    else
                                    {
                                        req_mode.reset();
                                        msg_handled = FMMR_Failed;
                                    }
                                }
                                else
                                    msg_handled = FMMR_Failed;
                            }
                            break;
                        }
                        default:
                        {
                            msg_handled = FMMR_Failed;
                            break;
                        }
                    }
                    break;
                }
                case 249:
                { // 切换绕圈（模式类指令，本层认领；参数无效/检查失败时由延迟回执拒绝）
                    mode_cmd = true;
                    req_mode = FlightModeFactory::createMode(AFunc_Circle);

                    // 模式检查
                    if (req_mode)
                    {
                        double params[7];
                        params[0] = msg.frame;            // frame
                        params[1] = msg.params[4];        // x
                        params[2] = msg.params[5];        // y
                        params[3] = msg.params[6];        // z
                        params[4] = msg.params[2];        // 偏航
                        params[5] = -msg.params[0] * 100; // 速度
                        params[6] = msg.params[1] * 100;  // 半径
                        if (!req_mode->set_params(params, 7))
                        { // 参数非法
                            req_mode.reset();
                            msg_handled = FMMR_Failed;
                            break;
                        }
                        req_mode->force_switch = true;

                        // 模式检查失败则退出
                        if (req_mode->check(rc, GLB_FlightModeFlags, comVars, &cfg, avCfg) != FMCR_OK)
                        {
                            req_mode.reset();
                            msg_handled = FMMR_Failed;
                        }
                        else
                        {
                            double logbuf[3] = {3, AFunc_Circle, 0};
                            SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            msg_handled = FMMR_Success;
                        }
                    }
                    else
                        msg_handled = FMMR_Failed;
                    break;
                }
                case 20:
                { // 返航到指定地点（模式类指令，本层认领；参数无效时由延迟回执拒绝）
                    mode_cmd = true;
                    if (msg.params[0] == 1)
                    {
                        if (isvalid(msg.params[4]) && isvalid(msg.params[5]) && msg.params[4] >= -90 && msg.params[4] <= 90 && msg.params[5] >= -180 && msg.params[5] <= 180)
                        {
                            // 计算返航高度(相对地面)
                            double req_alt_local;
                            if (get_localZ_fromOptimalGlobal(msg.params[6] * 100, &req_alt_local))
                            {
                                // 获取起飞点local z
                                // 相对地面高 = local_z - homeZ
                                double homeZ;
                                getHomeLocalZ(&homeZ);
                                req_alt_local -= homeZ;

                                // 创建模式
                                req_mode = FlightModeFactory::createMode(AFunc_RTL2Point);
                                if (req_mode)
                                {
                                    double logbuf[3] = {4, AFunc_RTL2Point, (double) (msg.params[1] * 100)};
                                    SDLog_Msg_DebugVect("changemode", logbuf, 3);

                                    double params[4];
                                    params[0] = msg.params[4];       // lat
                                    params[1] = msg.params[5];       // lon
                                    params[2] = req_alt_local;       // alt
                                    params[3] = msg.params[1] * 100; // rtl height
                                    req_mode->set_params(params, 4);

                                    msg_handled = FMMR_Success;
                                }
                            }
                        }
                    }
                    else
                    {
                        req_mode = FlightModeFactory::createMode(AFunc_RTL);
                        {
                            double logbuf[3] = {4, AFunc_RTL, 0};
                            SDLog_Msg_DebugVect("changemode", logbuf, 3);
                        }
                        msg_handled = FMMR_Success;
                    }
                    break;
                }
                case 21:
                { // 降落（模式类指令，本层认领）
                    mode_cmd = true;
                    req_reason = MODE_CODE_REASON_APP;
                    req_mode = FlightModeFactory::createMode(AFunc_Land);
                    if (req_mode)
                    {
                        double logbuf[3] = {5, AFunc_Land, (double) (msg.params[6] * 100)};
                        SDLog_Msg_DebugVect("changemode", logbuf, 3);

                        double lnd_params[1];
                        lnd_params[0] = msg.params[6] * 100;
                        req_mode->set_params(lnd_params, 1);

                        msg_handled = FMMR_Success;
                    }
                    break;
                }
            }

            /* 模式类指令回执延迟到模式实际采用后（req_mode切换成功处）；
             * 本帧未采用则保持Failed，ModeLoopFin回DENIED。
             * case内设置的Success在此统一改回Failed，仅req_mode实际采用成功时才重新置Success */
            if (mode_cmd)
            {
                msg_handled = FMMR_Failed;
                req_mode_from_msg = req_mode != nullptr;
            }
        }

        if (rc.available)
        { // 接收机可用

            // 重置无接收机时间
            comVars.noRcStartTIME.set_invalid();

            // 如果接收机变化重置状态
            // 重置模式开关状态
            if (rcName != last_rcName)
            {
                ModeButtonZone = 255;
                ModeButtonPendingZone = 255;
                ModeButtonDebounceCounter = 0;
                SticksOutDebounceCounter = 0;
                for (uint8_t i = 0; i < 3; ++i)
                {
                    ButtonsZone[i] = 255;
                    ButtonsPendingZone[i] = 255;
                    ButtonsDebounceCounter[i] = 0;
                    ButtonsSwTIME[i].set_invalid();
                }
                last_rcName = rcName;
            }

            // 一键上锁(功能码1003) - 边沿检测，开关需先拨低再拨高才触发
            bool switchBtLock = false;
            for (uint8_t i = 0; i < 3; ++i)
            {
                if (cfg.Bt234Func[i][0] == 1003 && rc.available_channels >= 6 + i)
                {
                    if (rc.data[5 + i] < 10.0f)
                        switchBt1003_seenLow = true;
                    if (switchBt1003_seenLow && rc.data[5 + i] > 90.0f)
                        switchBtLock = true;
                    break;
                }
            }

            // 手势强制加锁(外八) 或 1003一键上锁
            if ((rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90) || switchBtLock)
            {
                if (++exit_mode_Gcounter >= freq * 1.5)
                {
                    {
                        double logbuf[3] = {5, (double) exit_mode_Gcounter, 0};
                        SDLog_Msg_DebugVect("exitmode", logbuf, 3);
                    }
                    Attitude_Control_Disable();
                    // 保存飞行统计
                    flightStats.flight_time_s[0] += (float) stats_time_accum;
                    flightStats.flight_dist_m[0] += (float) stats_dist_accum;
                    store_flightStats();
                    if (mode_cmd)
                        ModeMsgAck(msg, false);
                    return MR_OK;
                }
            }
            else
                exit_mode_Gcounter = 0;

            // 检测模式开关更改飞行模式(带防抖，过滤RC短时间跳动脉冲)
            uint8_t new_ModeButtonZone;
            if (rc.available_channels >= 5)
                new_ModeButtonZone = get_RcButtonZone(rc.data[4], ModeButtonZone);
            else
                new_ModeButtonZone = 255;
            if (ModeButtonZone > 5 && new_ModeButtonZone <= 5)
            { // 首次检测到有效位置，直接同步（不触发模式切换）
                ModeButtonZone = new_ModeButtonZone;
            }
            else if (new_ModeButtonZone <= 5 && ModeButtonZone <= 5 && new_ModeButtonZone != ModeButtonZone)
            { // 检测到按钮变化，进行防抖确认
                if (new_ModeButtonZone == ModeButtonPendingZone)
                { // 与pending值一致，累加计数
                    if (++ModeButtonDebounceCounter >= button_debounce_frames)
                    { // 连续确认帧数达到阈值，确认切换
                        req_mode = FlightModeFactory::createMode(cfg.Bt1AFunc[new_ModeButtonZone][0]);
                        req_mode_from_msg = false;
                        {
                            double logbuf[3] = {6, (double) cfg.Bt1AFunc[new_ModeButtonZone][0], (double) new_ModeButtonZone};
                            SDLog_Msg_DebugVect("changemode", logbuf, 3);
                        }
                        // 遥控模式按钮切换
                        req_reason = MODE_CODE_REASON_NONE;
                        ModeButtonZone = new_ModeButtonZone;
                        ModeButtonDebounceCounter = 0;
                        ModeButtonPendingZone = 255;
                    }
                }
                else
                { // 新值与pending不同，重新开始计数
                    ModeButtonPendingZone = new_ModeButtonZone;
                    ModeButtonDebounceCounter = 1;
                }
            }
            else
            { // 无变化或无效值，复位防抖
                ModeButtonPendingZone = 255;
                ModeButtonDebounceCounter = 0;
            }

            // 检测模式更改按钮
            for (uint8_t i = 0; i < 3; ++i)
            {
                uint32_t btn_func = cfg.Bt234Func[i][0];
                if (btn_func > 0)
                {
                    // 检测模式开关更改飞行模式(带防抖，过滤RC短时间跳动脉冲)
                    uint8_t new_ButtonZone;
                    if (rc.available_channels >= i + 6)
                        new_ButtonZone = get_RcButtonZone(rc.data[i + 5], ButtonsZone[i]);
                    else
                        new_ButtonZone = 255;
                    if (ButtonsZone[i] > 5 && new_ButtonZone <= 5)
                    { // 首次检测到有效位置，直接同步（不触发功能切换）
                        ButtonsZone[i] = new_ButtonZone;
                    }
                    else if (new_ButtonZone <= 5 && ButtonsZone[i] <= 5 && new_ButtonZone != ButtonsZone[i])
                    { // 检测到按钮变化，进行防抖确认
                        if (new_ButtonZone == ButtonsPendingZone[i])
                        { // 与pending值一致，累加计数
                            if (++ButtonsDebounceCounter[i] >= button_debounce_frames)
                            { // 连续确认帧数达到阈值，确认变化
                                ButtonsDebounceCounter[i] = 0;
                                ButtonsPendingZone[i] = 255;
                                // 防止按钮频繁变化(跳舵)
                                if (ButtonsSwTIME[i].get_pass_time() > 0.5)
                                {
                                    if (btn_func < 1000)
                                    { // 切换到指定模式编号
                                        std::unique_ptr<FlightMode_Base> new_req_mode = FlightModeFactory::createMode(btn_func);
                                        if (new_req_mode != nullptr)
                                        {
                                            if (currentFlightMode && currentFlightMode->get_modeId() == new_req_mode->get_modeId() &&
                                                currentFlightMode->get_functionCode() == new_req_mode->get_functionCode())
                                            { // 模式一致尝试切换到其他模式
                                                if (ModeButtonZone <= 5)
                                                {
                                                    new_req_mode.reset();
                                                    new_req_mode = FlightModeFactory::createMode(cfg.Bt1AFunc[ModeButtonZone][0]);
                                                    if (new_req_mode != nullptr)
                                                    {
                                                        {
                                                            double logbuf[3] = {7, (double) cfg.Bt1AFunc[ModeButtonZone][0], (double) new_ButtonZone};
                                                            SDLog_Msg_DebugVect("changemode", logbuf, 3);
                                                        }
                                                        req_mode = std::move(new_req_mode);
                                                        req_mode_from_msg = false;
                                                        new_req_mode.reset();
                                                    }
                                                }
                                            }
                                            else
                                            { // 切换到选定模式
                                                req_mode = std::move(new_req_mode);
                                                req_mode_from_msg = false;
                                                {
                                                    double logbuf[3] = {7, (double) btn_func, (double) new_ButtonZone};
                                                    SDLog_Msg_DebugVect("changemode", logbuf, 3);
                                                }
                                                new_req_mode.reset();
                                            }
                                        }
                                    }
                                    else
                                    {
                                        switch (btn_func)
                                        {
                                            case 1000:
                                            { // 切换无头状态
                                                if (GLB_FlightModeFlags & FMF_HeadlessEna)
                                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_HeadlessEna);
                                                else
                                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_HeadlessEna);
                                                break;
                                            }
                                            case 1001:
                                            { // 切换避障状态
                                                if (GLB_FlightModeFlags & FMF_AvoidanceEna)
                                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_AvoidanceEna);
                                                else
                                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_AvoidanceEna);
                                                break;
                                            }
                                            case 1002:
                                            { // 切换仿地状态
                                                if (GLB_FlightModeFlags & FMF_TerrainFollowingEna)
                                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_TerrainFollowingEna);
                                                else
                                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags | (int) FMF_TerrainFollowingEna);
                                                break;
                                            }
                                            case 1011:
                                            { // 轨迹记录1
                                                if (fmodule_recordTrajectory == nullptr)
                                                {
                                                    fmodule_recordTrajectory = std::make_unique<FlightModule_RecordTrajectory>(1, round(0.1 / h), 0.1);
                                                    GLB_FlightModeFlags = (FlightModeFlag) (((int) GLB_FlightModeFlags & ~(int) FMF_RecordTrajectory_Mask) | (1 << 12));
                                                }
                                                else
                                                {
                                                    fmodule_recordTrajectory->stop_record();
                                                    fmodule_recordTrajectory.reset();
                                                    GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_RecordTrajectory_Mask);
                                                }
                                                break;
                                            }
                                        }
                                    }
                                }
                                ButtonsSwTIME[i] = TIME::now();
                                ButtonsZone[i] = new_ButtonZone;
                            }
                        }
                        else
                        { // 新值与pending不同，重新开始计数
                            ButtonsPendingZone[i] = new_ButtonZone;
                            ButtonsDebounceCounter[i] = 1;
                        }
                    }
                    else
                    { // 无变化或无效值，复位防抖
                        ButtonsPendingZone[i] = 255;
                        ButtonsDebounceCounter[i] = 0;
                    }
                }
                else
                {
                    ButtonsZone[i] = 255;
                    ButtonsPendingZone[i] = 255;
                    ButtonsDebounceCounter[i] = 0;
                    ButtonsSwTIME[i].set_invalid();
                }
            }

            // 检测遥控器不回中更改飞行模式(带防抖，过滤RC短时间跳动脉冲)
            bool sticks_in_neutral = in_symmetry_range_mid(rc.data[0], 50, cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[1], 50, cfg.NeutralZone[0]) &&
                                     in_symmetry_range_mid(rc.data[2], 50, cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg.NeutralZone[0]);
            if (!sticks_in_neutral)
            { // 摇杆没回中 - 累加防抖计数(防止溢出)
                if (SticksOutDebounceCounter < button_debounce_frames)
                    ++SticksOutDebounceCounter;
                if (SticksOutDebounceCounter >= button_debounce_frames)
                { // 持续不回中达到阈值，执行模式切换
                    if (currentFlightMode && currentFlightMode->get_fmClass() == FMC_Automatic && ((cfg.configs[0] & MCfg_NoRcCtrlInAuto_Bit) == 0))
                    {
                        // 摇杆打断自动模式 - 返航中断(悬停)置21, 其余自动模式(任务/降落等)置用户打杆接管
                        {
                            uint8_t curId = currentFlightMode->get_modeId();
                            if (curId == AFunc_RTL || curId == AFunc_RTL2Point)
                                req_reason = MODE_CODE_REASON_RTL_CANCELED;
                            else
                                req_reason = MODE_CODE_REASON_RC_TAKEOVER;
                        }
                        req_mode_from_msg = false;
                        // 摇杆打断自动模式 - 返航中断(悬停)置21, 其余自动模式(任务/降落等)置用户打杆接管
                        {
                            uint8_t curId = currentFlightMode->get_modeId();
                            if (curId == AFunc_RTL || curId == AFunc_RTL2Point)
                                req_reason = MODE_CODE_REASON_RTL_CANCELED;
                            else
                                req_reason = MODE_CODE_REASON_RC_TAKEOVER;
                        }

                        if (ModeButtonZone <= 5)
                        {
                            req_mode = FlightModeFactory::createMode(cfg.Bt1AFunc[ModeButtonZone][0]);
                            if (req_mode == nullptr || req_mode->get_fmClass() == FMC_Automatic)
                                req_mode = FlightModeFactory::createMode(AFunc_PosHold);
                        }
                        else
                            req_mode = FlightModeFactory::createMode(AFunc_PosHold);
                        {
                            double logbuf[3] = {8, (double) (currentFlightMode ? currentFlightMode->get_modeId() : -1), (double) rcvMode};
                            SDLog_Msg_DebugVect("changemode", logbuf, 3);
                        }
                        rcvMode = currentFlightMode->get_modeId();
                        rcvTIME = TIME::now();
                    }
                    else
                    {
                        rcvMode = -1;
                    }
                }
            }
            else
            { // 摇杆回中 - 复位防抖计数
                SticksOutDebounceCounter = 0;

                /*回舵恢复模式*/
                bool rcv = false;
                if (rcvMode >= 0)
                {
                    if ((cfg.configs[0] & MCfg_RcvAuto_Bit) && rcvTIME.get_pass_time() < 4.5)
                    {
                        std::unique_ptr<FlightMode_Base> new_req_mode = FlightModeFactory::createMode(rcvMode);
                        if (new_req_mode != nullptr)
                        {
                            req_mode = std::move(new_req_mode);
                            req_mode_from_msg = false;
                            {
                                double logbuf[3] = {9, (double) rcvMode, rcvTIME.get_pass_time()};
                                SDLog_Msg_DebugVect("changemode", logbuf, 3);
                            }
                            rcv = true;
                        }
                    }
                    rcvMode = -1;
                }
                /*回舵恢复模式*/
            }
        }
        else
        { // 接收机不可用

            // 重置遥控状态
            if (comVars.noRcStartTIME.is_valid() == false)
                comVars.noRcStartTIME = TIME::now();

            // 重置模式开关状态
            ModeButtonZone = 255;
            ModeButtonPendingZone = 255;
            ModeButtonDebounceCounter = 0;
            SticksOutDebounceCounter = 0;
            for (uint8_t i = 0; i < 3; ++i)
            {
                ButtonsZone[i] = 255;
                ButtonsPendingZone[i] = 255;
                ButtonsDebounceCounter[i] = 0;
                ButtonsSwTIME[i].set_invalid();
            }
            last_rcName = "";
            // 重置遥控器上锁动作计数器
            exit_mode_Gcounter = 0;
            // 重置低电量接管防抖计数器
            NDSticksOutDebounceCounter = 0;
        }

        // 低电量进入降落
        if (lpw_state == LPW_STATE::ND_LAND)
        {
            if (currentFlightMode)
            {
                bool nd_switch_manual = false;
                if (rc.available)
                { // 返航
                    bool sticks_in_neutral = in_symmetry_range_mid(rc.data[0], 50, cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[1], 50, cfg.NeutralZone[0]) &&
                                             in_symmetry_range_mid(rc.data[2], 50, cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg.NeutralZone[0]);
                    if (!sticks_in_neutral && (currentFlightMode->get_modeId() != AFunc_Land || currentFlightMode->get_fmClass() == FMC_Automatic))
                    { // 摇杆没回中 - 累加防抖计数(防止溢出)
                        if (NDSticksOutDebounceCounter < button_debounce_frames)
                            ++NDSticksOutDebounceCounter;
                        if (NDSticksOutDebounceCounter >= button_debounce_frames)
                        { // 持续不回中达到阈值，执行模式切换
                            nd_switch_manual = true;
                            if (currentFlightMode->get_modeId() != AFunc_PosHold || currentFlightMode->get_functionCode() != 3)
                            {
                                {
                                    double logbuf[3] = {10, (double) currentFlightMode->get_modeId(), (double) rc.available};
                                    SDLog_Msg_DebugVect("changemode", logbuf, 3);
                                }
                                currentFlightMode = FlightModeFactory::createMode(AFunc_PosHold);
                            }
                        }
                    }
                    else
                        NDSticksOutDebounceCounter = 0;
                }
                if (currentFlightMode->get_modeId() != AFunc_Land && !nd_switch_manual)
                {
                    {
                        double logbuf[3] = {10, (double) currentFlightMode->get_modeId(), 0};
                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                    }
                    // 低电压降落 - 电压严重过低
                    setCurrentFlyMode(AFunc_Land, MODE_CODE_REASON_LVOLT_LAND);
                    currentFlightMode = FlightModeFactory::createMode(AFunc_Land);
                }
            }
            else
            {
                {
                    double logbuf[3] = {10, -1, 0};
                    SDLog_Msg_DebugVect("changemode", logbuf, 3);
                }
                setCurrentFlyMode(AFunc_Land, MODE_CODE_REASON_LVOLT_LAND);
                currentFlightMode = FlightModeFactory::createMode(AFunc_Land);
            }
        }

        // 低电量进入返航
        else if (lpw_state == LPW_STATE::ND_RTL)
        {
            if (currentFlightMode)
            {
                bool nd_switch_manual = false;
                if (rc.available)
                { // 返航
                    bool sticks_in_neutral = in_symmetry_range_mid(rc.data[0], 50, cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[1], 50, cfg.NeutralZone[0]) &&
                                             in_symmetry_range_mid(rc.data[2], 50, cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg.NeutralZone[0]);
                    if (!sticks_in_neutral && (currentFlightMode->get_modeId() != AFunc_RTL || currentFlightMode->get_fmClass() == FMC_Automatic))
                    { // 摇杆没回中 - 累加防抖计数(防止溢出)
                        if (NDSticksOutDebounceCounter < button_debounce_frames)
                            ++NDSticksOutDebounceCounter;
                        if (NDSticksOutDebounceCounter >= button_debounce_frames)
                        { // 持续不回中达到阈值，执行模式切换
                            nd_switch_manual = true;
                            if (currentFlightMode->get_modeId() != AFunc_PosHold || currentFlightMode->get_functionCode() != 3)
                            {
                                {
                                    double logbuf[3] = {11, (double) currentFlightMode->get_modeId(), (double) rc.available};
                                    SDLog_Msg_DebugVect("changemode", logbuf, 3);
                                }
                                currentFlightMode = FlightModeFactory::createMode(AFunc_PosHold);
                            }
                        }
                    }
                    else
                        NDSticksOutDebounceCounter = 0;
                }
                if (currentFlightMode->get_modeId() != AFunc_RTL && !nd_switch_manual)
                {
                    {
                        double logbuf[3] = {11, (double) currentFlightMode->get_modeId(), 0};
                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                    }
                    // 低电压返航
                    setCurrentFlyMode(AFunc_RTL, MODE_CODE_REASON_LVOLT_RTL);
                    currentFlightMode = FlightModeFactory::createMode(AFunc_RTL);
                }
            }
            else
            {
                {
                    double logbuf[3] = {11, -1, 0};
                    SDLog_Msg_DebugVect("changemode", logbuf, 3);
                }
                setCurrentFlyMode(AFunc_RTL, MODE_CODE_REASON_LVOLT_RTL);
                currentFlightMode = FlightModeFactory::createMode(AFunc_RTL);
            }
        }

        // 执行模式更改
        else if (req_mode != nullptr)
        {
            FlightModeCheckResult req_fmcr = req_mode->check(rc, GLB_FlightModeFlags, comVars, &cfg, avCfg);
            if (req_fmcr == FMCR_OK)
            {
                bool req_applied = true;
                if (currentFlightMode == nullptr || currentFlightMode->get_modeId() != req_mode->get_modeId() || req_mode->force_switch)
                {
                    // 模式切换原因跟随生效
                    setPendingFlyModeReason(req_reason);
                    currentFlightMode = std::move(req_mode);
                }
                else if (currentFlightMode->get_functionCode() != req_mode->get_functionCode())
                {
                    req_applied = currentFlightMode->set_functionCode(req_mode->get_functionCode());
                    req_mode.reset();
                }
                // 指令请求的模式实际采用成功，才把延迟回执标记为成功
                if (req_mode_from_msg && req_applied)
                    msg_handled = FMMR_Success;
            }
            else if (req_mode_from_msg)
            { /* 模式检查失败：带原因报警（msg_handled已由mode_cmd置为Failed，ModeLoopFin 将回复 DENIED） */
                sendLedSignal(LEDSignal_Err1);
                const char *text;
                switch (req_fmcr)
                {
                    case FMCR_Err_NoPositioning:
                        text = "Mode switch denied: no positioning.";
                        break;
                    case FMCR_Err_TakenOff:
                        text = "Mode switch denied: unavailable after takeoff.";
                        break;
                    case FMCR_Err_NotTakeOff:
                        text = "Mode switch denied: requires takeoff first.";
                        break;
                    case FMCR_Err_ParamInvalid:
                        text = "Mode switch denied: invalid parameters.";
                        break;
                    default:
                        text = "Mode switch denied: mode check failed.";
                        break;
                }
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
                                                             &msg_sd, MAV_SEVERITY_WARNING, text, 0, 0);
                            mavlink_send_msg(port, &msg_sd);
                            mavlink_unlock_chan(i);
                        }
                    }
                }
                {
                    double logbuf[3] = {13, (double) req_mode->get_modeId(), (double) req_fmcr};
                    SDLog_Msg_DebugVect("changemode", logbuf, 3);
                }
            }
        }

        // 执行轨迹复演记录
        if (fmodule_recordTrajectory)
        {
            if (!fmodule_recordTrajectory->record())
            {
                fmodule_recordTrajectory.reset();
                GLB_FlightModeFlags = (FlightModeFlag) ((int) GLB_FlightModeFlags & ~(int) FMF_RecordTrajectory_Mask);
            }
        }

        // 执行飞行模式
        if (currentFlightMode == nullptr)
        { // 飞行模式为空执行安全模式
          // 关闭喷洒，手动控制
            setPump1(PumpOffManualPercent);

            // 控制权交给安全模式
            enter_MSafe(true);

            /*判断退出模式*/
            if (inFlight == false)
            {
                {
                    double logbuf[3] = {6, 0, 0};
                    SDLog_Msg_DebugVect("exitmode", logbuf, 3);
                }
                Attitude_Control_Disable();
                // 保存飞行统计
                flightStats.flight_time_s[0] += (float) stats_time_accum;
                flightStats.flight_dist_m[0] += (float) stats_dist_accum;
                store_flightStats();
                if (mode_cmd)
                    ModeMsgAck(msg, false);
                return MR_OK;
            }
            /*判断退出模式*/
        }
        else
        {
            FlightModeMsgResult fmmr = FMMR_Unprocessed;
            FlightModeResult fmr = currentFlightMode->main_func(rc, GLB_FlightModeFlags, comVars, msg_available && !mode_cmd ? &msg : 0, &fmmr, &cfg, avCfg, h);
            switch (fmr)
            {
                case FMR_RqDisarm_Land:
                { // 要求上锁-已落地
                    {
                        double logbuf[3] = {7, (double) (currentFlightMode ? currentFlightMode->get_modeId() : -1), (double) (inFlight ? 1 : 0)};
                        SDLog_Msg_DebugVect("exitmode", logbuf, 3);
                    }
                    Attitude_Control_Disable();

                    /* RTL/Land等落地退出后，其check()在未起飞状态必失败(FMCR_Err_NotTakeOff)，
                     * 若停留在该模式会导致下次解锁被拒绝。切换回当前模式按钮所在模式(检查可用)，
                     * 按钮模式不可用时回退PosHold，下次解锁时M32初始化检查会再次验证并给出明确提示。
                     * 切换逻辑参考FMR_OK_RqSwitchMode分支 */
                    {
                        uint8_t landed_modeId = currentFlightMode->get_modeId();
                        if (landed_modeId == AFunc_RTL || landed_modeId == AFunc_Land || landed_modeId == AFunc_RTL2Point)
                        {
                            AFunc next_mode = AFunc_PosHold;
                            if (ModeButtonZone <= 5)
                            {
                                std::unique_ptr<FlightMode_Base> btn_mode = FlightModeFactory::createMode(cfg.Bt1AFunc[ModeButtonZone][0]);
                                if (btn_mode && btn_mode->check(rc, GLB_FlightModeFlags, comVars, &cfg, avCfg) == FMCR_OK)
                                    next_mode = (AFunc) btn_mode->get_modeId();
                            }
                            setCurrentFlyMode(next_mode);
                            double logbuf[3] = {14, (double) next_mode, (double) landed_modeId};
                            SDLog_Msg_DebugVect("changemode", logbuf, 3);
                        }
                    }

                    // 保存飞行统计
                    flightStats.flight_time_s[0] += (float) stats_time_accum;
                    flightStats.flight_dist_m[0] += (float) stats_dist_accum;
                    store_flightStats();
                    if (mode_cmd)
                        ModeMsgAck(msg, msg_handled == FMMR_Success);
                    return MR_OK;
                    break;
                }

                case FMR_Err_RqSwitchRTL:
                case FMR_OK_RqSwitchRTL:
                { // 需要切换到返航模式
                    // 模式内部请求的返航: 失控-6(遥控信号丢失), 围栏-106, 任务无固定解-103, 其余无标准原因置0
                    setPendingFlyModeReason(MODE_CODE_REASON_NONE);
                    {
                        Receiver rcX;
                        getReceiver(&rcX, 0, 0.02);
                        if (!rcX.available)
                            setPendingFlyModeReason(MODE_CODE_REASON_RC_LOST);
                        else
                        {
                            // 围栏触发中的返航动作
                            FenceBreachStatus fenceSt;
                            if (getFenceStatus(&fenceSt) && fenceSt.active && (fenceSt.mitigation == FENCE_MITIGATE_RTL || fenceSt.mitigation == FENCE_MITIGATE_LAND))
                                setPendingFlyModeReason(MODE_CODE_REASON_FENCE);
                            else if (currentFlightMode)
                            {
                                uint8_t reqSrcId = currentFlightMode->get_modeId();
                                if (reqSrcId == AFunc_Mission && !isGPSFixedSolution())
                                    setPendingFlyModeReason(MODE_CODE_REASON_GPS_NOT_FIXED);
                            }
                        }
                    }
                    if (mode_cmd && fmr == FMR_Err_RqSwitchRTL)
                        msg_handled = FMMR_Failed;
                    // 模式内部请求的返航: 失控-6(遥控信号丢失), 围栏-106, 任务无固定解-103, 其余无标准原因置0
                    setPendingFlyModeReason(MODE_CODE_REASON_NONE);
                    {
                        Receiver rcX;
                        getReceiver(&rcX, 0, 0.02);
                        if (!rcX.available)
                            setPendingFlyModeReason(MODE_CODE_REASON_RC_LOST);
                        else
                        {
                            // 围栏触发中的返航动作
                            FenceBreachStatus fenceSt;
                            if (getFenceStatus(&fenceSt) && fenceSt.active && (fenceSt.mitigation == FENCE_MITIGATE_RTL || fenceSt.mitigation == FENCE_MITIGATE_LAND))
                                setPendingFlyModeReason(MODE_CODE_REASON_FENCE);
                            else if (currentFlightMode)
                            {
                                uint8_t reqSrcId = currentFlightMode->get_modeId();
                                if ((reqSrcId == AFunc_Mission || reqSrcId == AFunc_VertMission) && !isGPSFixedSolution())
                                    setPendingFlyModeReason(MODE_CODE_REASON_GPS_NOT_FIXED);
                            }
                        }
                    }
                    {
                        double logbuf[3] = {12, (double) AFunc_RTL, (double) fmr};
                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                    }
                    currentFlightMode = FlightModeFactory::createMode(AFunc_RTL);
                    break;
                }

                case FMR_Err_RqSwitchLand:
                case FMR_OK_RqSwitchLand:
                { // 需要切换到降落模式
                    // 模式内部请求的降落(定点失控降落等): 失控-6, 围栏-106, 其余视为无外部触发
                    setPendingFlyModeReason(MODE_CODE_REASON_NONE);
                    {
                        Receiver rcX;
                        getReceiver(&rcX, 0, 0.02);
                        if (!rcX.available)
                            setPendingFlyModeReason(MODE_CODE_REASON_RC_LOST);
                        else
                        {
                            // 围栏触发中的降落动作
                            FenceBreachStatus fenceSt;
                            if (getFenceStatus(&fenceSt) && fenceSt.active && (fenceSt.mitigation == FENCE_MITIGATE_RTL || fenceSt.mitigation == FENCE_MITIGATE_LAND))
                                setPendingFlyModeReason(MODE_CODE_REASON_FENCE);
                        }
                    }
                    if (mode_cmd && fmr == FMR_Err_RqSwitchLand)
                        msg_handled = FMMR_Failed;
                    // 模式内部请求的降落(定点失控降落等): 失控-6, 围栏-106, 其余视为无外部触发
                    setPendingFlyModeReason(MODE_CODE_REASON_NONE);
                    {
                        Receiver rcX;
                        getReceiver(&rcX, 0, 0.02);
                        if (!rcX.available)
                            setPendingFlyModeReason(MODE_CODE_REASON_RC_LOST);
                        else
                        {
                            // 围栏触发中的降落动作
                            FenceBreachStatus fenceSt;
                            if (getFenceStatus(&fenceSt) && fenceSt.active && (fenceSt.mitigation == FENCE_MITIGATE_RTL || fenceSt.mitigation == FENCE_MITIGATE_LAND))
                                setPendingFlyModeReason(MODE_CODE_REASON_FENCE);
                        }
                    }
                    {
                        double logbuf[3] = {12, (double) AFunc_Land, (double) fmr};
                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                    }
                    currentFlightMode = FlightModeFactory::createMode(AFunc_Land);
                    break;
                }

                case FMR_OK_RqSwitchLoiter:
                { // 需要切换到跟踪模式
                    {
                        double logbuf[3] = {12, (double) AFunc_Loiter, (double) fmr};
                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                    }
                    currentFlightMode = FlightModeFactory::createMode(AFunc_Loiter);
                    break;
                }
                case FMR_OK_RqSwitchPosHold:
                { // 需要切换到定点模式
                    // 围栏悬停动作触发的定点切换置围栏原因(106)
                    {
                        FenceBreachStatus fenceSt;
                        setPendingFlyModeReason(MODE_CODE_REASON_NONE);
                        if (getFenceStatus(&fenceSt) && fenceSt.active)
                            setPendingFlyModeReason(MODE_CODE_REASON_FENCE);
                    }
                    {
                        double logbuf[3] = {12, (double) AFunc_PosHold, (double) fmr};
                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                    }
                    currentFlightMode = FlightModeFactory::createMode(AFunc_PosHold);
                    break;
                }

                case FMR_OK_RqSwitchMode:
                case FMR_Err_ParamInvalid:
                case FMR_Err_NoPositioning:
                case FMR_Err_NoMission:
                case FMR_Err_NoHomePoint:
                { // 需要切换到其他模式
                    if (mode_cmd && fmr != FMR_OK_RqSwitchMode)
                        msg_handled = FMMR_Failed;
                    {
                        double logbuf[3] = {12, (double) (ModeButtonZone <= 5 ? cfg.Bt1AFunc[ModeButtonZone][0] : AFunc_PosHold), (double) fmr};
                        SDLog_Msg_DebugVect("changemode", logbuf, 3);
                    }
                    if (ModeButtonZone <= 5)
                    {
                        currentFlightMode = FlightModeFactory::createMode(cfg.Bt1AFunc[ModeButtonZone][0]);
                        if (currentFlightMode == nullptr || (currentFlightMode->check(rc, GLB_FlightModeFlags, comVars, &cfg, avCfg) != FMCR_OK))
                            currentFlightMode = FlightModeFactory::createMode(AFunc_PosHold);
                    }
                    else
                        currentFlightMode = FlightModeFactory::createMode(AFunc_PosHold);
                    break;
                }

                case FMR_OK:
                    break;
            }
            if (!mode_cmd && fmmr == FMMR_Success)
                msg_handled = FMMR_Success;
            else if (!mode_cmd && fmmr == FMMR_Failed)
                msg_handled = FMMR_Failed;
        }

        // 根据当前飞行模式更新LED灯语
        if (currentFlightMode != nullptr)
        {
            uint8_t modeId = currentFlightMode->get_modeId();
            if (modeId == AFunc_RTL || modeId == AFunc_Land || modeId == AFunc_RTL2Point)
                setLedMode(LEDMode_FlyingRTL);
            else if (modeId == AFunc_PosHold || modeId == AFunc_Loiter)
            {
                bool pos_ena;
                is_Position_Control_Enabled(&pos_ena);
                if (pos_ena)
                    setLedMode(LEDMode_FlyingManualPos);
                else
                    setLedMode(LEDMode_FlyingManualAlt);
            }
            else if (modeId == AFunc_Manual || modeId == AFunc_Stabilize || modeId == AFunc_Rattitude)
                setLedMode(LEDMode_FlyingManual);
            else
            {
                FlightModeClass fmc = currentFlightMode->get_fmClass();
                if (fmc == FMC_Automatic)
                    setLedMode(LEDMode_FlyingAuto);
                else if (fmc == FMC_SemiAutomatic)
                    setLedMode(LEDMode_FlyingSemiAuto);
                else
                    setLedMode(LEDMode_FlyingManualAlt);
            }
        }

    ModeLoopFin:
        /*返回消息处理结果*/
        if (msg_available && msg_handled != FMMR_Unprocessed)
        {
            ModeMsgAck(msg, msg_handled == FMMR_Success);
        }
        /*返回消息处理结果*/
    }
    // 保存飞行统计
    {
        double logbuf[3] = {8, 0, 0};
        SDLog_Msg_DebugVect("exitmode", logbuf, 3);
    }
    flightStats.flight_time_s[0] += (float) stats_time_accum;
    flightStats.flight_dist_m[0] += (float) stats_dist_accum;
    store_flightStats();
    return MR_OK;
}
