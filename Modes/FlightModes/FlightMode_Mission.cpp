#include "FlightMode_Mission.hpp"

#include "AuxFuncs.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Missions.hpp"
#include "Modes.hpp"
#include "NavCmdProcess.hpp"
#include "Sensors.hpp"
#include "StorageSystem.hpp"
#include "basic.hpp"

FlightModeClass FlightMode_Mission::get_fmClass()
{
    return FMC_Automatic;
}

bool FlightMode_Mission::set_params(double params[], uint16_t params_cnt)
{
    return true;
}

void FlightMode_Mission::send_mission_report()
{
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    {
        const Port *port = get_CommuPort(i);
        if (port && port->write != 0)
        {
            mavlink_message_t msg_sd;
            if (mavlink_lock_chan(i, 0.01))
            {
                MissionItemReport rpt = get_missionItemReport();
                mavlink_msg_mission_report_pack_chan(get_CommulinkSysId(),  // system id
                                                     get_CommulinkCompId(), // component id
                                                     i,                     // chan
                                                     &msg_sd,
                                                     TIME::get_System_Run_Time() * 1e6, // time_usec
                                                     rpt.seq,                           // seq
                                                     rpt.cmd,                           // cmd
                                                     rpt.mission_count,                 // mission_count
                                                     rpt.progress,                      // progress
                                                     rpt.result                         // result
                );
                mavlink_send_msg(port, &msg_sd, 0, 0.01);
                mavlink_unlock_chan(i);
            }
        }
    }
}

FlightModeCheckResult FlightMode_Mission::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    if (get_Position_MSStatus() != MS_Ready)
        return FMCR_Err_NoPositioning;

    return FMCR_OK;
}

// 获取航点类型
bool FlightMode_Mission::get_wp_type(const MissionInf &wp, WP_TYPE *type, WP_TYPE_FLAG *flag)
{
    if (wp.cmd == MAV_CMD_NAV_WAYPOINT)
    {
        if (flag)
            *flag = (WP_TYPE_FLAG) 0;
        switch (wp.frame)
        {
            case MAV_FRAME_GLOBAL:
            case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            case MAV_FRAME_GLOBAL_INT:
            case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            case MAV_FRAME_LOCAL_NED:
            case MAV_FRAME_LOCAL_ENU:
            {
                if (type)
                    *type = WP_TYPE::Normal;
                break;
            }
            case MAV_FRAME_GLOBAL_TERRAIN_ALT:
            case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
            {
                if (type)
                    *type = WP_TYPE::Terrain;
                break;
            }
            default:
                return false;
        }
        if (wp.params[0] > 0.01 || wp.params[1] <= 0)
        {
            if (flag)
                *flag = (WP_TYPE_FLAG) (*flag | WP_TYPE_FLAG_HOLD);
        }
        return true;
    }
    return false;
}

bool FlightMode_Mission::get_next_wp(MissionInf *next_wp, uint16_t *next_wp_ind)
{
    uint16_t rdWpInd = getCurrentMissionInd() + 1;
    while (1)
    {
        // 读取下一个航点
        MissionInf rdMissionInf;
        if (ReadMission(rdWpInd, &rdMissionInf) == false)
        { // 无航点返回失败
            return false;
        }

        if (rdMissionInf.cmd == 177)
        { // 首先检查航点跳转
            if ((doJumpWP == rdWpInd && doJumpCount < rdMissionInf.params[1]) || (doJumpWP != rdWpInd && 0 < rdMissionInf.params[1]))
            {
                rdWpInd = rdMissionInf.params[0] - 1;
                continue;
            }
        }
        else if (check_NavCmd(rdMissionInf.cmd, rdMissionInf.frame, rdMissionInf.params))
        { // 指令可执行
            if (next_wp)
                *next_wp = rdMissionInf;
            if (next_wp_ind)
                *next_wp_ind = rdWpInd;
            return true;
        }
        ++rdWpInd;
    }
}

bool FlightMode_Mission::run_to_next_wp()
{
    while (1)
    {
        if (setCurrentMission(getCurrentMissionInd() + 1))
        {
        doJumpReadMission:
            if (ReadCurrentMission(&current_mission_inf, &current_mission_ind))
            {
                if (current_mission_inf.cmd == 177)
                { // 首先处理航点跳转
                    if (doJumpWP != getCurrentMissionInd())
                    {
                        doJumpWP = getCurrentMissionInd();
                        doJumpCount = 0;
                    }
                    ++doJumpCount;
                    if (doJumpCount > current_mission_inf.params[1])
                    { // 循环次数已达到
                        continue;
                    }
                    else
                    {
                        if (current_mission_inf.params[0] > 0)
                        {
                            if (setCurrentMission(current_mission_inf.params[0] - 1))
                                goto doJumpReadMission;
                            else
                            { // 无航点信息
                                return false;
                            }
                        }
                        else
                        { // 跳转航点错误
                            return false;
                        }
                    }
                }
                else if (Process_InflightCmd(current_mission_inf.cmd, current_mission_inf.params) == false)
                {
                    if (check_NavCmd(current_mission_inf.cmd, current_mission_inf.frame, current_mission_inf.params))
                    { // 指令可执行
                        return true;
                    }
                }
            }
            else
            { // 无航点信息
                return false;
            }
        }
        else
        { // 无航点信息
            return false;
        }
    }
}

FlightMode_Mission::FlightMode_Mission()
    : FlightMode_Base("Mission", AFunc_Mission, 0)
{
    // 复位状态机
    step = 0;
    // 复位航点跳转状态
    doJumpWP = 0;
    doJumpCount = 0;
    // 复位避障状态
    avDistance_filted = -1;
    avDistance_timeout_counter = 0;
    // 复位任务状态
    init_NavCmdInf(&navInf);
}

FlightMode_Mission::~FlightMode_Mission() {}

FlightModeResult FlightMode_Mission::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                               const AvoidanceCfg *avCfg, double h)
{
    // 计算运行频率
    double freq = 1.0 / h;

    // 打开位置控制器
    Altitude_Control_Enable();
    Position_Control_Enable();
    bool pos_ena;
    is_Position_Control_Enabled(&pos_ena);
    if (pos_ena == false)
    { // 位置控制器无法打开返回手动模式
        return FMR_Err_NoPositioning;
    }

    if (cfg->configs[0] & MCfg_NoAutoWithouFixed_Bit)
    { // 无fix不允许自动
        bool getFixed = false;
        Position_Sensor gps_sensor;
        for (uint8_t i = default_gps1_sensor_index; i < default_gps1_sensor_index + default_max_gps_sensors; ++i)
        {
            if (GetPositionSensor(i, &gps_sensor) && gps_sensor.inf.addition_inf[1] == 6)
            {
                getFixed = true;
                break;
            }
        }
        if (!getFixed)
        { // 无FIX返航
            return FMR_Err_RqSwitchRTL;
        }
    }

    if (cfg->configs[0] & MCfg_WpNRcRTL_Bit)
    { // 任务模式没有遥控时返航
        if (!rc.available)
        { // 无FIX返航
            return FMR_Err_RqSwitchRTL;
        }
    }

    // 检测是否在围栏内
    if (avCfg->fenceEnable[0] & FenceEnable_CpxFenceFlag)
    { // 检测围栏
        vector3<double> pos;
        get_Position_Ctrl(&pos);
        vector3<double> tVelVec;
        bool res = is_insideFence(pos, tVelVec, 0);
        if (!res)
        { // 不在围栏内
            FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], true);
            setFenceBreachActive(true, FENCE_BREACH_BOUNDARY, fenceInf.mitigation);
            return fenceInf.modeRs;
        }
    }
    if ((avCfg->fenceMaxAlt[0] > 0.5))
    { // 简单围栏
        double homeZ = 0;
        getHomeLocalZ(&homeZ, 0, 0.01);
        vector3<double> position;
        get_Position_Ctrl(&position);
        double heightAboveGround = position.z - homeZ;

        if ((avCfg->fenceEnable[0] & FenceEnable_SplFenceFlag) || (avCfg->fenceEnable[0] & FenceEnable_CpxFenceFlag))
        {
            if (heightAboveGround > avCfg->fenceMaxAlt[0] * 100)
            { // 超出围栏高度
                FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], true);
                setFenceBreachActive(true, FENCE_BREACH_MAXALT, fenceInf.mitigation);
                return fenceInf.modeRs;
            }
        }

        if ((avCfg->fenceEnable[0] & FenceEnable_SplFenceFlag))
        {
            vector2<double> homeP;
            if (getHomePoint(&homeP))
            {
                vector3<double> pos;
                get_Position_Ctrl(&pos);
                vector3<double> tVelVec;
                // 判断距离围栏边界距离
                double dis = -1;
                bool res = is_insideCircleFence(pos, tVelVec, homeP.x, homeP.y, avCfg->fenceRadius[0] * 100, &dis);
                if (res == false)
                { // 在围栏外
                    FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], true);
                    setFenceBreachActive(true, FENCE_BREACH_BOUNDARY, fenceInf.mitigation);
                    return fenceInf.modeRs;
                }
            }
        }
    }

    switch (step)
    {
        case 0:
        { // 刹车等待进入模式
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            Attitude_Control_set_YawLock();

            // 上报模式状态 - 执行航点任务
            setCurrentFlyMode(AFunc_Mission, MODE_CODE_REASON_APP);

            // 复位拍照间距
            CamTriggDist = 0;
            // 复位水泵
            setPump1(PumpOffAutoPercent);

            extern bool inMissionFlight;
            inMissionFlight = false;

            // Reset mission report
            { // scoped for set_missionItemReport
                MissionItemReport rpt = {};
                rpt.seq = 0;
                rpt.cmd = 0;
                rpt.mission_count = getMissionsCount();
                rpt.progress = 0;
                rpt.result = ACFLY_MISSION_REPORT_EXECUTING;
                set_missionItemReport(rpt);
            }

            // Wait for braking to complete
            Position_ControlMode alt_mode, pos_mode;
            get_Altitude_ControlMode(&alt_mode);
            get_Position_ControlMode(&pos_mode);
            if (alt_mode == Position_ControlMode_Position && pos_mode == Position_ControlMode_Position)
            { // 刹车完成
                ++navInf.counter2;
                // 等待1秒再进入任务飞行
                if (navInf.counter2 >= 1 * freq)
                {
                    if (ReadCurrentMission(&current_mission_inf, &current_mission_ind))
                    { // 载入下一航点成功
                        // 初始化任务信息
                        init_NavCmdInf(&navInf);
                        if (current_mission_ind == restoreWpInf.CurrentWp[0])
                        { // 恢复航线飞行
                        }
                        else
                        { // 当前和记录的航点不一致 直接前往当前行点
                            // 复位当前航点信息
                            reset_RestoreWpInf(restoreWpInf);
                            storeRAM_restoreWpInf();
                        }
                    }
                    else
                    { // 获取不到航点信息
                        // 先试着把航点设置为首个
                        setCurrentMission(0);
                        // 复位当前航点信息
                        reset_RestoreWpInf(restoreWpInf);
                        storeRAM_restoreWpInf();
                        if (ReadCurrentMission(&current_mission_inf, &current_mission_ind))
                        { // 载入下一航点成功
                        }
                        else
                        { // 无航点信息返回手动模式
                            return FMR_Err_NoMission;
                        }
                    }

                    if (current_mission_ind == 0)
                    { // 航点为首个时首先排除航线头指令和不能识别的航点
                        reset_RestoreWpInf(restoreWpInf);
                        storeRAM_restoreWpInf();

                        while (1)
                        {
                            if (Process_InflightCmd(current_mission_inf.cmd, current_mission_inf.params))
                            { // 尝试执行指令成功
                                if (setCurrentMission(getCurrentMissionInd() + 1))
                                {
                                    if (ReadCurrentMission(&current_mission_inf, &current_mission_ind))
                                    {
                                        continue;
                                    }
                                    else
                                    { // 无航点信息返回手动模式
                                        setCurrentMission(0);
                                        return FMR_Err_NoMission;
                                    }
                                }
                                else
                                { // 无航点信息返回手动模式
                                    setCurrentMission(0);
                                    return FMR_Err_NoMission;
                                }
                            }
                            else
                            { // 尝试排除无效指令
                                if (check_NavCmd(current_mission_inf.cmd, current_mission_inf.frame, current_mission_inf.params))
                                { // 指令可执行退出
                                    restoreWpInf.CurrentWp[0] = current_mission_ind;
                                    storeRAM_restoreWpInf();
                                    break;
                                }
                                else
                                { // 命令不可执行
                                    // 跳过
                                    if (setCurrentMission(getCurrentMissionInd() + 1))
                                    {
                                        if (ReadCurrentMission(&current_mission_inf, &current_mission_ind))
                                        {
                                            continue;
                                        }
                                        else
                                        { // 无航点信息返回手动模式
                                            setCurrentMission(0);
                                            return FMR_Err_NoMission;
                                        }
                                    }
                                    else
                                    { // 无航点信息返回手动模式
                                        setCurrentMission(0);
                                        return FMR_Err_NoMission;
                                    }
                                }
                            }
                        }
                    }

                    // Report first mission item as STARTED
                    { // scoped for set_missionItemReport
                        MissionItemReport rpt = {};
                        rpt.seq = current_mission_ind;
                        rpt.cmd = current_mission_inf.cmd;
                        rpt.mission_count = getMissionsCount();
                        rpt.progress = 0;
                        rpt.result = ACFLY_MISSION_REPORT_STARTED;
                        set_missionItemReport(rpt);
                    }
                    inMissionFlight = true;

                    // Enter approach state
                    if (current_mission_inf.cmd != MAV_CMD_NAV_WAYPOINT)
                    { // 不需要恢复直接进入任务飞行
                        step = 100;
                    }
                    else
                    { // 进入入航状态
                        step = 10;
                    }
                    // 复位航点跳转状态
                    doJumpWP = 0;
                    doJumpCount = 0;
                    // 初始化航线预存状态
                    prestored = false;
                    // 初始化任务信息
                    ms_step = 0;
                    init_NavCmdInf(&navInf);
                }
            }
            else
                navInf.counter2 = 0;
            break;
        } // 入航
        case 10:
        { // 未起飞先起飞
            if (restoreWpInf.line_fs < 0)
                restoreWpInf.line_fs = 0;
            bool inFlight;
            get_is_inFlight(&inFlight);
            if (inFlight)
            {
                CamTriggDist = restoreWpInf.CamTrigDist;
                ++step;
            }
            else
            {
                Position_Control_set_XYLock();
                Position_Control_set_TargetVelocityZ(50);
            }
            break;
        }
        case 11:
        { // 高度调整
            // 锁定xy
            Position_Control_set_XYLock();
            // 求Z偏移距离
            double z_offset = 0;
            // 求恢复航线长度
            double AB_length = safe_sqrt(sq(restoreWpInf.line_x) + sq(restoreWpInf.line_y) + sq(restoreWpInf.line_z));
            if (AB_length > 0.1)
            {
                double inv_AB_length = 1.0 / AB_length;
                double line_fs;
                if (restoreWpInf.line_fs > 0)
                    line_fs = restoreWpInf.line_fs;
                else
                    line_fs = 0;
                double ufs = AB_length - line_fs;
                if (ufs < 0)
                    ufs = 0;
                z_offset += ufs * restoreWpInf.line_z * inv_AB_length;
            }
            double params[7] = {0};
            params[3] = std::nan("");
            params[6] = current_mission_inf.params[6] + z_offset * 0.01;
            vector3<double> OC;
            NavCmd16_WAYPOINT_GetInfo(current_mission_inf.frame, params, 0, &OC, 0);
            // 执行结果
            if (OC.z > 0)
            {
                wpRNeedDecent = false;
                bool res = false;
                switch (current_mission_inf.frame)
                {
                    case MAV_FRAME_GLOBAL_INT:
                    case MAV_FRAME_GLOBAL:
                    {
                        res = Position_Control_set_TargetPositionZGlobal(current_mission_inf.params[6] * 100 + z_offset, 0);
                        break;
                    }
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    {
                        res = Position_Control_set_TargetPositionZRA(current_mission_inf.params[6] * 100 + z_offset, 0);
                        break;
                    }
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    {
                        Position_Control_set_ZLock();
                        res = true;
                        break;
                    }

                    case MAV_FRAME_LOCAL_NED:
                        res = Position_Control_set_TargetPositionZ(-(current_mission_inf.params[6] * 100 + z_offset), 0);
                        break;

                    case MAV_FRAME_LOCAL_ENU:
                        res = Position_Control_set_TargetPositionZ(current_mission_inf.params[6] * 100 + z_offset, 0);
                        break;

                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    case MAV_FRAME_LOCAL_OFFSET_NED:
                        res = Position_Control_set_TargetPositionZRelative(-(current_mission_inf.params[6] * 100 + z_offset), 0);
                        break;

                    case MAV_FRAME_BODY_FLU:
                    {
                        res = Position_Control_set_TargetPositionZRelative(current_mission_inf.params[6] * 100 + z_offset, 0);
                        break;
                    }

                    default:
                        return FMR_Err_ParamInvalid;
                }

                if (res)
                    ++step;
                else
                    return FMR_Err_ParamInvalid;
            }
            else
            {
                wpRNeedDecent = true;
                step += 2;
            }
            break;
        }
        case 12:
        { // 等待高度调整完成
            // 锁定xy
            Position_Control_set_XYLock();
            Position_ControlMode alt_mode;
            get_Altitude_ControlMode(&alt_mode);
            if (alt_mode == Position_ControlMode_Position)
                ++step;
            break;
        }
        case 13:
        { // 旋转偏航

            // 锁定XYZ
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            // 求恢复航线长度
            double AB_length = safe_sqrt(sq(restoreWpInf.line_x) + sq(restoreWpInf.line_y) + sq(restoreWpInf.line_z));
            // 求AB向量长度倒数
            double offset_x = 0, offset_y = 0;
            if (AB_length > 0.1)
            {
                double inv_AB_length = 1.0 / AB_length;
                double line_fs;
                if (restoreWpInf.line_fs > 0)
                    line_fs = restoreWpInf.line_fs;
                else
                    line_fs = 0;
                double ufs = AB_length - line_fs;
                if (ufs < 0)
                    ufs = 0;
                offset_x = ufs * restoreWpInf.line_x * inv_AB_length;
                offset_y = ufs * restoreWpInf.line_y * inv_AB_length;
            }

            double LA, LB;
            switch (current_mission_inf.frame)
            {
                case MAV_FRAME_GLOBAL:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                case MAV_FRAME_GLOBAL_INT:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                { // 全球定位
                    if (current_mission_inf.params[4] < -90 || current_mission_inf.params[4] > 90 || current_mission_inf.params[5] < -180 || current_mission_inf.params[5] > 180)
                    { // 经纬度为不正确不转偏航
                        step += 2;
                        return FMR_OK;
                    }

                    // 获取最优全球定位传感器信息
                    PosSensorHealthInf2 global_inf;
                    if (get_OptimalGlobal_XY(&global_inf) == false)
                    {
                        return FMR_Err_NoMission;
                    }
                    // 获取指定经纬度平面坐标
                    double x, y;
                    map_projection_project(&global_inf.mp, current_mission_inf.params[4], current_mission_inf.params[5], &x, &y);
                    x -= global_inf.HOffset.x;
                    y -= global_inf.HOffset.y;
                    x += offset_x;
                    y += offset_y;
                    LA = y - global_inf.PositionENU.y;
                    LB = x - global_inf.PositionENU.x;
                    break;
                }

                case MAV_FRAME_LOCAL_NED:
                {
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    double x, y;
                    x = current_mission_inf.params[5] * 100;
                    y = current_mission_inf.params[4] * 100;
                    x += offset_x;
                    y += offset_y;
                    LA = y - position.y;
                    LB = x - position.x;
                    break;
                }

                case MAV_FRAME_LOCAL_ENU:
                {
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    double x, y;
                    x = current_mission_inf.params[4] * 100;
                    y = current_mission_inf.params[5] * 100;
                    x += offset_x;
                    y += offset_y;
                    LA = y - position.y;
                    LB = x - position.x;
                    break;
                }

                default:
                    return FMR_Err_ParamInvalid;
            }

            if (sq(LA) + sq(LB) > sq(500))
                Attitude_Control_set_Target_Yaw(atan2(LA, LB));
            ++step;

            break;
        }
        case 14:
        { // 等待偏航旋转开始航点飞行
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            // 求恢复航线长度
            double AB_length = safe_sqrt(sq(restoreWpInf.line_x) + sq(restoreWpInf.line_y) + sq(restoreWpInf.line_z));
            // 获取是否旋转完成
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                // 求AB向量长度倒数
                double offset_x = 0, offset_y = 0;
                if (AB_length > 0.1)
                {
                    double inv_AB_length = 1.0 / AB_length;
                    double line_fs;
                    if (restoreWpInf.line_fs > 0)
                        line_fs = restoreWpInf.line_fs;
                    else
                        line_fs = 0;
                    double ufs = AB_length - line_fs;
                    if (ufs < 0)
                        ufs = 0;
                    offset_x = ufs * restoreWpInf.line_x * inv_AB_length;
                    offset_y = ufs * restoreWpInf.line_y * inv_AB_length;
                }
                double Tx, Ty;
                switch (current_mission_inf.frame)
                {
                    case MAV_FRAME_GLOBAL:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    case MAV_FRAME_GLOBAL_INT:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    { // 全球定位
                        if (current_mission_inf.params[4] < -90 || current_mission_inf.params[4] > 90 || current_mission_inf.params[5] < -180 || current_mission_inf.params[5] > 180)
                        { // 经纬度为不正确退出
                            return FMR_Err_ParamInvalid;
                        }

                        // 获取最优全球定位传感器信息
                        PosSensorHealthInf2 global_inf;
                        if (get_OptimalGlobal_XY(&global_inf) == false)
                        {
                            return FMR_Err_NoPositioning;
                        }
                        // 获取指定经纬度平面坐标
                        double x, y;
                        map_projection_project(&global_inf.mp, current_mission_inf.params[4], current_mission_inf.params[5], &x, &y);
                        x -= global_inf.HOffset.x;
                        y -= global_inf.HOffset.y;
                        x += offset_x;
                        y += offset_y;
                        Tx = x;
                        Ty = y;
                        break;
                    }

                    case MAV_FRAME_LOCAL_NED:
                    {
                        double x, y;
                        x = current_mission_inf.params[5] * 100;
                        y = current_mission_inf.params[4] * 100;
                        x += offset_x;
                        y += offset_y;
                        Tx = x;
                        Ty = y;
                        break;
                    }

                    case MAV_FRAME_LOCAL_ENU:
                    {
                        double x, y;
                        x = current_mission_inf.params[4] * 100;
                        y = current_mission_inf.params[5] * 100;
                        x += offset_x;
                        y += offset_y;
                        Tx = x;
                        Ty = y;
                        break;
                    }

                    default:
                        return FMR_Err_ParamInvalid;
                }

                // 复位避障状态
                avDistance_filted = -1;
                avDistance_timeout_counter = 0;
                // 开始入航航线
                bool res = Position_Control_set_TargetPositionXY(Tx, Ty, 0);
                if (res)
                    ++step;
                else
                    return FMR_Err_ParamInvalid;
            }
            break;
        }
        case 15:
        { // 等待航线飞行完成

            // 获取航线飞行状态
            Position_ControlMode alt_mode, pos_mode;
            double frontDir;
            vector3<double> frontVec;
            get_Altitude_ControlMode(&alt_mode);
            get_Position_ControlMode(&pos_mode, &frontDir, &frontVec);

            // 计算避障
            // 前方障碍物距离
            double AvDistance = -1;
            // 航线最大飞行位置
            double AvDist = -1;
            if (Is_AutoMode(pos_mode))
            { // 航线避障
                if (AvMode_Enabled(avCfg->AvoidanceMode[0]))
                { // 需要进行避障
                    get_AvLineDistanceEnu(&AvDistance, frontVec, avCfg->wheelbase[0]);
                    if (AvDistance >= 0)
                    {
                        // 对障碍物距离进行滤波
                        if (avDistance_filted < 0)
                            avDistance_filted = AvDistance;
                        if (AvDistance < avDistance_filted)
                            avDistance_filted = AvDistance;
                        else
                            avDistance_filted += avDistance_filter_P * h * (AvDistance - avDistance_filted);
                    }
                    else if (avDistance_filted >= 0)
                    { // 无障碍物
                        if (++avDistance_timeout_counter >= avDistance_timeout_T * freq)
                        {
                            avDistance_filted = -1;
                        }
                    }

                    // 计算航线刹车点
                    if (avDistance_filted >= 0)
                    {
                        AvDist = avDistance_filted - (avCfg->AvoidanceDist[0] + 0.5f * avCfg->wheelbase[0]);
                        if (AvDist < 0)
                            AvDist = 0;
                    }
                }

                // 上报航线避障刹车悬停(触发条件与PosHold avLocked一致: v^2 >= 2as)
                // 必须在仿地限速缩小AvDist之前执行
                if (AvDist >= 0 && frontVec.get_square() > 0.001)
                {
                    vector3<double> dir = frontVec;
                    dir.normalize();
                    vector3<double> vel;
                    get_VelocityENU_Ctrl(&vel);
                    double velRoute = vel * dir;
                    setObstacleAvoidanceHoverActive(sq(velRoute) >= 2 * getPosCtrlCfg()->maxAutoAccXY[0] * AvDist);
                }
                else
                    setObstacleAvoidanceHoverActive(false);
            }
            else
                setObstacleAvoidanceHoverActive(false);

            bool terrain_mode = false;
            if (current_mission_inf.frame == MAV_FRAME_GLOBAL_TERRAIN_ALT || current_mission_inf.frame == MAV_FRAME_GLOBAL_TERRAIN_ALT_INT)
            {
                terrain_mode = true;

                bool thrNeutral = true;
                if (rc.available && !in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
                    thrNeutral = false;

                if (thrNeutral)
                {
                    PosSensorHealthInf1 pos_inf;
                    get_OptimalRange_Z(&pos_inf);
                    Position_Sensor_Data sensor;
                    if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(current_mission_inf.params[6]))
                    { // 根据测高传感器调整对地高度
                        vector3<double> pos;
                        get_Position_Ctrl(&pos);
                        double target_height;
                        target_height = current_mission_inf.params[6] * 100;

                        double err = target_height - sensor.position.z;
                        Position_Control_set_TargetVelocityZ(constrain(err, 200.0));

                        // 根据对地高度限速
                        double maxFrontPointL = -1;
                        double abs_err = fabs(err);

                        double errLimit = getPosCtrlCfg()->maxVelXY[0] - 3 * (abs_err - 50);
                        double groundLimit = 1.0 * (sensor.position.z - 20);
                        if (groundLimit < 20)
                            groundLimit = 20;

                        if (errLimit < groundLimit)
                            maxFrontPointL = errLimit;
                        else
                            maxFrontPointL = groundLimit;

                        if (maxFrontPointL >= 0 && (AvDist < 0 || maxFrontPointL < AvDist))
                            AvDist = maxFrontPointL;
                    }
                    else
                    { // 无高度传感器锁定高度
                        Position_Control_set_ZLock();
                    }
                }
                else
                {
                    double thr_stick = remove_deadband(rc.data[0] - 50.0, (double) cfg->NeutralZone[0]);
                    if (thr_stick > 0)
                        thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
                    else
                        thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
                    Position_Control_set_TargetVelocityZ(thr_stick);
                }
            }
            else
            { // 不需要仿地
                Position_Control_set_ZLock();
            }

            // 设置避障距离
            Position_Control_set_RouteLineAvoidanceRelative(AvDist);

            if (pos_mode == Position_ControlMode_Position)
            { // 已成功移动到上次航线位置
                Position_Control_set_XYZAutoSpeed(restoreWpInf.vel);
                step = 100;
            }
            break;
        }

        // 航线飞行
        case 100:
        {
            // Hold SUCCESS/FAILURE result so Commulink thread can send it
            if (report_hold_counter > 0)
            {
                --report_hold_counter;
                if (report_hold_counter % 2 == 0)
                    send_mission_report();
            }
            else
            {
                MissionItemReport rpt = get_missionItemReport();
                // Reset progress to 0 by default (non-waypoint commands have zero-length vectors)
                rpt.progress = 0;
                // Compute progress from RestoreWpInf line flight path
                if (restoreWpInf.line_fs >= 0)
                {
                    double lineLen = safe_sqrt(sq(restoreWpInf.line_x) + sq(restoreWpInf.line_y) + sq(restoreWpInf.line_z));
                    if (lineLen > 0.01)
                    {
                        double pct = (restoreWpInf.line_fs / lineLen) * 100.0;
                        if (pct < 0)
                            pct = 0;
                        if (pct > 100)
                            pct = 100;
                        rpt.progress = (uint8_t) pct;
                    }
                }
                rpt.seq = current_mission_ind;
                rpt.cmd = current_mission_inf.cmd;
                rpt.mission_count = getMissionsCount();
                rpt.result = ACFLY_MISSION_REPORT_EXECUTING;
                set_missionItemReport(rpt);
            }

            NAV_CMD_RS res = NCR_Err_ParamInvalid;
            switch (ms_step)
            {
                case 0:
                { // 未进行连续任务
                    // 复位连续任务飞行距离
                    traveledDist = 0;
                    if (current_mission_inf.cmd == 177)
                    { // 执行航点跳转

                        // 上报模式状态
                        setCurrentFlyMode(AFunc_Mission, MODE_CODE_REASON_APP);

                        if (doJumpWP != getCurrentMissionInd())
                        {
                            doJumpWP = getCurrentMissionInd();
                            doJumpCount = 0;
                        }
                        ++doJumpCount;
                        if (doJumpCount > current_mission_inf.params[1])
                            res = NCR_Fin;
                        else
                        {
                            if (current_mission_inf.params[0] > 0)
                                res = (NAV_CMD_RS) (current_mission_inf.params[0] - 1);
                            else
                                res = (NAV_CMD_RS) 0;
                        }
                    }
                    else if (Process_InflightCmd(current_mission_inf.cmd, current_mission_inf.params) == false)
                    { // 尝试执行指令失败-尝试执行航点任务
                        WP_TYPE n_wp_type;
                        WP_TYPE_FLAG n_wp_type_flag;
                        if (get_wp_type(current_mission_inf, &n_wp_type, &n_wp_type_flag))
                        {
                            // 反馈可以接受指令
                            res = NCR_Processing_CmdManageable;
                            // 上报模式状态
                            setCurrentFlyMode(AFunc_Mission, MODE_CODE_REASON_APP);
                            // 初始化任务信息
                            wp_type = n_wp_type;
                            init_NavCmdInf(&navInf);
                            ms_step = 1;
                            if (!(n_wp_type_flag & WP_TYPE_FLAG_HOLD))
                            {
                                // 进入连续任务
                                wait_quit = false;
                            }
                            else
                            {
                                wait_quit = true;
                            }
                        }
                        else
                        { // 不进行连续任务
                            // 直接执行指令
                            res = Process_NavCmd(current_mission_inf.cmd, freq, current_mission_inf.frame, current_mission_inf.params, &navInf, rc, flag, comVars, cfg, avCfg);
                        }
                    }
                    break;
                }

                    // 连续任务
                case 1:
                { // 旋转偏航
                    Position_Control_set_XYLock();
                    Position_Control_set_ZLock();
                    Attitude_Control_set_YawLock();

                    if (isvalid(current_mission_inf.params[3]) == false)
                    { // 机头指向航点方向

                        // 获取指向下一航点向量
                        vector3<double> OC;
                        NavCmd16_WAYPOINT_GetInfo(current_mission_inf.frame, current_mission_inf.params, 0, &OC, 0);

                        if (sq(OC.x) + sq(OC.y) > sq(100))
                            Attitude_Control_set_Target_Yaw(atan2(OC.y, OC.x));
                    }
                    else if (current_mission_inf.params[3] > -360 && current_mission_inf.params[3] < 360)
                    { // 指定偏航朝向
                        Attitude_Control_set_Target_Yaw(degree2rad(90 - current_mission_inf.params[3]));
                    }

                    // 反馈可以接受指令
                    res = NCR_Processing_CmdManageable;
                    // 进入下一状态
                    ++ms_step;
                    break;
                }
                case 2:
                { // 等待偏航旋转完成
                    Position_Control_set_XYLock();
                    Position_Control_set_ZLock(); // 等待偏航旋转完成
                    double yawTrackErr;
                    Attitude_Control_get_YawTrackErr(&yawTrackErr);
                    if (yawTrackErr < 0.01)
                        ++ms_step;
                    // 反馈可以接受指令
                    res = NCR_Processing_CmdManageable;
                    break;
                }
                case 3:
                { // 先执行第一条航线并尝试再预存一条航线

                    // 求下一个航点坐标
                    vector3<double> pC;
                    NavCmd16_WAYPOINT_GetInfo(current_mission_inf.frame, current_mission_inf.params, 0, 0, &pC);
                    // 执行航线(不预存)
                    switch (wp_type)
                    {
                        case WP_TYPE::Normal:
                        {
                            Position_Control_set_TargetPositionXYZ(pC.x, pC.y, pC.z, 0, MissionPrestoreMode_Null);
                            break;
                        }
                        case WP_TYPE::Terrain:
                        {
                            Position_Control_set_TargetPositionXY(pC.x, pC.y, 0, MissionPrestoreMode_Null);
                            break;
                        }
                    }

                    if (!wait_quit)
                    {
                        Position_Control_set_TurnRaius(current_mission_inf.params[1] * 100);

                        // 获取下一段航线预存
                        MissionInf next_wp;
                        uint16_t next_wp_ind;
                        WP_TYPE n_wp_type;
                        WP_TYPE_FLAG n_wp_type_flag;
                        if (get_next_wp(&next_wp, &next_wp_ind) && get_wp_type(next_wp, &n_wp_type, &n_wp_type_flag) && n_wp_type == wp_type)
                        {
                            // 求下一个航点坐标
                            vector3<double> pC;
                            NavCmd16_WAYPOINT_GetInfo(next_wp.frame, next_wp.params, 0, 0, &pC);
                            // 执行航线(预存)
                            switch (wp_type)
                            {
                                case WP_TYPE::Normal:
                                {
                                    Position_Control_set_TargetPositionXYZ(pC.x, pC.y, pC.z, 0, MissionPrestoreMode_PrestoreOrSet, &prestored);
                                    break;
                                }
                                case WP_TYPE::Terrain:
                                {
                                    Position_Control_set_TargetPositionXY(pC.x, pC.y, 0, MissionPrestoreMode_PrestoreOrSet, &prestored);
                                    break;
                                }
                            }
                            Position_Control_set_TurnRaius(next_wp.params[1] * 100);
                            // 如果是这点需要hold则准备退出
                            if (n_wp_type_flag & WP_TYPE_FLAG_HOLD)
                                wait_quit = true;
                        }
                    }
                    else
                        prestored = false;

                    // 反馈可以接受指令
                    res = NCR_Processing_CmdManageable;
                    // 进入下一状态
                    ++ms_step;
                    break;
                }
                case 4:
                { // 执行连续任务
                    // 获取航线飞行状态
                    Position_ControlMode alt_mode, pos_mode;
                    double frontDir;
                    vector3<double> frontVec;
                    get_Altitude_ControlMode(&alt_mode);
                    get_Position_ControlMode(&pos_mode, &frontDir, &frontVec);

                    if (CamTriggDist > 0)
                    { // 积分求飞行路程
                        vector3<double> vel;
                        get_VelocityENU_Ctrl(&vel);
                        traveledDist += safe_sqrt(vel.get_square()) * h;
                    }
                    else
                    {
                        navInf.temp[3] = 0;
                        traveledDist = 0;
                    }

                    if (CamTriggDist > 0)
                    {
                        if (traveledDist >= 0)
                        {
                            int mult = (int) (traveledDist / CamTriggDist) + 1;
                            if (mult > navInf.temp[3])
                            {
                                AuxCamTakePhotoAsync();
                                navInf.temp[3] = mult;
                            }
                        }
                    }
                    // 存储飞行状态
                    if (pos_mode == Position_ControlMode_RouteLine3D || pos_mode == Position_ControlMode_RouteLine)
                    { // 直线飞行
                        vector3<double> line_AB;
                        double flightDistance = -1;
                        if (Position_Control_get_LineFlightABDistance(&line_AB, &flightDistance))
                        {
                            restoreWpInf.CurrentWp[0] = current_mission_ind;
                            restoreWpInf.line_x = line_AB.x;
                            restoreWpInf.line_y = line_AB.y;
                            restoreWpInf.line_z = line_AB.z;
                            restoreWpInf.line_fs = flightDistance;
                            restoreWpInf.wpR = current_mission_inf.params[1] * 100;
                            restoreWpInf.CamTrigDist = CamTriggDist;
                            double vel;
                            Position_Control_get_XYZAutoSpeed(&vel);
                            restoreWpInf.vel = vel;
                            storeRAM_restoreWpInf();
                        }
                    }

                    // 调整航向
                    if (isvalid(current_mission_inf.params[3]) == false)
                    {
                        switch (pos_mode)
                        {
                            case Position_ControlMode_RouteLine:
                            case Position_ControlMode_RouteLine3D:
                            case Position_ControlMode_MissionEndArc3D:
                            case Position_ControlMode_MissionEndArc:
                            {
                                if (sq(frontVec.x) + sq(frontVec.y) > sq(100))
                                    Attitude_Control_set_Target_Yaw(frontDir);
                                break;
                            }
                            default:
                            {
                                break;
                            }
                        }
                    }

                    // 计算避障
                    // 前方障碍物距离
                    double AvDistance = -1;
                    // 航线最大飞行位置
                    double AvDist = -1;
                    if (Is_AutoMode(pos_mode))
                    { // 航线避障
                        if (AvMode_Enabled(avCfg->AvoidanceMode[0]))
                        { // 需要进行避障
                            get_AvLineDistanceEnu(&AvDistance, frontVec, avCfg->wheelbase[0]);
                            if (AvDistance >= 0)
                            {
                                // 对障碍物距离进行滤波
                                if (avDistance_filted < 0)
                                    avDistance_filted = AvDistance;
                                if (AvDistance < avDistance_filted)
                                    avDistance_filted = AvDistance;
                                else
                                    avDistance_filted += avDistance_filter_P * h * (AvDistance - avDistance_filted);
                            }
                            else if (avDistance_filted >= 0)
                            { // 无障碍物
                                if (++avDistance_timeout_counter >= avDistance_timeout_T * freq)
                                {
                                    avDistance_filted = -1;
                                }
                            }

                            // 计算航线刹车点
                            if (avDistance_filted >= 0)
                            {
                                AvDist = avDistance_filted - (avCfg->AvoidanceDist[0] + 0.5f * avCfg->wheelbase[0]);
                                if (AvDist < 0)
                                    AvDist = 0;
                            }
                        }

                        // 上报航线避障刹车悬停(触发条件与PosHold avLocked一致: v^2 >= 2as)
                        // 必须在仿地限速缩小AvDist之前执行
                        if (AvDist >= 0 && frontVec.get_square() > 0.001)
                        {
                            vector3<double> dir = frontVec;
                            dir.normalize();
                            vector3<double> vel;
                            get_VelocityENU_Ctrl(&vel);
                            double velRoute = vel * dir;
                            setObstacleAvoidanceHoverActive(sq(velRoute) >= 2 * getPosCtrlCfg()->maxAutoAccXY[0] * AvDist);
                        }
                        else
                            setObstacleAvoidanceHoverActive(false);
                    }
                    else
                        setObstacleAvoidanceHoverActive(false);

                    // 仿地高度调整
                    if (wp_type == WP_TYPE::Terrain)
                    {
                        bool thrNeutral = true;
                        if (rc.available && !in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
                            thrNeutral = false;
                        if (thrNeutral)
                        {
                            PosSensorHealthInf1 pos_inf;
                            get_OptimalRange_Z(&pos_inf);
                            Position_Sensor_Data sensor;
                            if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(current_mission_inf.params[6]))
                            { // 根据测高传感器调整对地高度
                                vector3<double> pos;
                                get_Position_Ctrl(&pos);
                                double target_height;
                                if (isvalid(current_mission_inf.params[6]) && current_mission_inf.params[6] * 100 > 5)
                                    target_height = current_mission_inf.params[6] * 100;
                                else if (navInf.temp[0] > 5)
                                    target_height = navInf.temp[0];
                                else
                                    target_height = 500;

                                // 计算当前等效高度
                                double current_height;
                                if (AvDistance < 0)
                                { // 前向距离不可用
                                    current_height = sensor.position.z;
                                }
                                else
                                { // 使用前向和下向最小距离
                                    if (AvDistance < sensor.position.z)
                                        current_height = AvDistance;
                                    else
                                        current_height = sensor.position.z;
                                }

                                // 计算目标速度
                                double err = target_height - current_height;
                                double maxVel;
                                if (err > 0)
                                    maxVel = getPosCtrlCfg()->maxAutoVelUp[0] * 0.7;
                                else
                                    maxVel = getPosCtrlCfg()->maxAutoVelDown[0] * 0.7;
                                Position_Control_set_TargetVelocityZ(constrain(err, 150.0));

                                // 根据对地高度限速
                                double maxFrontPointL = -1;
                                double abs_err = fabs(err);
                                double errLimit = -1;
                                double errLimitRange = target_height * 0.1;
                                if (abs_err > errLimitRange + 0.1)
                                {
                                    errLimit = errLimitRange / (abs_err - errLimitRange);
                                    if (errLimit < 0.3)
                                        errLimit = 0;
                                    else
                                        errLimit *= 5;
                                }
                                double groundLimit = 1.0 * (current_height - 20);
                                if (groundLimit < 20)
                                    groundLimit = 20;

                                if (errLimit >= 0 && errLimit < groundLimit)
                                    maxFrontPointL = errLimit;
                                else
                                    maxFrontPointL = groundLimit;

                                if (maxFrontPointL >= 0 && (AvDist < 0 || maxFrontPointL < AvDist))
                                    AvDist = maxFrontPointL;
                            }
                            else
                            { // 无高度传感器锁定高度
                                Position_Control_set_ZLock();
                            }
                        }
                    }

                    // 设置避障距离
                    Position_Control_set_RouteLineAvoidanceRelative(AvDist);

                    if (wait_quit)
                    { // 等待任务退出
                        // 反馈可以接受指令
                        res = NCR_Processing_CmdManageable;

                        if (prestored)
                        { // 已预存航线
                            if (get_prestoreMissionState() == false)
                            { // 预存航点已飞完

                                // 直接运行到下一个航点(这个航点应当已预存)
                                if (run_to_next_wp())
                                {
                                    // 保存任务信息
                                    vector3<double> BA;
                                    NavCmd16_WAYPOINT_GetInfo(current_mission_inf.frame, current_mission_inf.params, 0, &BA, 0);
                                    restoreWpInf.CurrentWp[0] = current_mission_ind;
                                    restoreWpInf.line_x = -BA.x;
                                    restoreWpInf.line_y = -BA.y;
                                    restoreWpInf.line_z = -BA.z;
                                    restoreWpInf.line_fs = 0;
                                    restoreWpInf.CamTrigDist = CamTriggDist;
                                    double vel;
                                    Position_Control_get_XYZAutoSpeed(&vel);
                                    restoreWpInf.vel = vel;
                                    storeRAM_restoreWpInf();

                                    // 更新预存状态
                                    prestored = false;
                                }
                            }
                        }
                        else if (pos_mode == Position_ControlMode_Position)
                        { // 任务结束
                            // hold
                            if (current_mission_inf.params[0] > 0.01)
                                ++ms_step;
                            else
                            { // 退出连续任务
                                ms_step = 0;
                                res = NCR_Fin;
                            }
                        }
                    }
                    else if (get_prestoreMissionState() == false)
                    { // 预存航点已飞完

                        // 反馈可以接受指令
                        res = NCR_Processing_CmdManageable;

                        if (!prestored && pos_mode == Position_ControlMode_Position)
                        { // 航点重合或已到点导致预存失败，直接推进到下一个可执行航点
                            if (run_to_next_wp())
                            {
                                prestored = false;
                                wait_quit = false;
                                ms_step = 3;
                            }
                            else
                            {
                                ms_step = 0;
                                res = NCR_Fin;
                            }
                            break;
                        }

                        // 如果之前已预存则直接运行到下一个航点(这个航点应当已预存)
                        if (prestored)
                            run_to_next_wp();

                        // 保存任务信息
                        vector3<double> BA;
                        NavCmd16_WAYPOINT_GetInfo(current_mission_inf.frame, current_mission_inf.params, 0, &BA, 0);
                        restoreWpInf.CurrentWp[0] = current_mission_ind;
                        restoreWpInf.line_x = -BA.x;
                        restoreWpInf.line_y = -BA.y;
                        restoreWpInf.line_z = -BA.z;
                        restoreWpInf.line_fs = 0;
                        restoreWpInf.CamTrigDist = CamTriggDist;
                        double vel;
                        Position_Control_get_XYZAutoSpeed(&vel);
                        restoreWpInf.vel = vel;
                        storeRAM_restoreWpInf();

                        // 获取下一段航线预存
                        MissionInf next_wp;
                        uint16_t next_wp_ind;
                        WP_TYPE n_wp_type;
                        WP_TYPE_FLAG n_wp_type_flag;
                        if (get_next_wp(&next_wp, &next_wp_ind) && get_wp_type(next_wp, &n_wp_type, &n_wp_type_flag) && n_wp_type == wp_type)
                        {
                            // 求下一个航点坐标
                            vector3<double> pC;
                            NavCmd16_WAYPOINT_GetInfo(next_wp.frame, next_wp.params, 0, 0, &pC);
                            // 执行航线(预存)
                            switch (wp_type)
                            {
                                case WP_TYPE::Normal:
                                {
                                    Position_Control_set_TargetPositionXYZ(pC.x, pC.y, pC.z, 0, MissionPrestoreMode_Prestore, &prestored);
                                    break;
                                }
                                case WP_TYPE::Terrain:
                                {
                                    Position_Control_set_TargetPositionXY(pC.x, pC.y, 0, MissionPrestoreMode_Prestore, &prestored);
                                    break;
                                }
                            }
                            Position_Control_set_TurnRaius(next_wp.params[1] * 100);
                            // 如果是这点需要hold则准备退出
                            if (n_wp_type_flag & WP_TYPE_FLAG_HOLD)
                                wait_quit = true;
                        }
                        else
                        { // 下一段航线不是连续航线
                          // 等待退出连续任务
                            prestored = false;
                            wait_quit = true;
                        }
                    }
                    else
                    { // 等待预存航线飞完
                      // 反馈可以接受指令
                        res = NCR_Processing_CmdManageable;
                    }
                    break;
                }
                case 5:
                { // hold
                    // 反馈可以接受指令
                    res = NCR_Processing_CmdManageable;

                    Position_Control_set_XYLock();
                    Position_Control_set_ZLock();
                    Attitude_Control_set_YawLock();
                    if (++navInf.counter2 >= current_mission_inf.params[0] * freq)
                    { // 退出连续任务
                        ms_step = 0;
                        res = NCR_Fin;
                    }
                    break;
                }
            }

            if (NavCmdRs_SuccessOrFault(res))
            { // 错误或执行完成

                // Update report only if not in hold period (preserve previous SUCCESS/FAILURE)
                if (report_hold_counter == 0)
                {
                    if (NavCmdRs_Fault(res))
                    {
                        MissionItemReport rpt = get_missionItemReport();
                        rpt.result = ACFLY_MISSION_REPORT_FAILURE;
                        set_missionItemReport(rpt);
                    }
                    else
                    {
                        MissionItemReport rpt = get_missionItemReport();
                        rpt.result = ACFLY_MISSION_REPORT_SUCCESS;
                        rpt.progress = 100;
                        set_missionItemReport(rpt);
                        send_mission_report();
                    }
                    // Hold the result for ~1.5s so Commulink thread can send it
                    report_hold_counter = (uint16_t) (0.1f * freq);
                }

                // 关闭喷洒
                setPump1(PumpOffAutoPercent);

                // 不自动执行返回
                if (current_mission_inf.autocontinue == 0)
                {
                    return FMR_OK_RqSwitchMode;
                }

                if (res == NCR_Land_RqDisarm)
                { // 已降落要求停止

                    // 切换下个航点
                    if (setCurrentMission(getCurrentMissionInd() + 1) == false)
                    { // 无航点信息返回手动模式
                        setCurrentMission(0);
                        // 复位当前航点信息
                        reset_RestoreWpInf(restoreWpInf);
                        storeRAM_restoreWpInf();
                    }

                    bool inFlight;
                    get_is_inFlight(&inFlight);
                    if (!inFlight)
                        return FMR_RqDisarm_Land;
                    else
                        return FMR_OK_RqSwitchMode;
                }
                else if (res < 0)
                { // 切换到下一模式
                    if (setCurrentMission(getCurrentMissionInd() + 1) == false)
                    { // 无航点信息返回手动模式
                        setCurrentMission(0);
                        // 复位当前航点信息
                        reset_RestoreWpInf(restoreWpInf);
                        storeRAM_restoreWpInf();
                        // 切换到其他模式
                        return FMR_OK_RqSwitchMode;
                    }
                    else
                    {
                        if (ReadCurrentMission(&current_mission_inf, &current_mission_ind))
                        { // 载入下一航点成功
                            // 初始化任务信息
                            init_NavCmdInf(&navInf);
                            // 更新任务状态
                            if (current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT)
                            { // waypoint任务更新航线信息
                                vector3<double> BA;
                                NavCmd16_WAYPOINT_GetInfo(current_mission_inf.frame, current_mission_inf.params, 0, &BA, 0);
                                restoreWpInf.CurrentWp[0] = current_mission_ind;
                                restoreWpInf.line_x = -BA.x;
                                restoreWpInf.line_y = -BA.y;
                                restoreWpInf.line_z = -BA.z;
                                restoreWpInf.line_fs = 0;
                                restoreWpInf.CamTrigDist = CamTriggDist;
                            }
                            else
                            { // 其他类型任务
                                restoreWpInf.CurrentWp[0] = current_mission_ind;
                                restoreWpInf.line_x = 0;
                                restoreWpInf.line_y = 0;
                                restoreWpInf.line_z = 0;
                                restoreWpInf.line_fs = 0;
                                restoreWpInf.CamTrigDist = CamTriggDist;
                            }
                        }
                        else
                        { // 无航点信息返回手动模式
                            setCurrentMission(0);
                            // 复位当前航点信息
                            reset_RestoreWpInf(restoreWpInf);
                            storeRAM_restoreWpInf();
                            // 切换到其他模式
                            return FMR_OK_RqSwitchMode;
                        }
                    }
                }
                else
                { // 切换到指定模式
                    if (setCurrentMission(res) == false)
                    { // 切换失败返回手动模式
                        setCurrentMission(0);
                        // 复位当前航点信息
                        reset_RestoreWpInf(restoreWpInf);
                        // 切换到其他模式
                        return FMR_OK_RqSwitchMode;
                    }
                    if (ReadCurrentMission(&current_mission_inf, &current_mission_ind))
                    { // 载入下一航点成功
                        // 初始化任务信息
                        init_NavCmdInf(&navInf);
                        // 更新任务状态
                        if (current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT)
                        { // waypoint任务更新航线信息
                            vector3<double> BA;
                            NavCmd16_WAYPOINT_GetInfo(current_mission_inf.frame, current_mission_inf.params, 0, &BA, 0);
                            restoreWpInf.CurrentWp[0] = current_mission_ind;
                            restoreWpInf.line_x = -BA.x;
                            restoreWpInf.line_y = -BA.y;
                            restoreWpInf.line_z = -BA.z;
                            restoreWpInf.line_fs = 0;
                            restoreWpInf.CamTrigDist = CamTriggDist;
                        }
                        else
                        { // 其他类型任务
                            restoreWpInf.CurrentWp[0] = current_mission_ind;
                            restoreWpInf.line_x = 0;
                            restoreWpInf.line_y = 0;
                            restoreWpInf.line_z = 0;
                            restoreWpInf.line_fs = 0;
                            restoreWpInf.CamTrigDist = CamTriggDist;
                        }
                    }
                    else
                    { // 无航点信息返回手动模式
                        setCurrentMission(0);
                        // 复位当前航点信息
                        reset_RestoreWpInf(restoreWpInf);
                        storeRAM_restoreWpInf();
                        // 切换到其他模式
                        return FMR_OK_RqSwitchMode;
                    }
                }
            }
            else
            { // 任务执行中
                if (res == NCR_Processing_CmdManageable)
                { // 可执行InFlightCmd

                    if (msg)
                    {
                        if (Process_InflightCmd(msg->cmd, msg->params))
                        {
                            if (msgRs)
                                *msgRs = FMMR_Success;
                        }
                    }
                }
            }

            break;
        }
    }
    return FMR_OK;
}
