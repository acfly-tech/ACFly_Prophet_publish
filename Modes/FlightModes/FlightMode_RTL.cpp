#include "FlightMode_RTL.hpp"

#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "Parameters.hpp"
#include "Sensors.hpp"
#include "StorageSystem.hpp"
#include "basic.hpp"
#include "followTarget.hpp"
#include "precLand.hpp"

FlightModeClass FlightMode_RTL::get_fmClass()
{
    return fmc;
}

bool FlightMode_RTL::set_params(double params[], uint16_t params_cnt)
{
    if (step != 0)
        return false;
    switch (function_code)
    {
        case 1:
        {
            if (params_cnt == 1)
            {
                rtl_ground_alt = params[0];

                paramSet = 1;
                return true;
            }
            break;
        }
        case 3:
        {
            if (params_cnt == 4)
            {
                rtl_lat = params[0];
                rtl_lon = params[1];
                rtl_ground_alt = params[2];
                rtl_height = params[3];

                paramSet = 3;
                return true;
            }
            break;
        }
    }
    return false;
}

bool FlightMode_RTL::set_functionCode(uint16_t function_code)
{
    if (step != 0)
        return false;
    this->function_code = function_code;
    this->paramSet = 0;
    return true;
}

FlightModeCheckResult FlightMode_RTL::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    bool inFlight;
    get_is_inFlight(&inFlight);

    if (function_code < 1 || function_code > 3)
        return FMCR_Err_ParamInvalid;
    if (!inFlight && function_code != 3)
        return FMCR_Err_NotTakeOff;
    if (function_code == 3 && !paramSet)
        return FMCR_Err_ParamInvalid;
    if (get_Position_MSStatus() != MS_Ready)
        return FMCR_Err_NoPositioning;

    return FMCR_OK;
}

FlightMode_RTL::FlightMode_RTL(uint16_t function_code)
    : FlightMode_Base("RTL", mapFunctionCodeToAFunc(function_code), function_code)
{
    // 复位参数
    paramSet = 0;
    // 复位状态机
    step = 0;
    // 复位退出模式计数器
    exit_counter1 = 0;
    exit_counter2 = 0;
    thr_raised = false;
    // 复位返航航线进入原因(直接降落实例保持NONE)
    rtl_entry_reason = MODE_CODE_REASON_NONE;
    precCtrl_TerminalLanding = false;
    precCtrl_int_x = 0;
    precCtrl_int_y = 0;
    precCtrl_int2_x = 0;
    precCtrl_int2_y = 0;
    precCtrl_target_x = 0;
    precCtrl_target_y = 0;
    precCtrl_target_px = 0;
    precCtrl_target_py = 0;
    precSensor_heightOffset = -10000000000;
    precSensor_yawFixed = false;
    precRetry_step = 0;
    precRetry_counter = 0;
    last_precSensor_updTIME.set_invalid();
    precRtl_yawSetup.set_invalid();
    // 复位返航航线进入原因(直接降落实例保持NONE)
    rtl_entry_reason = MODE_CODE_REASON_NONE;
    // 重置飞行模式类别
    fmc = FMC_Automatic;
    // 读取安全模式参数
    ReadParamGroup("MSafe", (uint64_t *) &safeCfg, 0);
}

FlightMode_RTL::~FlightMode_RTL() {}

void FlightMode_RTL::init_landInf(bool to_startPoint, double pos_z)
{
    // 读取精准降落参数
    precInf.ctrlKp[0] = 8;
    precInf.ctrlKi[0] = 1;
    ReadParamGroup("precLand", (uint64_t *) &precInf, 0);
    // 复位精准降落传感器基准的高度偏移
    precSensor_heightOffset = -10000000000;
    // 复位上次精准降落传感器更新时间
    last_precSensor_updTIME.set_invalid();
    // 复位精准降落传感器偏航修正状态
    precSensor_yawFixed = false;

    // 复位末段进入标志
    precCtrl_TerminalLanding = false;
    // 复位精准降落控制积分
    precCtrl_int_x = 0;
    precCtrl_int_y = 0;
    precCtrl_int2_x = 0;
    precCtrl_int2_y = 0;
    // 复位精准降落控制期望位置
    vector3<double> pos;
    get_Position_Ctrl(&pos);
    precCtrl_target_x = pos.x;
    precCtrl_target_y = pos.y;
    // 复位精准降落控制理论目标点位置
    precCtrl_target_px = pos.x;
    precCtrl_target_py = pos.y;
    // 复位精准降落上升重试步骤
    precRetry_step = 0;
    // 复位精准降落上升重试计数器
    precRetry_counter = 0;

    // 设置是否返回起飞点
    rtl_to_startPoint = to_startPoint;
    // 设置地面高度
    rtl_pos_z = pos_z;
}
void FlightMode_RTL::init_followLandInf()
{
    // 读取精准降落参数
    precInf.ctrlKp[0] = 8;
    precInf.ctrlKi[0] = 1;
    ReadParamGroup("precLand", (uint64_t *) &precInf, 0);
    // 复位精准降落传感器基准的高度偏移
    precSensor_heightOffset = -10000000000;
    // 复位上次精准降落传感器更新时间
    last_precSensor_updTIME.set_invalid();
    // 复位精准降落传感器偏航修正状态
    precSensor_yawFixed = false;

    // 复位末段进入标志
    precCtrl_TerminalLanding = false;
    // 复位精准降落控制积分
    precCtrl_int_x = 0;
    precCtrl_int_y = 0;
    precCtrl_int2_x = 0;
    precCtrl_int2_y = 0;
    // 复位精准降落控制期望位置
    precCtrl_target_x = 0;
    precCtrl_target_y = 0;
    // 复位精准降落控制理论目标点位置
    precCtrl_target_px = 0;
    precCtrl_target_py = 0;
    // 复位精准降落上升重试步骤
    precRetry_step = 0;
    // 复位精准降落上升重试计数器
    precRetry_counter = 0;
    // 复位精准降落偏航标志
    precRtl_yawSetup.set_invalid();
}
// 获取备降点
FlightMode_RTL::AL_TYPE FlightMode_RTL::get_ALInfo()
{
    // 备件点范围不正确直接退出
    if (safeCfg.MaxALHRange[0] <= 0)
        return AL_TYPE::NONE;

    // 判断是否是相对距离点
    if (SfRtMode_ALinRelativePos(safeCfg.SfRtMode[0]))
    {
        // 先带符号解释存储值：uint32 直接参与运算时负偏移会回绕，
        // 且 sq() 作用于 uint32 会溢出 32 位，导致范围检查结果不可靠
        double al_rel_x = (int32_t) safeCfg.AL1_lat[0];
        double al_rel_y = (int32_t) safeCfg.AL1_lon[0];
        if ((al_rel_x != 0 || al_rel_y != 0) && sq(al_rel_x) + sq(al_rel_y) < sq(safeCfg.MaxALHRange[0]) && safeCfg.AL1_alt[0] >= 0.5 * 100)
        {
            AL_lat = al_rel_x;
            AL_lon = al_rel_y;
            AL_alt = safeCfg.AL1_alt[0];
            return AL_TYPE::RELATIVE;
        }
        return AL_TYPE::NONE;
    }

    // 获取当前位置
    vector3<double> pos;
    get_Position_Ctrl(&pos);
    // 找出范围内最近的备降点
    double minDist = -1;
    for (uint8_t i = 0; i < MAX_AL_POINTS; ++i)
    {
        // 范围检查前先带符号解释经纬度（存储为 uint32 位模式）
        int32_t al_lat_i = (int32_t) safeCfg.AL1_lat[i * 6];
        int32_t al_lon_i = (int32_t) safeCfg.AL1_lon[i * 6];
        if (al_lat_i == 0 || al_lon_i == 0 || safeCfg.AL1_alt[i * 6] < 0.5 * 100)
            break;
        if (al_lat_i < -90e7 || al_lat_i > 90e7 || al_lon_i < -180e7 || al_lon_i > 180e7)
            break;

        double lat = al_lat_i * 1e-7;
        double lon = al_lon_i * 1e-7;

        double x, y;
        get_localXY_fromOptimalGlobal(lat, lon, &x, &y);
        double new_dist = sq(x - pos.x) + sq(y - pos.y);
        if (minDist < 0 || new_dist < minDist)
        {
            if (new_dist < sq(safeCfg.MaxALHRange[0]))
            {
                minDist = new_dist;
                AL_lat = lat;
                AL_lon = lon;
                AL_alt = safeCfg.AL1_alt[i * 6];
            }
        }
    }
    if (minDist < 0)
        return AL_TYPE::NONE;
    else
        return AL_TYPE::GLOBAL;
}

// 计算分级降落速度(cm/s 正值)
// 高于一级降落高度按一级降落速度下降(RtLand1Vel 非正值时用maxAutoVelDown)
// 低于二级降落高度按LandVel下降, 两级高度之间线性过渡
// 一级高度低于二级高度时自动交换, 两高度相等时全程LandVel
double FlightMode_RTL::getLandVel(double height, double maxAutoVelDown) const
{
    double h1 = safeCfg.RtLand1Height[0];
    double h2 = safeCfg.RtLand2Height[0];
    if (h1 < 0)
        h1 = 0;
    if (h2 < 0)
        h2 = 0;
    if (h1 < h2)
    { // 交换保证 h1 >= h2
        double t = h1;
        h1 = h2;
        h2 = t;
    }

    // 一级降落速度(非正值代表使用maxAutoVelDown并限幅)
    double v1 = safeCfg.RtLand1Vel[0];
    if (v1 <= 0 || v1 > maxAutoVelDown)
        v1 = maxAutoVelDown;

    // 二级降落速度维持现状使用LandVel
    double v2 = getPosCtrlCfg()->LandVel[0];

    if (height >= h1)
        return v1;
    if (height <= h2)
        return v2;

    // h2~h1 之间线性过渡
    return v2 + (v1 - v2) * (height - h2) / (h1 - h2);
}

FlightModeResult FlightMode_RTL::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                           const AvoidanceCfg *avCfg, double h)
{
    // 获取起飞状态
    bool inFlight;
    get_is_inFlight(&inFlight);
    // 计算模式运行频率
    const double freq = 1.0 / h;

    switch (step)
    {
        case 0:
        {
            // 重置飞行模式类别
            fmc = FMC_Automatic;

            // 打开位置控制器
            Altitude_Control_Enable();
            Position_Control_Enable();

            // 获取位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);

            if (pos_ena)
            { // 位置控制器已打开

                // 锁定位置
                Position_Control_set_XYLock();
                Position_Control_set_ZLock();

                // 上报模式状态 (必须放到锁定位置后 避免从安全模式切回没有改变模式)
                // 按实例真实mode_id上报: function_code=1为降落模式, 2/3为返航模式
                setCurrentFlyMode((AFunc) get_modeId());

                Position_ControlMode pos_mode, alt_mode;
                get_Position_ControlMode(&pos_mode);
                get_Altitude_ControlMode(&alt_mode);
                if (pos_mode == Position_ControlMode_Position && alt_mode == Position_ControlMode_Position)
                { // 刹车完成进入返航状态
                    switch (function_code)
                    {
                        case 1:
                        { // 直接降落 - 功能码1实例即降落模式, 上报App/外部请求reason不在此覆盖
                            if (paramSet == 1 && isvalid(rtl_ground_alt))
                            { // 地面高信息可用
                                // 获取起飞点local z
                                double homeZ;
                                getHomeLocalZ(&homeZ);

                                // 设置降落点地面local z
                                rtl_pos_z = rtl_ground_alt + homeZ;
                            }
                            else
                            {
                                // 设置降落点地面local z为很高让飞机最低速度降落
                                rtl_pos_z = 1000000000;
                            }

                            init_landInf(false, rtl_pos_z);
                            step = 100;
                            break;
                        }
                        case 2:
                        { // 返航到起飞点降落
                            // 记录进入原因: 只延续触发型/任务型诱因(失控/围栏/低电压/无固定解/任务返航107)
                            // 其余(App5等下发语义)不延续, 降落段按function_code分支处理
                            uint16_t entry_r = getCurrentFlyModeReason();
                            if (entry_r == MODE_CODE_REASON_RC_LOST || entry_r == MODE_CODE_REASON_FENCE || entry_r == MODE_CODE_REASON_LVOLT_RTL || entry_r == MODE_CODE_REASON_GPS_NOT_FIXED ||
                                entry_r == MODE_CODE_REASON_MISSION_NAV)
                                rtl_entry_reason = entry_r;
                            else
                                rtl_entry_reason = MODE_CODE_REASON_NONE;
                            step = 10;
                            break;
                        }
                        case 3:
                        { // 返航到指定点
                            // 记录进入原因: 只延续触发型/任务型诱因(失控/围栏/低电压/无固定解/任务返航107)
                            // 其余(App5等下发语义)不延续, 降落段按function_code分支处理
                            uint16_t entry_r = getCurrentFlyModeReason();
                            if (entry_r == MODE_CODE_REASON_RC_LOST || entry_r == MODE_CODE_REASON_FENCE || entry_r == MODE_CODE_REASON_LVOLT_RTL || entry_r == MODE_CODE_REASON_GPS_NOT_FIXED ||
                                entry_r == MODE_CODE_REASON_MISSION_NAV)
                                rtl_entry_reason = entry_r;
                            else
                                rtl_entry_reason = MODE_CODE_REASON_NONE;
                            step = 50;
                            break;
                        }
                    }
                }
            }
            else
            { // 位置控制器打不开

                Attitude_Control_set_Target_RollPitch(0, 0);
                Position_Control_set_ZLock();

                // 上报模式状态 (必须放到锁定位置后 避免从安全模式切回没有改变模式)
                // 按实例真实mode_id上报: function_code=1为降落模式, 2/3为返航模式
                setCurrentFlyMode((AFunc) get_modeId());

                step = 1;
            }
            break;
        }
            // 位置控制器未打开进行定高飞行等待位置控制
        case 1:
        {
            // 打开位置控制器
            Altitude_Control_Enable();
            Position_Control_Enable();

            // 获取位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);

            if (!pos_ena)
            {
                if (rc.available)
                {
                    double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxLean[0] / 50.0);
                    Attitude_Control_set_Target_RollPitch(remove_deadband(rc.data[3] - 50, cfg->NeutralZone[0]) * RPCtrlScale, remove_deadband(rc.data[2] - 50, cfg->NeutralZone[0]) * RPCtrlScale);
                }
                else
                {
                    Attitude_Control_set_Target_RollPitch(0, 0);
                }
            }
            else
            { // 定位恢复
                Position_Control_set_XYLock();
                Position_Control_set_ZLock();
                step = 0;
            }

            // 油门杆控制高度(速度方式)
            rcCtrl_Alt_v(rc, flag, comVars, msg, msgRs, cfg, avCfg, h);

            // 偏航杆在中间锁偏航
            // 不在中间控制偏航速度
            double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
            if (!rc.available || in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
                Attitude_Control_set_YawLock();
            else
                Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);

            // 判断锁定
            if (rc.available && !thr_raised)
            {
                if (rc.data[0] > 35)
                    thr_raised = true;
            }
            if (!inFlight && rc.available)
            {
                if (rc.data[0] < 30)
                {
                    ++exit_counter1;
                    if (comVars.taken_off)
                    {
                        if (exit_counter1 > 0.15 * freq)
                            return FMR_RqDisarm_Land;
                    }
                    else
                    {
                        if (exit_counter1 > (thr_raised ? 0.15 : 10) * freq)
                            return FMR_RqDisarm_Land;
                    }
                }
                else
                    exit_counter1 = 0;

                if (rc.data[0] < 25 && rc.data[1] < 25)
                {
                    ++exit_counter2;
                    if (exit_counter2 > 0.3 * freq)
                        return FMR_RqDisarm_Land;
                }
                else
                    exit_counter2 = 0;
            }
            else
            {
                exit_counter1 = 0;
                exit_counter2 = 0;
            }

            break;
        }

        // 需要返航到起飞点
        case 10:
        {
            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            followSensor followS;
            current_followSInd = get_followSensor(&followS);
            if (current_followSInd && followS.available && isFollowDataType_NeedRTL(followS.dataType) && followS.updateTime.get_pass_time() < 2 &&
                (!isFollowDataType_GlobalXY(followS.dataType) || followS.globalXYPosAvailable))
            { // 跟随可用
                // 初始化跟随
                vector3<double> pos;
                get_Position_Ctrl(&pos);
                followPos = pos;
                followRTL_currentVel = 0;

                // 将followPos设置为 pos - followS.pos
                // followTPos设置为 followS.pos
                // 当前目标位置TPos = followPos + followTPos = pos
                // 下一步会将followPos缓慢归0(返回跟随传感器的地方)
                // 使TPos = followTPos
                if (isFollowDataType_XY(followS.dataType))
                { // 跟随目标可用
                    followPos.x -= followS.pos.x;
                    followPos.y -= followS.pos.y;

                    followTPos.x = followS.pos.x;
                    followTPos.y = followS.pos.y;
                    followTVel.x = followS.vel.x;
                    followTVel.y = followS.vel.y;
                }
                if (isFollowDataType_Z(followS.dataType))
                { // 跟随目标可用
                    followPos.z -= followS.pos.z;

                    followTPos.z = followS.pos.z;
                    followTVel.z = followS.vel.z;
                }

                // 进入跟随返航
                counter1 = 0;
                followRtl_step = 0;
                step = 15;
                precCtrl_int_x = 0;
                precCtrl_int_y = 0;
                precCtrl_int2_x = 0;
                precCtrl_int2_y = 0;
                followYawAvailable = false;
                // 复位yaw偏移
                followYawOffset = 0;
                followRtl_yawSetup = false;
            }
            else
            { // 跟随不可用
                step = 20;
            }
            break;
        }

        // 跟随返航
        case 15:
        {
            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            setCurrentFlyMode(AFunc_RTL);

            // 期望加速度
            vector3<double> TAcc;

            followSensor followS;
            uint16_t followSInd = get_followSensor(&followS);
            if (followSInd == current_followSInd && followS.available && isFollowDataType_NeedRTL(followS.dataType) && (!isFollowDataType_GlobalXY(followS.dataType) || followS.globalXYPosAvailable))
            { // 跟随可用
                // 计算传感器数据滞后的时间
                double upT = followS.updateTime.get_pass_time() + followS.delay;
                if (isFollowDataType_XY(followS.dataType))
                {
                    // 速度*时间 插值 平滑传感器期望位置
                    followTPos.x = followS.pos.x + followS.vel.x * upT;
                    followTPos.y = followS.pos.y + followS.vel.y * upT;
                    // 跟随期望速度
                    followTVel.x = followS.vel.x;
                    followTVel.y = followS.vel.y;
                    // 跟随期望加速度
                    TAcc.x = followS.acc.x;
                    TAcc.y = followS.acc.y;
                }
                if (isFollowDataType_Z(followS.dataType))
                {
                    // 跟随期望位置
                    followTPos.z = followS.pos.z + followS.vel.z * upT;
                    // 跟随期望速度
                    followTVel.z = followS.vel.z;
                    // 跟随期望加速度
                    TAcc.z = followS.acc.z;
                }

                if (followS.yawAvailable() != followYawAvailable)
                {
                    followYawAvailable = followS.yawAvailable();
                    followRtl_yawSetup = false;
                }
                if (followS.yawAvailable())
                { // 跟随yaw可用
                    followYaw = followS.yaw;
                    followYawRate = followS.yawRate;
                    if (!followRtl_yawSetup)
                    {
                        followRtl_yawSetup = true;
                        followYawOffset = 0;
                    }
                }
                else
                { // 跟随yaw不可用
                    if (!followRtl_yawSetup)
                    {
                        followRtl_yawSetup = true;
                        // 复位followYawOffset到当前航向
                        Quaternion quat;
                        get_AirframeY_quat(&quat);
                        followYawOffset = quat.getYaw();
                    }
                }
            }
            else
            { // 跟随不可用
                step = 0;
                return FMR_OK;
            }

            double roll_sitck_d = 0;
            double pitch_sitck_d = 0;
            double thr_stick_d = 0;

            vector3<double> missionTVel;
            switch (followRtl_step)
            {
                case 0:
                { // 第一步回到起飞点

                    // 计算剩余距离
                    double rtlRm = safe_sqrt(sq(followPos.x) + sq(followPos.y));

                    // 计算飞行速度
                    double acc = getPosCtrlCfg()->maxAutoAccXY[0] * 0.5; // 加速度
                    const double mtDist = 500;                           // 保留距离
                    const double mtVel = 200;                            // 保留距离内飞行速度
                    double accDist = 0.5 * sq(followRTL_currentVel) / acc;
                    // 需要减速
                    if (accDist >= rtlRm)
                    {
                        followRTL_currentVel -= acc * h;
                        if (followRTL_currentVel < 1)
                            followRTL_currentVel = 1;
                    }
                    else
                    { // 前馈加速到最高速度
                        double max_v = safeCfg.RtSpeed[0];
                        if (followRTL_currentVel < max_v)
                            followRTL_currentVel += acc * h;
                        if (followRTL_currentVel > max_v)
                            followRTL_currentVel = max_v;
                    }

                    // 计算任务速度
                    if (rtlRm > 0.0001)
                    {
                        double msVel = followRTL_currentVel - 250;
                        if (msVel < 0)
                            msVel = 0;
                        double scale = msVel / rtlRm;
                        missionTVel.x = -followPos.x * scale;
                        missionTVel.y = -followPos.y * scale;
                    }

                    // 计算期望位置
                    vector3<double> tPos, pos;
                    get_Position_Ctrl(&pos);
                    get_TargetPosInf(0, 0, &tPos, 0);
                    if (rtlRm > 0.0001 && sq(tPos.x - pos.x) + sq(tPos.y - pos.y) < sq(10 * 100))
                    {
                        double new_rtlRm = rtlRm - followRTL_currentVel * h;
                        if (new_rtlRm < 0)
                            new_rtlRm = 0;
                        double rtlRmScale = new_rtlRm / rtlRm;
                        followPos.x *= rtlRmScale;
                        followPos.y *= rtlRmScale;
                        rtlRm = new_rtlRm;
                    }

                    // 判断是否返航结束
                    if (rtlRm < 0.01)
                    {
                        if (++counter1 >= 3 * freq)
                        {
                            counter1 = 0;
                            init_followLandInf();
                            ++followRtl_step;
                        }
                    }
                    break;
                }

                default:
                case 1:
                { // 降落

                    // 获取末端降落速度
                    double landVel = getPosCtrlCfg()->LandVel[0];

                    // 已降落要求上锁
                    if (!inFlight)
                        return FMR_RqDisarm_Land;

                    // 计算精准降落高度
                    bool precHeight_available = false;
                    double height = 0;
                    if (precSensor_heightOffset > -10000000000)
                    {
                        precHeight_available = true;
                        height = followPos.z - precSensor_heightOffset;
                    }

                    // 降落过程允许摇杆操作
                    if (rc.available && !precHeight_available)
                    {
                        roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]);
                        pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]);
                        thr_stick_d = remove_deadband(rc.data[0] - 50.0, (double) cfg->NeutralZone[0]);

                        // 控制偏航速度
                        double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                        followYawOffset += remove_deadband(50.0 - rc.data[1], (double) cfg->NeutralZone[0]) * YCtrlScale * h;
                        if (followYawOffset > Pi)
                            followYawOffset -= 2 * Pi;
                        else if (followYawOffset < -Pi)
                            followYawOffset += 2 * Pi;
                    }

                    // 是否需要独立控制高度
                    bool precAltCtrl = false;
                    // 获取精准降落传感器
                    precLandSensor precSensor;
                    if (precLandSensor_get(&precSensor) && precSensor.available)
                    {
                        // 获取控制误差
                        vector3<double> posErr;
                        get_history_PositionErr(&posErr, 0);
                        // 计算控制误差修正误差
                        vector2<double> posErr_fixErr;
                        posErr_fixErr.x = posErr.x - precCtrl_int_x;
                        posErr_fixErr.y = posErr.y - precCtrl_int_y;

                        // 计算修正率
                        double kp_scale = 1.0;
                        if (precSensor.pos.z < -1000)
                            kp_scale = 0.1;
                        else if (precSensor.pos.z < -500)
                            kp_scale = 0.15;
                        else if (precSensor.pos.z < -250)
                            kp_scale = 0.3;
                        else
                        {
                        }

                        // 判断传感器更新修正位置
                        if (precSensor.updateTime != last_precSensor_updTIME)
                        {
                            double vv[3];
                            vv[0] = precSensor.pos.x * 0.01;
                            vv[1] = precSensor.pos.y * 0.01;
                            vv[2] = precSensor.pos.z * 0.01;
                            SDLog_Msg_DebugVect("prec", vv, 3);
                            // 计算修正频率
                            double h;
                            if (last_precSensor_updTIME.is_valid() == false)
                                h = 0;
                            else
                                h = last_precSensor_updTIME.get_pass_time();
                            if (h > 1.5)
                                h = 1.5;
                            // 保存上次修正时间
                            last_precSensor_updTIME = precSensor.updateTime;

                            // 修正航向
                            if (precSensor.yaw_available)
                            {
                                double ang = precSensor.yaw;
                                ang = fmod(ang, 2 * Pi);
                                if (ang > Pi)
                                    ang -= 2 * Pi;
                                else if (ang < -Pi)
                                    ang += 2 * Pi;

                                if (!precRtl_yawSetup.is_valid())
                                {
                                    followYawOffset += ang;
                                    precRtl_yawSetup = TIME::now();
                                }
                                else
                                {
                                    double h = precRtl_yawSetup.get_pass_time_st();
                                    if (h > 1)
                                        h = 1;
                                    followYawOffset += 0.33 * h * ang;
                                }
                                if (followYawOffset > Pi)
                                    followYawOffset -= 2 * Pi;
                                else if (followYawOffset < -Pi)
                                    followYawOffset += 2 * Pi;
                            }

                            // 计算精准降落传感器基准的高度偏移
                            precSensor_heightOffset = followPos.z + precSensor.pos.z;
                            precHeight_available = true;
                            height = -precSensor.pos.z;

                            // 获取历史控制误差
                            vector3<double> his_posErr;
                            get_history_PositionErr(&his_posErr, precSensor.delay);

                            // 计算误差
                            double err_x = precSensor.pos.x;
                            double err_y = precSensor.pos.y;
                            // 误差积分2(控制误差之外的误差积分)
                            // 总误差(视觉误差) = 消除后的控制误差(posErr-precCtrl_int) + 其他误差(precCtrl_int2)
                            //														precCtrl_int2_x += h  * precInf.ctrlKi[0] * (err_x - posErr.x +
                            // precCtrl_int_x
                            //- precCtrl_int2_x); 														precCtrl_int2_y += h  *
                            // precInf.ctrlKi[0] * (err_y - posErr.y + precCtrl_int_y - precCtrl_int2_x);
                            // precCtrl_int2_x
                            // = constrain(precCtrl_int2_x, 50.0); 														precCtrl_int2_y =
                            // constrain(precCtrl_int2_y, 50.0); 储存理论期望位置
                            precCtrl_target_px = precCtrl_target_x + precCtrl_int_x + precCtrl_int2_x + err_x - his_posErr.x;
                            precCtrl_target_py = precCtrl_target_y + precCtrl_int_y + precCtrl_int2_y + err_y - his_posErr.y;
                        }

                        double vv[6];
                        vv[0] = precCtrl_target_px * 0.01;
                        vv[1] = precCtrl_target_py * 0.01;
                        vv[2] = precCtrl_target_x * 0.01;
                        vv[3] = precCtrl_target_y * 0.01;
                        vv[4] = precCtrl_int_x * 0.01;
                        vv[5] = precCtrl_int_y * 0.01;
                        SDLog_Msg_DebugVect("prec2", vv, 6);

                        // 控制误差积分
                        if (!precCtrl_TerminalLanding)
                        {
                            if (fabs(precCtrl_int_x) > fabs(posErr.x) + 35)
                                precCtrl_int_x = sign(precCtrl_int_x) * (fabs(posErr.x) + 35);
                            if (fabs(precCtrl_int_y) > fabs(posErr.y) + 35)
                                precCtrl_int_y = sign(precCtrl_int_y) * (fabs(posErr.y) + 35);
                            precCtrl_int_x += h * precInf.ctrlKi[0] * constrain(posErr_fixErr.x, 50.0);
                            precCtrl_int_y += h * precInf.ctrlKi[0] * constrain(posErr_fixErr.y, 50.0);
                            precCtrl_int_x = constrain(precCtrl_int_x, 100.0);
                            precCtrl_int_y = constrain(precCtrl_int_y, 100.0);
                        }
                        // 将期望位置缓慢移动到理论目标位置
                        double err_x = precCtrl_target_px - precCtrl_target_x;
                        double err_y = precCtrl_target_py - precCtrl_target_y;
                        precCtrl_target_x += kp_scale * precInf.ctrlKp[0] * h * (err_x);
                        precCtrl_target_y += kp_scale * precInf.ctrlKp[0] * h * (err_y);
                        // 更新followPos
                        followPos.x = precCtrl_target_x;
                        followPos.y = precCtrl_target_y;

                        // 误差过大停止下降
                        if (precHeight_available && height < 250 && !precCtrl_TerminalLanding)
                        {
                            if (sq(precSensor.pos.x) + sq(precSensor.pos.y) > sq(precInf.maxDynamicErrRaw[0]) || posErr_fixErr.get_square() > sq(precInf.maxDynamicErrCtrl[0]))
                            {
                                if (height > 25)
                                {
                                    precAltCtrl = true;
                                    followPos.z -= 0 * h;
                                }
                                else if (height > 15)
                                {
                                    precAltCtrl = true;
                                    followPos.z += 30 * h;
                                }
                            }
                        }
                        if (height < 15)
                        {
                            precCtrl_TerminalLanding = true;
                        }
                    }
                    else
                    { // 精准降落传感器失效
                        if (precHeight_available)
                        { // 精准降落传感器高度可用
                            // 获取控制误差
                            vector3<double> posErr;
                            get_history_PositionErr(&posErr, 0);
                            // 计算控制误差修正误差
                            vector2<double> posErr_fixErr;
                            posErr_fixErr.x = posErr.x - precCtrl_int_x;
                            posErr_fixErr.y = posErr.y - precCtrl_int_y;
                            // 误差过大停止下降
                            if (posErr_fixErr.get_square() > sq(precInf.maxDynamicErrCtrl[0]) && !precCtrl_TerminalLanding)
                            {
                                if (height > 25)
                                {
                                    precAltCtrl = true;
                                    followPos.z -= 0 * h;
                                }
                                else if (height > 15)
                                {
                                    precAltCtrl = true;
                                    followPos.z += 30 * h;
                                }
                            }
                            if (height < 15)
                            {
                                precCtrl_TerminalLanding = true;
                            }

                            // 控制误差积分
                            if (!precCtrl_TerminalLanding)
                            {
                                if (fabs(precCtrl_int_x) > fabs(posErr.x) + 35)
                                    precCtrl_int_x = sign(precCtrl_int_x) * (fabs(posErr.x) + 35);
                                if (fabs(precCtrl_int_y) > fabs(posErr.y) + 35)
                                    precCtrl_int_y = sign(precCtrl_int_y) * (fabs(posErr.y) + 35);
                                precCtrl_int_x += h * precInf.ctrlKi[0] * constrain(posErr_fixErr.x, 50.0);
                                precCtrl_int_y += h * precInf.ctrlKi[0] * constrain(posErr_fixErr.y, 50.0);
                                precCtrl_int_x = constrain(precCtrl_int_x, 100.0);
                                precCtrl_int_y = constrain(precCtrl_int_y, 100.0);
                            }
                            // 将期望位置缓慢移动到理论目标位置
                            double err_x = precCtrl_target_px - precCtrl_target_x;
                            double err_y = precCtrl_target_py - precCtrl_target_y;
                            precCtrl_target_x += 0.35 * precInf.ctrlKp[0] * h * (err_x);
                            precCtrl_target_y += 0.35 * precInf.ctrlKp[0] * h * (err_y);
                            // 更新followPos
                            followPos.x = precCtrl_target_x;
                            followPos.y = precCtrl_target_y;
                        }
                        else
                        { // 精准降落高度不可用
                        }
                    }

                    // 调整高度
                    if (!precAltCtrl)
                    {
                        if (precCtrl_TerminalLanding)
                        {
                            followPos.z -= 60 * h;
                        }
                        else if (precHeight_available)
                        {
                            // 分级降落速度(跟随返航上限为maxVelDown的0.7倍)
                            followPos.z -= getLandVel(height, getPosCtrlCfg()->maxVelDown[0] * 0.7) * h;
                        }
                        else
                            followPos.z -= landVel * h;
                    }

                    break;
                }
            }

            // XY速度前馈
            vector3<double> followTV = missionTVel;
            if (isFollowDataType_XY(followS.dataType))
            { // 跟随目标可用
                followTV.x += followTVel.x;
                followTV.y += followTVel.y;
            }
            // Z速度前馈
            if (isFollowDataType_Z(followS.dataType))
            { // 跟随目标可用
                followTV.z += followTVel.z;
            }

            // 计算期望位置速度
            vector3<double> TVel;
            rcCtrl_Pos_p(roll_sitck_d, pitch_sitck_d, followPos.x, followPos.y, TVel.x, TVel.y, followTV.x, followTV.y, rc, flag, comVars, msg, msgRs, cfg, avCfg, h);
            rcCtrl_Alt_p(thr_stick_d, followPos.z, TVel.z, followTV.z, rc, flag, comVars, msg, msgRs, cfg, avCfg, h);

            vector3<double> TPos;
            TPos.x = followPos.x + precCtrl_int_x + precCtrl_int2_x;
            TPos.y = followPos.y + precCtrl_int_y + precCtrl_int2_y;
            TPos.z = followPos.z;

            // XY位置跟随
            if (isFollowDataType_XY(followS.dataType))
            { // 跟随目标可用
                TPos.x += followTPos.x;
                TPos.y += followTPos.y;
            }

            // Z位置跟随
            if (isFollowDataType_Z(followS.dataType))
            { // 跟随目标可用
                TPos.z += followTPos.z;
            }
            // 水平控制
            Position_Control_set_TargetPosVelAccXY_OffBoard(TPos.x, TPos.y, TVel.x, TVel.y, TAcc.x, TAcc.y);
            // 垂直控制
            Position_Control_set_TargetPosVelAccZ_OffBoard(TPos.z, TVel.z, TAcc.z);
            if (followYawAvailable)
                Attitude_Control_set_Target_Yaw_Offboard(followYaw + followYawOffset, followYawRate);
            else
                Attitude_Control_set_Target_Yaw_Offboard(followYawOffset, 0);
            break;
        }

            // 返航到起飞点
        case 20:
        { // 调整高度

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            bool res = false;
            if (inFlight)
            {
                // 计算距离起飞点距离
                bool homeAvailable = false;
                vector2<double> homeP;
                if (getHomeLatLon(&homeP))
                { // 返航至经纬度

                    // 获取最优全球定位传感器信息
                    double x, y;
                    if (get_localXY_fromOptimalGlobal(homeP.x, homeP.y, &x, &y))
                    {
                        // 求LA LB
                        vector3<double> t_pos;
                        get_TargetPosInf(0, 0, &t_pos, 0);
                        rtl_lat = x;
                        rtl_lon = y;

                        homeAvailable = true;
                    }
                }
                if (!homeAvailable && getHomePoint(&homeP))
                { // 返航至Local坐标
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    rtl_lat = homeP.x;
                    rtl_lon = homeP.y;

                    homeAvailable = true;
                }
                if (homeAvailable == false)
                {
                    return FMR_Err_NoHomePoint;
                }

                vector3<double> pos;
                get_Position_Ctrl(&pos);
                double LA = rtl_lon - pos.y;
                double LB = rtl_lat - pos.x;

                double homeZ;
                getHomeLocalZ(&homeZ);

                if (sq(LA) + sq(LB) > sq(safeCfg.RtHRange[0]))
                { // 距离较远才需要升高返航
                    if (safeCfg.RtHeight[0] > 0 && homeZ + safeCfg.RtHeight[0] > pos.z)
                    { // 返航高度大于当前高度才进行升高
                        res = Position_Control_set_TargetPositionZRA(safeCfg.RtHeight[0]);
                    }
                    else
                        res = Position_Control_set_ZLock();
                }
                else
                    res = Position_Control_set_ZLock();
            }
            else // 未起飞
            {
                return FMR_RqDisarm_Land;
            }

            if (res)
            {
                counter1 = 0;
                ++step;
            }
            else
                return FMR_Err_ParamInvalid;
            break;
        }
        case 21:
        { // 等待高度调整完成旋转偏航

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            // 锁定水平位置
            Position_Control_set_XYLock();
            // 锁定yaw
            Attitude_Control_set_YawLock();

            // 等待高度调整完成
            Position_ControlMode pos_mode, alt_mode;
            get_Position_ControlMode(&pos_mode);
            get_Altitude_ControlMode(&alt_mode);
            if (pos_mode == Position_ControlMode_Position && alt_mode == Position_ControlMode_Position)
            {
                // 设置偏航角(机头转向返航点) - 保持机头模式下跳过转头
                if (!SfRtMode_KeepYawRTL(safeCfg.SfRtMode[0]))
                {
                    vector3<double> pos;
                    get_Position_Ctrl(&pos);
                    double LA = rtl_lon - pos.y;
                    double LB = rtl_lat - pos.x;

                    if (sq(LA) + sq(LB) > sq(safeCfg.RtHRange[0]))
                        Attitude_Control_set_Target_Yaw(atan2(LA, LB));
                }
                ++step;
            }
            break;
        }
        case 22:
        { // 等待偏航旋转开始航点飞行

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                bool res;
                res = Position_Control_set_TargetPositionXY(rtl_lat, rtl_lon, safeCfg.RtSpeed[0]);
                if (res)
                    ++step;
                else
                    return FMR_Err_NoPositioning;
            }
            break;
        }
        case 23:
        { // 等待飞行完成

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            Position_Control_set_ZLock();
            Position_ControlMode pos_mode;
            get_Position_ControlMode(&pos_mode);
            if (pos_mode == Position_ControlMode_Position)
            { // 位置调整完成
                Position_Control_set_XYLock();

                if (SfRtMode_KeepYawRTL(safeCfg.SfRtMode[0]))
                { // 保持机头模式 - 锁定当前航向不转回起飞航向
                    Attitude_Control_set_YawLock();
                }
                else
                {
                    // 旋转至起飞航向
                    double homeYaw;
                    getHomeLocalZ(0, &homeYaw);
                    Attitude_Control_set_Target_Yaw(homeYaw);
                }

                ++step;
            }
            break;
        }
        case 24:
        { // 等待偏航旋转
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                // 进入降落状态
                init_landInf(true, 0);
                step = 100;
            }
            break;
        }

        // 返航到指定点
        case 50:
        { // 调整高度

            // 检查参数是否有效
            if (paramSet != function_code)
                return FMR_Err_ParamInvalid;

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            bool res = false;
            if (isvalid(rtl_ground_alt))
            { // 地面高信息可用
                if (inFlight)
                {
                    // 获取起飞点local z
                    double homeZ;
                    getHomeLocalZ(&homeZ);

                    // 设置降落点地面local z
                    rtl_pos_z = rtl_ground_alt + homeZ;

                    // 计算应到达的高度
                    double reqAlt;
                    if (isvalid(rtl_height))
                    { // 地面站提供了返航高度
                        reqAlt = homeZ + rtl_height;
                    }
                    else
                    { // 地面站未提供返航高度
                        reqAlt = rtl_ground_alt + homeZ + safeCfg.RtHeight[0];
                        if (reqAlt < safeCfg.RtHeight[0] + homeZ)
                            reqAlt = homeZ + safeCfg.RtHeight[0];
                    }

                    // 目标高度在目前高度以上时升高
                    vector3<double> pos;
                    get_Position_Ctrl(&pos);
                    if (pos.z < reqAlt)
                        res = Position_Control_set_TargetPositionZ(reqAlt);
                    else
                        res = Position_Control_set_ZLock();
                }
                else
                { // 未起飞
                    // 获取地面local z
                    double homeZ;
                    getHomeLocalZ(&homeZ);

                    // 设置降落点地面local z
                    rtl_pos_z = rtl_ground_alt + homeZ;

                    double reqAlt;
                    if (isvalid(rtl_height))
                    { // 地面站提供了返航高度
                        reqAlt = homeZ + rtl_height;
                    }
                    else
                    { // 地面站未提供返航高度
                        reqAlt = rtl_ground_alt + homeZ + safeCfg.RtHeight[0];
                        if (reqAlt - homeZ < safeCfg.RtHeight[0])
                            reqAlt = homeZ + safeCfg.RtHeight[0];
                    }

                    // 起飞
                    res = Position_Control_Takeoff_Height(reqAlt);
                }
            }
            else
            { // 海拔信息不可用

                // 设置降落点地面local z为很高让飞机最低速度降落
                rtl_pos_z = 1000000000;

                if (inFlight)
                {
                    double RtHeight;
                    if (isvalid(rtl_height))
                        RtHeight = rtl_height;
                    else
                    {
                        RtHeight = safeCfg.RtHeight[0];
                    }

                    double homeZ;
                    getHomeLocalZ(&homeZ);
                    vector3<double> pos;
                    get_Position_Ctrl(&pos);
                    if (RtHeight > 0 && homeZ + RtHeight > pos.z)
                    { // 返航高度大于当前高度才进行升高
                        res = Position_Control_set_TargetPositionZRA(RtHeight);
                    }
                    else
                        res = Position_Control_set_ZLock();
                }
                else // 未起飞
                {
                    double RtHeight;
                    if (isvalid(rtl_height))
                        RtHeight = rtl_height;
                    else
                    {
                        RtHeight = safeCfg.RtHeight[0];
                    }
                    res = Position_Control_Takeoff_HeightRelative(RtHeight);
                }
            }

            if (res)
                ++step;
            else
                return FMR_Err_ParamInvalid;
            break;
        }
        case 51:
        { // 等待高度调整完成

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            Position_ControlMode alt_mode;
            get_Altitude_ControlMode(&alt_mode);
            Position_Control_set_XYLock();
            if (alt_mode == Position_ControlMode_Position)
            { // 高度调整完成

                // 旋转偏航
                double x, y;
                if (get_localXY_fromOptimalGlobal(rtl_lat, rtl_lon, &x, &y))
                {
                    vector3<double> pos;
                    get_Position_Ctrl(&pos);
                    double LA = y - pos.y;
                    double LB = x - pos.x;

                    // 设置偏航角
                    if (sq(LA) + sq(LB) > sq(200))
                        Attitude_Control_set_Target_Yaw(atan2(LA, LB));
                    ++step;
                }
                else
                    return FMR_Err_NoPositioning;
            }
            break;
        }
        case 52:
        { // 等待偏航旋转
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                bool res = Position_Control_set_TargetPositionXY_LatLon(rtl_lat, rtl_lon, -1);
                if (res)
                    ++step;
                else
                    return FMR_Err_NoPositioning;
            }
            break;
        }
        case 53:
        { // 等待飞行完成

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            Position_Control_set_ZLock();
            Position_ControlMode pos_mode;
            get_Position_ControlMode(&pos_mode);
            if (pos_mode == Position_ControlMode_Position)
            { // 位置调整完成
                Position_Control_set_XYLock();

                // 进入降落状态
                init_landInf(false, rtl_pos_z);
                step = 100;
            }
            break;
        }

            // 原地降落
        case 100:
        { // 原地降落

            // 设置模式状态 - 进入降落段继承返航航线的诱因(围栏/失控/低电压)
            // 巡航线无诱因(App/遥控请求返航)时报返航到家后降落(12)
            // function_code=1直接降落实例(如App降落指令)不覆盖已有的切换reason
            if (function_code != 1)
            { // 巡航线进入的降落段
                if (rtl_entry_reason != MODE_CODE_REASON_NONE)
                    setCurrentFlyMode(AFunc_Land, rtl_entry_reason);
                else
                    setCurrentFlyMode(AFunc_Land, MODE_CODE_REASON_RTL_HOME_LAND);
            }
            else
                setCurrentFlyMode(AFunc_Land);

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
            {
                step = 0;
                return FMR_OK;
            }

            // 获取对地高度
            bool precHeight_available = false;
            vector3<double> pos;
            get_Position_Ctrl(&pos);
            double height;
            if (precSensor_heightOffset > -100000000)
            { // 精准降落信息可用
                precHeight_available = true;
                height = pos.z - precSensor_heightOffset;
            }
            else
            { // 精准降落信息不可用

                // 计算距离地面高度
                if (rtl_to_startPoint)
                { // 返航回起飞点
                    double homeZ;
                    getHomeLocalZ(&homeZ);
                    vector3<double> pos;
                    get_Position_Ctrl(&pos);
                    height = pos.z - homeZ;

                    double homeSenosrZ;
                    if (getHomeOptSensorZ(&homeSenosrZ))
                    {
                        if (homeSenosrZ < height)
                            height = homeSenosrZ;
                    }
                }
                else
                { // 降落点非起飞点
                    height = pos.z - rtl_pos_z;
                }
                PosSensorHealthInf1 pos_inf;
                Position_Sensor_Data sensor;
                if (get_OptimalRange_Z(&pos_inf) && pos_inf.available && GetPositionSensorData(pos_inf.sensor_ind, &sensor))
                {
                    if (sensor.available)
                    {
                        if (sensor.position.z < height || rtl_pos_z <= 1000000000)
                            height = sensor.position.z;
                    }
                }
            }
            // 获取精准降落传感器
            bool precAltCtrl = false;
            precLandSensor precSensor;
            if (precLandSensor_get(&precSensor) && precSensor.available)
            {
                // 获取控制误差
                vector3<double> posErr;
                get_history_PositionErr(&posErr, 0);
                // 计算控制误差修正误差
                vector2<double> posErr_fixErr;
                posErr_fixErr.x = posErr.x - precCtrl_int_x;
                posErr_fixErr.y = posErr.y - precCtrl_int_y;

                // 计算修正率
                double kp_scale = 1.0;
                if (height > 1000)
                    kp_scale = 0.1;
                else if (height > 500)
                    kp_scale = 0.15;
                else if (height > 250)
                    kp_scale = 0.3;
                else
                {
                }
                if (precSensor.updateTime != last_precSensor_updTIME)
                {
                    // 重置精准降落上升重试步骤
                    precRetry_step = 0;

                    // 计算修正频率
                    double h;
                    if (last_precSensor_updTIME.is_valid() == false)
                        h = 0;
                    else
                        h = last_precSensor_updTIME.get_pass_time();
                    if (h > 1.5)
                        h = 1.5;
                    // 保存上次修正时间
                    last_precSensor_updTIME = precSensor.updateTime;

                    // 计算精准降落传感器基准的高度偏移
                    precSensor_heightOffset = pos.z + precSensor.pos.z;
                    precHeight_available = true;
                    height = -precSensor.pos.z;

                    // 修正偏航
                    if (precSensor.yaw_available && !precSensor_yawFixed && height > 300)
                    {
                        double ang = precSensor.yaw;
                        ang = fmod(ang, 2 * Pi);
                        if (ang > Pi)
                            ang -= 2 * Pi;
                        else if (ang < -Pi)
                            ang += 2 * Pi;
                        if (!precSensor_yawFixed)
                            Attitude_Control_set_Target_YawRelative(ang);
                        else
                            Attitude_Control_set_Target_YawRelative(0.2 * h * constrain(ang, degree2rad(30.0)));
                        precSensor_yawFixed = true;
                    }

                    // 计算误差
                    double err_x = precSensor.pos.x;
                    double err_y = precSensor.pos.y;
                    // 获取历史控制误差
                    vector3<double> his_posErr;
                    get_history_PositionErr(&his_posErr, 0);
                    // 误差积分2(控制误差之外的误差积分)
                    // 总误差(视觉误差) = 消除后的控制误差(posErr-precCtrl_int) + 其他误差(precCtrl_int2)
                    //										precCtrl_int2_x += h  * precInf.ctrlKi[0] * (err_x - posErr.x + precCtrl_int_x - precCtrl_int2_x);
                    //										precCtrl_int2_y += h  * precInf.ctrlKi[0] * (err_y - posErr.y + precCtrl_int_y - precCtrl_int2_x);
                    //										precCtrl_int2_x = constrain(precCtrl_int2_x, 50.0);
                    //										precCtrl_int2_y = constrain(precCtrl_int2_y, 50.0);
                    // 储存理论期望位置
                    //                    precCtrl_target_px = pos.x + err_x;
                    //                    precCtrl_target_py = pos.y + err_y;
                    precCtrl_target_px = precCtrl_target_x + precCtrl_int_x + precCtrl_int2_x + err_x - his_posErr.x;
                    precCtrl_target_py = precCtrl_target_y + precCtrl_int_y + precCtrl_int2_y + err_y - his_posErr.y;
                }

                // 误差积分
                if (fabs(precCtrl_int_x) > fabs(posErr.x) + 35)
                    precCtrl_int_x = sign(precCtrl_int_x) * (fabs(posErr.x) + 35);
                if (fabs(precCtrl_int_y) > fabs(posErr.y) + 35)
                    precCtrl_int_y = sign(precCtrl_int_y) * (fabs(posErr.y) + 35);
                precCtrl_int_x += h * precInf.ctrlKi[0] * constrain(posErr_fixErr.x, 50.0);
                precCtrl_int_y += h * precInf.ctrlKi[0] * constrain(posErr_fixErr.y, 50.0);
                precCtrl_int_x = constrain(precCtrl_int_x, 100.0);
                precCtrl_int_y = constrain(precCtrl_int_y, 100.0);
                // 将期望位置缓慢移动到理论目标位置
                double err_x = precCtrl_target_px - precCtrl_target_x;
                double err_y = precCtrl_target_py - precCtrl_target_y;
                precCtrl_target_x += kp_scale * precInf.ctrlKp[0] * h * (err_x);
                precCtrl_target_y += kp_scale * precInf.ctrlKp[0] * h * (err_y);

                // 误差过大停止下降
                if (height <= 250 && !precCtrl_TerminalLanding)
                {
                    if (sq(precSensor.pos.x) + sq(precSensor.pos.y) > sq(precInf.maxStaticErrRaw[0]) || posErr_fixErr.get_square() > sq(precInf.maxStaticErrCtrl[0]))
                    {
                        if (height > 25)
                        {
                            precAltCtrl = true;
                            Position_Control_set_ZLock();
                        }
                        else if (height > 15)
                        {
                            precAltCtrl = true;
                            Position_Control_set_TargetVelocityZ(30);
                        }
                    }
                }
                if (height < 15)
                {
                    precCtrl_TerminalLanding = true;
                }
            }
            else
            { // 精准降落传感器失效
                // last_precSensor_updTIME.set_invalid();

                if (precHeight_available)
                {
                    // 获取控制误差
                    vector3<double> posErr;
                    get_history_PositionErr(&posErr, 0);
                    // 计算控制误差修正误差
                    vector2<double> posErr_fixErr;
                    posErr_fixErr.x = posErr.x - precCtrl_int_x;
                    posErr_fixErr.y = posErr.y - precCtrl_int_y;
                    if (posErr_fixErr.get_square() > sq(precInf.maxStaticErrCtrl[0]) && !precCtrl_TerminalLanding)
                    {
                        if (height > 25)
                        {
                            precAltCtrl = true;
                            Position_Control_set_ZLock();
                        }
                        else if (height > 15)
                        {
                            precAltCtrl = true;
                            Position_Control_set_TargetVelocityZ(30);
                        }
                    }
                    if (height < 15)
                    {
                        precCtrl_TerminalLanding = true;
                    }
                    // 误差积分
                    if (fabs(precCtrl_int_x) > fabs(posErr.x) + 35)
                        precCtrl_int_x = sign(precCtrl_int_x) * (fabs(posErr.x) + 35);
                    if (fabs(precCtrl_int_y) > fabs(posErr.y) + 35)
                        precCtrl_int_y = sign(precCtrl_int_y) * (fabs(posErr.y) + 35);
                    precCtrl_int_x += h * precInf.ctrlKi[0] * constrain(posErr_fixErr.x, 50.0);
                    precCtrl_int_y += h * precInf.ctrlKi[0] * constrain(posErr_fixErr.y, 50.0);
                    precCtrl_int_x = constrain(precCtrl_int_x, 100.0);
                    precCtrl_int_y = constrain(precCtrl_int_y, 100.0);
                    // 将期望位置缓慢移动到理论目标位置
                    double err_x = precCtrl_target_px - precCtrl_target_x;
                    double err_y = precCtrl_target_py - precCtrl_target_y;
                    precCtrl_target_x += 0.35 * precInf.ctrlKp[0] * h * (err_x);
                    precCtrl_target_y += 0.35 * precInf.ctrlKp[0] * h * (err_y);
                }

                if (precRetry_step == 1)
                { // 上升重试
                    precAltCtrl = true;
                    if (last_precSensor_updTIME.get_pass_time() > 3.5)
                    { // 精准降落失败
                        if (last_precSensor_updTIME.get_pass_time() < 13.5)
                        { // 先上升重试
                            Position_Control_set_TargetVelocityZ(50);
                        }
                        else
                        { // 降落失败
                            al_type = get_ALInfo();
                            if (al_type != AL_TYPE::NONE)
                            { // 前往备降点
                                Position_Control_set_TargetPositionZRelative(AL_alt);

                                // 进入备降流程
                                step = 150;
                            }
                            else
                            { // 无备降点直接降落(分级降落速度)
                                float sp_f = getPosCtrlCfg()->maxAutoVelDown[0];
                                if (sp_f < 100)
                                    sp_f = 100;
                                Position_Control_set_TargetVelocityZ(-getLandVel(height, sp_f));
                            }
                        }
                    }
                    else
                        Position_Control_set_ZLock();
                }
                else if (precHeight_available && height > 45 && height < 135)
                { // 降落传感器出现过
                    if (++precRetry_counter >= 3)
                    {
                        al_type = get_ALInfo();
                        if (al_type != AL_TYPE::NONE)
                        { // 前往备降点
                            precAltCtrl = true;
                            Position_Control_set_TargetPositionZRelative(AL_alt);

                            // 进入备降流程
                            step = 150;
                        }
                    }
                    else
                        precRetry_step = 1;
                }
                else if (!precHeight_available && height < 700)
                { // 精准降落传感器未出现过
                    al_type = get_ALInfo();
                    if (al_type != AL_TYPE::NONE)
                    { // 前往备降点
                        precAltCtrl = true;
                        Position_Control_set_TargetPositionZRelative(AL_alt);

                        // 进入备降流程
                        step = 150;
                    }
                }
            }

            // 获取降落速度
            float sp = getPosCtrlCfg()->maxAutoVelDown[0];
            if (sp < 100)
                sp = 100;

            // 设置模式类别
            // 精准降落传感器可用时强制为自动模式
            // 否则允许手动操作
            if (precHeight_available)
                fmc = FMC_Automatic;
            else if (cfg->configs[0] & MCfg_RtlLandingAuto_Bit)
                fmc = FMC_Automatic;
            else
                fmc = FMC_SemiAutomatic;

            // 降落
            if (!precAltCtrl)
            {
                // 半自动模式下允许手动调整高度
                if (fmc != FMC_Automatic && rc.available && !in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
                {
                    // 油门杆控制高度(速度方式)
                    rcCtrl_Alt_v(rc, flag, comVars, msg, msgRs, cfg, avCfg, h);
                }
                else
                {
                    if (!precCtrl_TerminalLanding)
                    {
                        // 分级降落: 一级高度以上用一级降落速度, 二级高度以下用LandVel, 之间线性过渡
                        Position_Control_set_TargetVelocityZ(-getLandVel(height, sp));
                    }
                    else
                        Position_Control_set_TargetVelocityZ(-60);
                }
            }

            // 半自动模式下允许手动调整位置
            vector3<double> TVel;
            if (fmc != FMC_Automatic && rc.available && (!in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]) || !in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0])))
            {
                // 调整位置
                double roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]);
                double pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]);
                rcCtrl_Pos_p(roll_sitck_d, pitch_sitck_d, precCtrl_target_x, precCtrl_target_y, TVel.x, TVel.y, 0, 0, rc, flag, comVars, msg, msgRs, cfg, avCfg, h);
            }
            Position_Control_set_TargetPosVelAccXY_OffBoard(precCtrl_target_x + precCtrl_int_x + precCtrl_int2_x, precCtrl_target_y + precCtrl_int_y + precCtrl_int2_y, TVel.x, TVel.y, 0, 0);

            // 半自动模式下允许手动调整偏航
            if (fmc != FMC_Automatic)
            {
                // 偏航杆在中间锁偏航
                // 不在中间控制偏航速度
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                if (!rc.available || in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
                    Attitude_Control_set_YawLock();
                else
                    Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);
            }

            if (inFlight == false)
            { // 降落完成
                return FMR_RqDisarm_Land;
            }
            break;
        }

        // 备降
        case 150:
        { // 前往备降点

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
                step = 0;

            // 重置飞行模式类别
            fmc = FMC_Automatic;

            Position_ControlMode alt_mode;
            get_Altitude_ControlMode(&alt_mode);
            Position_Control_set_XYLock();
            if (alt_mode == Position_ControlMode_Position)
            {
                bool res = false;
                switch (al_type)
                {
                    case AL_TYPE::GLOBAL:
                    {
                        res = Position_Control_set_TargetPositionXY_LatLon(AL_lat, AL_lon);
                        Position_Control_set_ZLock();
                        break;
                    }
                    case AL_TYPE::RELATIVE:
                    {
                        res = Position_Control_set_TargetPositionXYRelative(AL_lat, AL_lon);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                if (res)
                    ++step;
                else
                    step = 152;
            }
            break;
        }
        case 151:
        { // 等待飞行完成

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
                step = 0;

            Position_ControlMode pos_mode;
            get_Position_ControlMode(&pos_mode);
            Position_Control_set_ZLock();
            if (pos_mode == Position_ControlMode_Position)
            {
                Position_Control_set_XYLock();
                ++step;
            }
            break;
        }
        case 152:
        { // 降落

            // 检查位置控制器是否打开
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);
            if (!pos_ena)
                step = 0;

            Position_Control_set_XYLock();

            // 获取对地高度
            bool precHeight_available = false;
            vector3<double> pos;
            get_Position_Ctrl(&pos);
            double height;
            if (precSensor_heightOffset > -100000000)
            { // 精准降落信息可用
                precHeight_available = true;
                height = pos.z - precSensor_heightOffset;
            }
            else
            { // 精准降落信息不可用
                height = pos.z - rtl_pos_z;
                if (rtl_to_startPoint)
                {
                    double homeSenosrZ;
                    if (getHomeOptSensorZ(&homeSenosrZ))
                    {
                        if (homeSenosrZ < height)
                            height = homeSenosrZ;
                    }
                }
                PosSensorHealthInf1 pos_inf;
                Position_Sensor_Data sensor;
                if (get_OptimalRange_Z(&pos_inf) && pos_inf.available && GetPositionSensorData(pos_inf.sensor_ind, &sensor))
                {
                    if (sensor.available)
                    {
                        if (sensor.position.z < height || rtl_pos_z <= 1000000000)
                            height = sensor.position.z;
                    }
                }
            }

            // 获取降落速度
            float sp = getPosCtrlCfg()->maxAutoVelDown[0];
            if (sp < 100)
                sp = 100;
            // 分级降落: 一级高度以上用一级降落速度, 二级高度以下用LandVel, 之间线性过渡
            Position_Control_set_TargetVelocityZ(-getLandVel(height, sp));

            if (inFlight == false)
            { // 降落完成
                return FMR_RqDisarm_Land;
            }
            break;
        }
    }

    return FMR_OK;
}