#include "FlightMode_PosHold.hpp"

#include "Avoidance.hpp"
#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "Sensors.hpp"
#include "basic.hpp"

FlightModeClass FlightMode_PosHold::get_fmClass()
{
    return FMC_Manual;
}

bool FlightMode_PosHold::set_params(double params[], uint16_t params_cnt)
{
    return false;
}

FlightModeCheckResult FlightMode_PosHold::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    switch (this->function_code)
    {
        case 1:
        { // 单独定高: 仅需高度定位
            if (get_Altitude_MSStatus() == MS_Ready)
                return FMCR_OK;
            else
                return FMCR_Err_NoPositioning;
            break;
        }
        case 3:
        { // 兼容定点+定高: 无高度定位不可运行; 有高度但无水平定位可降级为定高运行(警告级)
            if (get_Altitude_MSStatus() != MS_Ready)
                return FMCR_Err_NoPositioning;
            if (get_Position_MSStatus() == MS_Ready)
                return FMCR_OK;
            return FMCR_Warn_NoPositioning;
        }
        case 2:
        {
            if (get_Position_MSStatus() == MS_Ready)
                return FMCR_OK;
            else
                return FMCR_Err_NoPositioning;
            break;
        }
        default:
        {
            return FMCR_Err_ParamInvalid;
        }
    }
}

FlightMode_PosHold::FlightMode_PosHold(uint16_t function_code)
    : FlightMode_Base("PosHold", AFunc_PosHold, function_code)
{
    // 复位避障锁
    avLocked = false;
    fenceLocked = false;
    setObstacleAvoidanceHoverActive(false);
    // 复位仿地高度
    tf_height = -1;
    // 复位执行cmd状态
    memset(&cmdMsg, 0, sizeof(cmdMsg));
    cmdInProcess = false;
    // 复位退出模式计数器
    exit_counter1 = 0;
    exit_counter2 = 0;
    thr_raised = false;
}

FlightMode_PosHold::~FlightMode_PosHold()
{
    setObstacleAvoidanceHoverActive(false);
}

FlightModeResult FlightMode_PosHold::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                               const AvoidanceCfg *avCfg, double h)
{
    // 每周期将avLocked镜像到共享的避障悬停心跳标志。
    // 必须在周期最先执行, 这样同周期内NavCmd航线刹车上报(cmdInProcess路径)
    // 之后仍可将其置true。
    setObstacleAvoidanceHoverActive(avLocked);
    // 获取起飞状态
    bool inFlight;
    get_is_inFlight(&inFlight);
    // 计算模式运行频率
    const double freq = 1.0 / h;

    // 水平位置控制
    if (rc.available)
    {
        // 根据function_code打开位置控制器
        bool posEna = false;
        switch (this->function_code)
        {
            case 1:
            { // 单独定高
                Position_Control_Disable();
                Altitude_Control_Enable();
                bool altEna;
                is_Altitude_Control_Enabled(&altEna);
                is_Position_Control_Enabled(&posEna);
                // 位置控制器打不开失败
                if (!altEna)
                    return FMR_Err_NoPositioning;

                setCurrentFlyMode(AFunc_AltHold);
                break;
            }
            case 2:
            { // 单独定点
              // 打开位置控制器
                Position_Control_Enable();
                is_Position_Control_Enabled(&posEna);
                // 位置控制器打不开失败
                if (!posEna)
                    return FMR_Err_NoPositioning;

                setCurrentFlyMode(cmdInProcess ? AFunc_PosHoldOrbit : AFunc_PosHold);
                break;
            }
            case 3:
            { // 定高+定点
                Position_Control_Enable();
                Altitude_Control_Enable();
                bool altEna;
                is_Altitude_Control_Enabled(&altEna);
                is_Position_Control_Enabled(&posEna);
                // 位置控制器打不开失败
                if (!altEna)
                    return FMR_Err_NoPositioning;

                if (posEna)
                    setCurrentFlyMode(cmdInProcess ? AFunc_PosHoldOrbit : AFunc_PosHold);
                else
                    setCurrentFlyMode(AFunc_AltHold);
                break;
            }
            default:
            {
                return FMR_Err_ParamInvalid;
            }
        }

        if (posEna)
        {
            if (in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]) &&
                in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]))
            { // 摇杆在中位

                // 检查指令 如指令可用则执行
                if (msg)
                {
                    if (Process_InflightCmd(msg->cmd, msg->params) == false)
                    {
                        if (check_NavCmd(msg->cmd, msg->frame, msg->params))
                        { // 指令可被执行
                            if (!cmdInProcess || memcmp(&cmdMsg, msg, sizeof(cmdMsg)) != 0)
                            {
                                init_NavCmdInf(&navInf);
                                cmdInProcess = true;
                                cmdMsg = *msg;
                                if (msgRs)
                                    *msgRs = FMMR_Success;
                            }
                        }
                    }
                    else
                    {
                        if (msgRs)
                            *msgRs = FMMR_Success;
                    }
                }

                if (cmdInProcess)
                { // 需要执行NavCmd
                    int16_t res = -100;
                    res = Process_NavCmd(cmdMsg.cmd, freq, cmdMsg.frame, cmdMsg.params, &navInf, rc, flag, comVars, cfg, avCfg);
                    if (NavCmdRs_SuccessOrFault(res))
                    { // NavCmd执行完成

                        // 复位指令执行状态
                        memset(&cmdMsg, 0, sizeof(cmdMsg));
                        cmdInProcess = false;

                        if (res == -10 && !inFlight)
                        {
                            return FMR_RqDisarm_Land;
                        }
                    }
                }
            }
            else
            { // 摇杆不在中位
                // 复位指令执行状态
                memset(&cmdMsg, 0, sizeof(cmdMsg));
                cmdInProcess = false;
            }

            if (!cmdInProcess)
            {
                // 计算避障最大速度
                double maxVelXY = getPosCtrlCfg()->maxVelXY[0];
                // 获取当前速度
                vector3<double> velocityEnu;
                get_VelocityENU_Ctrl(&velocityEnu);

                // 计算目标速度
                double roll_sitck = rc.data[3] - 50.0;
                double pitch_sitck = rc.data[2] - 50.0;
                double roll_sitck_d = remove_deadband(roll_sitck, (double) cfg->NeutralZone[0]);
                double pitch_sitck_d = remove_deadband(pitch_sitck, (double) cfg->NeutralZone[0]);
                vector2<double> targetVel, targetVelNT;
                if (flag & FMF_HeadlessEna)
                {
                    if (!comVars.headless_inited)
                    {
                        flag = (FlightModeFlag) ((uint32_t) flag & (~(uint32_t) FMF_HeadlessEna));
                    }
                }
                if (flag & FMF_HeadlessEna)
                {
                    double sin_Yaw, cos_Yaw;
                    fast_sin_cos(comVars.headless_yaw, &sin_Yaw, &cos_Yaw);
                    targetVel.x = BodyHeading2ENU_x(pitch_sitck_d, -roll_sitck_d, sin_Yaw, cos_Yaw);
                    targetVel.y = BodyHeading2ENU_y(pitch_sitck_d, -roll_sitck_d, sin_Yaw, cos_Yaw);
                    targetVelNT.x = BodyHeading2ENU_x(pitch_sitck, -roll_sitck, sin_Yaw, cos_Yaw);
                    targetVelNT.y = BodyHeading2ENU_y(pitch_sitck, -roll_sitck, sin_Yaw, cos_Yaw);
                }
                else
                {
                    double yaw;
                    double yaw_declination;
                    get_YawDeclination(&yaw_declination);
                    Attitude_Control_get_TargetTrackYaw(&yaw);
                    yaw += yaw_declination;
                    double sin_Yaw, cos_Yaw;
                    fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                    targetVel.x = BodyHeading2ENU_x(pitch_sitck_d, -roll_sitck_d, sin_Yaw, cos_Yaw);
                    targetVel.y = BodyHeading2ENU_y(pitch_sitck_d, -roll_sitck_d, sin_Yaw, cos_Yaw);
                    targetVelNT.x = BodyHeading2ENU_x(pitch_sitck, -roll_sitck, sin_Yaw, cos_Yaw);
                    targetVelNT.y = BodyHeading2ENU_y(pitch_sitck, -roll_sitck, sin_Yaw, cos_Yaw);
                }

                // 计算避障
                // if (flag & FMF_AvoidanceEna)
                //{
                //    if (AvMode_Enabled(avCfg->AvoidanceMode[0]) || getAvTargetsCount() == 0)
                //    {
                //        flag = (FlightModeFlag)((uint32_t)flag & (~(uint32_t)FMF_AvoidanceEna));
                //    }
                //}
                if (flag & FMF_AvoidanceEna)
                { // 需要进行避障
                    if (!avLocked)
                    { // 未避障锁定才进行避障计算
                        double AvDistance = -1;
                        vector3<double> tVelVec(targetVelNT.x, targetVelNT.y, 0);
                        double avDist = avCfg->AvoidanceDist[0] + 0.5f * avCfg->wheelbase[0];
                        get_AvLineDistanceEnu(&AvDistance, tVelVec, avCfg->wheelbase[0]);

                        if (AvDistance >= 0)
                        {
                            tVelVec.normalize();
                            double currentVel = tVelVec * velocityEnu;
                            double sRm = AvDistance - avDist;
                            if (sq(currentVel) >= 2 * (0.5 * getPosCtrlCfg()->maxAccXY[0]) * sRm)
                            {
                                avLocked = true;
                                setObstacleAvoidanceHoverActive(true);
                            }
                        }
                    }
                }

                // 检测复杂围栏
                if (avCfg->fenceEnable[0] & FenceEnable_CpxFenceFlag)
                {
                    if (!avLocked)
                    {
                        vector3<double> pos;
                        get_Position_Ctrl(&pos);
                        vector3<double> tVelVec(targetVelNT.x, targetVelNT.y, 0);
                        // 判断距离围栏边界距离
                        double dis = -1;
                        bool res = is_insideFence(pos, tVelVec, &dis);
                        double avDist = 0.5f * avCfg->wheelbase[0];
                        if (res)
                        { // 在围栏内
                            // 判断边界距离刹车
                            if (dis >= 0)
                            { // 有围栏信息
                                tVelVec.normalize();
                                double currentVel = tVelVec * velocityEnu;
                                double sRm = dis - avDist;
                                if (sq(currentVel) >= 2 * (0.5 * getPosCtrlCfg()->maxAccXY[0]) * sRm)
                                {
                                    fenceLocked = true;
                                    FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], false);
                                    setFenceBreachActive(true, FENCE_BREACH_BOUNDARY, fenceInf.mitigation);
                                    if (fenceInf.modeRs != FMR_OK)
                                        return fenceInf.modeRs;
                                }
                            }
                        }
                        else if (dis < 0)
                        { // 在围栏外
                            // 如果速度方向朝向范围外则刹车
                            fenceLocked = true;
                            FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], false);
                            setFenceBreachActive(true, FENCE_BREACH_BOUNDARY, fenceInf.mitigation);
                            if (fenceInf.modeRs != FMR_OK)
                                return fenceInf.modeRs;
                        }
                    }
                }
                // 检测简单围栏
                if ((avCfg->fenceEnable[0] & FenceEnable_SplFenceFlag) && (avCfg->fenceRadius[0] > 0.5))
                {
                    if (!avLocked)
                    {
                        vector2<double> homeP;
                        if (getHomePoint(&homeP))
                        {
                            vector3<double> pos;
                            get_Position_Ctrl(&pos);
                            vector3<double> tVelVec(targetVelNT.x, targetVelNT.y, 0);
                            // 判断距离围栏边界距离
                            double dis = -1;
                            bool res = is_insideCircleFence(pos, tVelVec, homeP.x, homeP.y, avCfg->fenceRadius[0] * 100, &dis);
                            if (res)
                            { // 在围栏内
                                // 判断边界距离刹车
                                if (dis >= 0)
                                { // 有围栏信息
                                    double avDist = 0.5f * avCfg->wheelbase[0];
                                    tVelVec.normalize();
                                    double currentVel = tVelVec * velocityEnu;
                                    double sRm = dis - avDist;
                                    if (sq(currentVel) >= 2 * (0.5 * getPosCtrlCfg()->maxAccXY[0]) * sRm)
                                    {
                                        fenceLocked = true;
                                        FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], false);
                                        setFenceBreachActive(true, FENCE_BREACH_BOUNDARY, fenceInf.mitigation);
                                        if (fenceInf.modeRs != FMR_OK)
                                            return fenceInf.modeRs;
                                    }
                                }
                            }
                            else if (dis < 0)
                            { // 在围栏外
                                // 如果速度方向朝向范围外则刹车
                                fenceLocked = true;
                                FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], false);
                                setFenceBreachActive(true, FENCE_BREACH_BOUNDARY, fenceInf.mitigation);
                                if (fenceInf.modeRs != FMR_OK)
                                    return fenceInf.modeRs;
                            }
                        }
                    }
                }

                if ((avLocked || fenceLocked) && in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]))
                {
                    avLocked = false;
                    fenceLocked = false;
                    setObstacleAvoidanceHoverActive(false);
                    setFenceBreachActive(false, 0, 0, 0.1);
                }
                else if (fenceLocked)
                {
                    FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], false);
                    setFenceBreachActive(true, FENCE_BREACH_BOUNDARY, fenceInf.mitigation);
                }

                // 俯仰横滚杆控水平速度
                if (avLocked || fenceLocked)
                    Position_Control_set_XYLockFast();
                else if (in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]))
                    Position_Control_set_XYLock();
                else
                {
                    double RPCtrlScale = atan2(getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc);
                    double XYCtrlScale = maxVelXY / 50.0;
                    double vel_stick_err = safe_sqrt(sq(velocityEnu.x / XYCtrlScale - targetVel.x) + sq(velocityEnu.y / XYCtrlScale - targetVel.y));
                    if (vel_stick_err > 50)
                        vel_stick_err = 50;
                    targetVel *= XYCtrlScale;
                    targetVel.constrain(maxVelXY);
                    Position_Control_set_TargetVelocityXY_AngleLimit(targetVel.x, targetVel.y, vel_stick_err * RPCtrlScale);
                }
            }
            else
            { // 正在执行任务复位避障锁
                // NavCmd航线状态会自行刷新共享心跳标志,
                // 此处不再显式清零(立即写false会覆盖同周期的航线刹车上报)
                avLocked = false;
                fenceLocked = false;
                setFenceBreachActive(false);
            }
        }
        else
        { // 无定位控制姿态

            // 复位避障锁
            avLocked = false;
            fenceLocked = false;
            setObstacleAvoidanceHoverActive(false);
            setFenceBreachActive(false);

            // 复位指令执行状态
            memset(&cmdMsg, 0, sizeof(cmdMsg));
            cmdInProcess = false;

            // 补偿风力扰动
            vector3<double> WindDisturbance;
            get_WindDisturbance(&WindDisturbance);
            Quaternion attitude;
            get_Attitude_quat(&attitude);
            double yaw = attitude.getYaw();
            double sin_Yaw, cos_Yaw;
            fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
            double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x(WindDisturbance.x, WindDisturbance.y, sin_Yaw, cos_Yaw);
            double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y(WindDisturbance.x, WindDisturbance.y, sin_Yaw, cos_Yaw);

            double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxLean[0] / 50.0);
            //						Attitude_Control_set_Target_RollPitch(
            //							( rc.data[3] - 50
            //)*RPCtrlScale - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
            //							( rc.data[2] - 50
            //)*RPCtrlScale - atan2( WindDisturbance_Bodyheading_x , GravityAcc )
            //						);
            Attitude_Control_set_Target_RollPitch(remove_deadband(rc.data[3] - 50, cfg->NeutralZone[0]) * RPCtrlScale, remove_deadband(rc.data[2] - 50, cfg->NeutralZone[0]) * RPCtrlScale);
        }

        // 油门杆控制垂直速度
        if (in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
        {
            if (!cmdInProcess)
            {
                Position_ControlMode alt_mode;
                get_Altitude_ControlMode(&alt_mode);
                if ((flag & FMF_TerrainFollowingEna) && inFlight && (tf_height >= 0 || alt_mode == Position_ControlMode_Position))
                {
                    PosSensorHealthInf1 pos_inf;
                    get_OptimalRange_Z(&pos_inf);
                    Position_Sensor_Data sensor;
                    if (GetPositionSensorData(pos_inf.sensor_ind, &sensor))
                    { // 根据测高传感器调整对地高度

                        // 初始化仿地高度
                        if (tf_height < 0)
                            tf_height = sensor.position.z;

                        // 计算目标速度
                        double err = tf_height - sensor.position.z;
                        double maxVel;
                        if (err > 0)
                            maxVel = getPosCtrlCfg()->maxAutoVelUp[0] * 0.7;
                        else
                            maxVel = getPosCtrlCfg()->maxAutoVelDown[0] * 0.7;
                        Position_Control_set_TargetVelocityZ(constrain(err, maxVel));
                    }
                    else
                    { // 无测高传感器
                        tf_height = -1;
                        Position_Control_set_ZLock();
                    }
                }
                else
                { // 不需要跟随地形
                    tf_height = -1;
                    Position_Control_set_ZLock();
                }
            }
        }
        else
        {
            // 计算油门摇杆位置
            double thr_stick = remove_deadband(rc.data[0] - 50.0, (double) cfg->NeutralZone[0]);

            // 复位地形跟随高度
            tf_height = -1;

            // 围栏限高检测（每周期计算，触发/解除及时上报）
            bool fenceH_Lock = false;
            if (!cmdInProcess)
            {
                const AvoidanceCfg *avCfg = getAvCfg();
                if (((avCfg->fenceEnable[0] & FenceEnable_SplFenceFlag) || (avCfg->fenceEnable[0] & FenceEnable_CpxFenceFlag)) && (avCfg->fenceMaxAlt[0] > 0.5))
                { // 需要围栏限高
                    double homeZ = 0;
                    getHomeLocalZ(&homeZ, 0, 0.01);
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    if (position.z - homeZ > avCfg->fenceMaxAlt[0] * 100)
                        fenceH_Lock = true;

                    // 上报围栏触发状态
                    if (fenceH_Lock)
                    {
                        //                    FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], false);
                        setFenceBreachActive(true, FENCE_BREACH_MAXALT, FENCE_MITIGATE_HOVER);
                        //                    if (fenceInf.modeRs != FMR_OK)
                        //                        return fenceInf.modeRs;
                    }
                }
            }

            if (thr_stick > 0)
            { // 上升受围栏限高
                if (fenceH_Lock)
                    Position_Control_set_ZLock();
                else
                {
                    thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
                    Position_Control_set_TargetVelocityZ(thr_stick);
                }
            }
            else
            { // 下降不受围栏限制
                thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
                Position_Control_set_TargetVelocityZ(thr_stick);
            }
        }

        // 偏航杆在中间锁偏航
        // 不在中间控制偏航速度
        double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
        if (in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
        {
            if (!cmdInProcess)
                Attitude_Control_set_YawLock();
        }
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
    }
    else
    {
        // 失控后保持地形高
        if ((flag & FMF_TerrainFollowingEna) && inFlight && tf_height >= 0)
        {
            PosSensorHealthInf1 pos_inf;
            get_OptimalRange_Z(&pos_inf);
            Position_Sensor_Data sensor;
            if (GetPositionSensorData(pos_inf.sensor_ind, &sensor))
            { // 根据测高传感器调整对地高度

                // 计算目标速度
                double err = tf_height - sensor.position.z;
                double maxVel;
                if (err > 0)
                    maxVel = getPosCtrlCfg()->maxAutoVelUp[0] * 0.7;
                else
                    maxVel = getPosCtrlCfg()->maxAutoVelDown[0] * 0.7;
                Position_Control_set_TargetVelocityZ(constrain(err, maxVel));
            }
            else
            { // 无测高传感器
                tf_height = -1;
                Position_Control_set_ZLock();
            }
        }

        if (comVars.noRcStartTIME.get_pass_time() > 0.5)
        {
            // 无遥控信号进入安全模式
            Altitude_Control_Enable();
            Position_Control_Enable();
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);

            // 复位避障锁
            avLocked = false;
            fenceLocked = false;
            setObstacleAvoidanceHoverActive(false);
            setFenceBreachActive(false);

            if ((cfg->configs[0] & MCfg_PosNRcHold_Bit) && pos_ena)
            { // 失控悬停

                // 检查指令 如指令可用则执行
                if (msg)
                {
                    if (Process_InflightCmd(msg->cmd, msg->params) == false)
                    {
                        if (check_NavCmd(msg->cmd, msg->frame, msg->params))
                        { // 指令可被执行
                            if (!cmdInProcess || memcmp(&cmdMsg, msg, sizeof(cmdMsg)) != 0)
                            {
                                init_NavCmdInf(&navInf);
                                cmdInProcess = true;
                                cmdMsg = *msg;
                                if (msgRs)
                                    *msgRs = FMMR_Success;
                            }
                        }
                    }
                    else
                    {
                        if (msgRs)
                            *msgRs = FMMR_Success;
                    }
                }

                if (cmdInProcess)
                { // 需要执行NavCmd
                    setCurrentFlyMode(AFunc_PosHoldOrbit);
                    int16_t res = -100;
                    res = Process_NavCmd(cmdMsg.cmd, freq, cmdMsg.frame, cmdMsg.params, &navInf, rc, flag, comVars, cfg, avCfg);
                    if (NavCmdRs_SuccessOrFault(res))
                    { // NavCmd执行完成

                        // 复位指令执行状态
                        memset(&cmdMsg, 0, sizeof(cmdMsg));
                        cmdInProcess = false;
                        setCurrentFlyMode(AFunc_PosHold);

                        if (res == -10 && !inFlight)
                        {
                            return FMR_RqDisarm_Land;
                        }
                    }
                }
                else
                {
                    setCurrentFlyMode(AFunc_PosHold);
                    Position_Control_set_ZLock();
                    Position_Control_set_XYLock();
                    Attitude_Control_set_YawLock();
                }
            }
            else
            { // 失控请求返航
                Position_Control_set_ZLock();
                Position_Control_set_XYLock();
                Attitude_Control_set_YawLock();

                float nRcT = 2;
                if (cfg->nRcT[0] > 0)
                    nRcT = cfg->nRcT[0];
                if (comVars.noRcStartTIME.get_pass_time() > nRcT)
                {
                    if ((cfg->configs[0] & MCfg_NRcLand_Bit))
                        return FMR_Err_RqSwitchLand;
                    else
                        return FMR_Err_RqSwitchRTL;
                }
            }
        }
    }
    return FMR_OK;
}