#include "FlightMode_Circle.hpp"

#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "StorageSystem.hpp"
#include "basic.hpp"

FlightModeClass FlightMode_Circle::get_fmClass()
{
    return FMC_SemiAutomatic;
}

bool FlightMode_Circle::set_params(double params[], uint16_t params_cnt)
{
    if (params_cnt == 7)
    {
        frame = (MAV_FRAME) params[0];
        switch (frame)
        {
            case MAV_FRAME_GLOBAL:
            case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            case MAV_FRAME_GLOBAL_INT:
            case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            case MAV_FRAME_GLOBAL_TERRAIN_ALT:
            case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
            case MAV_FRAME_LOCAL_NED:
            case MAV_FRAME_LOCAL_ENU:
            {
                break;
            }
            default:
                return false;
        }
        rO.x = params[1];
        rO.y = params[2];
        rO.z = params[3];
        rYaw = params[4];
        rSp = params[5];
        r = params[6];
        paramSet = true;

        return true;
    }
    return false;
}

FlightModeCheckResult FlightMode_Circle::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    bool inFlight;
    get_is_inFlight(&inFlight);

    if (!inFlight)
        return FMCR_Err_NotTakeOff;
    if (!paramSet)
        return FMCR_Err_ParamInvalid;
    if (get_Position_MSStatus() != MS_Ready)
        return FMCR_Err_NoPositioning;

    return FMCR_OK;
}

FlightMode_Circle::FlightMode_Circle()
    : FlightMode_Base("Circle", AFunc_Circle, 0)
{
    // 复位参数
    paramSet = false;
    // 复位初始速度
    sp_filted = 0;
    // 复位退出模式计数器
    exit_counter1 = 0;
    exit_counter2 = 0;
    thr_raised = false;
    // 复位状态机
    step = 0;
}

FlightMode_Circle::~FlightMode_Circle() {}

FlightModeResult FlightMode_Circle::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                              const AvoidanceCfg *avCfg, double h)
{
    // 获取起飞状态
    bool inFlight;
    get_is_inFlight(&inFlight);
    // 计算模式运行频率
    const double freq = 1.0 / h;

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

    switch (step)
    {
        case 0:
        {
            // 上报模式状态
            setCurrentFlyMode(AFunc_Circle);

            // 打开位置控制器
            Altitude_Control_Enable();
            Position_Control_Enable();

            // 获取位置控制状态
            Position_ControlMode pos_mode;
            get_Position_ControlMode(&pos_mode);

            // 等待刹车完成
            Position_Control_set_XYLock();
            if (pos_mode == Position_ControlMode_Position)
            {
                // 求圆心坐标
                switch (frame)
                {
                    case MAV_FRAME_GLOBAL:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    case MAV_FRAME_GLOBAL_INT:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    {
                        get_localXY_fromOptimalGlobal(rO.x, rO.y, &origin.x, &origin.y);
                        break;
                    }
                    case MAV_FRAME_LOCAL_ENU:
                    {
                        origin.x = rO.x;
                        origin.y = rO.y;
                        break;
                    }
                    case MAV_FRAME_LOCAL_NED:
                    {
                        origin.x = rO.y;
                        origin.y = rO.x;
                        break;
                    }
                    default:
                        return FMR_Err_ParamInvalid;
                }

                // 转向
                if (rYaw >= 1000)
                    Attitude_Control_set_Target_Yaw(degree2rad(90 - (rYaw - 1000)));
                else if (rYaw <= -1000)
                    Attitude_Control_set_Target_Yaw(degree2rad(90 - (rYaw + 1000)));
                else
                { // 指向圆心为0度
                    vector3<double> pos;
                    get_Position_Ctrl(&pos);
                    double dir = atan2(origin.y - pos.y, origin.x - pos.x);
                    Attitude_Control_set_Target_Yaw(degree2rad(rYaw) + dir);
                }
                ++step;
            }
            break;
        }

        case 1:
        {
            // 等待转向完成
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                if (!isvalid(r))
                { // 半径自适应
                }
                else
                { // 移动到指定半径位置
                    vector3<double> pos;
                    get_Position_Ctrl(&pos);
                    vector2<double> dirVec(origin.x - pos.x, origin.y - pos.y);
                    double dirVelLen = dirVec.get_square();
                    if (dirVelLen > 1 && r > 1)
                    {
                        dirVelLen = safe_sqrt(dirVelLen);
                        vector2<double> dirVec_normed = dirVec * (1.0 / dirVelLen);
                        vector2<double> relVec = dirVec_normed * (dirVelLen - r);
                        Position_Control_set_TargetPositionXYRelative(relVec.x, relVec.y);
                    }
                    else
                        return FMR_Err_ParamInvalid;
                }
                ++step;
            }
            break;
        }

        case 2:
        {
            // 等待位置调整完成
            Position_ControlMode pos_mode;
            get_Position_ControlMode(&pos_mode);
            if (pos_mode == Position_ControlMode_Position)
            {
                ++step;
            }
            break;
        }

        case 3:
        {
            double circleY;
            if (rYaw >= 1000)
                circleY = degree2rad(90 - (rYaw - 1000)) + 1000;
            else if (rYaw <= -1000)
                circleY = degree2rad(90 - (rYaw + 1000)) + 1000;
            else // 指向圆心为0度
                circleY = degree2rad(rYaw);
            bool res = Position_Control_do_Circle(sp_filted, origin, circleY);
            if (res)
            {
                if (rSp >= 0)
                {
                    sp_filted += 50 * h;
                    if (sp_filted >= rSp)
                    {
                        sp_filted = rSp;
                        ++step;
                    }
                }
                else
                {
                    sp_filted -= 50 * h;
                    if (sp_filted <= rSp)
                    {
                        sp_filted = rSp;
                        ++step;
                    }
                }
            }
            break;
        }

        case 4:
        {
            // 进行绕圈调整
            double roll_sitck_d = 0;
            double pitch_sitck_d = 0;
            double yaw_sitck_d = 0;
            if (rc.available)
            {
                roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]);
                pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]);
                yaw_sitck_d = remove_deadband(rc.data[1] - 50.0, (double) cfg->NeutralZone[0]);
            }
            Position_Control_do_ManualCircleRelative(5 * roll_sitck_d * h, -3 * pitch_sitck_d * h, 3 * yaw_sitck_d * h);
            break;
        }
    }

    // 控制高度
    if (in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
        Position_Control_set_ZLock();
    else
    {
        double thr_stick = remove_deadband(rc.data[0] - 50.0, (double) cfg->NeutralZone[0]);

        // 获取避障参数
        const AvoidanceCfg *avCfg = getAvCfg();
        if (((avCfg->fenceEnable[0] & FenceEnable_SplFenceFlag) || (avCfg->fenceEnable[0] & FenceEnable_CpxFenceFlag)) && (avCfg->fenceMaxAlt[0] > 0.5))
        { // 需要围栏限高
            // 计算是否超出围栏高度
            bool lockH = false;
            double homeZ = 0;
            getHomeLocalZ(&homeZ, 0, 0.01);
            vector3<double> position;
            get_Position_Ctrl(&position);
            double heightAboveGround = position.z - homeZ;
            if (heightAboveGround > avCfg->fenceMaxAlt[0] * 100)
                lockH = true;

            // 上报围栏触发状态
            if (lockH)
            {
                FenceActionInf fenceInf = get_FenceActionInf(avCfg->fenceAction[0], true);
                setFenceBreachActive(true, FENCE_BREACH_MAXALT, fenceInf.mitigation);
                if (fenceInf.modeRs != FMR_OK)
                    return fenceInf.modeRs;
            }

            if (thr_stick > 0)
            {
                if (lockH)
                    Position_Control_set_ZLock();
                else
                {
                    thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
                    Position_Control_set_TargetVelocityZ(thr_stick);
                }
            }
            else
            {
                thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
                Position_Control_set_TargetVelocityZ(thr_stick);
            }
        }
        else
        { // 不需要围栏
            if (thr_stick > 0)
                thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
            else
                thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
            Position_Control_set_TargetVelocityZ(thr_stick);
        }
    }

    return FMR_OK;
}