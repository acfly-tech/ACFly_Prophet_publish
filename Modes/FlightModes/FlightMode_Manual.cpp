#include "FlightMode_Manual.hpp"

#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "basic.hpp"

FlightModeClass FlightMode_Manual::get_fmClass()
{
    return FMC_Manual;
}

bool FlightMode_Manual::set_params(double params[], uint16_t params_cnt)
{
    return false;
}

FlightModeCheckResult FlightMode_Manual::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    switch (this->function_code)
    {
        case 1:
        case 2:
        case 3:
        {
            return FMCR_OK;
        }
        default:
        {
            return FMCR_Err_ParamInvalid;
        }
    }
    return FMCR_Err_ParamInvalid;
}

FlightMode_Manual::FlightMode_Manual(uint16_t function_code)
    : FlightMode_Base("Manual", AFunc_Manual, function_code)
{
    Attitude_Control_set_YawLock();

    // 复位初始油门计数器
    mthr_prepCounter = 0;
    // 复位退出模式计数器
    exit_counter1 = 0;
    exit_counter2 = 0;
}

FlightMode_Manual::~FlightMode_Manual() {}

FlightModeResult FlightMode_Manual::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                              const AvoidanceCfg *avCfg, double h)
{
    // 计算模式运行频率
    const double freq = 1.0 / h;

    // 获取起飞状态
    bool inFlight;
    get_is_inFlight(&inFlight);

    // 校验function_code
    switch (this->function_code)
    {
        case 1:
        { // 半自稳
            setCurrentFlyMode(AFunc_Manual);
            break;
        }
        case 2:
        { // 增稳
            setCurrentFlyMode(AFunc_Stabilize);
            break;
        }
        case 3:
        { // 手动
            setCurrentFlyMode(AFunc_Rattitude);
            break;
        }
        default:
        {
            return FMR_Err_ParamInvalid;
        }
    }

    // 水平位置控制
    if (rc.available)
    { // 手动飞行

        // 关闭高度控制器仅打开姿态控制器
        Altitude_Control_Disable();
        Attitude_Control_Enable();

        // 获取控制器打开状态
        bool posEna;
        bool altEna;
        is_Altitude_Control_Enabled(&altEna);
        is_Position_Control_Enabled(&posEna);

        // 初始化初始油门
        if (altEna)
            mthr_prepCounter = 0;
        if (mthr_prepCounter == 0)
            get_Target_Throttle(&mthr_thrFilted);

        // 判断锁定
        if (!inFlight)
        {
            if (rc.data[0] < 15)
            {
                ++exit_counter1;
                if (comVars.taken_off)
                {
                    if (exit_counter1 > 0.15 * freq)
                        return FMR_RqDisarm_Land;
                }
                else
                {
                    if (exit_counter1 > 30.0 * freq)
                        return FMR_RqDisarm_Land;
                }
            }
            else
                exit_counter1 = 0;

            if (rc.data[0] < 15 && rc.data[1] < 15)
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

        if (altEna)
        { // 定位还未关闭(通常因为仍然在安全模式下)

            // 油门杆控制垂直速度
            if (in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
                Position_Control_set_ZLock();
            else
            {
                double thr_stick = remove_deadband(rc.data[0] - 50.0, (double) cfg->NeutralZone[0]);
                Position_Control_set_TargetVelocityZ(thr_stick);
            }

            // 偏航杆在中间锁偏航
            // 不在中间控制偏航速度
            double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
            if (in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
                Attitude_Control_set_YawLock();
            else
                Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);

            if (posEna)
            {
                if (in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]))
                    Position_Control_set_XYLock();
                else
                {
                    // 计算摇杆位置
                    double roll_sitck = rc.data[3] - 50.0;
                    double pitch_sitck = rc.data[2] - 50.0;
                    double roll_sitck_d = remove_deadband(roll_sitck, (double) cfg->NeutralZone[0]);
                    double pitch_sitck_d = remove_deadband(pitch_sitck, (double) cfg->NeutralZone[0]);

                    // 计算目标速度
                    vector2<double> targetVel;
                    double sin_Yaw, cos_Yaw;
                    fast_sin_cos(comVars.headless_yaw, &sin_Yaw, &cos_Yaw);
                    targetVel.x = BodyHeading2ENU_x(pitch_sitck_d, -roll_sitck_d, sin_Yaw, cos_Yaw);
                    targetVel.y = BodyHeading2ENU_y(pitch_sitck_d, -roll_sitck_d, sin_Yaw, cos_Yaw);

                    // 获取当前速度
                    vector3<double> velocityEnu;
                    get_VelocityENU_Ctrl(&velocityEnu);
                    // 获取最大速度
                    double maxVelXY = getPosCtrlCfg()->maxVelXY[0];
                    // 计算输出速度舵量
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
            {
                double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxLean[0] / 50.0);
                Attitude_Control_set_Target_RollPitch(remove_deadband(rc.data[3] - 50, cfg->NeutralZone[0]) * RPCtrlScale, remove_deadband(rc.data[2] - 50, cfg->NeutralZone[0]) * RPCtrlScale);
            }
        }
        else
        { // 已进入手动控制

            // 计算输出油门
            double targetThr = rc.data[0];
            if (abs(mthr_prepCounter) >= 100000)
                Attitude_Control_set_Throttle(targetThr);
            else
            { // 刚进入模式时油门缓慢变化防止高度变化太剧烈
                if (abs(mthr_prepCounter) >= 2.5 * freq)
                    mthr_prepCounter = 1000000;

                if (targetThr > mthr_thrFilted && mthr_prepCounter >= 0)
                    ++mthr_prepCounter;
                else if (targetThr < mthr_thrFilted && mthr_prepCounter <= 0)
                    --mthr_prepCounter;
                else
                    mthr_prepCounter = 1000000;

                mthr_thrFilted += mthr_filter_P * h * (targetThr - mthr_thrFilted);
                Attitude_Control_set_Throttle(mthr_thrFilted);
            }

            // 俯仰横滚杆控俯仰横滚
            if (this->function_code == 3)
            { // 半自稳
                double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxRPSp[0] / 50.0);
                double outRol = remove_deadband(rc.data[3] - 50.0, (double) cfg->ratitudeZone[0]);
                bool outRolZero = (outRol == 0);
                outRol *= 1.0 / (50.0 - (double) cfg->NeutralZone[0]);
                outRol = sign(outRol) * pow(fabs(outRol), (double) cfg->RolRateE[0]);
                outRol *= 50.0;
                double outPit = remove_deadband(rc.data[2] - 50.0, (double) cfg->ratitudeZone[0]);
                bool outPitZero = (outPit == 0);
                outPit *= 1.0 / (50.0 - (double) cfg->NeutralZone[0]);
                outPit = sign(outPit) * pow(fabs(outPit), (double) cfg->PitRateE[0]);
                outPit *= 50.0;
                if (outRolZero && outPitZero)
                { // 半自稳回平
                    Attitude_Control_set_Target_RollPitch(remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]) * RPCtrlScale,
                                                          remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]) * RPCtrlScale);
                }
                else
                {
                    Attitude_Control_set_Target_RollPitchRate(outRol * RPCtrlScale, outPit * RPCtrlScale);
                }
            }
            else if (this->function_code == 1)
            { // 手动
                double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxRPSp[0] / 50.0);
                double outRol = remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]);
                outRol *= 1.0 / (50.0 - (double) cfg->NeutralZone[0]);
                outRol = sign(outRol) * pow(fabs(outRol), (double) cfg->RolRateE[0]);
                outRol *= 50.0;
                double outPit = remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]);
                outPit *= 1.0 / (50.0 - (double) cfg->NeutralZone[0]);
                outPit = sign(outPit) * pow(fabs(outPit), (double) cfg->PitRateE[0]);
                outPit *= 50.0;
                Attitude_Control_set_Target_RollPitchRate(outRol * RPCtrlScale, outPit * RPCtrlScale);
            }
            else
            {
                double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxLean[0] / 50.0);
                Attitude_Control_set_Target_RollPitch(remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]) * RPCtrlScale,
                                                      remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]) * RPCtrlScale);
            }

            // 偏航杆在中间锁偏航
            // 不在中间控制偏航速度
            if (this->function_code == 1)
            { // 手动
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                double out = remove_deadband(50.0 - rc.data[1], (double) cfg->NeutralZone[0]);
                out *= 1.0 / (50.0 - (double) cfg->NeutralZone[0]);
                out = sign(out) * pow(fabs(out), (double) cfg->YawRateE[0]);
                out *= 50.0;
                Attitude_Control_set_Target_YawRate(out * YCtrlScale);
            }
            else
            {
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                if (in_symmetry_range_mid(rc.data[1], 50.0, (double) cfg->NeutralZone[0]))
                    Attitude_Control_set_YawLock();
                else
                    Attitude_Control_set_Target_YawRate((50.0 - rc.data[1]) * YCtrlScale);
            }
        }
    }
    else
    { // 失控
        if (comVars.noRcStartTIME.get_pass_time() > 0.5)
        {
            // 复位初始油门计数器
            mthr_prepCounter = 0;

            // 无遥控信号进入安全模式
            Altitude_Control_Enable();
            Position_Control_Enable();
            bool pos_ena;
            is_Position_Control_Enabled(&pos_ena);

            if ((cfg->configs[0] & MCfg_PosNRcHold_Bit) && pos_ena)
            { // 失控悬停
                Position_Control_set_ZLock();
                Position_Control_set_XYLock();
                Attitude_Control_set_YawLock();
            }
            else
            { // 失控请求返航
                if (comVars.noRcStartTIME.get_pass_time() > 2.0)
                    return FMR_OK_RqSwitchRTL;
            }
        }
    }
    return FMR_OK;
}