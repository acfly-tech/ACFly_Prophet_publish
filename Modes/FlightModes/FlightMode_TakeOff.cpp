#include "FlightMode_TakeOff.hpp"

#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "basic.hpp"
#include "followTarget.hpp"

FlightModeClass FlightMode_TakeOff::get_fmClass()
{
    return FMC_Automatic;
}

bool FlightMode_TakeOff::set_params(double params[], uint16_t params_cnt)
{
    if (params_cnt == 2 && (params[0] == 1 || params[0] == 2))
    {
        if (params[0] == 1)
            relAlt = false;
        else
            relAlt = true;
        height = params[1];
        paramSet = true;

        return true;
    }
    return false;
}

FlightModeCheckResult FlightMode_TakeOff::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    bool inFlight;
    get_is_inFlight(&inFlight);

    if (inFlight)
        return FMCR_Err_TakenOff;
    if (!paramSet)
        return FMCR_Err_ParamInvalid;
    if (get_Position_MSStatus() != MS_Ready)
        return FMCR_Err_NoPositioning;

    return FMCR_OK;
}

FlightMode_TakeOff::FlightMode_TakeOff()
    : FlightMode_Base("TakeOff", AFunc_TakeOff, 0)
{
    // 复位参数
    paramSet = false;
    // 复位跟随传感器编号
    lastFollowSInd = -1;
    // 复位跟随传感器类型
    followType = FOLLOW_TYPE::NONE;
    // 复位状态机
    step = 0;
}

FlightMode_TakeOff::~FlightMode_TakeOff() {}

FlightModeResult FlightMode_TakeOff::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                               const AvoidanceCfg *avCfg, double h)
{
    switch (step)
    {
        case 0:
        {
            // 上报模式状态
            setCurrentFlyMode(AFunc_TakeOff);

            // 打开位置控制器
            Altitude_Control_Enable();
            Position_Control_Enable();
            bool res = false;
            if (!relAlt)
                res = Position_Control_Takeoff_HeightGlobal(height);
            else
                res = Position_Control_Takeoff_HeightRelative(height);
            if (!res)
                return FMR_Err_ParamInvalid;

            // 初始化刹车点
            vector3<double> pos;
            get_Position_Ctrl(&pos);
            vector3<double> vel;
            get_VelocityENU_Ctrl(&vel);
            double stopT = 0.5 * safe_sqrt(vel.get_square()) / (0.5 * getPosCtrlCfg()->maxAccXY[0]);
            followPos = pos + vel * stopT;
            // 复位跟随
            followType = FOLLOW_TYPE::NONE;
            followTPos.zero();
            followTVel.zero();
            // 复位偏航
            Attitude_Control_set_YawLock();
            followYawType = last_followYawType = FOLLOW_YAW_TYPE::NONE;

            ++step;
            break;
        }
        case 1:
        {
            // 期望加速度
            vector3<double> TAcc;
            // 尝试获取跟随传感器
            followSensor followS;
            uint16_t followSInd = get_followSensor(&followS);

            // 获取起飞状态
            bool inFlight;
            get_is_inFlight(&inFlight);

            if (followSInd && followS.available && followS.updateTime.get_pass_time() < 2 && (!isFollowDataType_GlobalXY(followS.dataType) || followS.globalXYPosAvailable))
            { // 跟随可用
                if (isFollowDataType_Point(followS.dataType))
                { // 点追踪
                    if (lastFollowSInd != followSInd || followType != FOLLOW_TYPE::POINT || !inFlight)
                    { // 传感器发生了更改
                      // 如果之前跟随过则重置目标点
                        if (followType != FOLLOW_TYPE::NONE)
                        {
                            followPos += followTPos;
                            followType = FOLLOW_TYPE::NONE;
                        }
                        lastFollowSInd = followSInd;
                        // 初始化传感器跟随原点
                        followOrigin.x = followS.pos.x;
                        followOrigin.y = followS.pos.y;
                        followOrigin.z = followS.pos.z;
                        // 复位跟随位置
                        last_followTPos.zero();
                        // 设置跟随类型
                        followType = FOLLOW_TYPE::POINT;
                    }
                    // 计算传感器数据滞后的时间
                    double upT = followS.updateTime.get_pass_time() + followS.delay;
                    // 速度*时间 插值 平滑传感器期望位置
                    followTPos.x = followS.pos.x + followS.vel.x * upT;
                    followTPos.y = followS.pos.y + followS.vel.y * upT;
                    followTPos.z = followS.pos.z + followS.vel.z * upT;
                    // 期望位置减去期望位置原点
                    followTPos -= followOrigin;
                    // 更新传感器期望速度
                    followTVel.x = followS.vel.x;
                    followTVel.y = followS.vel.y;
                    followTVel.z = followS.vel.z;
                    // 跟随期望加速度
                    TAcc.x = followS.acc.x;
                    TAcc.y = followS.acc.y;
                    TAcc.z = followS.acc.z;
                    // 复位
                    if (followS.yawAvailable())
                    {
                        followYawType = FOLLOW_YAW_TYPE::FOLLOW;
                        followYaw = followS.yaw;
                        followYawRate = followS.yawRate;
                    }
                    else
                        followYawType = FOLLOW_YAW_TYPE::NONE;
                }
                else
                { // 跟随不可用
                    if (followType != FOLLOW_TYPE::NONE)
                    { // 重置当前目标点
                        followPos += followTPos;
                        followTPos.zero();
                        followTVel.zero();
                        followType = FOLLOW_TYPE::NONE;
                        followYawType = FOLLOW_YAW_TYPE::NONE;
                    }
                }
            }
            else
            { // 跟随不可用
                if (followType != FOLLOW_TYPE::NONE)
                { // 重置当前目标点
                    followPos += followTPos;
                    followTPos.zero();
                    followTVel.zero();
                    followType = FOLLOW_TYPE::NONE;
                    followYawType = FOLLOW_YAW_TYPE::NONE;
                }
            }

            // 未起飞初始化followPos
            if (!inFlight)
            {
                // 初始化刹车点
                vector3<double> pos;
                get_Position_Ctrl(&pos);
                vector3<double> vel;
                get_VelocityENU_Ctrl(&vel);
                double stopT = 0.5 * safe_sqrt(vel.get_square()) / (0.5 * getPosCtrlCfg()->maxAccXY[0]);
                followPos = pos + vel * stopT;
            }

            // 偏航模式变化重置偏航状态
            if (followYawType != last_followYawType)
            {
                followYawOffset = 0;
                last_followYawType = followYawType;
            }

            double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
            if (followYawType == FOLLOW_YAW_TYPE::FOLLOW_OFFSET && rc.available)
            {
                followYawOffset += (50 - rc.data[1]) * YCtrlScale * h;
                if (followYawOffset > Pi)
                    followYawOffset -= 2 * Pi;
                else if (followYawOffset < -Pi)
                    followYawOffset += 2 * Pi;
            }
            if (followYawType == FOLLOW_YAW_TYPE::FOLLOW)
                Attitude_Control_set_Target_Yaw_Offboard(followYaw, followYawRate);
            else if (followYawType == FOLLOW_YAW_TYPE::FOLLOW_OFFSET)
                Attitude_Control_set_Target_Yaw_Offboard(followYaw + followYawOffset, followYawRate);
            else
            { // 偏航不跟踪
                if (!rc.available || in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
                    Attitude_Control_set_YawLock();
                else
                    Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);
            }

            // XY速度前馈
            vector3<double> followTV;
            if (isFollowDataType_XY(followS.dataType))
            { // 跟随目标可用
                followTV.x += followTVel.x;
                followTV.y += followTVel.y;
            }

            // 计算期望位置速度
            double thr_stick_d = 0;
            double roll_sitck_d = 0;
            double pitch_sitck_d = 0;
            //			if (rc.available)
            //			{
            //				thr_stick_d = remove_deadband(rc.data[0] - 50.0, (double)cfg->NeutralZone[0]);
            //				roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double)cfg->NeutralZone[0]);
            //				pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double)cfg->NeutralZone[0]);
            //			}
            vector3<double> TVel;
            rcCtrl_Pos_p(roll_sitck_d, pitch_sitck_d, followPos.x, followPos.y, TVel.x, TVel.y, followTV.x, followTV.y, rc, flag, comVars, msg, msgRs, cfg, avCfg, h);

            vector3<double> TPos;
            TPos.x = followPos.x;
            TPos.y = followPos.y;

            // XY位置跟随
            if (isFollowDataType_XY(followS.dataType))
            { // 跟随目标可用
                TPos.x += followTPos.x;
                TPos.y += followTPos.y;
            }

            // 水平控制
            Position_Control_set_TargetPosVelAccXY_OffBoard(TPos.x, TPos.y, TVel.x, TVel.y, TAcc.x, TAcc.y);

            // 等待起飞完成
            Position_ControlMode alt_mode;
            get_Altitude_ControlMode(&alt_mode);
            if (alt_mode == Position_ControlMode_Position)
            {
                if (followType != FOLLOW_TYPE::NONE)
                    return FMR_OK_RqSwitchLoiter;
                else
                    return FMR_OK_RqSwitchMode;
            }

            break;
        }
    }

    return FMR_OK;
}