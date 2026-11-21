#include "FlightMode_Follow.hpp"

#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "StorageSystem.hpp"
#include "basic.hpp"
#include "followTarget.hpp"

FlightModeClass FlightMode_Follow::get_fmClass()
{
    return FMC_SemiAutomatic;
}

bool FlightMode_Follow::set_params(double params[], uint16_t params_cnt)
{
    return false;
}

FlightModeCheckResult FlightMode_Follow::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    switch (this->function_code)
    {
        case 1:
        case 3:
        {
            if (get_Altitude_MSStatus() == MS_Ready)
                return FMCR_OK;
            else
                return FMCR_Err_NoPositioning;
            break;
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

FlightMode_Follow::FlightMode_Follow(uint16_t function_code)
    : FlightMode_Base("Follow", AFunc_Loiter, function_code)
{
    // 复位状态机
    step = 0;
    // 复位跟随传感器编号
    lastFollowSInd = -1;
    // 复位跟随传感器类型
    followType = FOLLOW_TYPE::NONE;
    // 复位退出模式计数器
    exit_counter1 = 0;
    exit_counter2 = 0;
    thr_raised = false;
}

FlightMode_Follow::~FlightMode_Follow() {}

FlightModeResult FlightMode_Follow::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                              const AvoidanceCfg *avCfg, double h)
{
    // 计算模式运行频率
    const double freq = 1.0 / h;

    // 根据function_code打开位置控制器
    bool posEna = false;
    switch (this->function_code)
    {
        case 1:
        { // 无定位跟随
            Position_Control_Disable();
            Altitude_Control_Enable();
            bool altEna;
            is_Altitude_Control_Enabled(&altEna);
            // 位置控制器打不开失败
            if (!altEna)
                return FMR_Err_NoPositioning;

            setCurrentFlyMode(AFunc_Loiter);
            break;
        }
        case 2:
        { // 有定位跟随
          // 打开位置控制器
            Position_Control_Enable();
            is_Position_Control_Enabled(&posEna);
            // 位置控制器打不开失败
            if (!posEna)
                return FMR_Err_NoPositioning;

            setCurrentFlyMode(AFunc_Loiter);
            break;
        }
        case 3:
        { // 兼容有定位和无定位跟随
            Position_Control_Enable();
            Altitude_Control_Enable();
            bool altEna;
            is_Altitude_Control_Enabled(&altEna);
            is_Position_Control_Enabled(&posEna);
            // 位置控制器打不开失败
            if (!altEna)
                return FMR_Err_NoPositioning;

            if (posEna)
                setCurrentFlyMode(AFunc_Loiter);
            else
                setCurrentFlyMode(AFunc_Loiter);
            break;
        }
        default:
        {
            return FMR_Err_ParamInvalid;
        }
    }

    // 获取起飞状态
    bool inFlight;
    get_is_inFlight(&inFlight);

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

    //// 未起飞重置所有状态
    // if( !inFlight)
    //   step = 0;

    if (posEna)
    { // 定位情况下追踪

        // 判断执行初始化操作
        switch (step)
        {
            default:
            { // 之前不在定位跟踪状态
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
                followVelFilted.zero();
                // 复位偏航
                Attitude_Control_set_YawLock();
                followYawType = last_followYawType = FOLLOW_YAW_TYPE::NONE;
                // 初始化step
                step = 100;
                break;
            }
            case 100:
            {
                break;
            }
        }

        // 期望加速度
        vector3<double> TAcc;

        // 跟随起飞状态
        bool follow_takeoff = false;

        // 尝试获取跟随传感器
        followSensor followS;
        uint16_t followSInd = get_followSensor(&followS);
        if (followSInd && followS.available && followS.updateTime.get_pass_time() < 2 && (!isFollowDataType_GlobalXY(followS.dataType) || followS.globalXYPosAvailable))
        { // 跟随可用
            if (isFollowDataType_Point(followS.dataType))
            { // 点追踪
                if (!inFlight && (followS.vel.z > 10 || followS.acc.z > 50))
                    follow_takeoff = true;
                if (lastFollowSInd != followSInd || followType != FOLLOW_TYPE::POINT || (!inFlight && !follow_takeoff))
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
            else if (isFollowDataType_Track(followS.dataType))
            { // 距离保持跟踪
                if (lastFollowSInd != followSInd || followType != FOLLOW_TYPE::TRACK || !inFlight)
                { // 传感器发生了更改
                  // 如果之前跟随过则重置目标点
                    if (followType != FOLLOW_TYPE::NONE)
                    {
                        followPos += followTPos;
                        followType = FOLLOW_TYPE::NONE;
                    }
                    lastFollowSInd = followSInd;
                    followOrigin.x = followS.pos.x;
                    followOrigin.y = followS.pos.y;
                    followOrigin.z = followS.pos.z;
                    // 复位跟随位置
                    last_followTPos.zero();
                    last_followTPos2.zero();
                    // 设置跟随类型
                    followType = FOLLOW_TYPE::TRACK;
                }
                // 更新传感器期望位置
                followTPos.x = followS.pos.x;
                followTPos.y = followS.pos.y;
                followTPos.z = followS.pos.z;

                // 求 当前位置 到 目标位置 的向量rVec
                vector3<double> tPos;
                get_TargetPosInf(0, 0, &tPos, 0);
                // get_Position_Ctrl(&tPos);
                vector2<double> rVec(followTPos.x - tPos.x, followTPos.y - tPos.y);
                double rVec_length = safe_sqrt(rVec.get_square());
                if (rVec_length > 20)
                {
                    followYawType = FOLLOW_YAW_TYPE::FOLLOW;
                    followYaw = atan2(rVec.y, rVec.x);
                    followYawRate = 0;
                }
                else
                    followYawType = FOLLOW_YAW_TYPE::NONE;

                // 将followTPos设置为 跟随原点 到 目前跟随位置 的向量
                followTPos -= followOrigin;

                // 计算跟随点位置变化 在 rVec方向上的分量 tVec
                vector2<double> tVec;
                tVec.x = followTPos.x - last_followTPos.x;
                tVec.y = followTPos.y - last_followTPos.y;
                last_followTPos = followTPos;
                if (rVec_length > 10)
                {
                    rVec *= (1.0 / rVec_length);
                    tVec = rVec * (tVec * rVec);
                    // 更新传感器期望位置 = 上次期望位置 + tVec(跟随点变化在rVec的分量)
                    followTPos.x = last_followTPos2.x + tVec.x;
                    followTPos.y = last_followTPos2.y + tVec.y;
                }
                else
                    followTPos = last_followTPos2;
                last_followTPos2 = followTPos;
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
        // 控制偏航
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
        // Z速度前馈
        if (isFollowDataType_Z(followS.dataType))
        { // 跟随目标可用
            followTV.z += followTVel.z;
        }

        // 计算期望位置速度
        double thr_stick_d = 0;
        double roll_sitck_d = 0;
        double pitch_sitck_d = 0;
        if (rc.available)
        {
            thr_stick_d = remove_deadband(rc.data[0] - 50.0, (double) cfg->NeutralZone[0]);
            roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]);
            pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]);
        }
        vector3<double> TVel;
        rcCtrl_Pos_p(roll_sitck_d, pitch_sitck_d, followPos.x, followPos.y, TVel.x, TVel.y, followTV.x, followTV.y, rc, flag, comVars, msg, msgRs, cfg, avCfg, h);
        rcCtrl_Alt_p(thr_stick_d, followPos.z, TVel.z, followTV.z, rc, flag, comVars, msg, msgRs, cfg, avCfg, h);

        vector3<double> TPos;
        TPos.x = followPos.x;
        TPos.y = followPos.y;
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

        // 高度控制
        if (!inFlight && (thr_stick_d > 1 || follow_takeoff))
        {
            vector3<double> pos;
            get_Position_Ctrl(&pos);
            Position_Control_set_TargetPosVelAccZ_OffBoard(pos.z + 15, TVel.z, 0);
        }
        else
        {
            Position_Control_set_TargetPosVelAccZ_OffBoard(TPos.z, TVel.z, TAcc.z);
        }
    }
    else
    { // 无定位追踪

        followSensor followS;
        uint16_t followSInd = get_followSensor(&followS);
        if (followSInd && followS.available && followS.updateTime.get_pass_time() < 2 && (!isFollowDataType_GlobalXY(followS.dataType) && isFollowDataType_XY(followS.dataType)))
        { // 跟随可用

            // 判断摇杆回中
            bool sticks_in_neutral = !rc.available || (in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]) &&
                                                       in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]));
            if (sticks_in_neutral)
            { // 遥感在中间执行无GPS跟随

                // 判断执行初始化操作
                switch (step)
                {
                    default:
                    { // 之前不在此状态
                      // 复位跟随传感器原点
                        followOrigin.x = followS.pos.x;
                        followOrigin.y = followS.pos.y;
                        // 复位上次位置误差
                        lastFollowPosErr.zero();
                        // 复位速度滤波值
                        followVelFilted.zero();
                        // 复位历史状态
                        last_followTPos = followS.pos;
                        last_followTPos2.zero();
                        // 初始化step
                        step = 200;
                        break;
                    }
                    case 200:
                    {
                        break;
                    }
                }

                vector3<double> pos;
                // get_Position_Ctrl(&pos);

                // 计算机头方向风扰
                vector3<double> WindDisturbance;
                get_WindDisturbance(&WindDisturbance);
                Quaternion attitude;
                get_Attitude_quat(&attitude);
                double yaw = attitude.getYaw();
                double sin_Yaw, cos_Yaw;
                fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x(WindDisturbance.x, WindDisturbance.y, sin_Yaw, cos_Yaw);
                double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y(WindDisturbance.x, WindDisturbance.y, sin_Yaw, cos_Yaw);

                vector3<double> followPos_filted = followS.pos;
                if (isFollowDataType_Track(followS.dataType))
                { // 跟踪传感器需要对垂直方向数据进行滤波
                    double posLen = sq(followS.pos.x) + sq(followS.pos.y);
                    if (posLen > sq(10))
                    { // 滤波
                        posLen = safe_sqrt(posLen);
                        double inv_posLen = 1.0 / posLen;

                        // 求轴向向量
                        vector3<double> tVec3 = followS.pos - last_followTPos;
                        vector2<double> tVec(tVec3.x, tVec3.y);
                        vector2<double> posDir(followS.pos.x, followS.pos.y);
                        posDir *= inv_posLen;
                        vector2<double> rVec = posDir * (tVec * posDir);

                        // 求垂直向向量
                        vector2<double> dVec = tVec - rVec;
                        const double dVecFilterP = 0.3;
                        const double dVecFilterL = 100;
                        last_followTPos2.x += 2 * Pi * dVecFilterP * h * (constrain(dVec.x - last_followTPos2.x, dVecFilterL));
                        last_followTPos2.y += 2 * Pi * dVecFilterP * h * (constrain(dVec.y - last_followTPos2.y, dVecFilterL));

                        // 滤波
                        followPos_filted = last_followTPos + vector3<double>(tVec.x, tVec.y, 0) + vector3<double>(last_followTPos2.x, last_followTPos2.y, 0);
                    }
                    last_followTPos = followS.pos;
                }

                // 计算跟随速度
                vector3<double> posErr = followS.pos - followOrigin - pos;
                vector3<double> posErr_filted = followPos_filted - followOrigin - pos;
                vector2<double> followVel;
                followVel.x = -(posErr.x - lastFollowPosErr.x) * freq;
                followVel.y = -(posErr.y - lastFollowPosErr.y) * freq;
                lastFollowPosErr.x = posErr.x;
                lastFollowPosErr.y = posErr.y;
                vector2<double> followVelErr = followVel - followVelFilted;
                followVelFilted += followVelErr * (2 * Pi * 0.2 * h);

                const double P1 = 0.3;
                const double P2 = 2.0;

                // 计算期望速度
                vector2<double> targetVel;
                targetVel.x = posErr_filted.x * P1;
                targetVel.y = posErr_filted.y * P1;
                targetVel.constrain(300.0);

                // 计算期望加速度
                vector2<double> targetAcc = (targetVel - followVelFilted) * P2;
                vector2<double> targetAccBody;
                targetAccBody.x = ENU2BodyHeading_x(targetAcc.x, targetAcc.y, sin_Yaw, cos_Yaw);
                targetAccBody.y = ENU2BodyHeading_y(targetAcc.x, targetAcc.y, sin_Yaw, cos_Yaw);

                // 使用当前倾角预测跟随速度
                Quaternion quat;
                get_AirframeY_quat(&quat);
                double pit = quat.getPitch();
                double rol = quat.getRoll();
                vector2<double> estAccBody;
                estAccBody.x = +tan(pit) * GravityAcc;
                estAccBody.y = -tan(rol) * GravityAcc;
                vector2<double> estAcc;
                estAcc.x = BodyHeading2ENU_x(estAccBody.x, estAccBody.y, sin_Yaw, cos_Yaw) + WindDisturbance.x;
                estAcc.y = BodyHeading2ENU_y(estAccBody.x, estAccBody.y, sin_Yaw, cos_Yaw) + WindDisturbance.y;
                followVelFilted += estAcc * h;

                // 偏航杆在中间锁偏航
                // 不在中间控制偏航速度
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                if (!rc.available || in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
                {
                    //									if( followS.yawAvailable() )
                    //										Attitude_Control_set_Target_Yaw(followS.yaw);
                    //									else
                    Attitude_Control_set_YawLock();
                }
                else
                    Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);

                // 计算倾角
                const double maxAngle = degree2rad(10.0);
                Attitude_Control_set_Target_RollPitch(-(constrain(atan2(targetAccBody.y, GravityAcc), maxAngle) - atan2(WindDisturbance_Bodyheading_y, GravityAcc)),
                                                      +(constrain(atan2(targetAccBody.x, GravityAcc), maxAngle) - atan2(WindDisturbance_Bodyheading_x, GravityAcc)));
            }
            else
            { // 摇杆不在中间手动控制
                step = 270;
                followVelFilted.zero();

                vector3<double> WindDisturbance;
                get_WindDisturbance(&WindDisturbance);
                Quaternion attitude;
                get_Attitude_quat(&attitude);
                double yaw = attitude.getYaw();
                double sin_Yaw, cos_Yaw;
                fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x(WindDisturbance.x, WindDisturbance.y, sin_Yaw, cos_Yaw);
                double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y(WindDisturbance.x, WindDisturbance.y, sin_Yaw, cos_Yaw);

                // 俯仰横滚杆控俯仰横滚
                double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxLean[0] / 50.0);
                Attitude_Control_set_Target_RollPitch((rc.data[3] - 50) * RPCtrlScale, (rc.data[2] - 50) * RPCtrlScale);

                // 偏航杆在中间锁偏航
                // 不在中间控制偏航速度
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                if (in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
                    Attitude_Control_set_YawLock();
                else
                    Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);
            }
        }
        else
        { // 跟随不可用
            step = 300;

            if (rc.available)
            {
                vector3<double> WindDisturbance;
                get_WindDisturbance(&WindDisturbance);
                Quaternion attitude;
                get_Attitude_quat(&attitude);
                double yaw = attitude.getYaw();
                double sin_Yaw, cos_Yaw;
                fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x(WindDisturbance.x, WindDisturbance.y, sin_Yaw, cos_Yaw);
                double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y(WindDisturbance.x, WindDisturbance.y, sin_Yaw, cos_Yaw);

                // 俯仰横滚杆控俯仰横滚
                double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxLean[0] / 50.0);
                //						Attitude_Control_set_Target_RollPitch(
                //							( rc.data[3] - 50 )*RPCtrlScale - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
                //							( rc.data[2] - 50 )*RPCtrlScale - atan2( WindDisturbance_Bodyheading_x , GravityAcc )
                //						);
                Attitude_Control_set_Target_RollPitch((rc.data[3] - 50) * RPCtrlScale, (rc.data[2] - 50) * RPCtrlScale);

                // 偏航杆在中间锁偏航
                // 不在中间控制偏航速度
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                if (in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
                    Attitude_Control_set_YawLock();
                else
                    Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);
            }
            else
            {
                if (comVars.noRcStartTIME.get_pass_time() > 0.5)
                {
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
        }

        // 油门杆控制垂直速度
        if (!rc.available || in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
            Position_Control_set_ZLock();
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

    return FMR_OK;
}