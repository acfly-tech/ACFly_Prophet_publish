#include "FlightMode_OffBoard.hpp"

#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "SlamCfg.hpp"
#include "StorageSystem.hpp"
#include "basic.hpp"

FlightModeClass FlightMode_OffBoard::get_fmClass()
{
    return FMC_SemiAutomatic;
}

bool FlightMode_OffBoard::set_params(double params[], uint16_t params_cnt)
{
    return true;
}

FlightModeCheckResult FlightMode_OffBoard::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    return FMCR_OK;
}

FlightMode_OffBoard::FlightMode_OffBoard()
    : FlightMode_Base("offboard", AFunc_Offboard, 0)
{
    // 复位退出模式计数器
    exit_counter1 = 0;
    exit_counter2 = 0;
    thr_raised = false;
    // 复位手动控制超时时间
    Offboard_ThrTIME = TIME::now();
    Offboard_YawTIME = TIME::now();
    Offboard_PitRolTIME = TIME::now();
    Offboard_ManualTIME = TIME::now();
    // 复位指令状态
    init_NavCmdInf(&navInf);
    cmdInProcess = false;
    memset(&cmdMsg, 0, sizeof(cmdMsg));
    // 关闭喷洒，手动控制
    // setPump1(PumpOffManualPercent);
}

FlightMode_OffBoard::~FlightMode_OffBoard() {}

FlightModeResult FlightMode_OffBoard::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                                const AvoidanceCfg *avCfg, double h)
{
    // 计算模式运行频率
    const double freq = 1.0 / h;

    // 打开位置控制器
    Altitude_Control_Enable();
    Position_Control_Enable();
    bool pos_ena;
    is_Position_Control_Enabled(&pos_ena);
    if (pos_ena == false)
    {
        return FMR_Err_NoPositioning;
    }

    // 上报模式状态
    setCurrentFlyMode(cmdInProcess ? AFunc_OffboardOrbit : AFunc_Offboard);

    // 获取飞行状态
    bool inFlight;
    get_is_inFlight(&inFlight);

    // 是否遥控器控制
    bool rcCtrl = false;

    if (rc.available)
    {
        // 判断锁定
        if (!thr_raised)
        {
            if (rc.data[0] > 35)
                thr_raised = true;
        }
        if (!inFlight)
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

        // 判断遥控器是否回中
        bool sticks_in_neutral = in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]) &&
                                 in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]);
        if (!sticks_in_neutral && (cfg->configs[0] & MCfg_NoRcCtrlInAuto_Bit) == 0)
            Offboard_ManualTIME = TIME::now();

        if (Offboard_ManualTIME.get_pass_time() < 1.5)
        { // 摇杆动作后一段时间内手动控制

            rcCtrl = true;
            cmdInProcess = false;
            Offboard_ThrTIME.set_invalid();
            Offboard_YawTIME.set_invalid();
            Offboard_PitRolTIME.set_invalid();

            // 判断摇杆是否回中
            bool thr_in_neutral = in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]);
            bool yaw_in_neutral = in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]);
            bool pitrol_in_neutral = in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]);

            // 油门控制 摇杆在中位且一段时间无控制才锁定
            if (thr_in_neutral)
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

            // 偏航控制 摇杆在中位且一段时间无控制才锁定
            if (yaw_in_neutral)
                Attitude_Control_set_YawLock();
            else
            {
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);
            }

            // 横滚俯仰控制 摇杆在中位且一段时间无控制才锁定
            if (pitrol_in_neutral)
                Position_Control_set_XYLock();
            else
            {
                double RPCtrlScale = atan2(getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc);
                double XYCtrlScale = getPosCtrlCfg()->maxVelXY[0] / 50.0;
                double roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]);
                double pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]);
                vector3<double> velocityFLU;
                get_VelocityFLU_Ctrl(&velocityFLU);
                double vel_stick_err_pitch = velocityFLU.x / XYCtrlScale - pitch_sitck_d;
                double vel_stick_err_roll = velocityFLU.y / XYCtrlScale - -roll_sitck_d;
                constrain_vector(vel_stick_err_roll, vel_stick_err_pitch, 50);
                Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(pitch_sitck_d * XYCtrlScale, -roll_sitck_d * XYCtrlScale, fabs(vel_stick_err_roll) * RPCtrlScale,
                                                                            fabs(vel_stick_err_pitch) * RPCtrlScale);
            }
        }
        else
        { // 不需要遥控控制
            if (!cmdInProcess)
            {
                if (Offboard_ThrTIME.get_pass_time() > 1.0)
                    Position_Control_set_ZLock();
                if (Offboard_YawTIME.get_pass_time() > 1.0)
                    Attitude_Control_set_YawLock();
                if (Offboard_PitRolTIME.get_pass_time() > 1.0)
                    Position_Control_set_XYLock();
            }
        }
    }
    else
    { // 无遥控器
        Offboard_ManualTIME.set_invalid();

        if (!cmdInProcess)
        {
            if (Offboard_ThrTIME.get_pass_time() > 1.0)
                Position_Control_set_ZLock();
            if (Offboard_YawTIME.get_pass_time() > 1.0)
                Attitude_Control_set_YawLock();
            if (Offboard_PitRolTIME.get_pass_time() > 1.0)
                Position_Control_set_XYLock();

            if (Offboard_ThrTIME.get_pass_time() > 2 && Offboard_PitRolTIME.get_pass_time() > 2)
            { // 无遥控信号且一段时间无信号
                // 进入安全模式
                return FMR_OK_RqSwitchRTL;
            }
        }
    }

    // 执行指令
    if (!rcCtrl && msg)
    {
        // 执行inflight cmd
        if (Process_InflightCmd(msg->cmd, msg->params))
        {
            if (msgRs)
                *msgRs = FMMR_Success;
        }
        else if (check_NavCmd(msg->cmd, msg->frame, msg->params))
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

        // 执行set_position_target指令
        else if (msg->cmd == 1084)
        { // SET_POSITION_TARGET_LOCAL_NED
            cmdInProcess = false;

            // 读取 slamcfg 参数，slam_frame 启用时把 LOCAL 帧重映射为 VISION 帧（走 SLAM 控制）
            SlamCfg slamcfg_rd;
            ReadParamGroup("slamcfg", (uint64_t *) &slamcfg_rd, 0);
            uint8_t eff_frame = msg->frame;
            if (slamcfg_rd.slam_frame[0])
            {
                if (eff_frame == MAV_FRAME_LOCAL_NED)
                    eff_frame = MAV_FRAME_VISION_NED;
                else if (eff_frame == MAV_FRAME_LOCAL_ENU)
                    eff_frame = MAV_FRAME_VISION_ENU;
            }

            uint16_t type_mask = msg->params[0];
            // thr
            if ((type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE) == 0)
            { // pva
                double posz = msg->params[3] * 100;
                double velz = 0;
                double accz = 0;
                if ((type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) == 0)
                {
                    velz = msg->params[6] * 100;
                    if ((type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) == 0)
                        accz = msg->params[9] * 100;
                }
                switch (eff_frame)
                {
                    case MAV_FRAME_VISION_NED:
                    {
                        Position_Control_set_TargetPosVelAccZ_SLAM_OffBoard(Slam_Index, -posz, -velz, -accz);
                        break;
                    }
                    case MAV_FRAME_VISION_ENU:
                    {
                        Position_Control_set_TargetPosVelAccZ_SLAM_OffBoard(Slam_Index, posz, velz, accz);
                        break;
                    }
                    case MAV_FRAME_LOCAL_NED:
                    {
                        Position_Control_set_TargetPosVelAccZ_OffBoard(-posz, -velz, -accz);
                        break;
                    }
                    default:
                    case MAV_FRAME_LOCAL_ENU:
                    {
                        Position_Control_set_TargetPosVelAccZ_OffBoard(posz, velz, accz);
                        break;
                    }
                    case MAV_FRAME_BODY_FLU:
                    {
                        Position_Control_set_TargetPosRelativeVelAccZ_OffBoard(posz, velz, accz);
                        break;
                    }
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    {
                        Position_Control_set_TargetPosRelativeVelAccZ_OffBoard(-posz, -velz, -accz);
                        break;
                    }
                }
                Offboard_ThrTIME = TIME::now();
            }
            else if ((type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) == 0)
            { // va
                double velz = 0;
                double accz = 0;
                velz = msg->params[6] * 100;
                if ((type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) == 0)
                    accz = msg->params[9] * 100;
                if (fabs(velz) < 0.1 && fabs(accz) < 0.1)
                { // 零垂直速度且零加速度指令(死区0.1): 锁定高度, 避免纯速度闭环下缓慢漂移
                    // 加速度非0时不锁定(纯加速度控制场景)
                    Position_Control_set_ZLock();
                }
                else
                {
                    switch (eff_frame)
                    {
                        case MAV_FRAME_VISION_NED:
                        {
                            Position_Control_set_TargetVelAccZ_SLAM_OffBoard(Slam_Index, -velz, -accz);
                            break;
                        }
                        case MAV_FRAME_VISION_ENU:
                        {
                            Position_Control_set_TargetVelAccZ_SLAM_OffBoard(Slam_Index, velz, accz);
                            break;
                        }
                        case MAV_FRAME_BODY_FRD:
                        case MAV_FRAME_BODY_NED:
                        case MAV_FRAME_BODY_OFFSET_NED:
                        case MAV_FRAME_LOCAL_NED:
                        {
                            Position_Control_set_TargetVelAccZ_OffBoard(-velz, -accz);
                            break;
                        }
                        default:
                        case MAV_FRAME_BODY_FLU:
                        case MAV_FRAME_LOCAL_ENU:
                        {
                            Position_Control_set_TargetVelAccZ_OffBoard(velz, accz);
                            break;
                        }
                    }
                }
                Offboard_ThrTIME = TIME::now();
            }
            // yaw
            if ((type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE) == 0)
            { // pva
                double Trate = 0;
                if ((type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) == 0)
                    Trate = -msg->params[11];
                if (slamcfg_rd.slam_frame[0])
                {
                    // SLAM 模式: yawOffset - SLAM yaw = AirframeY 绝对航向
                    // slam_quat 构造时对 yaw/q2/q3 取负(MavlinkRCProcess.cpp:3145/3234),
                    // 故 yawOffset = ψ_Y - (-ψ_s) = ψ_Y + ψ_s(常数帧偏移)。
                    // 正确目标: ψ_Y_target = yawOffset - ψ_s_target。
                    double Tyaw = -msg->params[10];
                    Position_Sensor_Data slamSensor;
                    if (GetPositionSensorData(11, &slamSensor) && slamSensor.available)
                        Tyaw += slamSensor.mp.lat0_rad;
                    Attitude_Control_set_Target_Yaw_Offboard(Tyaw, Trate);
                }
                else
                {
                    // 普通模式: NED→ENU 绝对航向
                    double Tyaw = 0.5 * Pi - msg->params[10];
                    Attitude_Control_set_Target_Yaw_Offboard(Tyaw, Trate);
                }
                Offboard_YawTIME = TIME::now();
            }
            else if ((type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) == 0)
            {
                double Trate = -msg->params[11];
                Attitude_Control_set_Target_YawRate(Trate);
                Offboard_YawTIME = TIME::now();
            }
            // pit rol
            if ((type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) == 0)
            { // pva
                double posx = msg->params[1] * 100;
                double posy = msg->params[2] * 100;
                double velx = 0, vely = 0;
                double accx = 0, accy = 0;
                if ((type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) == 0)
                {
                    velx = msg->params[4] * 100;
                    vely = msg->params[5] * 100;
                    if ((type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) == 0)
                    {
                        accx = msg->params[7] * 100;
                        accy = msg->params[8] * 100;
                    }
                }
                switch (eff_frame)
                {
                    case MAV_FRAME_VISION_NED:
                    {
                        Position_Control_set_TargetPosVelAccXY_SLAM_OffBoard(Slam_Index, posx, -posy, velx, -vely, accx, -accy);
                        break;
                    }
                    case MAV_FRAME_VISION_ENU:
                    {
                        Position_Control_set_TargetPosVelAccXY_SLAM_OffBoard(Slam_Index, posx, posy, velx, vely, accx, accy);
                        break;
                    }
                    case MAV_FRAME_LOCAL_NED:
                    {
                        Position_Control_set_TargetPosVelAccXY_OffBoard(posy, posx, vely, velx, accy, accx);
                        break;
                    }
                    default:
                    case MAV_FRAME_LOCAL_ENU:
                    {
                        Position_Control_set_TargetPosVelAccXY_OffBoard(posx, posy, velx, vely, accx, accy);
                        break;
                    }
                    case MAV_FRAME_BODY_FLU:
                    {
                        double yaw;
                        double yaw_declination;
                        get_YawDeclination(&yaw_declination);
                        Attitude_Control_get_TargetTrackYaw(&yaw);
                        yaw += yaw_declination;
                        double sin_Yaw, cos_Yaw;
                        fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                        double posx_enu = BodyHeading2ENU_x(posx, posy, sin_Yaw, cos_Yaw);
                        double posy_enu = BodyHeading2ENU_y(posx, posy, sin_Yaw, cos_Yaw);
                        double velx_enu = BodyHeading2ENU_x(velx, vely, sin_Yaw, cos_Yaw);
                        double vely_enu = BodyHeading2ENU_y(velx, vely, sin_Yaw, cos_Yaw);
                        double accx_enu = BodyHeading2ENU_x(accx, accy, sin_Yaw, cos_Yaw);
                        double accy_enu = BodyHeading2ENU_y(accx, accy, sin_Yaw, cos_Yaw);
                        Position_Control_set_TargetPosRelativeVelAccXY_OffBoard(posx_enu, posy_enu, velx_enu, vely_enu, accx_enu, accy_enu);
                        break;
                    }
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    {
                        double yaw;
                        double yaw_declination;
                        get_YawDeclination(&yaw_declination);
                        Attitude_Control_get_TargetTrackYaw(&yaw);
                        yaw += yaw_declination;
                        double sin_Yaw, cos_Yaw;
                        fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                        double posx_enu = BodyHeading2ENU_x(posx, -posy, sin_Yaw, cos_Yaw);
                        double posy_enu = BodyHeading2ENU_y(posx, -posy, sin_Yaw, cos_Yaw);
                        double velx_enu = BodyHeading2ENU_x(velx, -vely, sin_Yaw, cos_Yaw);
                        double vely_enu = BodyHeading2ENU_y(velx, -vely, sin_Yaw, cos_Yaw);
                        double accx_enu = BodyHeading2ENU_x(accx, -accy, sin_Yaw, cos_Yaw);
                        double accy_enu = BodyHeading2ENU_y(accx, -accy, sin_Yaw, cos_Yaw);
                        Position_Control_set_TargetPosRelativeVelAccXY_OffBoard(posx_enu, posy_enu, velx_enu, vely_enu, accx_enu, accy_enu);
                        break;
                    }
                }
                Offboard_PitRolTIME = TIME::now();
            }
            else if ((type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) == 0)
            { // va
                double velx = 0, vely = 0;
                double accx = 0, accy = 0;
                velx = msg->params[4] * 100;
                vely = msg->params[5] * 100;
                if ((type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) == 0)
                {
                    accx = msg->params[7] * 100;
                    accy = msg->params[8] * 100;
                }
                if (sq(velx) + sq(vely) < sq(0.1) && sq(accx) + sq(accy) < sq(0.1))
                { // 零水平速度且零加速度指令(死区0.1): 锁定当前位置, 避免纯速度闭环下位置开环缓慢漂移
                    // 旋转变换不改变模长, 原始指令域判断零速即可; 加速度非0时不锁定(纯加速度控制场景)
                    Position_Control_set_XYLock();
                }
                else
                {
                    switch (eff_frame)
                    {
                        case MAV_FRAME_VISION_NED:
                        {
                            Position_Control_set_TargetVelAccXY_SLAM_OffBoard(Slam_Index, velx, -vely, accx, -accy);
                            break;
                        }
                        case MAV_FRAME_VISION_ENU:
                        {
                            Position_Control_set_TargetVelAccXY_SLAM_OffBoard(Slam_Index, velx, vely, accx, accy);
                            break;
                        }
                        case MAV_FRAME_LOCAL_NED:
                        {
                            Position_Control_set_TargetVelAccXY_OffBoard(vely, velx, accy, accx);
                            break;
                        }
                        default:
                        case MAV_FRAME_LOCAL_ENU:
                        {
                            Position_Control_set_TargetVelAccXY_OffBoard(velx, vely, accx, accy);
                            break;
                        }
                        case MAV_FRAME_BODY_FLU:
                        {
                            double yaw;
                            double yaw_declination;
                            get_YawDeclination(&yaw_declination);
                            Attitude_Control_get_TargetTrackYaw(&yaw);
                            yaw += yaw_declination;
                            double sin_Yaw, cos_Yaw;
                            fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                            double velx_enu = BodyHeading2ENU_x(velx, vely, sin_Yaw, cos_Yaw);
                            double vely_enu = BodyHeading2ENU_y(velx, vely, sin_Yaw, cos_Yaw);
                            double accx_enu = BodyHeading2ENU_x(accx, accy, sin_Yaw, cos_Yaw);
                            double accy_enu = BodyHeading2ENU_y(accx, accy, sin_Yaw, cos_Yaw);
                            Position_Control_set_TargetVelAccXY_OffBoard(velx_enu, vely_enu, accx_enu, accy_enu);
                            break;
                        }
                        case MAV_FRAME_BODY_FRD:
                        case MAV_FRAME_BODY_NED:
                        case MAV_FRAME_BODY_OFFSET_NED:
                        {
                            double yaw;
                            double yaw_declination;
                            get_YawDeclination(&yaw_declination);
                            Attitude_Control_get_TargetTrackYaw(&yaw);
                            yaw += yaw_declination;
                            double sin_Yaw, cos_Yaw;
                            fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                            double velx_enu = BodyHeading2ENU_x(velx, -vely, sin_Yaw, cos_Yaw);
                            double vely_enu = BodyHeading2ENU_y(velx, -vely, sin_Yaw, cos_Yaw);
                            double accx_enu = BodyHeading2ENU_x(accx, -accy, sin_Yaw, cos_Yaw);
                            double accy_enu = BodyHeading2ENU_y(accx, -accy, sin_Yaw, cos_Yaw);
                            Position_Control_set_TargetVelAccXY_OffBoard(velx_enu, vely_enu, accx_enu, accy_enu);
                            break;
                        }
                    }
                }
                Offboard_PitRolTIME = TIME::now();
            }

            // 发送ACK
            uint8_t port_id = msg->cmd_type & 0xf;
            const Port *port = get_CommuPort(port_id);
            if (port && port->write)
            {
                if (mavlink_lock_chan(port_id, 0.01))
                {
                    mavlink_message_t msg_sd;

                    mavlink_msg_position_target_local_ned_pack_chan(get_CommulinkSysId(),  // system id
                                                                    get_CommulinkCompId(), // component id
                                                                    port_id,               // chan
                                                                    &msg_sd, TIME::get_System_Run_Time() * 1e3, msg->frame, msg->params[0], msg->params[1], msg->params[2], msg->params[3],
                                                                    msg->params[4], msg->params[5], msg->params[6], msg->params[7], msg->params[8], msg->params[9], msg->params[10], msg->params[11]);
                    mavlink_send_msg(port, &msg_sd, 0, -1);
                    mavlink_unlock_chan(port_id);
                }
            }
        }
        else if (msg->cmd == 1086)
        { // SET_POSITION_TARGET_GLOBAL_INT
            cmdInProcess = false;

            uint16_t type_mask = msg->params[0];
            // thr
            if ((type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE) == 0)
            { // pva
                double posz = msg->params[3] * 100;
                double velz = 0;
                double accz = 0;
                if ((type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) == 0)
                {
                    velz = msg->params[6] * 100;
                    if ((type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) == 0)
                        accz = msg->params[9] * 100;
                }
                switch (msg->frame)
                {
                    case MAV_FRAME_GLOBAL_INT:
                    case MAV_FRAME_GLOBAL:
                    {
                        Position_Control_set_TargetPosGlobalVelAccZ_OffBoard(posz, -velz, -accz);
                        break;
                    }
                    default:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    {
                        Position_Control_set_TargetPosZRAVelAccZ_OffBoard(posz, -velz, -accz);
                        break;
                    }
                }
                Offboard_ThrTIME = TIME::now();
            }
            else if ((type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) == 0)
            { // va
                double velz = 0;
                double accz = 0;
                velz = msg->params[6] * 100;
                if ((type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) == 0)
                    accz = msg->params[9] * 100;
                Position_Control_set_TargetVelAccZ_OffBoard(-velz, -accz);
                Offboard_ThrTIME = TIME::now();
            }
            // yaw
            if ((type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE) == 0)
            { // pva
                double Tyaw = 0.5 * Pi - msg->params[10];
                double Trate = 0;
                if ((type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) == 0)
                    Trate = -msg->params[11];
                Attitude_Control_set_Target_Yaw_Offboard(Tyaw, Trate);
                Offboard_YawTIME = TIME::now();
            }
            else if ((type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) == 0)
            {
                double Trate = -msg->params[11];
                Attitude_Control_set_Target_YawRate(Trate);
                Offboard_YawTIME = TIME::now();
            }
            // pit rol
            if ((type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) == 0)
            { // pva
                double lat = msg->params[1];
                double lon = msg->params[2];
                double velx = 0, vely = 0;
                double accx = 0, accy = 0;
                if ((type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) == 0)
                {
                    velx = msg->params[4] * 100;
                    vely = msg->params[5] * 100;
                    if ((type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) == 0)
                    {
                        accx = msg->params[7] * 100;
                        accy = msg->params[8] * 100;
                    }
                }
                Position_Control_set_TargetGlobalPosVelAccXY_OffBoard(lat, lon, vely, velx, accy, accx);
                Offboard_PitRolTIME = TIME::now();
            }
            else if ((type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) == 0)
            { // va
                double velx = 0, vely = 0;
                double accx = 0, accy = 0;
                velx = msg->params[4] * 100;
                vely = msg->params[5] * 100;
                if ((type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) == 0 && (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) == 0)
                {
                    accx = msg->params[7] * 100;
                    accy = msg->params[8] * 100;
                }
                Position_Control_set_TargetVelAccXY_OffBoard(vely, velx, accy, accx);
                Offboard_PitRolTIME = TIME::now();
            }

            // 发送ACK
            uint8_t port_id = msg->cmd_type & 0xf;
            const Port *port = get_CommuPort(port_id);
            if (port && port->write)
            {
                if (mavlink_lock_chan(port_id, 0.01))
                {
                    mavlink_message_t msg_sd;

                    mavlink_msg_position_target_local_ned_pack_chan(get_CommulinkSysId(),  // system id
                                                                    get_CommulinkCompId(), // component id
                                                                    port_id,               // chan
                                                                    &msg_sd, TIME::get_System_Run_Time() * 1e3, msg->frame, msg->params[0], msg->params[1], msg->params[2], msg->params[3],
                                                                    msg->params[4], msg->params[5], msg->params[6], msg->params[7], msg->params[8], msg->params[9], msg->params[10], msg->params[11]);
                    mavlink_send_msg(port, &msg_sd, 0, -1);
                    mavlink_unlock_chan(port_id);
                }
            }
        }
    }

    // 执行指令
    if (cmdInProcess)
    {
        int16_t res = -100;
        res = Process_NavCmd(cmdMsg.cmd, freq, cmdMsg.frame, cmdMsg.params, &navInf, rc, flag, comVars, cfg, avCfg);
        if (NavCmdRs_SuccessOrFault(res))
        { // NavCmd执行完成
            cmdInProcess = false;

            if (res == NCR_Land_RqDisarm && !inFlight)
            {
                return FMR_RqDisarm_Land;
            }
        }
    }
    return FMR_OK;
}
