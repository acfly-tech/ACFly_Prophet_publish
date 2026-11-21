#include "FlightModes.hpp"

#include "ControlSystem.hpp"
#include "MeasurementSystem.hpp"
#include "basic.hpp"

#include "FlightMode_Follow.hpp"
#include "FlightMode_Manual.hpp"
#include "FlightMode_PosHold.hpp"

#include "FlightMode_AutoMission.hpp"
#include "FlightMode_Circle.hpp"
#include "FlightMode_OffBoard.hpp"

#include "FlightMode_Mission.hpp"
#include "FlightMode_RTL.hpp"
#include "FlightMode_TakeOff.hpp"
#include "FlightMode_TrajectoryReplay.hpp"

const FlightModeFactory::Creator *FlightModeFactory::creators(uint16_t *size)
{
    static const Creator creators[] = {
        /*000-*/ nullptr,
        /*001-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_PosHold>(1); },
        /*002-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_PosHold>(3); },
        /*003-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_Follow>(3); },
        /*004-*/ nullptr,
        /*005-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_Circle>(); },
        /*006-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_Manual>(1); },
        /*007-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_Manual>(2); },
        /*008-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_Manual>(3); },
        /*009-*/ nullptr,
        /*010-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_OffBoard>(); },
        /*011-*/ nullptr,
        /*012-*/ nullptr,
        /*013-*/ nullptr,
        /*014-*/ nullptr,
        /*015-*/ nullptr,
        /*016-*/ nullptr,
        /*017-*/ nullptr,
        /*018-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_TrajectoryReplay>(); },
        /*019-*/ nullptr,
        /*020-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_TakeOff>(); },
        /*021-*/ nullptr,
        /*022-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_Mission>(); },
        /*023-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_RTL>(2); },
        /*024-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_RTL>(1); },
        /*025-*/ nullptr,
        /*026-*/ nullptr,
        /*027-*/ nullptr,
        /*028-*/ nullptr,
        /*029-*/ nullptr,
        /*030-*/ nullptr,
        /*031-*/ nullptr,
        /*032-*/ nullptr,
        /*033-*/ nullptr,
        /*034-*/ nullptr,
        /*035-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_AutoMission>(); },
        /*036-*/ []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<FlightMode_RTL>(3); },
        /*037-*/ nullptr,
        /*038-*/ nullptr,
        /*039-*/ nullptr,
        /*040-*/ nullptr,
        /*041-*/ nullptr,
        /*042-*/ nullptr,
        /*043-*/ nullptr,
        /*044-*/ nullptr,
        /*045-*/ nullptr,
        /*046-*/ nullptr,
        /*047-*/ nullptr,
        /*048-*/ nullptr,
        /*049-*/ nullptr,
        /*050-*/ nullptr,
        /*051-*/ nullptr,
        /*052-*/ nullptr,
        /*053-*/ nullptr,
        /*054-*/ nullptr,
        /*055-*/ nullptr,
        /*056-*/ nullptr,
        /*057-*/ nullptr,
        /*058-*/ nullptr,
        /*059-*/ nullptr,
        /*060-*/ nullptr,
        /*061-*/ nullptr,
        /*062-*/ nullptr,
        /*063-*/ nullptr,
        /*064-*/ nullptr,
        /*065-*/ nullptr,
        /*066-*/ nullptr,
        /*067-*/ nullptr,
        /*068-*/ nullptr,
        /*069-*/ nullptr,
        /*070-*/ nullptr,
        /*071-*/ nullptr,
        /*072-*/ nullptr,
        /*073-*/ nullptr,
        /*074-*/ nullptr,
        /*075-*/ nullptr,
        /*076-*/ nullptr,
        /*077-*/ nullptr,
        /*078-*/ nullptr,
        /*079-*/ nullptr,
        /*080-*/ nullptr,
        /*081-*/ nullptr,
        /*082-*/ nullptr,
        /*083-*/ nullptr,
        /*084-*/ nullptr,
        /*085-*/ nullptr,
        /*086-*/ nullptr,
        /*087-*/ nullptr,
        /*088-*/ nullptr,
        /*089-*/ nullptr,
        /*090-*/ nullptr,
        /*091-*/ nullptr,
        /*092-*/ nullptr,
        /*093-*/ nullptr,
        /*094-*/ nullptr,
        /*095-*/ nullptr,
        /*096-*/ nullptr,
        /*097-*/ nullptr,
        /*098-*/ nullptr,
        /*099-*/ nullptr,
        /*100-*/ nullptr,
        /*101-*/ nullptr,
        /*102-*/ nullptr,
        /*103-*/ nullptr,
        /*104-*/ nullptr,
        /*105-*/ nullptr,
        /*106-*/ nullptr,
        /*107-*/ nullptr,
        /*108-*/ nullptr,
        /*109-*/ nullptr,
        /*110-*/ nullptr,
        /*111-*/ nullptr,
        /*112-*/ nullptr,
        /*113-*/ nullptr,
        /*114-*/ nullptr,
        /*115-*/ nullptr,
        /*116-*/ nullptr,
        /*117-*/ nullptr,
        /*118-*/ nullptr,
        /*119-*/ nullptr,
        /*120-*/ nullptr,
        /*121-*/ nullptr,
        /*122-*/ nullptr,
        /*123-*/ nullptr,
        /*124-*/ nullptr,
        /*125-*/ nullptr,
        /*126-*/ nullptr,
        /*127-*/ nullptr,
        /*128-*/ nullptr,
        /*129-*/ nullptr,
        /*130-*/ nullptr,
        /*131-*/ nullptr,
        /*132-*/ nullptr,
        /*133-*/ nullptr,
        /*134-*/ nullptr,
        /*135-*/ nullptr,
        /*136-*/ nullptr,
        /*137-*/ nullptr,
        /*138-*/ nullptr,
        /*139-*/ nullptr,
        /*140-*/ nullptr,
        /*141-*/ nullptr,
        /*142-*/ nullptr,
        /*143-*/ nullptr,
        /*144-*/ nullptr,
        /*145-*/ nullptr,
        /*146-*/ nullptr,
        /*147-*/ nullptr,
        /*148-*/ nullptr,
        /*149-*/ nullptr,
        /*150-*/ nullptr,
        /*151-*/ nullptr,
        /*152-*/ nullptr,
        /*153-*/ nullptr,
        /*154-*/ nullptr,
        /*155-*/ nullptr,
        /*156-*/ nullptr,
        /*157-*/ nullptr,
        /*158-*/ nullptr,
        /*159-*/ nullptr,
        /*160-*/ nullptr,
        /*161-*/ nullptr,
        /*162-*/ nullptr,
        /*163-*/ nullptr,
        /*164-*/ nullptr,
        /*165-*/ nullptr,
        /*166-*/ nullptr,
        /*167-*/ nullptr,
        /*168-*/ nullptr,
        /*169-*/ nullptr,
        /*170-*/ nullptr,
        /*171-*/ nullptr,
        /*172-*/ nullptr,
        /*173-*/ nullptr,
        /*174-*/ nullptr,
        /*175-*/ nullptr,
        /*176-*/ nullptr,
        /*177-*/ nullptr,
        /*178-*/ nullptr,
        /*179-*/ nullptr,
        /*180-*/ nullptr,
        /*181-*/ nullptr,
        /*182-*/ nullptr,
        /*183-*/ nullptr,
        /*184-*/ nullptr,
        /*185-*/ nullptr,
        /*186-*/ nullptr,
        /*187-*/ nullptr,
        /*188-*/ nullptr,
        /*189-*/ nullptr,
        /*190-*/ nullptr,
        /*191-*/ nullptr,
        /*192-*/ nullptr,
        /*193-*/ nullptr,
        /*194-*/ nullptr,
        /*195-*/ nullptr,
        /*196-*/ nullptr,
        /*197-*/ nullptr,
        /*198-*/ nullptr,
        /*199-*/ nullptr,
        /*200-*/ nullptr,
        /*201-*/ nullptr,
        /*202-*/ nullptr,
        /*203-*/ nullptr,
        /*204-*/ nullptr,
        /*205-*/ nullptr,
        /*206-*/ nullptr,
        /*207-*/ nullptr,
        /*208-*/ nullptr,
        /*209-*/ nullptr,
        /*210-*/ nullptr,
        /*211-*/ nullptr,
        /*212-*/ nullptr,
        /*213-*/ nullptr,
        /*214-*/ nullptr,
        /*215-*/ nullptr,
        /*216-*/ nullptr,
        /*217-*/ nullptr,
        /*218-*/ nullptr,
        /*219-*/ nullptr,
        /*220-*/ nullptr,
        /*221-*/ nullptr,
        /*222-*/ nullptr,
        /*223-*/ nullptr,
        /*224-*/ nullptr,
        /*225-*/ nullptr,
        /*226-*/ nullptr,
        /*227-*/ nullptr,
        /*228-*/ nullptr,
        /*229-*/ nullptr,
        /*230-*/ nullptr,
        /*231-*/ nullptr,
        /*232-*/ nullptr,
        /*233-*/ nullptr,
        /*234-*/ nullptr,
        /*235-*/ nullptr,
        /*236-*/ nullptr,
        /*237-*/ nullptr,
        /*238-*/ nullptr,
        /*239-*/ nullptr,
        /*240-*/ nullptr,
        /*241-*/ nullptr,
        /*242-*/ nullptr,
        /*243-*/ nullptr,
        /*244-*/ nullptr,
        /*245-*/ nullptr,
        /*246-*/ nullptr,
        /*247-*/ nullptr,
        /*248-*/ nullptr,
        /*249-*/ nullptr,
        /*250-*/ nullptr,
        /*251-*/ nullptr,
        /*252-*/ nullptr,
        /*253-*/ nullptr,
        /*254-*/ nullptr,
        /*255-*/ nullptr,
        /*256-*/ nullptr,
    };
    if (size)
        *size = sizeof(creators) / sizeof(Creator);
    return creators;
}

// 模式开关标志
FlightModeFlag GLB_FlightModeFlags = (FlightModeFlag) 0;

// 解析围栏动作参数
//  动作: 0-悬停 1-返航 2-降落
FenceActionInf get_FenceActionInf(uint32_t fenceAction, bool mission)
{
    uint8_t action = mission ? ((fenceAction >> 8) & 0xFF) : (fenceAction & 0xFF);
    FenceActionInf res;
    switch (action)
    {
        case 0: // 悬停
            res.mitigation = FENCE_MITIGATE_HOVER;
            res.modeRs = mission ? FMR_OK_RqSwitchPosHold : FMR_OK;
            break;
        case 1: // 返航
            res.mitigation = FENCE_MITIGATE_RTL;
            res.modeRs = FMR_Err_RqSwitchRTL;
            break;
        case 2: // 降落
            res.mitigation = FENCE_MITIGATE_LAND;
            res.modeRs = FMR_Err_RqSwitchLand;
            break;
        default:
            res.mitigation = FENCE_MITIGATE_VEL_LIMIT;
            res.modeRs = FMR_Err_RqSwitchRTL;
            break;
    }
    return res;
}

// 通用控制--高度-速度方式
void FlightMode_Base::rcCtrl_Alt_v(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                   const AvoidanceCfg *avCfg, double h)
{
    // 油门杆控制垂直速度
    if (!rc.available || in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
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
                setFenceBreachActive(true, FENCE_BREACH_MAXALT, FENCE_MITIGATE_HOVER);

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
}

// 通用遥控器控制--高度-位置闭环
void FlightMode_Base::rcCtrl_Alt_p(double thr_sitck_d, double &tPos_z, double &tVel_z, double followTVel_z, const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars,
                                   const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg, double h)
{
    // Z期望速度
    double TvelZ = 0;
    double maxVelErrDis;
    if (thr_sitck_d > 0)
    {
        TvelZ = thr_sitck_d * getPosCtrlCfg()->maxVelUp[0] / 50;
        maxVelErrDis = getPosCtrlCfg()->maxAccUp[0] / getPosCtrlCfg()->P2[0];
    }
    else
    {
        TvelZ = thr_sitck_d * getPosCtrlCfg()->maxVelDown[0] / 50;
        maxVelErrDis = getPosCtrlCfg()->maxAccDown[0] / getPosCtrlCfg()->P2[0];
    }

    // 判断摇杆中位
    bool thrStickNeutral = thr_sitck_d == 0;

    // 限制期望速度Z
    vector3<double> velocityEnu;
    get_VelocityENU(&velocityEnu);
    double velErrZ = TvelZ - velocityEnu.z;
    velErrZ += followTVel_z;
    double velErrDis = fabs(velErrZ);

    double TvelZC;
    if (velErrDis > maxVelErrDis && !thrStickNeutral)
    {
        velErrZ *= maxVelErrDis / velErrDis;
        TvelZC = velErrZ + velocityEnu.z;
    }
    else
    {
        TvelZC = TvelZ;
    }

    // 调整期望位置速度
    tPos_z += TvelZC * h;
    tVel_z = TvelZ * getPosCtrlCfg()->VelXYFF[0] + followTVel_z;
}

// 通用遥控器控制--水平位置-位置闭环
void FlightMode_Base::rcCtrl_Pos_p(double roll_sitck_d, double pitch_sitck_d, double &tPos_x, double &tPos_y, double &tVel_x, double &tVel_y, double followTVel_x, double followTVel_y,
                                   const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                   const AvoidanceCfg *avCfg, double h)
{
    // XY期望速度
    vector2<double> TvelEnu;
    vector3<double> velocityFLU;
    get_VelocityFLU_Ctrl(&velocityFLU);

    vector2<double> TvelFlu;
    double XYCtrlScale = getPosCtrlCfg()->maxVelXY[0] / 50.0;
    TvelFlu.x = pitch_sitck_d * XYCtrlScale;
    TvelFlu.y = -roll_sitck_d * XYCtrlScale;

    // 转弯补偿
#define CIR_F
    double w;
    Attitude_Control_get_YawTrackVel(&w);
    vector2<double> TvelCir;
    if (sq(TvelFlu.x) + sq(TvelFlu.y) > sq(velocityFLU.x) + sq(velocityFLU.y))
    {
        TvelCir.x = -velocityFLU.y * w;
        TvelCir.y = velocityFLU.x * w;
    }
    else
    {
        TvelCir.x = -TvelFlu.y * w;
        TvelCir.y = TvelFlu.x * w;
    }
    TvelFlu += TvelCir * (1.0 / getPosCtrlCfg()->P1[0]);

    // 获取ENU系期望速度
    double yaw;
    double yaw_declination;
    get_YawDeclination(&yaw_declination);
    Attitude_Control_get_TargetTrackYaw(&yaw);
    yaw += yaw_declination;
    double sin_Yaw, cos_Yaw;
    fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
    TvelEnu.x = BodyHeading2ENU_x(TvelFlu.x, TvelFlu.y, sin_Yaw, cos_Yaw);
    TvelEnu.y = BodyHeading2ENU_y(TvelFlu.x, TvelFlu.y, sin_Yaw, cos_Yaw);

    // 判断摇杆中位
    bool rpStickNeutral = (roll_sitck_d == 0 && pitch_sitck_d == 0);

    // 限制期望速度XY
    vector3<double> velocityEnu;
    get_VelocityENU(&velocityEnu);
    vector2<double> velEnu(velocityEnu.x, velocityEnu.y);
    vector2<double> velErr = TvelEnu - velEnu;
    // 跟随速度修正误差
    velErr.x += followTVel_x;
    velErr.y += followTVel_y;
    double velErrDis = velErr.get_square();
    double maxVelErrDis = getPosCtrlCfg()->maxAccXY[0] / getPosCtrlCfg()->P2[0];
    vector2<double> TvelEnuC;
    if (velErrDis > sq(maxVelErrDis) && !rpStickNeutral)
    {
        velErrDis = safe_sqrt(velErrDis);
        velErr *= maxVelErrDis / velErrDis;
        TvelEnuC = velErr + velEnu;
    }
    else
    {
        TvelEnuC = TvelEnu;
    }

    // 调整期望位置速度
    tPos_x += TvelEnuC.x * h;
    tPos_y += TvelEnuC.y * h;
    tVel_x = TvelEnu.x * getPosCtrlCfg()->VelXYFF[0] + followTVel_x;
    tVel_y = TvelEnu.y * getPosCtrlCfg()->VelXYFF[0] + followTVel_y;
}

void init_FlightModes() {}