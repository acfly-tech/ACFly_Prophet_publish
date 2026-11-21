/*
        MSafe安全模式
        用户模式一定时间不进行控制
        或调用enter_MSafe会进入此模式

        进入安全模式后（ForceMSafeCtrl）
        用户模式控制将失效

        用户模式同时进行水平（XY位置或姿态）
        和垂直高度控制可退出MSafe获得控制权

        ！！不建议一般用户更改此文件！！
        ！！此文件需经过验证再发布！！
*/

#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "NavCmdProcess.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "StorageSystem.hpp"
#include "ctrl_Main.hpp"
#include "followTarget.hpp"
#include "precLand.hpp"
#include "task.h"

// 安全模式任务句柄
TaskHandle_t MSafeTaskHandle;
// 强制返航（降落）
bool ForceRTL = false;

// 电量状态
static PWR_STATE pwr_state = PWR_STATE::NORMAL;
PWR_STATE get_pwr_state()
{
    return pwr_state;
}

#define MSafeRate 20
static void MSafe_Server(void *pvParameters)
{
    // 准确周期延时
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    double h = 1.0 / MSafeRate;

    // 模式
    uint8_t ModeButtonZone = 255;
    AFunc cMode = AFunc_PosHold;

    // 参数
    MSafeCfg cfg;
    ModeFuncCfg MFunc_cfg;
    cfg.SfRtMode[0] = 1;             // SfRtMode
    cfg.RtSpeed[0] = 1000;           // 返航速度
    cfg.RtHRange[0] = 3000;          // 短距离返航范围
    cfg.RtHeight[0] = 5000;          // 返航高度
    cfg.RtLand1Height[0] = 20 * 100; // 一级降落高度
    cfg.RtLand2Height[0] = 10 * 100; // 二级降落高度
    cfg.RtLand1Vel[0] = -1;          // 一级降落速度(-1使用maxAutoVelDown)
    cfg.FcRTLVolt[0] = 0;            // 强制返航电压
    cfg.FcLandVolt[0] = 0;           // 强制降落电压
    const AvoidanceCfg *avCfg = getAvCfg();
    uint16_t cfg_update_counter = 60000;
    // 模式通用变量
    FlightMode_CommonVars comVars;

    // 电量状态判断
    uint16_t pwr_LVolt_Rtl_counter = 0;
    uint16_t pwr_LVolt_Land_counter = 0;

    // 跟随模式
    vector3<double> followPos;
    vector3<double> followOrigin;
    vector3<double> followTPos;
    vector3<double> followTVel;
    double followTYaw = 0;
    int8_t followYawSource = -1;
    // 跟随模式返航当前速度
    double followRTL_currentVel = 0;

    // 是否刚进入自动模式
    // 16-32有定位全自动模式
    // 32-48无定位自动模式
    uint8_t firstAuto = 0;
    NavCmdInf navInf;
    init_NavCmdInf(&navInf);
    uint16_t current_mission_ind;
    // 返航高度
    double RtHeight = -1;
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, h * configTICK_RATE_HZ);

        // 获取电压
        float mainBatVolt = 0;
        getCurrentBatteryTotalVoltRawFilted(&mainBatVolt);

        // 5秒更新读取一次配置
        if (++cfg_update_counter >= 5 * MSafeRate)
        {
            ReadParamGroup("MSafe", (uint64_t *) &cfg, 0);
            // 读取模式配置
            ReadParamGroup("MFunc", (uint64_t *) &MFunc_cfg, 0);
            // 获取避障设置
            avCfg = getAvCfg();
        }

        // 获取是否打开控制器
        // 控制器没打开不进行进一步操作
        bool attCtrlEna;
        is_Attitude_Control_Enabled(&attCtrlEna);
        if (attCtrlEna == false)
        {
            // 退出安全模式
            ForceMSafeCtrl = false;
            // 复位自动控制计时器
            firstAuto = 0;

            // 复位低电量检测
            if (mainBatVolt <= 7 || ((cfg.FcRTLVolt[0] <= 5 || mainBatVolt > cfg.FcRTLVolt[0]) && (cfg.FcLandVolt[0] <= 5 || mainBatVolt > cfg.FcLandVolt[0])))
                pwr_state = PWR_STATE::NORMAL;
            continue;
        }

        // 判断低电量
        if (mainBatVolt > 7)
        {
            if (cfg.FcRTLVolt[0] > 5)
            { // 判断返航电压
                if (mainBatVolt < cfg.FcRTLVolt[0])
                {
                    if (pwr_state < PWR_STATE::LVOLT_REQ_RTL)
                    {
                        if (++pwr_LVolt_Rtl_counter >= 3 * MSafeRate)
                        {
                            pwr_LVolt_Rtl_counter = 0;
                            pwr_state = PWR_STATE::LVOLT_REQ_RTL;
                        }
                    }
                    else
                        pwr_LVolt_Rtl_counter = 0;
                }
                else
                    pwr_LVolt_Rtl_counter = 0;
            }
            if (cfg.FcLandVolt[0] > 5)
            { // 判断降落电压
                if (mainBatVolt < cfg.FcLandVolt[0])
                {
                    if (pwr_state < PWR_STATE::LVOLT_REQ_LAND)
                    {
                        if (++pwr_LVolt_Land_counter >= 3 * MSafeRate)
                        {
                            pwr_LVolt_Land_counter = 0;
                            pwr_state = PWR_STATE::LVOLT_REQ_LAND;
                        }
                    }
                    else
                        pwr_LVolt_Land_counter = 0;
                }
                else
                    pwr_LVolt_Land_counter = 0;
            }
        }
        else
            pwr_LVolt_Rtl_counter = pwr_LVolt_Land_counter = 0;

        // 获取上次控制时间
        TIME lastXYCtrlTime, lastZCtrlTime;
        get_lastXYCtrlTime(&lastXYCtrlTime);
        get_lastZCtrlTime(&lastZCtrlTime);

        if ((SfRtMode_LpwMSafe(cfg.SfRtMode[0]) && pwr_state > PWR_STATE::NORMAL) || lastXYCtrlTime.get_pass_time() > 1.5 || lastZCtrlTime.get_pass_time() > 1.5)
        { // 低电量或控制超时
            // 强制进入MSafe控制
            ForceMSafeCtrl = true;
            // 打开高度控制器
            Altitude_Control_Enable();

            // 关闭喷洒，手动控制
            setPump1(PumpOffManualPercent);

            // 获取接收机
            Receiver rc;
            getReceiver(&rc);

            // 低电量时强制返航
            if (pwr_state > PWR_STATE::NORMAL)
                ForceRTL = true;

            // 判断是否使用遥控器控制
            // 无遥控器或ForceRTL且遥控回中时
            // 执行返航
            bool UseRcCtrl = false;
            bool sticks_in_neutral = true;
            if (rc.available)
            {
                sticks_in_neutral = in_symmetry_range_mid(rc.data[0], 50, (double) MFunc_cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[1], 50, (double) MFunc_cfg.NeutralZone[0]) &&
                                    in_symmetry_range_mid(rc.data[2], 50, (double) MFunc_cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, (double) MFunc_cfg.NeutralZone[0]);
                if (ForceRTL || pwr_state > PWR_STATE::NORMAL)
                {
                    if (!sticks_in_neutral)
                        UseRcCtrl = true;
                }
                else
                    UseRcCtrl = true;
            }

            if (UseRcCtrl)
            { // 使用遥控器控制
            rcCtrl:
                // 复位自动控制计时器
                firstAuto = 0;

                uint8_t new_ModeButtonZone;
                if (rc.available_channels >= 5)
                    new_ModeButtonZone = get_RcButtonZone(rc.data[4], ModeButtonZone);
                else
                    new_ModeButtonZone = 255;
                if (new_ModeButtonZone <= 5 && ModeButtonZone <= 5 && new_ModeButtonZone != ModeButtonZone)
                { // 模式按钮改变更改模式
                    cMode = (AFunc) MFunc_cfg.Bt1AFunc[new_ModeButtonZone][0];
                }
                ModeButtonZone = new_ModeButtonZone;

                // 根据5通状态选择定点定高
                if (cMode == AFunc_AltHold)
                    Position_Control_Disable();
                else
                    Position_Control_Enable();

                bool posCtrlEna;
                is_Position_Control_Enabled(&posCtrlEna);

                // 发布模式 - 遥控信号丢失时悬停为失控悬停
                if (posCtrlEna)
                    setCurrentFlyMode(AFunc_PosHold, MODE_CODE_REASON_RC_LOST);
                else
                    setCurrentFlyMode(AFunc_AltHold, MODE_CODE_REASON_RC_LOST);

                // 油门杆控制垂直速度
                if (in_symmetry_range_mid(rc.data[0], 50, MFunc_cfg.NeutralZone[0]))
                    Position_Control_set_ZLock();
                else
                {
                    double thr_stick = remove_deadband(rc.data[0] - 50.0, (double) MFunc_cfg.NeutralZone[0]);
                    if (thr_stick > 0)
                        thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
                    else
                        thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
                    Position_Control_set_TargetVelocityZ(thr_stick);
                }

                // 偏航杆在中间锁偏航
                // 不在中间控制偏航速度
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                if (in_symmetry_range_mid(rc.data[1], 50, MFunc_cfg.NeutralZone[0]))
                    Attitude_Control_set_YawLock();
                else
                    Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);

                // 水平控制
                if (posCtrlEna)
                {
                    // 俯仰横滚杆控水平速度
                    if (in_symmetry_range_mid(rc.data[3], 50, MFunc_cfg.NeutralZone[0]) && in_symmetry_range_mid(rc.data[2], 50, MFunc_cfg.NeutralZone[0]))
                        Position_Control_set_XYLock();
                    else
                    {
                        double RPCtrlScale = atan2(getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc);
                        double XYCtrlScale = getPosCtrlCfg()->maxVelXY[0] / 50.0;
                        double roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) MFunc_cfg.NeutralZone[0]);
                        double pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) MFunc_cfg.NeutralZone[0]);
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
                {
                    // 俯仰横滚杆控俯仰横滚
                    double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxLean[0] / 50.0);
                    Attitude_Control_set_Target_RollPitch((rc.data[3] - 50) * RPCtrlScale, (rc.data[2] - 50) * RPCtrlScale);
                }
            }
            else
            { // 无遥控信号

                // 尝试打开水平位置控制器
                Position_Control_Enable();

                // 重置模式配置
                ModeButtonZone = 255;
                cMode = AFunc_PosHold;

                bool posCtrlEna;
                is_Position_Control_Enabled(&posCtrlEna);

                if (ForceRTL || SfRtMode_AutoRTL(cfg.SfRtMode[0]))
                { // 需要返航
                    if (posCtrlEna)
                    { // 已打开水平位置控制器
                    proccessRTL:
                        // 上报返航模式 - 覆盖整个失控返航过程(刹车/爬升/回程/到家上空)
                        // 此后由降落段 setCurrentFlyMode(AFunc_Land) 接管
                        if (rc.available == false || (lastXYCtrlTime.get_pass_time() > 1.5 || lastZCtrlTime.get_pass_time() > 1.5))
                            setCurrentFlyMode(AFunc_RTL, MODE_CODE_REASON_RC_LOST);
                        else
                            setCurrentFlyMode(AFunc_RTL, MODE_CODE_REASON_LVOLT_RTL);

                        if (firstAuto < 32)
                        { // 刚进入失控状态

                            followSensor followS;
                            if (get_followSensor(&followS) && followS.available && isFollowDataType_NeedRTL(followS.dataType) && followS.updateTime.get_pass_time() < 2 &&
                                (!isFollowDataType_GlobalXY(followS.dataType) || followS.globalXYPosAvailable))
                            { // 跟随可用
                                // 初始化跟随
                                vector3<double> pos;
                                get_Position_Ctrl(&pos);
                                followPos = pos;
                                followRTL_currentVel = 0;

                                if (isFollowDataType_XY(followS.dataType))
                                { // 跟随目标可用
                                    followPos.x -= followS.pos.x;
                                    followPos.y -= followS.pos.y;
                                }
                                if (isFollowDataType_Z(followS.dataType))
                                { // 跟随目标可用
                                    followPos.z -= followS.pos.z;
                                }

                                // followOrigin.x = followS.pos.x;
                                // followOrigin.y = followS.pos.y;
                                // followOrigin.z = followS.pos.z;

                                followTPos.x = followS.pos.x;
                                followTPos.y = followS.pos.y;
                                followTPos.z = followS.pos.z;
                                followTPos -= followOrigin;
                                followTVel.x = followS.vel.x;
                                followTVel.y = followS.vel.y;
                                followTVel.z = followS.vel.z;

                                if (followS.yawAvailable())
                                {
                                    Quaternion quat;
                                    get_AirframeY_quat(&quat);
                                    double yaw = quat.getYaw();
                                    followTYaw = fmod(yaw - followS.yaw, 2 * Pi);
                                    if (followTYaw > Pi)
                                        followTYaw -= 2 * Pi;
                                    else if (followTYaw < -Pi)
                                        followTYaw += 2 * Pi;
                                    followYawSource = 1;
                                }
                                else
                                    followYawSource = -1;

                                // 进入跟随返航
                                init_NavCmdInf(&navInf);
                                current_mission_ind = 0;
                                firstAuto = 64;
                            }
                            else
                            { // 跟随不可用

                                if (!sticks_in_neutral)
                                    goto rcCtrl;

                                // 刹车锁位置
                                Attitude_Control_set_YawLock();
                                Position_Control_set_XYLock();
                                Position_Control_set_ZLock();
                                if (firstAuto != 16)
                                { // 初始化等待
                                    init_NavCmdInf(&navInf);
                                    firstAuto = 16;
                                }
                                else
                                { // 刹车后等待2秒
                                    Position_ControlMode alt_mode, pos_mode;
                                    get_Altitude_ControlMode(&alt_mode);
                                    get_Position_ControlMode(&pos_mode);
                                    if (alt_mode == Position_ControlMode_Position && pos_mode == Position_ControlMode_Position)
                                    {
                                        if (++navInf.counter2 >= 2 * MSafeRate)
                                        {
                                            init_NavCmdInf(&navInf);
                                            current_mission_ind = 0;
                                            firstAuto = 32;
                                        }
                                    }
                                }
                            }
                        }
                        else if (firstAuto == 64)
                        { // 跟随模式返航

                            followSensor followS;
                            if (get_followSensor(&followS) && followS.available && isFollowDataType_NeedRTL(followS.dataType) &&
                                (!isFollowDataType_GlobalXY(followS.dataType) || followS.globalXYPosAvailable))
                            { // 跟随可用
                                followTPos.x = followS.pos.x;
                                followTPos.y = followS.pos.y;
                                followTPos.z = followS.pos.z;
                                followTPos -= followOrigin;
                                followTVel.x = followS.vel.x;
                                followTVel.y = followS.vel.y;
                                followTVel.z = followS.vel.z;
                            }
                            else
                            { // 跟随不可用
                                firstAuto = 0;
                                goto proccessRTL;
                            }

                            double roll_sitck_d = 0;
                            double pitch_sitck_d = 0;
                            double thr_stick = 0;

                            vector3<double> missionTVel;
                            switch (current_mission_ind)
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
                                        double max_v = cfg.RtSpeed[0];
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

                                    // 检查航向
                                    if (followYawSource == 1 && !followS.yawAvailable())
                                    {
                                        followYawSource = -1;
                                    }
                                    else if (followYawSource != 1 && followS.yawAvailable())
                                    {
                                        Quaternion quat;
                                        get_AirframeY_quat(&quat);
                                        double yaw = quat.getYaw();
                                        followTYaw = fmod(yaw - followS.yaw, 2 * Pi);
                                        if (followTYaw > Pi)
                                            followTYaw -= 2 * Pi;
                                        else if (followTYaw < -Pi)
                                            followTYaw += 2 * Pi;
                                        followYawSource = 1;
                                    }

                                    // 控制偏航
                                    if (in_symmetry_range_mid(rc.data[1], 50, MFunc_cfg.NeutralZone[0]))
                                    {
                                        if (followYawSource == 1)
                                            Attitude_Control_set_Target_Yaw_Offboard(followTYaw + followS.yaw, 0);
                                        else
                                            Attitude_Control_set_YawLock();
                                    }
                                    else
                                    {
                                        double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                                        double targetYawRate = (50 - rc.data[1]) * YCtrlScale;
                                        if (followYawSource == 1)
                                        {
                                            followTYaw = fmod(followTYaw + h * targetYawRate, 2 * Pi);
                                            if (followTYaw > Pi)
                                                followTYaw -= 2 * Pi;
                                            else if (followTYaw < -Pi)
                                                followTYaw += 2 * Pi;
                                            Attitude_Control_set_Target_Yaw_Offboard(followTYaw + followS.yaw, remove_deadband(targetYawRate, degree2rad(50.0)));
                                        }
                                        else
                                            Attitude_Control_set_Target_YawRate(targetYawRate);
                                    }

                                    // 判断是否返航结束
                                    if (rtlRm < 0.01)
                                    {
                                        if (++navInf.counter2 >= 3 * MSafeRate)
                                        {
                                            init_NavCmdInf(&navInf);

                                            precLandSensor precSensor;
                                            if (precLandSensor_get(&precSensor) && precSensor.available && precSensor.yaw_available)
                                            {
                                                Attitude_Control_set_Target_Yaw(precSensor.yaw);
                                                followYawSource = 2;
                                                followTYaw = 0;
                                            }
                                            else if (followS.yawAvailable())
                                            {
                                                Attitude_Control_set_Target_Yaw(followS.yaw);
                                                followYawSource = 1;
                                                followTYaw = 0;
                                            }
                                            else
                                            {
                                                Attitude_Control_set_YawLock();
                                                followYawSource = -1;
                                            }
                                            ++current_mission_ind;
                                        }
                                    }
                                    break;
                                }

                                case 1: // 转偏航
                                {
                                    Position_Control_set_XYLock();
                                    Position_Control_set_ZLock();
                                    double yawTrackErr;
                                    Attitude_Control_get_YawTrackErr(&yawTrackErr);
                                    if (yawTrackErr < 0.01)
                                    {
                                        init_NavCmdInf(&navInf);
                                        ++current_mission_ind;
                                    }
                                    break;
                                }

                                default:
                                case 2:
                                { // 降落

                                    // 降落过程允许摇杆操作
                                    roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) MFunc_cfg.NeutralZone[0]);
                                    pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) MFunc_cfg.NeutralZone[0]);
                                    thr_stick = remove_deadband(rc.data[0] - 50.0, (double) MFunc_cfg.NeutralZone[0]);

                                    // 降落速度
                                    double landVel = getPosCtrlCfg()->LandVel[0];

                                    static double ddd[2];
                                    precLandSensor precSensor;
                                    if (precLandSensor_get(&precSensor) && precSensor.available)
                                    {
                                        if (precSensor.z_available)
                                        {
                                            if (precSensor.pos.z < -1500)
                                                landVel = 250;
                                            else if (precSensor.pos.z < -1000)
                                                landVel = 150;
                                        }

                                        if (precSensor.updateTime != *(TIME *) &navInf.temp[1])
                                        {
                                            vector3<double> tPos, pos;
                                            get_Position_Ctrl(&pos);
                                            get_TargetPosInf(0, 0, &tPos, 0);

                                            if (navInf.temp[0] == 1)
                                            {
                                                // followPos.x += precSensor.pos.x - navInf.temp[2];
                                                // followPos.y += precSensor.pos.y - navInf.temp[3];
                                                // followPos.x += precSensor.pos.x;
                                                // followPos.y += precSensor.pos.y;
                                            }
                                            navInf.temp[0] = 1;
                                            *(TIME *) &navInf.temp[1] = precSensor.updateTime;
                                            navInf.temp[2] = tPos.x - pos.x;
                                            navInf.temp[3] = tPos.y - pos.y;

                                            ddd[0] = precSensor.pos.x;
                                            ddd[1] = precSensor.pos.y;

                                            vector3<double> posErr;
                                            get_history_PositionErr(&posErr, precSensor.delay);
                                            followPos.x += precSensor.pos.x - posErr.x;
                                            followPos.y += precSensor.pos.y - posErr.y;
                                        }
                                    }
                                    else
                                        Position_Control_set_XYLock();

                                    double maxLandVel = getPosCtrlCfg()->maxVelDown[0] * 0.8;
                                    if (landVel > maxLandVel)
                                        landVel = maxLandVel;
                                    followPos.z -= landVel * h;

                                    // 检查航向
                                    if (followYawSource == 1 && !followS.yawAvailable())
                                    {
                                        followYawSource = -1;
                                    }
                                    else if (followYawSource == 2 && !precSensor.yaw_available)
                                    {
                                        followYawSource = -1;
                                    }
                                    if (followYawSource == -1 && precSensor.yaw_available)
                                    {
                                        followTYaw = 0;
                                        followYawSource = 2;
                                    }
                                    if (followYawSource == -1 && followS.yawAvailable())
                                    {
                                        followTYaw = 0;
                                        followYawSource = 1;
                                    }

                                    // 控制偏航
                                    if (in_symmetry_range_mid(rc.data[1], 50, MFunc_cfg.NeutralZone[0]))
                                    {
                                        if (followYawSource == 1)
                                            Attitude_Control_set_Target_Yaw_Offboard(followTYaw + followS.yaw, 0);
                                        else if (followYawSource == 2)
                                            Attitude_Control_set_Target_Yaw_Offboard(followTYaw + precSensor.yaw, 0);
                                        else
                                            Attitude_Control_set_YawLock();
                                    }
                                    else
                                    {
                                        double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                                        double targetYawRate = (50 - rc.data[1]) * YCtrlScale;
                                        if (followYawSource >= 0)
                                        {
                                            followTYaw = fmod(followTYaw + h * targetYawRate, 2 * Pi);
                                            if (followTYaw > Pi)
                                                followTYaw -= 2 * Pi;
                                            else if (followTYaw < -Pi)
                                                followTYaw += 2 * Pi;
                                            if (followYawSource == 1)
                                                Attitude_Control_set_Target_Yaw_Offboard(followTYaw + followS.yaw, 0);
                                            else if (followYawSource == 2)
                                                Attitude_Control_set_Target_Yaw_Offboard(followTYaw + precSensor.yaw, remove_deadband(targetYawRate, degree2rad(50.0)));
                                        }
                                        else
                                            Attitude_Control_set_Target_YawRate(targetYawRate);
                                    }

                                    double vvv[6];
                                    vvv[0] = followPos.x * 0.01;
                                    vvv[1] = followPos.y * 0.01;
                                    vvv[2] = followPos.z * 0.01;
                                    vvv[3] = ddd[0] * 0.01;
                                    vvv[4] = ddd[1] * 0.01;
                                    vvv[5] = landVel;
                                    SDLog_Msg_DebugVect("prec", vvv, 6);

                                    break;
                                }
                            }

                            // XY期望速度
                            // double XYCtrlScale = getPosCtrlCfg()->maxVelXY[0] / 50.0;
                            double XYCtrlScale = 500 / 50.0;
                            vector3<double> velocityFLU;
                            get_VelocityFLU_Ctrl(&velocityFLU);
                            double vel_stick_err_pitch = velocityFLU.x / XYCtrlScale - pitch_sitck_d;
                            double vel_stick_err_roll = velocityFLU.y / XYCtrlScale - -roll_sitck_d;
                            constrain_vector(vel_stick_err_roll, vel_stick_err_pitch, 50);
                            vector2<double> TvelFlu;
                            TvelFlu.x = pitch_sitck_d * XYCtrlScale;
                            TvelFlu.y = -roll_sitck_d * XYCtrlScale;

                            // Z期望速度
                            double TvelZ;
                            if (thr_stick > 0)
                                TvelZ = thr_stick * getPosCtrlCfg()->maxVelUp[0] / 50;
                            else
                                TvelZ = thr_stick * getPosCtrlCfg()->maxVelDown[0] / 50;

                            // 获取ENU系期望速度
                            double yaw;
                            double yaw_declination;
                            get_YawDeclination(&yaw_declination);
                            Attitude_Control_get_TargetTrackYaw(&yaw);
                            yaw += yaw_declination;
                            double sin_Yaw, cos_Yaw;
                            fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                            vector2<double> TvelEnu;
                            TvelEnu.x = BodyHeading2ENU_x(TvelFlu.x, TvelFlu.y, sin_Yaw, cos_Yaw);
                            TvelEnu.y = BodyHeading2ENU_y(TvelFlu.x, TvelFlu.y, sin_Yaw, cos_Yaw);

                            // 限制期望速度XY
                            vector3<double> velocityEnu;
                            get_VelocityENU(&velocityEnu);
                            vector2<double> velEnu(velocityEnu.x, velocityEnu.y);
                            vector2<double> velErr = TvelEnu - velEnu;
                            velErr.x += missionTVel.x;
                            velErr.y += missionTVel.y;
                            if (isFollowDataType_XY(followS.dataType))
                            { // 跟随目标可用
                                velErr.x += followTVel.x;
                                velErr.y += followTVel.y;
                            }
                            double velErrDis = velErr.get_square();
                            double maxVelErrDis = getPosCtrlCfg()->maxAccXY[0] / getPosCtrlCfg()->P2[0];
                            vector2<double> TvelEnuC;
                            if (velErrDis > sq(maxVelErrDis) && (roll_sitck_d != 0 || roll_sitck_d != 0))
                            {
                                velErrDis = safe_sqrt(velErrDis);
                                velErr *= maxVelErrDis / velErrDis;
                                TvelEnuC = velErr + velEnu;
                            }
                            else
                            {
                                TvelEnuC = TvelEnu;
                            }

                            // 限制期望速度Z
                            double velErrZ = TvelZ - velocityEnu.z;
                            velErrZ += missionTVel.z;
                            if (isFollowDataType_Z(followS.dataType))
                            { // 跟随目标可用
                                velErrZ += followTVel.z;
                            }
                            velErrDis = fabs(velErrZ);
                            double TvelZC;
                            if (velErrDis > maxVelErrDis && thr_stick != 0)
                            {
                                velErrZ *= maxVelErrDis / velErrDis;
                                TvelZC = velErrZ + velocityEnu.z;
                            }
                            else
                            {
                                TvelZC = TvelZ;
                            }

                            // 计算期望速度
                            vector3<double> TVel;
                            TVel.x = TvelEnu.x * getPosCtrlCfg()->VelXYFF[0];
                            TVel.y = TvelEnu.y * getPosCtrlCfg()->VelXYFF[0];
                            TVel.z = TvelZ * getPosCtrlCfg()->VelXYFF[0];

                            // 计算期望位置
                            followPos.x += TvelEnuC.x * h;
                            followPos.y += TvelEnuC.y * h;
                            followPos.z += TvelZC * h;
                            vector3<double> TPos;
                            TPos.x = followPos.x;
                            TPos.y = followPos.y;
                            TPos.z = followPos.z;

                            // XY速度前馈
                            TVel.x += missionTVel.x;
                            TVel.y += missionTVel.y;
                            if (isFollowDataType_XY(followS.dataType))
                            { // 跟随目标可用
                                TPos.x += followTPos.x;
                                TPos.y += followTPos.y;
                                TVel.x += followTVel.x;
                                TVel.y += followTVel.y;
                            }

                            // Z速度前馈
                            TVel.z += missionTVel.z;
                            if (isFollowDataType_Z(followS.dataType))
                            { // 跟随目标可用
                                TPos.z += followTPos.z;
                                TVel.z += followTVel.z;
                            }

                            // 水平控制
                            Position_Control_set_TargetPosVelAccXY_OffBoard(TPos.x, TPos.y, TVel.x, TVel.y, 0, 0);

                            Position_Control_set_TargetPosVelAccZ_OffBoard(TPos.z, TVel.z, 0);
                        }
                        else
                        { // 返航回起飞点

                            if (!sticks_in_neutral)
                                goto rcCtrl;

                            // 降落电压强制降落
                            if (pwr_state == PWR_STATE::LVOLT_REQ_LAND)
                            {
                                // 上报模式状态 - 智能低电压返航后降落
                                setCurrentFlyMode(AFunc_Land, MODE_CODE_REASON_LVOLT_RTL_LAND);
                                current_mission_ind = 3;
                            }
                            else
                            {
                                // 上报模式状态 - 低电压返航(此前失控时为遥控信号丢失返航)
                                if (rc.available == false || (lastXYCtrlTime.get_pass_time() > 1.5 || lastZCtrlTime.get_pass_time() > 1.5))
                                    setCurrentFlyMode(AFunc_RTL, MODE_CODE_REASON_RC_LOST);
                                else
                                    setCurrentFlyMode(AFunc_RTL, MODE_CODE_REASON_LVOLT_RTL);
                            }

                            switch (current_mission_ind)
                            {
                                case 0:
                                { // 判断返航距离是否需要升高
                                    vector2<double> homeP;
                                    if (getHomeLatLon(&homeP))
                                    { // 返航至经纬度

                                        // 获取最优全球定位传感器信息
                                        PosSensorHealthInf2 global_inf;
                                        if (get_OptimalGlobal_XY(&global_inf) == false)
                                        {
                                            if (getHomePoint(&homeP))
                                                goto RtLocal;
                                            else
                                            { // 无返航点
                                                // 直接降落
                                                current_mission_ind = 3;
                                                break;
                                            }
                                        }
                                        // 获取指定经纬度平面坐标
                                        double x, y;
                                        map_projection_project(&global_inf.mp, homeP.x, homeP.y, &x, &y);
                                        x -= global_inf.HOffset.x;
                                        y -= global_inf.HOffset.y;
                                        double RtDistanceSq = sq(y - global_inf.PositionENU.y) + sq(x - global_inf.PositionENU.x);

                                        // 判断是否大于升高返航距离
                                        if (RtDistanceSq > sq(cfg.RtHRange[0]))
                                        { // 升高返航
                                            RtHeight = cfg.RtHeight[0];
                                            double homeZ;
                                            getHomeLocalZ(&homeZ);
                                            vector3<double> pos;
                                            get_Position_Ctrl(&pos);
                                            if (RtHeight > 0 && homeZ + RtHeight > pos.z)
                                            { // 返航高度大于当前高度才进行升高
                                            }
                                            else
                                                RtHeight = -1;
                                        }
                                        else
                                            RtHeight = -1;

                                        // 切换至升高状态
                                        init_NavCmdInf(&navInf);
                                        ++current_mission_ind;
                                    }
                                    else if (getHomePoint(&homeP))
                                    { // 返航至Local坐标
                                    RtLocal:
                                        vector3<double> position;
                                        get_Position_Ctrl(&position);
                                        double RtDistanceSq = sq(homeP.y - position.y) + sq(homeP.x - position.x);

                                        // 判断是否大于升高返航距离
                                        if (RtDistanceSq > sq(cfg.RtHRange[0]))
                                        { // 升高返航
                                            RtHeight = cfg.RtHeight[0];
                                            double homeZ;
                                            getHomeLocalZ(&homeZ);
                                            vector3<double> pos;
                                            get_Position_Ctrl(&pos);
                                            if (RtHeight > 0 && homeZ + RtHeight > pos.z)
                                            { // 返航高度大于当前高度才进行升高
                                            }
                                            else
                                                RtHeight = -1;
                                        }
                                        else
                                            RtHeight = -1;

                                        // 切换至升高状态
                                        init_NavCmdInf(&navInf);
                                        ++current_mission_ind;
                                    }
                                    else
                                    { // 无返航点
                                        // 直接降落
                                        current_mission_ind = 3;
                                        break;
                                    }
                                    break;
                                }

                                case 1:
                                { // 升高到指定对地高度
                                    double homeZ;
                                    getHomeLocalZ(&homeZ);
                                    vector3<double> pos;
                                    get_Position_Ctrl(&pos);
                                    if (RtHeight > 0)
                                    {
                                        double params[7];
                                        params[0] = 0;
                                        params[1] = 0;
                                        params[2] = 0;
                                        params[3] = nan("");
                                        params[4] = 230;
                                        params[5] = 230;
                                        params[6] = RtHeight * 0.01;
                                        int16_t res =
                                            Process_NavCmd(MAV_CMD_NAV_WAYPOINT, MSafeRate, MAV_FRAME_GLOBAL_RELATIVE_ALT, params, &navInf, rc, GLB_FlightModeFlags, comVars, &MFunc_cfg, avCfg);
                                        if (NavCmdRs_SuccessOrFault(res))
                                        {
                                            init_NavCmdInf(&navInf);
                                            ++current_mission_ind;
                                        }
                                    }
                                    else
                                    {
                                        init_NavCmdInf(&navInf);
                                        ++current_mission_ind;
                                    }
                                    break;
                                }

                                case 2:
                                { // 回到Home点
                                    double params[7];
                                    params[0] = 0;
                                    params[1] = 0;
                                    params[2] = 0;
                                    params[3] = 0;
                                    params[4] = 0;
                                    params[5] = 0;
                                    params[6] = 0;
                                    int16_t res =
                                        Process_NavCmd(MAV_CMD_NAV_RETURN_TO_LAUNCH, MSafeRate, MAV_FRAME_GLOBAL_RELATIVE_ALT, params, &navInf, rc, GLB_FlightModeFlags, comVars, &MFunc_cfg, avCfg);
                                    if (NavCmdRs_SuccessOrFault(res))
                                    {
                                        init_NavCmdInf(&navInf);
                                        ++current_mission_ind;
                                    }
                                    break;
                                }

                                default:
                                case 3:
                                { // 降落
                                    Position_Control_set_XYLock();
                                    Position_Control_set_TargetVelocityZ(-50);
                                    break;
                                }
                            }
                        }
                    }
                    else
                    { // 无法打开位置控制器
                        // 让姿态回复水平
                        Attitude_Control_set_Target_RollPitch(0, 0);
                        // 下降
                        Position_Control_set_TargetVelocityZ(-50);
                        // 复位自动控制计时器
                        firstAuto = 0;
                    }
                }
                else
                { // 失控悬停

                    // 复位自动控制计时器
                    firstAuto = 0;

                    // 悬停
                    Attitude_Control_set_YawLock();
                    Position_Control_set_XYLock();
                    Position_Control_set_ZLock();
                }
            }
            // 已降落关闭控制器并退出MSafe
            bool inFlight;
            get_is_inFlight(&inFlight);
            if (inFlight == false)
            {
                Attitude_Control_Disable();
                os_delay(0.5);
                ForceMSafeCtrl = false;
            }
        }
        else
        {
            ForceMSafeCtrl = false;

            // 复位自动控制计时器
            firstAuto = 0;
        }
    }
}

void init_MSafe()
{
    // 注册参数
    MSafeCfg initial_cfg;
    initial_cfg.SfRtMode[0] = 1;             // SfRtMode
    initial_cfg.RtSpeed[0] = 1000;           // RtSpeed
    initial_cfg.RtHRange[0] = 3000;          // RtHRange
    initial_cfg.RtHeight[0] = 5000;          // RtHeight
    initial_cfg.RtLand1Height[0] = 20 * 100; // 一级降落高度
    initial_cfg.RtLand2Height[0] = 10 * 100; // 二级降落高度
    initial_cfg.RtLand1Vel[0] = -1;          // 一级降落速度(-1使用maxAutoVelDown)
    initial_cfg.FcAlarmVolt[0] = 0;          // 警报电压
    initial_cfg.FcRTLVolt[0] = 0;            // 强制返航电压
    initial_cfg.FcLandVolt[0] = 0;           // 强制降落电压
    initial_cfg.MaxALHRange[0] = 50 * 100;   // 备降点最大范围cm
    // 备降点1
    initial_cfg.AL1_lat[0] = 0;
    initial_cfg.AL1_lon[0] = 0;
    initial_cfg.AL1_alt[0] = 10 * 100;
    // 备降点2
    initial_cfg.AL2_lat[0] = 0;
    initial_cfg.AL2_lon[0] = 0;
    initial_cfg.AL2_alt[0] = 10 * 100;
    // 备降点3
    initial_cfg.AL3_lat[0] = 0;
    initial_cfg.AL3_lon[0] = 0;
    initial_cfg.AL3_alt[0] = 10 * 100;

    MAV_PARAM_TYPE param_types[] = {
        MAV_PARAM_TYPE_UINT32, // SfRtMode
        MAV_PARAM_TYPE_REAL32, // RtSpeed

        MAV_PARAM_TYPE_REAL32, // RtHRange
        MAV_PARAM_TYPE_REAL32, // RtHeight
        MAV_PARAM_TYPE_REAL32, // 一级降落高度
        MAV_PARAM_TYPE_REAL32, // 二级降落高度
        MAV_PARAM_TYPE_REAL32, // 一级降落速度

        MAV_PARAM_TYPE_REAL32, // 警报电压
        MAV_PARAM_TYPE_REAL32, // 强制返航电压
        MAV_PARAM_TYPE_REAL32, // 强制降落电压

        MAV_PARAM_TYPE_REAL32, // 备降点最大范围cm

        // 备降点1
        MAV_PARAM_TYPE_INT32,
        MAV_PARAM_TYPE_INT32,
        MAV_PARAM_TYPE_REAL32,

        // 备降点2
        MAV_PARAM_TYPE_INT32,
        MAV_PARAM_TYPE_INT32,
        MAV_PARAM_TYPE_REAL32,

        // 备降点3
        MAV_PARAM_TYPE_INT32,
        MAV_PARAM_TYPE_INT32,
        MAV_PARAM_TYPE_REAL32,
    };
    SName param_names[] = {
        "Sf_SfRtMode", // SfRtMode
        "Sf_RtSpeed",  // RtSpeed

        "Sf_GbRtHRange", // GbRtHRange
        "Sf_GbRtHeight", // GbRtHeight
        "Sf_RtLand1H",   // 一级降落高度
        "Sf_RtLand2H",   // 二级降落高度
        "Sf_RtLand1V",   // 一级降落速度

        "Sf_AlarmVolt",  // 警报电压
        "Sf_FcRTLVolt",  // 强制返航电压
        "Sf_FcLandVolt", // 强制降落电压

        "Sf_ALMaxDist", // 备降点最大范围cm

        // 备降点1
        "Sf_AL1_lat",
        "Sf_AL1_lon",
        "Sf_AL1_alt",

        // 备降点1
        "Sf_AL2_lat",
        "Sf_AL2_lon",
        "Sf_AL2_alt",

        // 备降点1
        "Sf_AL3_lat",
        "Sf_AL3_lon",
        "Sf_AL3_alt",
    };
    InitialParametersInit((uint64_t *) &initial_cfg, param_types, param_names, sizeof(MSafeCfg) / 8);
    ParamGroupRegister("MSafe", 3, sizeof(MSafeCfg) / 8, param_types, param_names, (uint64_t *) &initial_cfg);

    xTaskCreate(MSafe_Server, "MSafe", 1200, NULL, SysPriority_SafeTask, &MSafeTaskHandle);
}