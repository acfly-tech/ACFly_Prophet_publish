/*
 * FlightMode_AutoMission - 二次开发自动任务模式 (AFunc 35)
 *
 * 用途: 机载任务的快速二次开发沙盒。
 *       任务流程: 刹车 -> 起飞 -> 绕正方形航线飞行 -> 自动降落。
 *       开发者只需修改 main_func 中的 step 状态机(===== 开发者任务开发区 =====)。
 *
 * 进入方式:
 *       1. 参数 MFunc_Bt2Func(或Bt3/Bt4Func) 设为 35: 专属任务拨杆,
 *          拨动切入任务, 再次拨动切回主拨杆档位模式;
 *       2. 参数 MFunc_Bt1AFuncN 把某个拨杆档位设为 35, 拨到该档位进入;
 *       3. 地面站 MAVLink SET_MODE 预选(解锁后直接运行任务)或空中切入
 *          (custom mode 0x23040000)。
 *
 * 解锁条件: 需要位置信息(水平+高度)就绪, 否则拒绝解锁。
 *
 * 任务打断与续飞:
 *       - 打杆: 不退出本模式, 立即切换手动接管(定点/定高手感),
 *         四杆回中后立即刹车并从断点步骤继续执行任务;
 *       - 空中失去水平定位: 不退出本模式, 自动降级为姿态定高悬停,
 *         任务暂停, 定位恢复且摇杆回中后自动续飞;
 *       - 拨杆/cmd176显式切出后重进本模式: 从断点步骤续飞;
 *       - 落地后进入 / 任务完成: 全新任务。
 *
 * 安全特性(由 M32_PosCtrl 外壳自动提供, 无需开发者实现):
 *       - 低电量自动返航/降落, 失控自动返航(MSafe独立任务);
 *       - 任务完成返回 FMR_RqDisarm_Land 自动降落并上锁;
 *       - 出错需要返航时 return FMR_Err_RqSwitchRTL。
 *
 * 常用控制原语参数速查 (MAVLink 标准, BODY_FLU 机体系, 单位: 米/秒/度):
 *       Process_NavCmd(cmd, freq, frame, params, ...) 的 params[7]:
 *               params[0] Hold时间(s), 到达航点后停留该时长才算完成
 *               params[1] Accept半径(m), 0为默认
 *               params[2] Pass半径(m), 0为穿过航点
 *               params[3] 偏航角(度), NaN=指向航点, 超过±360=不改变偏航
 *               params[4] 前向距离(m), 机体系x
 *               params[5] 左向距离(m), 机体系y
 *               params[6] 相对高度(m), 相对起飞点
 *       cmd = MAV_CMD_NAV_TAKEOFF 时 params[6] 为起飞高度
 *       返回值 res: NavCmdRs_SuccessOrFault(res) 为真表示本步完成
 */

#include "FlightMode_AutoMission.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "NavCmdProcess.hpp"
#include "Receiver.hpp"

#include "AC_Math.hpp"
#include "vector3.hpp"

// 跨实例断点
uint8_t FlightMode_AutoMission::s_saved_step = 0;
uint8_t FlightMode_AutoMission::s_saved_loop = 0;

bool FlightMode_AutoMission::set_params(double params[], uint16_t params_cnt)
{
    return true;
}

FlightMode_AutoMission::FlightMode_AutoMission()
    : FlightMode_Base("AutoMission", AFunc_AutoMission, 0)
{
    land_counter = 0;
    suspended = false;
    recovering = false;
    finished = false;
    // 复位任务状态
    init_NavCmdInf(&navInf);

    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight && s_saved_step >= 1 && s_saved_step <= 6)
    { // 空中重进且有断点: 携带断点, 刹车后续飞
        entry_resume_step = s_saved_step;
        loop_count = s_saved_loop;
    }
    else
    { // 地面进入或无断点: 全新任务, 清除陈旧断点
        entry_resume_step = 0;
        loop_count = 0;
        s_saved_step = 0;
    }
    step = 0;
}

FlightMode_AutoMission::~FlightMode_AutoMission()
{
    if (finished == false && step >= 1 && step <= 6)
    { // 任务被显式打断(拨杆/cmd176/低电注入): 保存断点供重进续飞
        s_saved_step = step;
        s_saved_loop = loop_count;
    }
    else
        s_saved_step = 0;
}

// 获取飞行模式类别
// FMC_SemiAutomatic: 打杆不触发M32外壳切出, 由本模式内部接管(打断续飞)
FlightModeClass FlightMode_AutoMission::get_fmClass()
{
    return FMC_SemiAutomatic;
}

// 模式进入检查: 需要位置信息(水平+高度), 与解锁条件一致
FlightModeCheckResult FlightMode_AutoMission::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    if (get_Position_MSStatus() != MS_Ready || get_Altitude_MSStatus() != MS_Ready)
        return FMCR_Err_NoPositioning;

    return FMCR_OK;
}

// 模式主函数: 由M32_PosCtrl以固定频率调用(非阻塞, 不允许死循环)
FlightModeResult FlightMode_AutoMission::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                                   const AvoidanceCfg *avCfg, double h)
{
    // 计算运行频率
    double freq = 1.0 / h;

    // 打开位置控制器(空中失去水平定位时使能失败, 控制器自动关闭XY控制,
    // 由挂起状态机降级为姿态定高, 不退出本模式)
    Altitude_Control_Enable();
    Position_Control_Enable();

    // 获取飞行状态
    bool inFlight;
    get_is_inFlight(&inFlight);

    // 检测打杆与水平定位丢失
    bool sticks_out = false;
    if (rc.available)
        sticks_out = !(in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]) &&
                       in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]));
    bool pos_lost = (get_Position_MSStatus() != MS_Ready);

    // 挂起触发(立即, 无防抖): 打杆接管 或 空中失去水平定位
    if ((sticks_out || (pos_lost && inFlight)) && suspended == false)
    {
        suspended = true;
        recovering = false;
        // 丢弃进行中的NavCmd中间状态, 恢复时当前步骤重新执行
        init_NavCmdInf(&navInf);
    }

    if (suspended)
    { // 挂起子态: 任务冻结, 手动控制(定点/定高)
        if (rc.available && sticks_out == false && pos_lost == false)
        { // 摇杆回中且定位恢复: 进入恢复刹车
            suspended = false;
            recovering = true;
            navInf.counter2 = 0;
        }
        else
        {
            if (rc.available)
            { // 手动控制(定点/定高手感)
                bool pos_ena;
                is_Position_Control_Enabled(&pos_ena);

                // 上报模式状态
                setCurrentFlyMode(pos_ena ? AFunc_PosHold : AFunc_AltHold);

                // 油门杆控制垂直速度
                if (in_symmetry_range_mid(rc.data[0], 50, cfg->NeutralZone[0]))
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

                // 水平控制
                if (in_symmetry_range_mid(rc.data[3], 50, cfg->NeutralZone[0]) && in_symmetry_range_mid(rc.data[2], 50, cfg->NeutralZone[0]))
                    Position_Control_set_XYLock();
                else if (pos_ena)
                { // 有定位: 杆量速度控制+倾角限制
                    double RPCtrlScale = atan2(getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc);
                    double XYCtrlScale = getPosCtrlCfg()->maxVelXY[0] / 50.0;
                    double roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]);
                    double pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]);
                    vector3<double> velocityFLU;
                    get_VelocityFLU_Ctrl(&velocityFLU);
                    double vel_stick_err_pitch = velocityFLU.x / XYCtrlScale - pitch_sitck_d;
                    double vel_stick_err_roll = velocityFLU.y / XYCtrlScale - -roll_sitck_d;
                    Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(pitch_sitck_d * XYCtrlScale, -roll_sitck_d * XYCtrlScale, fabs(vel_stick_err_roll) * RPCtrlScale,
                                                                                fabs(vel_stick_err_pitch) * RPCtrlScale);
                }
                else
                { // 无定位: 姿态定高控制
                    double RPCtrlScale = degree2rad(getAttCtrlCfg()->maxLean[0] / 50.0);
                    Attitude_Control_set_Target_RollPitch(remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]) * RPCtrlScale,
                                                          remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]) * RPCtrlScale);
                }

                // 偏航杆在中间锁偏航, 不在中间控制偏航速度
                double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
                if (in_symmetry_range_mid(rc.data[1], 50, cfg->NeutralZone[0]))
                    Attitude_Control_set_YawLock();
                else
                    Attitude_Control_set_Target_YawRate((50 - rc.data[1]) * YCtrlScale);
            }
            return FMR_OK;
        }
    }

    if (recovering)
    { // 恢复刹车: 刹车并等待1秒后继续任务
        Position_Control_set_XYLock();
        Position_Control_set_ZLock();
        // 锁定偏航: 清除挂起期间残留的角速度目标(否则松杆后持续旋转)
        Attitude_Control_set_YawLock();

        Position_ControlMode alt_mode, pos_mode;
        get_Altitude_ControlMode(&alt_mode);
        get_Position_ControlMode(&pos_mode);
        if (alt_mode == Position_ControlMode_Position && pos_mode == Position_ControlMode_Position)
        { // 刹车完成, 等待1秒
            ++navInf.counter2;
            if (navInf.counter2 >= 1 * freq)
            {
                recovering = false;
                navInf.counter2 = 0;
            }
        }
        else
            navInf.counter2 = 0;
        return FMR_OK;
    }

    // 上报模式状态
    setCurrentFlyMode(AFunc_AutoMission);

    /* ============================================================
     * ==================  开发者任务开发区  =========================
     * step状态机: 每个case执行一步任务动作
     * 步骤完成( NavCmdRs_SuccessOrFault(res) 为真 )后 ++step 进入下一步
     * 任务全部完成后 return FMR_RqDisarm_Land 自动降落上锁
     * 出错需要返航时 return FMR_Err_RqSwitchRTL
     * ============================================================ */
    switch (step)
    {
        // 刹车等待: 原地悬停1秒, 等位置控制器稳定后再开始任务
        case 0:
        {
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();

            // 等待刹车完成
            Position_ControlMode alt_mode, pos_mode;
            get_Altitude_ControlMode(&alt_mode);
            get_Position_ControlMode(&pos_mode);
            if (alt_mode == Position_ControlMode_Position && pos_mode == Position_ControlMode_Position)
            { // 刹车完成, 等待1秒
                ++navInf.counter2;
                if (navInf.counter2 >= 1 * freq)
                {
                    init_NavCmdInf(&navInf);
                    if (entry_resume_step >= 1 && entry_resume_step <= 6)
                    { // 断点续飞: 跳回保存的步骤
                        step = entry_resume_step;
                        entry_resume_step = 0;
                    }
                    else
                    {
                        // 已在空中则跳过起飞直接巡航(支持空中切入)
                        bool takeoff_inFlight;
                        get_is_inFlight(&takeoff_inFlight);
                        if (takeoff_inFlight)
                            step = 2;
                        else
                            step = 1;
                    }
                }
            }
            else
                navInf.counter2 = 0;
            break;
        }

        // 起飞: 相对当前高度爬升0.5米
        case 1:
        {
            double params[7];
            params[0] = 0; // pitch最小俯仰(多旋翼忽略)
            params[1] = 0;
            params[2] = 0;
            params[3] = nan(""); // 偏航: NaN=保持当前偏航
            params[4] = 0;       // 前向偏移0
            params[5] = 0;       // 左向偏移0
            params[6] = 0.5;     // 起飞高度: 0.5米
            NAV_CMD_RS res = Process_NavCmd(MAV_CMD_NAV_TAKEOFF, freq, MAV_FRAME_BODY_FLU, params, &navInf, rc, flag, comVars, cfg, avCfg);
            if (NavCmdRs_SuccessOrFault(res))
            { // 起飞完成
                init_NavCmdInf(&navInf);
                ++step;
            }
            break;
        }

        // 正方形航线第1边: 向前飞1米
        case 2:
        {
            double params[7];
            params[0] = 3; // 到达后停留3秒(到位判据)
            params[1] = 0;
            params[2] = 0;
            params[3] = 1000; // 偏航: 超过±360=不改变偏航
            params[4] = 1;    // 前向1米
            params[5] = 0;
            params[6] = 0; // 高度不变
            NAV_CMD_RS res = Process_NavCmd(MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf, rc, flag, comVars, cfg, avCfg);
            if (NavCmdRs_SuccessOrFault(res))
            { // 到达航点
                init_NavCmdInf(&navInf);
                ++step;
            }
            break;
        }

        // 正方形航线第2边: 向左飞1米
        case 3:
        {
            double params[7];
            params[0] = 3;
            params[1] = 0;
            params[2] = 0;
            params[3] = 1000;
            params[4] = 0;
            params[5] = 1; // 左向1米
            params[6] = 0;
            NAV_CMD_RS res = Process_NavCmd(MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf, rc, flag, comVars, cfg, avCfg);
            if (NavCmdRs_SuccessOrFault(res))
            {
                init_NavCmdInf(&navInf);
                ++step;
            }
            break;
        }

        // 正方形航线第3边: 向后飞1米
        case 4:
        {
            double params[7];
            params[0] = 3;
            params[1] = 0;
            params[2] = 0;
            params[3] = 1000;
            params[4] = -1; // 后向1米
            params[5] = 0;
            params[6] = 0;
            NAV_CMD_RS res = Process_NavCmd(MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf, rc, flag, comVars, cfg, avCfg);
            if (NavCmdRs_SuccessOrFault(res))
            {
                init_NavCmdInf(&navInf);
                ++step;
            }
            break;
        }

        // 正方形航线第4边: 向右飞1米, 回到起点
        case 5:
        {
            double params[7];
            params[0] = 3;
            params[1] = 0;
            params[2] = 0;
            params[3] = 1000;
            params[4] = 0;
            params[5] = -1; // 右向1米
            params[6] = 0;
            NAV_CMD_RS res = Process_NavCmd(MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf, rc, flag, comVars, cfg, avCfg);
            if (NavCmdRs_SuccessOrFault(res))
            { // 一圈完成
                init_NavCmdInf(&navInf);
                if (++loop_count >= AUTOMISSION_LOOPS)
                    ++step; // 圈数够了进入降落
                else
                    step = 2; // 继续绕圈
            }
            break;
        }

        // 自动降落: 原地垂直下降, 落地后上锁
        case 6:
        {
            Position_Control_set_XYLock();
            Position_Control_set_TargetVelocityZ(-getPosCtrlCfg()->LandVel[0]); // LandVel为配置的降落速度

            // 落地检测: inFlight为假并持续0.5秒认定已落地
            bool land_inFlight;
            get_is_inFlight(&land_inFlight);
            if (land_inFlight == false)
            {
                if (++land_counter >= 25)
                { // 任务全部完成: 清除断点并要求上锁(M32外壳自动执行)
                    finished = true;
                    s_saved_step = 0;
                    return FMR_RqDisarm_Land;
                }
            }
            else
                land_counter = 0;
            break;
        }

        default:
            step = 0;
            break;
    }
    /* ==================  开发者任务开发区结束  ==================== */

    return FMR_OK;
}
