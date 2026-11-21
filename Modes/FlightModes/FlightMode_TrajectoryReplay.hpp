#pragma once

#include "FlightModes.hpp"

#include "FlightModule_RecordTrajectory.hpp"
#include "NavCmdProcess.hpp"
#include "TD4.hpp"
#include "ff.h"

class FlightMode_TrajectoryReplay : public FlightMode_Base
{
  private:
    // 轨迹复演类型enum
    enum class REPLAY_TYPE
    {
        // 从记录起点开始
        FROM_START_POINT = 0,
        // 从当前点开始
        FROM_CURRENT_POINT,
        // 从当前点开始逐渐接近
        APPROACHING_POINT,
        // 从当前点和偏航开始
        FROM_CURRENT_POINT_YAW,
    };
    bool is_valid_replay_type(int val)
    {
        return val >= static_cast<int>(REPLAY_TYPE::FROM_START_POINT) && val <= static_cast<int>(REPLAY_TYPE::FROM_CURRENT_POINT_YAW);
    }

    // 反向复演调整起点经纬度
    bool calc_reversePoint(FlightModule_RecordTrajectory::REC_HEADER &header);

    // 轨迹复演类型
    REPLAY_TYPE replay_type;
    // 轨迹复演反向
    bool reverse;
    // 偏航反向
    bool yaw_reverse;
    // 偏航偏移
    double yaw_offset;
    // 轨迹复演记录编号
    uint8_t tra_ind;
    // APPROACHING_POINT接近速度cm/s
    double approaching_speed = 30;
    // 反向TD4求解器
    const static uint8_t REVERSE_SOLVER_M = 4;
    RealTimeTD4Inverter<REVERSE_SOLVER_M> reverse_solver[3];
    TD4_Lite acc_td4[3];
    // 未来点个数(会使用td4逆向求解器自动计算)
    uint8_t future_points_cnt;
    // 未来加速度数组
    vector3<double> *future_accs;
    // 逆向求解器当前时间累积
    double reverse_solver_cur_h[3];
    // 获取所有未来点
    bool get_all_future_points(const vector3<double> &current_tVel);
    // 获取未来时刻加速度
    vector3<double> calc_future_acc(double future_t);

    // 文件
    FIL fil;
    bool file_opened;
    // 运行时间间隔
    double rec_h;
    // 当前积分时间
    double cur_h;
    // 总点数
    uint32_t total_points;
    // 已运行的点数
    uint32_t checked_points;
    // 剩余的接近距离
    vector3<double> approaching_s;
    // 接近距离叠加值
    vector3<double> approaching_addon;

    // 任务状态
    NavCmdInf navInf;
    double nav_params[8];

    // 状态机
    uint8_t step;
    // 是否首个点
    bool first_point;
    // 手动跟随的期望位置(随手动操控变化)
    vector3<double> followPos;
    // 自动跟随的期望位置(随跟随传感器位置变化)
    vector3<double> followTPos;
    // 距离保持跟踪的上一次跟随传感器位置
    vector3<double> last_followTPos;
    // 自动跟随的期望速度(跟随传感器的速度)
    vector3<double> followTVel;
    // 自动跟随的期望加速度(跟随传感器的加速度)
    vector3<double> followTAcc;
    // 跟随原点(跟随传感器的原点)
    vector3<double> followOrigin;
    // 跟随yaw
    double followYaw;
    double followYawRate;
    // yaw偏移
    double followYawOffset;

  public:
    /**
     * @brief 轨迹复演模式
     */
    FlightMode_TrajectoryReplay();

    virtual ~FlightMode_TrajectoryReplay();

    // 获取飞行模式类别
    virtual FlightModeClass get_fmClass();

    /**
     * @brief 设置飞行模式参数
     * @param params[0] 轨迹复演记录编号
     * @param params[1] 轨迹复演类型REPLAY_TYPE
     * @param params[2] 1=轨迹复演反向
     * @param params_cnt 必须为3
     */
    virtual bool set_params(double params[], uint16_t params_cnt);

    // 模式进入检查
    virtual FlightModeCheckResult check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg);

    // 模式主函数
    virtual FlightModeResult main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                       const AvoidanceCfg *avCfg, double h);
};