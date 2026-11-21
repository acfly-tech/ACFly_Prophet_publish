#pragma once

#include "FlightModes.hpp"

#include "Missions.hpp"
#include "NavCmdProcess.hpp"

class FlightMode_Mission : public FlightMode_Base
{
  private:
    // 航点类型enum
    enum class WP_TYPE
    {
        Normal,
        Terrain,
    };
    enum WP_TYPE_FLAG
    {
        WP_TYPE_FLAG_HOLD = (1 << 0),
    };

    // 状态机
    uint8_t step;
    uint8_t ms_step;
    // 任务状态
    NavCmdInf navInf;
    // 当前任务
    uint16_t current_mission_ind;
    MissionInf current_mission_inf;
    // 恢复任务是否需要下降
    bool wpRNeedDecent;
    // 航点跳转
    uint16_t doJumpCount;
    uint16_t doJumpWP;
    // 航线是否已预存
    bool prestored;
    // 当前任务类型
    WP_TYPE wp_type;
    // 任务等待退出
    bool wait_quit;
    // 连续任务飞行距离
    double traveledDist;
    // Mission report hold counter (holds SUCCESS/FAILURE for ~1.5s so it gets sent)
    uint16_t report_hold_counter = 0;

    // 障碍物距离滤波值
    double avDistance_filted;
    // 无障碍物超时计数
    uint16_t avDistance_timeout_counter = 0;

    // 障碍物距离滤波系数
    const double avDistance_filter_P = 0.5;
    // 无障碍物超时时间
    const double avDistance_timeout_T = 3.0;

    // 获取当前任务类型
    bool get_wp_type(const MissionInf &wp, WP_TYPE *type, WP_TYPE_FLAG *flag);
    // 获取下一航点
    bool get_next_wp(MissionInf *next_wp, uint16_t *next_wp_ind);
    // 运行至下一航点
    bool run_to_next_wp();

    // 发送任务状态报告
    void send_mission_report();

  public:
    /**
     * @brief 起飞模式
     */
    FlightMode_Mission();

    virtual ~FlightMode_Mission();

    // 获取飞行模式类别
    virtual FlightModeClass get_fmClass();

    /**
     * @brief 设置飞行模式参数
     */
    virtual bool set_params(double params[], uint16_t params_cnt);

    // 模式进入检查
    virtual FlightModeCheckResult check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg);

    // 模式主函数
    virtual FlightModeResult main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                       const AvoidanceCfg *avCfg, double h);
};