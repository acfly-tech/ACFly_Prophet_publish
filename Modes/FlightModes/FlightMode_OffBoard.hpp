#pragma once

#include "FlightModes.hpp"

#include "NavCmdProcess.hpp"
#include "timeBase.hpp"

class FlightMode_OffBoard : public FlightMode_Base
{
  private:
    TIME Offboard_ThrTIME;
    TIME Offboard_YawTIME;
    TIME Offboard_PitRolTIME;
    TIME Offboard_ManualTIME;

    uint16_t exit_counter1;
    uint16_t exit_counter2;
    bool thr_raised;

    // 任务状态
    NavCmdInf navInf;

    // 指令执行是否完成（手动模式）
    bool cmdInProcess = false;
    ModeMsg cmdMsg;

    // SLAM传感器序列号
    uint8_t Slam_Index = 11;

  public:
    /**
     * @brief 起飞模式
     */
    FlightMode_OffBoard();

    virtual ~FlightMode_OffBoard();

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