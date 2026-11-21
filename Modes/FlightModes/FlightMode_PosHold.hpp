#pragma once

#include "FlightModes.hpp"

#include "NavCmdProcess.hpp"

class FlightMode_PosHold : public FlightMode_Base
{
  private:
    // 避障锁定
    bool avLocked;
    // 围栏锁定
    bool fenceLocked;
    // 仿地高度
    double tf_height;
    // 是否在执行cmd
    bool cmdInProcess;
    NavCmdInf navInf;
    ModeMsg cmdMsg;
    // 退出模式计数器
    uint16_t exit_counter1;
    uint16_t exit_counter2;
    bool thr_raised;

  public:
    /**
     * @brief 位置控制模式
     * @param function_code 3=兼容定点+定高 2=单独定点 1=单独定高
     */
    FlightMode_PosHold(uint16_t function_code);

    virtual ~FlightMode_PosHold();

    // 获取飞行模式类别
    virtual FlightModeClass get_fmClass();

    // 设置飞行模式参数
    virtual bool set_params(double params[], uint16_t params_cnt);

    // 模式进入检查
    virtual FlightModeCheckResult check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg);

    // 模式主函数
    virtual FlightModeResult main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                       const AvoidanceCfg *avCfg, double h);
};