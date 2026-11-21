#pragma once

#include "FlightModes.hpp"

#include "NavCmdProcess.hpp"

class FlightMode_Manual : public FlightMode_Base
{
  private:
    // 初始油门滤波值
    double mthr_thrFilted = -1;
    // 初始油门计数器
    int32_t mthr_prepCounter = 0;
    // 退出模式计数器
    uint16_t exit_counter1;
    uint16_t exit_counter2;

    // 初始油门滤波器系数
    const double mthr_filter_P = 1.0;

  public:
    /**
     * @brief 手动控制模式
     * @param function_code 1=手动 2=增稳 3=半自稳
     */
    FlightMode_Manual(uint16_t function_code);

    virtual ~FlightMode_Manual();

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