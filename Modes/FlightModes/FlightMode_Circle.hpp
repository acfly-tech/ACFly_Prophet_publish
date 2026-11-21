#pragma once

#include "FlightModes.hpp"

#include "NavCmdProcess.hpp"

class FlightMode_Circle : public FlightMode_Base
{
  private:
    // 起飞参数是否已设置
    bool paramSet;
    // 坐标系
    MAV_FRAME frame;
    // 中心点坐标
    vector3<double> rO;
    vector2<double> origin;
    // 半径
    double r;
    // 环绕偏航角
    double rYaw;
    // 环绕速度
    double rSp;
    // 初始速度滤波
    double sp_filted;
    // 退出模式计数器
    uint16_t exit_counter1;
    uint16_t exit_counter2;
    bool thr_raised;

    // 状态机
    uint8_t step;

  public:
    /**
     * @brief 起飞模式
     */
    FlightMode_Circle();

    virtual ~FlightMode_Circle();

    // 获取飞行模式类别
    virtual FlightModeClass get_fmClass();

    /**
     * @brief 设置飞行模式参数
     * @param params[0] 坐标系
     * @param params[1] x
     * @param params[2] y
     * @param params[3] z
     * @param params[4] 偏航角
     * @param params[5] 速度
     * @param params[6] 半径
     * @param params_cnt 必须为7
     */
    virtual bool set_params(double params[], uint16_t params_cnt);

    // 模式进入检查
    virtual FlightModeCheckResult check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg);

    // 模式主函数
    virtual FlightModeResult main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                       const AvoidanceCfg *avCfg, double h);
};