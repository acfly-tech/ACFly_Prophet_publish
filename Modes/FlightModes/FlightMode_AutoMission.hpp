#pragma once

#include "FlightModes.hpp"

#include "NavCmdProcess.hpp"

// 任务绕圈数: 完成起飞后绕正方形飞行 AUTOMISSION_LOOPS 圈, 然后自动降落
#define AUTOMISSION_LOOPS 2

class FlightMode_AutoMission : public FlightMode_Base
{
  private:
    // 跨实例断点(显式切出模式后重进续飞, 0=无断点)
    static uint8_t s_saved_step;
    static uint8_t s_saved_loop;

    // 状态机步骤
    uint8_t step;
    // 任务中间信息
    NavCmdInf navInf;
    // 已完成圈数
    uint8_t loop_count;
    // 落地检测计数器
    uint16_t land_counter;
    // 挂起子态: 打杆手动接管或丢水平定位暂停任务
    bool suspended;
    // 恢复刹车中(摇杆回中/定位恢复后先刹车再续飞)
    bool recovering;
    // 任务已结束(完成或出错), 析构时不保存断点
    bool finished;
    // 构造时携带的断点步骤(刹车完成后跳回)
    uint8_t entry_resume_step;

  public:
    /**
     * @brief 二次开发自动任务模式
     */
    FlightMode_AutoMission();

    virtual ~FlightMode_AutoMission();

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
