#pragma once

#include "FlightModes.hpp"

#include "NavCmdProcess.hpp"

class FlightMode_TakeOff : public FlightMode_Base
{
  private:
    // 跟随类型enum
    enum class FOLLOW_TYPE
    {
        NONE,
        POINT,
        TRACK,
    };
    // 跟随偏航类型enum
    enum class FOLLOW_YAW_TYPE
    {
        // 不跟随
        NONE,
        // 完全跟随传感器
        FOLLOW,
        // 跟随传感器但可进行offset调整
        FOLLOW_OFFSET,
    };

    // 起飞参数是否已设置
    bool paramSet;
    // 相对高度起飞
    bool relAlt;
    // 高度cm
    double height;

    // 状态机
    uint8_t step;
    // 上次跟随传感器序号
    int32_t lastFollowSInd = -1;
    // 跟随传感器类型
    FOLLOW_TYPE followType;
    // 手动跟随的期望位置(随手动操控变化)
    vector3<double> followPos;
    // 自动跟随的期望位置(随跟随传感器位置变化)
    vector3<double> followTPos;
    // 自动跟随的期望速度(跟随传感器的速度)
    vector3<double> followTVel;
    // 跟随原点(跟随传感器的原点)
    vector3<double> followOrigin;
    // 距离保持跟踪的上一次跟随传感器位置
    vector3<double> last_followTPos;
    // 距离保持跟踪的上一次期望位置
    vector3<double> last_followTPos2;
    // 偏航跟随模式
    FOLLOW_YAW_TYPE followYawType;
    FOLLOW_YAW_TYPE last_followYawType;
    // 跟随yaw
    double followYaw;
    double followYawRate;
    // yaw偏移
    double followYawOffset;

  public:
    /**
     * @brief 起飞模式
     */
    FlightMode_TakeOff();

    virtual ~FlightMode_TakeOff();

    // 获取飞行模式类别
    virtual FlightModeClass get_fmClass();

    /**
     * @brief 设置飞行模式参数
     * @param params[0] 1=海拔高 2=相对高
     * @param params[1] 高度cm
     * @param params_cnt 必须为2
     */
    virtual bool set_params(double params[], uint16_t params_cnt);

    // 模式进入检查
    virtual FlightModeCheckResult check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg);

    // 模式主函数
    virtual FlightModeResult main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                       const AvoidanceCfg *avCfg, double h);
};