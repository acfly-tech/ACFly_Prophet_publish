#pragma once

#include "FlightModes.hpp"

#include "NavCmdProcess.hpp"
#include "precLand.hpp"

class FlightMode_RTL : public FlightMode_Base
{
  private:
    // 起飞参数是否已设置
    uint8_t paramSet;
    // 返航点经纬度参数
    double rtl_lat;
    double rtl_lon;
    // 返航点地面相对高参数
    double rtl_ground_alt;
    // 返航高度相对高参数
    double rtl_height;

    // 安全模式参数
    MSafeCfg safeCfg;
    // 计数器1
    uint16_t counter1;

    // 返回的是否起飞点(会采样起飞点传感器进行高度判断)
    bool rtl_to_startPoint;
    // 返航位置local高度
    double rtl_pos_z;

    // 跟随返航步骤
    uint8_t followRtl_step;
    // 跟随返航当前期望速度
    double followRTL_currentVel;
    // 当前跟随传感器序号
    uint16_t current_followSInd;
    // 手动跟随的期望位置(随手动操控变化)
    vector3<double> followPos;
    // 自动跟随的期望位置(随跟随传感器位置变化)
    vector3<double> followTPos;
    // 自动跟随的期望速度(跟随传感器的速度)
    vector3<double> followTVel;
    // 跟随yaw
    bool followYawAvailable;
    double followYaw;
    double followYawRate;
    // 跟随yaw偏移
    double followYawOffset;
    // 跟随yaw偏移设定标志
    bool followRtl_yawSetup;
    // 跟随yaw偏移设定标志
    TIME precRtl_yawSetup;

    // 精准降落参数
    PrecLandInf precInf;
    // 精准降落末段进入标志
    bool precCtrl_TerminalLanding;
    // 精准降落传感器基准的高度偏移(=local高度 - 传感器高度)
    double precSensor_heightOffset;
    // 上次精准降落传感器更新时间
    TIME last_precSensor_updTIME;
    // 精准降落传感器是否已经修正偏航
    bool precSensor_yawFixed;
    // 精准降落控制积分
    double precCtrl_int_x;
    double precCtrl_int_y;
    double precCtrl_int2_x;
    double precCtrl_int2_y;
    // 精准降落控制期望位置
    double precCtrl_target_x;
    double precCtrl_target_y;
    // 精准降落控制理论目标点位置
    double precCtrl_target_px;
    double precCtrl_target_py;
    // 精准降落上升重试步骤
    uint8_t precRetry_step;
    // 精准降落上升重试计数器
    uint8_t precRetry_counter;

    // 备降点类型
    enum class AL_TYPE
    {
        NONE,
        GLOBAL,
        RELATIVE,
    };
    AL_TYPE al_type;
    // 备降点位置
    double AL_lat;
    double AL_lon;
    double AL_alt;

    // 退出模式计数器
    uint16_t exit_counter1;
    uint16_t exit_counter2;
    bool thr_raised;

    // 进入返航航线时记录的原因(直接降落实例保持NONE)
    // 降落段继承此原因上报: 围栏-106, 失控-6, 低电压-2,
    // App/遥控请求-NONE(降落段补报RTL_HOME_LAND=12)
    uint16_t rtl_entry_reason;

    // 模式类别
    FlightModeClass fmc;

    // 状态机
    uint8_t step;

    /**
     * @brief 初始化降落信息
     * @param to_startPoint 返回的是否起飞点
     * @param pos_z 降落点地面的local z
     */
    void init_landInf(bool to_startPoint, double pos_z);
    // 初始化跟随降落信息
    void init_followLandInf();
    // 获取备降点
    AL_TYPE get_ALInfo();

    /**
     * @brief 计算分级降落速度(cm/s 正值)
     * @param height 当前对地高度cm
     * @param maxAutoVelDown 一级降落速度上限cm/s(非正值或超限时使用此值)
     * 高于RtLand1Height按RtLand1Vel下降(非正值用maxAutoVelDown)
     * 低于RtLand2Height按LandVel下降, 两级高度间线性过渡
     */
    double getLandVel(double height, double maxAutoVelDown) const;

    // 根据function_code获取模式
    static constexpr AFunc mapFunctionCodeToAFunc(uint16_t function_code)
    {
        switch (function_code)
        {
            case 1:
                return AFunc_Land;
            case 2:
                return AFunc_RTL;
            case 3:
                return AFunc_RTL2Point;
        }
        return AFunc_RTL;
    }

  public:
    /**
     * @brief 返航模式
     * @param function_code 1=仅降落 2=返航到起飞点 3=起飞并返航到指定点
     */
    FlightMode_RTL(uint16_t function_code);

    virtual bool set_functionCode(uint16_t function_code);

    virtual ~FlightMode_RTL();

    // 获取飞行模式类别
    virtual FlightModeClass get_fmClass();

    /**
     * @brief 设置飞行模式参数
     * @brief funcition_code == 1
     * @param params[0] 返航点地面相对起飞点高度cm
     * @param params_cnt 必须为1
     * @brief funcition_code == 3
     * @param params[0] 返航lat
     * @param params[1] 返航lon
     * @param params[2] 返航点地面相对起飞点高度cm
     * @param params[3] 返航飞行高度cm(相对起飞点高)
     * @param params_cnt 必须为4
     */
    virtual bool set_params(double params[], uint16_t params_cnt);

    // 模式进入检查
    virtual FlightModeCheckResult check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg);

    // 模式主函数
    virtual FlightModeResult main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                       const AvoidanceCfg *avCfg, double h);
};