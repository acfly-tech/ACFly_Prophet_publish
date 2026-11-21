#pragma once

#include "Avoidance.hpp"
#include "Basic.hpp"
#include "Modes.hpp"
#include "Receiver.hpp"
#include <functional>
#include <map>
#include <memory>

// 飞行模式工厂
class FlightMode_Base;
// class FlightModeFactory
//{
//   public:
//     // 使用函数指针来创建对象
//     using Creator = std::function<std::unique_ptr<FlightMode_Base>()>;

//    // 注册子类
//    template <typename T> static void registerMode(int id)
//    {
//        creators()[id] = []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<T>();
//        };
//    }

//    // 根据序号创建对象
//    static std::unique_ptr<FlightMode_Base> createMode(int id)
//    {
//        auto it = creators().find(id);
//        if (it != creators().end())
//        {
//            return it->second(); // 调用对应的创建函数
//        }
//        return nullptr; // 或者抛出异常，表示无效的ID
//    }

//  private:
//    // 使用静态map存储ID到创建函数的映射
//    static std::map<int, Creator> & creators()
//    {
//        static std::map<int, Creator> creatorMap;
//        return creatorMap;
//    }
//};
#define FLIGHT_MODES_CNT 256
class FlightModeFactory
{
  public:
    // 使用函数指针来创建对象
    using Creator = std::function<std::unique_ptr<FlightMode_Base>()>;

    // 注册子类
    // template <typename T> static void registerMode(int id)
    //{
    //    creators()[id] = []() -> std::unique_ptr<FlightMode_Base> { return std::make_unique<T>();
    //    };
    //}

    // 根据序号创建对象
    static std::unique_ptr<FlightMode_Base> createMode(int id)
    {
        uint16_t size;
        const Creator *creatorsList = creators(&size);
        if (id >= 0 && id < size && creatorsList[id] != nullptr)
            return creatorsList[id]();
        else
            return nullptr;
    }

  private:
    // 使用静态map存储ID到创建函数的映射
    static const Creator *creators(uint16_t *size);
};

// 飞行模式类别
enum FlightModeClass
{
    // 手动控制模式
    FMC_Manual,
    // 半自动模式
    FMC_SemiAutomatic,
    // 全自动模式
    FMC_Automatic,
};

// 飞行模式使能标志
enum FlightModeFlag : uint32_t
{
    // 仿地开关
    FMF_TerrainFollowingEna = (1 << 0),
    // 无头开关
    FMF_HeadlessEna = (1 << 1),
    // 避障开关
    FMF_AvoidanceEna = (1 << 2),
};
#define FMF_RecordTrajectory_Mask ((1 << 15) | (1 << 14) | (1 << 13) | (1 << 12))

// 飞行模式运行结果
enum FlightModeResult
{
    // 正常且可以继续运行
    FMR_OK = 0,
    // 运行完成 需要退出到别的模式
    FMR_OK_RqSwitchMode,
    // 运行完成 需要退出返航模式
    FMR_OK_RqSwitchRTL,
    // 运行完成 需要退出到降落模式
    FMR_OK_RqSwitchLand,

    // 要求锁定-已落地
    FMR_RqDisarm_Land,

    // 无定位 需要退出到别的模式
    FMR_Err_NoPositioning,
    // 无任务 需要退出到别的模式
    FMR_Err_NoMission,
    // 运行失败 需要退出返航模式
    FMR_Err_RqSwitchRTL,
    // 运行失败 需要退出到降落模式
    FMR_Err_RqSwitchLand,

    // 失败参数无效
    FMR_Err_ParamInvalid,
    // 无返航点失败
    FMR_Err_NoHomePoint,

    // 运行完成 切换到跟踪模式
    FMR_OK_RqSwitchLoiter,
    // 运行完成 切换到定点模式
    FMR_OK_RqSwitchPosHold,
};

// 根据围栏动作参数返回需要切换的模式
//  动作: 0-悬停 1-返航 2-降落
//  手动飞行动作(位0-7): 悬停返回FMR_OK(保持当前模式悬停刹车)
//  任务飞行动作(位8-15): 悬停返回FMR_OK_RqSwitchPosHold(切定点模式悬停)
struct FenceActionInf
{
    // 上报动作(FENCE_MITIGATE_*)
    uint8_t mitigation;
    // 模式切换结果(FlightModeResult)
    FlightModeResult modeRs;
};
// 解析围栏动作参数
//  fenceAction: 围栏动作参数
//  mission: true-任务飞行动作(位8-15) false-手动飞行动作(位0-7)
FenceActionInf get_FenceActionInf(uint32_t fenceAction, bool mission);

// 飞行进入检查结果
enum FlightModeCheckResult
{
    // 正常检查正常可以继续运行
    FMCR_OK = 0,

    // 无定位不能进入
    FMCR_Err_NoPositioning,
    // 已起飞不能进入
    FMCR_Err_TakenOff,
    // 未起飞不能进入
    FMCR_Err_NotTakeOff,

    // 失败参数无效
    FMCR_Err_ParamInvalid,

    /* 警告级检查结果(>=100): 模式可运行但功能降级
     * 飞行中切换照常放行; 解锁时由M32报警并拒绝解锁 */
    // 无水平定位 (如PosHold无定位时将降级为定高)
    FMCR_Warn_NoPositioning = 100,
};

// 检查结果是否允许进入/切换模式 (OK或警告级)
inline bool FMCR_Allowed(FlightModeCheckResult r)
{
    return r == FMCR_OK || r >= FMCR_Warn_NoPositioning;
}

// ModeMsg处理结果
enum FlightModeMsgResult
{
    // 未处理
    FMMR_Unprocessed = 0,

    // 返回成功
    FMMR_Success,
    // 返回失败
    FMMR_Failed,
};

// 飞行模式通用变量
class FlightMode_CommonVars
{
  public:
    // 无rc持续时间(应在主模式更新)
    TIME noRcStartTIME;
    void resetVars_rc()
    {
        noRcStartTIME.set_invalid();
    }

    // 起飞状态(应在主模式更新)
    bool taken_off;
    void resetVars_takeoff()
    {
        taken_off = false;
    }

    // 无头模式
    bool headless_inited;
    double headless_yaw;
    void resetVars_headless()
    {
        headless_inited = false;
    }
    void set_headlessYaw(double yaw)
    {
        headless_yaw = yaw;
        headless_inited = true;
    }

    // 重置所有变量
    void resetVars()
    {
        resetVars_rc();
        resetVars_headless();
        resetVars_takeoff();
    }
    FlightMode_CommonVars()
    {
        resetVars();
    }
};

class FlightMode_Base
{
  private:
    // 模式id
    uint8_t mode_id;
    // 模式名称
    SName name;

  protected:
    // 功能代码
    uint16_t function_code;

    // 通用遥控器控制--高度-速度方式
    void rcCtrl_Alt_v(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg,
                      double h);

    // 通用遥控器控制--高度-位置闭环
    void rcCtrl_Alt_p(double thr_sitck_d, double &tPos_z, double &tVel_z, double followTVel_z, const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg,
                      FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg, double h);

    // 通用遥控器控制--水平位置-位置闭环
    void rcCtrl_Pos_p(double roll_sitck_d, double pitch_sitck_d, double &tPos_x, double &tPos_y, double &tVel_x, double &tVel_y, double followTVel_x, double followTVel_y, const Receiver &rc,
                      FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg, double h);

  public:
    FlightMode_Base(SName name, uint8_t mode_id, uint16_t function_code)
    {
        this->name = name;
        this->mode_id = mode_id;
        this->function_code = function_code;

        this->force_switch = false;
        // FlightModeFactory.registerMode(this, mode_id);
    }

    // 需要强制切换模式
    bool force_switch;

    virtual ~FlightMode_Base() = default;

    // 获取模式id
    uint8_t get_modeId()
    {
        return this->mode_id;
    }

    // 获取飞行模式类别
    virtual FlightModeClass get_fmClass() = 0;

    // 获取功能代码
    uint16_t get_functionCode() const
    {
        return function_code;
    }

    // 设置功能代码
    virtual bool set_functionCode(uint16_t function_code)
    {
        this->function_code = function_code;
        return true;
    }

    // 设置飞行模式参数
    virtual bool set_params(double params[], uint16_t params_cnt) = 0;

    // 模式进入检查
    virtual FlightModeCheckResult check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg) = 0;

    // 模式主函数
    virtual FlightModeResult main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs, const ModeFuncCfg *cfg,
                                       const AvoidanceCfg *avCfg, double h) = 0;
};

// 全局模式开关标志
extern FlightModeFlag GLB_FlightModeFlags;
