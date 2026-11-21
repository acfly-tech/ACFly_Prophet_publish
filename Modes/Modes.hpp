#pragma once

#include "Basic.hpp"
#include "Receiver.hpp"
#include "mavlink.h"

// 模式收发消息
#define CMD_TYPE_MAVLINK (1 << 4)
// 遥控Aux功能消息（一键起降返航等）
#define CMD_TYPE_RC_AUX (1 << 5)
// 外部设备消息（机巢、集群等）
#define CMD_TYPE_EXTERNAL_DEV (1 << 6)
#define CMD_TYPE_MASK 0xf0
#define CMD_TYPE_PORT_MASK 0x0f

/* acfly内部ModeMsg cmd号: 采用 0x8000+mavlink msgid 编码(bit15置位表示内部合成命令)。
 * 与MAVLink命令号空间(uint16, 相机/云台已用到2505)彻底隔离,
 * 不会再被InFlightCmdProcess误当作相机/云台命令消费。
 * 新增内部ModeMsg cmd必须使用本编码!
 *  ACFLY_CMD_SET_POSITION_TARGET_LOCAL_NED  = 0x8000+84 (mavlink msg84)
 *  ACFLY_CMD_SET_POSITION_TARGET_GLOBAL_INT = 0x8000+86 (mavlink msg86)
 */
#define ACFLY_INTERNAL_CMD_BIT 0x8000
#define ACFLY_CMD_SET_POSITION_TARGET_LOCAL_NED (ACFLY_INTERNAL_CMD_BIT | 84)
#define ACFLY_CMD_SET_POSITION_TARGET_GLOBAL_INT (ACFLY_INTERNAL_CMD_BIT | 86)

struct ModeMsg
{
    // 高4位：1-mavlink消息
    // 低4位：接收port
    uint8_t cmd_type;
    uint8_t sd_sysid;
    uint8_t sd_compid;
    uint8_t frame;
    uint32_t cmd;
    double params[16];
};
bool ModeReceiveMsg(ModeMsg *msg, double TIMEOUT);
bool SendMsgToMode(ModeMsg msg, double TIMEOUT);

void ModeMsgAck(const ModeMsg &msg, bool accepted);
// 仅由 Modes 任务在模式入口检查完成后调用，不等待整个飞行主循环返回
void ModeEntryAck(bool accepted);

// 模式功能选项
struct ModeFuncCfg
{
    // PA-按钮1解锁前功能（模式序号）
    uint8_t Bt1PAFunc[6][8];

    // A-按钮1解锁后功能（1-定高模式 2-位置模式 3-运动模式 22-任务模式 23-返航模式）
    uint8_t Bt1AFunc[6][8];

    // 按钮2-4功能
    // 0-1000: 切换到指定模式
    // 1000: 切换无头状态
    // 1001: 切换避障状态
    // 1002: 切换仿地状态
    uint32_t Bt234Func[3][2];

    // 安全按钮
    //  0：无
    //  2-4：对应按钮按下(100%)强制锁定电机
    //  10：油门最下偏航最做强制锁定电机
    uint8_t SafeBt[8];

    // 中位死区
    float NeutralZone[2];
    // 位置速度响应曲线系数
    float PosVelAlpha[2];
    // 姿态角速率响应曲线系数
    float RolRateE[2];
    float PitRateE[2];
    float YawRateE[2];
    // 半自稳模式摇杆区域
    float ratitudeZone[2];

    // 失控返航时间
    float nRcT[2];

    // 配置选项开关
    //  0-解锁后复位到0号航点
    //  1-航线过程失控返航
    //  2-自动模式禁止摇杆操作
    //  3-摇杆操作退出自动模式内3秒自动恢复自动模式(防止误触)

    // 8-航向异常禁止解锁
    // 9-无定位禁止解锁
    // 10-电压不正常允许解锁
    // 11-位置解算异常允许解锁
    // 12-SD卡容量不足不允许解锁
    // 13-无避障传感器允许在避障模式解锁

    // 24-无fix状态不允许执行航线
    // 27-返航降落阶段全自动模式
    uint32_t configs[2];

    // 超声波定高(Trig/Echo)启用开关: 0=关闭 1=启用
    uint8_t USonicEn[8];

} __ALIGNED_PACKED;
#define is_AFunc_auto(x) (x >= 20)
#define is_AFunc_NoPos(x) (x == 1 || x == 6 || x == 7 || x == 8)

// 解锁复位航线到航点0
#define MCfg_RstWp0_Bit (1 << 0)
// 自动模式失控返航
#define MCfg_WpNRcRTL_Bit (1 << 1)
// 自动模式禁止摇杆干预
#define MCfg_NoRcCtrlInAuto_Bit (1 << 2)
// 摇杆干预后回中需要恢复到原自动模式
#define MCfg_RcvAuto_Bit (1 << 3)
// 手动模式失控悬停
#define MCfg_PosNRcHold_Bit (1 << 6)
// 失控后降落
#define MCfg_NRcLand_Bit (1 << 7)

// 航向异常禁止解锁
#define MCfg_NoUnlockHeadingAbnormal_Bit (1 << 8)
// 无定位禁止解锁
#define MCfg_NoUnlockNoPositioning_Bit (1 << 9)
// 电压异常允许解锁
#define MCfg_CanUnlockVoltageAbnormal_Bit (1 << 10)
// 位置解算异常允许解锁
#define MCfg_CanUnlockPositioningAbnormal_Bit (1 << 11)
// SD卡异常禁止解锁
#define MCfg_NoUnlockSdInsufficient_Bit (1 << 12)
// 无避障传感器允许解锁
#define MCfg_CanUnlockWithouAVSYS_Bit (1 << 13)
// 无Fixed禁止任务
#define MCfg_NoAutoWithouFixed_Bit (1 << 24)

// 返航降落阶段全自动模式
#define MCfg_RtlLandingAuto_Bit (1 << 27)

// 固定解才允许解锁起飞
#define MCfg_NoUnlockWithouFixed_Bit (1 << 25)

// 上云API mode_code_reason (飞行器进入当前状态的原因)
// 只实现飞控完全支持的原因项
// 方言(mavlink.h)已定义同名枚举时使用方言定义，本地定义仅作兜底
#ifndef HAVE_ENUM_MODE_CODE_REASON
enum MODE_CODE_REASON
{
    // 无意义
    MODE_CODE_REASON_NONE = 0,
    // 电压不足（返航、降落）
    MODE_CODE_REASON_LVOLT_RTL = 2,
    // 电压严重过低（降落）
    MODE_CODE_REASON_LVOLT_LAND = 3,
    // 遥控器按键请求（起飞、返航、降落）
    MODE_CODE_REASON_RC_BUTTON = 4,
    // App请求（起飞、返航、降落）
    MODE_CODE_REASON_APP = 5,
    // 遥控信号丢失（返航、降落、悬停）
    MODE_CODE_REASON_RC_LOST = 6,
    // 外部设备触发（机巢、集群等）
    MODE_CODE_REASON_EXTERNAL = 7,
    // 执行航点任务时请求（起飞）
    MODE_CODE_REASON_MISSION_TAKEOFF = 11,
    // 返航阶段到达Home点上方后请求（降落）
    MODE_CODE_REASON_RTL_HOME_LAND = 12,
    // 智能低电量返航后进入（降落）
    MODE_CODE_REASON_LVOLT_RTL_LAND = 17,
    // 返航取消 (悬停)
    MODE_CODE_REASON_RTL_CANCELED = 21,
    // 自定义扩展原因 (100+, 厂商私有)
    // 姿态解锁系统未初始化完成 (待机)
    MODE_CODE_REASON_ATTITUDE_NOT_READY = 100,
    // 高度解算系统尚未初始化成功 (待机)
    MODE_CODE_REASON_ALTITUDE_NOT_READY = 101,
    // 位置解算系统未初始化成功, 无定位或定位精度不够 (待机/起飞准备)
    MODE_CODE_REASON_NO_POSITION = 102,
    // 定位尚未固定解 (起飞准备)
    MODE_CODE_REASON_GPS_NOT_FIXED = 103,
    // 测向尚未固定解 (起飞准备)
    MODE_CODE_REASON_DAO_NOT_FIXED = 104,
    // 用户打杆接管, 中断自动任务/降落 (手动飞行)
    MODE_CODE_REASON_RC_TAKEOVER = 105,
    // 触发围栏 (返航/降落/悬停)
    MODE_CODE_REASON_FENCE = 106,
    // 航线任务中的自动返航/降落脚点 (返航/降落)
    MODE_CODE_REASON_MISSION_NAV = 107,
};
#endif

// 飞行模式定义
enum AFunc
{
    AFunc_AltHold = 1,
    AFunc_PosHold = 2,
    AFunc_PosHoldOrbit = 4, // 定点执行NavCmd中（Orbit子模式）
    // AFunc_PosHoldAv = 11,
    // AFunc_PosHoldNH = 12, // 无头模式
    // AFunc_PosHoldNHAv = 13,
    AFunc_Loiter = 3,
    AFunc_Circle = 5,
    AFunc_Manual = 6,
    AFunc_Stabilize = 7,
    AFunc_Rattitude = 8,
    AFunc_Offboard = 10,
    AFunc_OffboardOrbit = 9, // Offboard执行NavCmd中（Orbit子模式）
    AFunc_Follow = 16,
    AFunc_TrajectoryReplay = 18,

    AFunc_TakeOff = 20,
    AFunc_Mission = 22,
    AFunc_RTL = 23,
    AFunc_Land = 24,
    AFunc_VertMission = 25,
    AFunc_WallHug = 26,
    AFunc_Strike = 27,
    AFunc_AutoMission = 35, // 二次开发自动任务模式
    AFunc_RTL2Point = 36,
};
// 模式状态存储
bool setCurrentFlyMode(AFunc cMode, uint16_t reason = MODE_CODE_REASON_NONE);
AFunc getCurrentFlyMode();
// 获取进入当前飞行模式的原因 (上云API mode_code_reason)
uint16_t getCurrentFlyModeReason();
// 设置挂起的模式切换原因 (M32主循环在模式生效前设置)
void setPendingFlyModeReason(uint16_t reason);
// 判断是否有GPS固定解 (任意GPS传感器fixtype==6)
bool isGPSFixedSolution();

// 获取遥控按钮对应的区域（0-5）
inline int8_t get_RcButtonZone(double rc, uint8_t current_zone)
{
    double st = 1.0 / 6 * 100;
    if (current_zone <= 5 && rc >= current_zone * st - 1 && rc < (current_zone + 1) * st + 1)
        return current_zone;

    if (rc < 1 * st)
        return 0;
    else if (rc < 2 * st)
        return 1;
    else if (rc < 3 * st)
        return 2;
    else if (rc < 4 * st)
        return 3;
    else if (rc < 5 * st)
        return 4;
    else
        return 5;
}

class Mode_Base;
void ModeRegister(Mode_Base *mode, uint8_t id);

enum ModeResult
{
    MR_OK = 0,
    MR_Err,
};

class Mode_Base
{
  private:
  public:
    SName name;
    Mode_Base(SName name, uint8_t mode_id)
    {
        this->name = name;
        ModeRegister(this, mode_id);
    }
    // 模式主函数
    virtual ModeResult main_func(void *param1, uint32_t param2) = 0;
    // 获取模式Mavlink模式标号
    virtual void get_MavlinkMode(ModeFuncCfg cfg, Receiver rc, uint8_t btn_zones[4], AFunc *mode)
    {
        *mode = AFunc_Stabilize;
        //			*mav_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        //			*mav_main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
        //			*mav_sub_mode = 0;
    }
};

// 当前航点记录（用于断点续飞）
struct RestoreWpInf
{
    // 当前航点序号
    int32_t CurrentWp[2];
    // 航线向量AB A-目标 B起点
    double line_x;
    double line_y;
    double line_z;
    // 航线完成距离
    double line_fs;
    // 定距拍照距离
    double CamTrigDist;
    // 飞行速度
    double vel;
    // 航点半径
    double wpR;
} __ALIGNED_PACKED;
static inline void reset_RestoreWpInf(RestoreWpInf &inf)
{
    inf.CurrentWp[0] = -1;
    inf.line_x = 0;
    inf.line_y = 0;
    inf.line_z = 0;
    inf.line_fs = -1;
    inf.CamTrigDist = 0;
    inf.vel = 0;
    inf.wpR = -1;
}

// Current mission item execution report (written by Modes thread, read by Commulink thread)
struct MissionItemReport
{
    // Current mission item sequence number
    uint16_t seq;
    // Current mission item command ID
    uint16_t cmd;
    // Total number of mission items
    uint16_t mission_count;
    // Current mission item progress, 0 to 100
    uint8_t progress;
    // Mission execution state (ACFLY_MISSION_REPORT_RESULT)
    uint8_t result;
};
// Thread-safe read: copies missionItemReport under mutex protection
MissionItemReport get_missionItemReport(double TIMEOUT = -1);
// Thread-safe write: assigns missionItemReport under mutex protection
void set_missionItemReport(const MissionItemReport &report);
// 航点恢复变量(非线程安全 仅限Modes线程使用)
extern RestoreWpInf restoreWpInf;
// 设置需要保存航点恢复变量到存储器
void set_RestoreWpInf_needStore();

// 读取航点恢复信息至restoreWpInf
bool read_restoreWpInf(double TIMEOUT = -1);
// 存储restoreWpInf至RAM
bool storeRAM_restoreWpInf(double TIMEOUT = -1);
// 存储restoreWpInf至存储器
bool store_restoreWpInf(double TIMEOUT = -1);

// 飞行统计信息（存储在"Init"参数组偏移4处）
#define FLIGHTSTATS_PARAM_OFFSET 4
#define FLIGHTSTATS_PARAM_COUNT 3
struct FlightStats
{
    uint32_t flight_count[2]; // 飞行架次
    float flight_time_s[2];   // 飞行总时间(秒)
    float flight_dist_m[2];   // 飞行总距离(米)
} __ALIGNED_PACKED;
// 飞行统计变量(非线程安全 仅限Modes/M32线程使用)
extern FlightStats flightStats;
// 设置需要保存飞行统计到存储器
void set_FlightStats_needStore();
// 读取飞行统计至flightStats
bool read_flightStats(double TIMEOUT = -1);
// 存储flightStats至RAM
bool storeRAM_flightStats(double TIMEOUT = -1);
// 存储flightStats至存储器
bool store_flightStats(double TIMEOUT = -1);

// 安全模式参数
struct MSafeCfg
{
    // 自动返航模式
    //  bit0 - 0:失控悬停 1:失控自动返航
    //  bit1 - 0:返航时机头转向返航点(到家后转回起飞航向) 1:返航全程保持机头朝向不变
    //  bit3 - 0:低电量安全模式不介入 1:低电量安全模式强制介入
    //  bit10 - 0:备降点为经纬度 1:备降点为相对距离
    uint32_t SfRtMode[2];

    // 返航速度cm/s
    float RtSpeed[2];

    // 升高返航范围cm
    float RtHRange[2];
    // 返航升高高度cm（对地）
    float RtHeight[2];
    // 一级降落高度cm（对地 高于此高度按一级降落速度下降）
    float RtLand1Height[2];
    // 二级降落高度cm（对地 低于此高度按LandVel下降 两级高度间线性过渡）
    float RtLand2Height[2];
    // 一级降落速度cm/s（负值代表使用maxAutoVelDown）
    float RtLand1Vel[2];

    // 报警电压
    float FcAlarmVolt[2];
    // 强制返航电压
    float FcRTLVolt[2];
    // 强制降落电压
    float FcLandVolt[2];

    // 备降点最大范围cm
    float MaxALHRange[2];
    // 备降点1
    uint32_t AL1_lat[2];
    uint32_t AL1_lon[2];
    float AL1_alt[2];
    // 备降点2
    uint32_t AL2_lat[2];
    uint32_t AL2_lon[2];
    float AL2_alt[2];
    // 备降点3
    uint32_t AL3_lat[2];
    uint32_t AL3_lon[2];
    float AL3_alt[2];
} __ALIGNED_PACKED;
// 失控返航
#define SfRtMode_AutoRTL(x) (x & (1 << 0))
// 返航全程保持机头朝向不变(不转向返航点/不转回起飞航向)
#define SfRtMode_KeepYawRTL(x) (x & (1 << 1))
// 低电量安全模式强制介入
#define SfRtMode_LpwMSafe(x) (x & (1 << 3))
// 备降点个数
#define MAX_AL_POINTS 3
// 备降点为相对坐标
#define SfRtMode_ALinRelativePos(x) (x & (1 << 10))

// 电量状态
enum class PWR_STATE
{
    // 电量正常
    NORMAL = 0,

    // 电压低返航
    LVOLT_REQ_RTL = 10,
    // 电压低降落
    LVOLT_REQ_LAND,
};
constexpr bool operator<(PWR_STATE a, PWR_STATE b)
{
    return static_cast<int>(a) < static_cast<int>(b);
}
constexpr bool operator>(PWR_STATE a, PWR_STATE b)
{
    return static_cast<int>(a) > static_cast<int>(b);
}
// 获取电量状态
PWR_STATE get_pwr_state();

void init_Modes();