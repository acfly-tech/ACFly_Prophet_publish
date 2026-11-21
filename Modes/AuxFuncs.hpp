#pragma once

#include "Basic.hpp"
#include "Receiver.hpp"

/*** 遥控-数字云台控制 ***/
#define DgYTRol 1          // 横滚
#define DgYTPit 2          // 俯仰
#define DgYTYaw 3          // 偏航
#define DgYTCenter 4       // 回中
#define DgYTZoom 5         // 变倍
#define DgYTMode 6         // 模式
#define DgYTFocus 7        // 聚焦
#define DgYTAutoFocus 8    // 自动聚焦
#define DgYTShoot 9        // 拍照
#define DgYTVideo 10       // 录像
#define DgYTPicInPic 11    // 画中画
#define DgYTOpenLaser 12   // 开启激光
#define DgYTOpenThermal 13 // 开启红外
#define DgYTAutoTrack 14   // 自动跟踪
/*** 遥控-数字云台控制 ***/

/*AuxFuncs
        0:无
        1-16:映射遥控器对应通道（raw data）
        25-40:用遥控器对应通道进行相机快门触发（raw_data）
        49-64:用遥控器对应通道进行云台控制（raw_data）
        73-96:舵机云台俯仰控制
        97-120:舵机云台横滚控制
        121-137:数字云台控制
        173-196:舵机云台俯仰往复控制
        197-220:舵机云台横滚往复控制
        221-244:摇摆喷头控制
        273-296:舵机云台2俯仰控制
        301-316:起落架控制
        501-516:遥控器开关量
        525-548:水泵控制
        950:断药开关
        1001-1016:映射虚拟摇杆通道
        1025-1048:虚拟摇杆相机快门触发
        1049-1072:虚拟摇杆无刷云台俯仰
        1073-1096:虚拟摇杆舵机云台俯仰
        1097-1120:虚拟摇杆舵机云台横滚
*/
struct AuxFuncsConfig
{
    // 辅助通道选项
    //  bit4: 0-舵机锁开启时舵机通道不能变化 1-舵机锁开启时强行将通道锁定到舵机锁参数值
    uint32_t auxCfg[2];

    // 舵机锁参数
    //  bit24:31-遥控器通道 为0不开启舵机锁
    //  bit0:16-舵机保持值
    uint32_t stLockParam[2];

    uint8_t Aux_YTSurveyPic[8]; // 数字云台是否需在测绘任务过程中拍照
    uint16_t Aux_CamOnPwm[4];
    uint16_t Aux_CamOffPwm[4];
    float Aux_CamShTime[2];
    uint8_t Aux_CamTrigEna[8];
    uint16_t Aux_BsYTPit0[4];
    uint16_t Aux_BsYTPit90[4];
    uint16_t Aux_StYTPit0[4];
    uint16_t Aux_StYTPit90[4];
    uint16_t Aux_StYT2Pit0[4];
    uint16_t Aux_StYT2Pit90[4];
    uint16_t Aux_StYTRolN45[4];
    uint16_t Aux_StYTRolP45[4];
    float Aux_YTPitMin[2];
    float Aux_YTPitMax[2];
    float Aux_YTRollMax[2];
    float Aux_Pump1Min[2];
    float Aux_Pump1St[2];
    float Aux_Pump1Max[2];
    float Aux_Pump1Sp[2];
    uint16_t Aux_GearOpenPwm[4];
    uint16_t Aux_GearClosePwm[4];

    /* 遥控通道1-16对应的数字云台功能与参数（放在结构体末尾） */
    uint16_t RcYTFunc[16][4];
    float RcYTParam1[16][2];
    float RcYTParam2[16][2];
};
#define AUX_CFG_STLOCK_FORCEVALUE_BIT (1 << 4)

/*数字云台通道*/
struct YTCtrl
{
    /*模式
            0-角度模式
            1-角速度模式
    */
    uint8_t mode;
    float value;
    TIME update_TIME;
};
bool get_DgYTRolCtrl(YTCtrl *res, double Sync_waitTime = -1);
bool get_DgYTPitCtrl(YTCtrl *res, double Sync_waitTime = -1);
bool get_DgYTYawCtrl(YTCtrl *res, double Sync_waitTime = -1);
bool get_DgYTZoomCtrl(YTCtrl *res, double Sync_waitTime = -1);
bool get_DgYTModeCtrl(YTCtrl *res, double Sync_waitTime = -1);
bool get_DgYTFocusCtrl(YTCtrl *res, double Sync_waitTime = -1);
bool get_DgYTShootCtrl(YTCtrl *res, double Sync_waitTime = -1);
bool get_DgYTReturnCtrl(YTCtrl *res, double Sync_waitTime = -1);

// 记录Pos
bool recordPos(bool acTrig);

// 数字云台功能接口
struct DgYTFuncStruct
{
    // 是否可用
    bool available;
    // 拍照: 0-不成功 1-成功 2-等待Trig脚位触发 3-等待内部触发
    //  acTrig: 内部触发(加触发数量)
    uint8_t (*takePhoto)();
    // 指定俯仰角
    bool (*setPitch)(double pit);
    // 指定偏航角
    bool (*setYaw)(double yaw);
};
int16_t dgYtFunc_Register(const DgYTFuncStruct &func);
bool dgYtFunc_UnRegister(int16_t ind);
bool get_dgYtFunc(int16_t ind, DgYTFuncStruct *func);
/*数字云台通道*/

// 初始化Aux处理
void init_process_AuxFuncs();
// 进行Aux处理
void process_AuxFuncs(const Receiver *rc, double h);

// 拍照
bool AuxCamTakePhoto();
// 异步拍照
bool AuxCamTakePhotoAsync(double TIMEOUT = 0.01);
// 自动控制云台角度
bool AuxGimbalSetAngle(double angle);

// 获取剩余药量百分比
float getTankRMPercent();

// 水泵1控制
//  percent: -1000:关闭且不可手动控制 1000:关闭可手动控制 0-100:给定转动百分比
bool setPump1(float percent);
#define PumpOffAutoPercent -1000
#define PumpOffManualPercent 1000
bool setPump1bySpeed(float sp);

// 设置对应通道PWM值
bool setAuxPWM(float PWMus, uint8_t ind);

void init_AuxFuncs();
