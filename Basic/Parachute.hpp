#pragma once

#include "Basic.hpp"

// 降落伞信息
struct ParachuteInf
{
    /*开伞触发方式
        0-仅手动（地面站指令/失控保护等，侧翻检测不触发开伞）
        1-侧翻检测触发（侧翻即视为坠机，立即开伞）
    */
    uint32_t triggerType;
    // 侧翻判定角度阈值(deg)
    float flipAngle;
    // 侧翻判定角速度误差阈值(rad/s)
    float flipRateErr;
    // 侧翻判定持续时间(s)
    float flipDuration;
} __ALIGNED_PACKED;

// 降落伞状态
struct ParachuteStatus
{
    // 数据更新时间
    TIME updateTime;
    // 降落伞就绪（上电自检通过，可以开伞）
    bool ready;
    // 降落伞是否已打开
    bool deployed;
    // 厂商代码
    uint8_t manufacturer;
};

/*
    注册降落伞

    返回值：
      非0:添加成功 返回key
      0:添加失败（已有降落伞或内存不足）
*/
uint32_t Parachute_register(double TIMEOUT = -1);
/*
    取消注册降落伞

    返回值：
    true:移除成功
    false:移除失败
*/
bool Parachute_unRegister(uint32_t key, double TIMEOUT = -1);

/*
    更新降落伞状态（由驱动调用）

      key: 降落伞密钥
      ready: 降落伞就绪状态
      deployed: 降落伞是否已打开
      manufacturer: 厂商代码

    返回值：
    true:成功
    false:失败
*/
bool Parachute_updateStatus(uint32_t key, bool ready, bool deployed, uint8_t manufacturer, double TIMEOUT = -1);

/*
    获取降落伞是否存在且在线

    返回值：
    true:降落伞存在且数据在线
    false:降落伞不存在或数据超时
*/
bool Parachute_isPresent(double TIMEOUT = 0.01);

/*
    获取降落伞状态（需先确认 Parachute_isPresent 为 true）

    返回值：
    true:成功
    false:失败
*/
bool Parachute_getStatus(ParachuteStatus *res_status, double TIMEOUT = 0.01);

/*
    请求开伞（立即发送开伞指令，降落伞就绪时生效）

    返回值：
    true:降落伞存在且开伞请求已提交
    false:降落伞不存在或请求失败
*/
bool Parachute_release(double TIMEOUT = 0.01);

/*
    撤销开伞请求（仅在降落伞尚未触发时有效）
*/
void Parachute_cancelRelease();

/*
    驱动内部接口：查询开伞请求（由降落伞驱动轮询调用）

      res_active: 是否处于开伞请求有效期内（需要发送开伞指令）

    返回值：
    true:成功
    false:失败（无降落伞）
*/
bool Parachute_getReleaseRequest(bool *res_active);
