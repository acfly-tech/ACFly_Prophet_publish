#include "Parachute.hpp"

#include "semphr.h"

// 数据在线超时：超过此时间未收到状态报文视为离线
#define PARACHUTE_ONLINE_TIMEOUT 3.0
// 开伞请求保持时间：驱动在此时间内持续重发开伞指令
#define PARACHUTE_RELEASE_HOLD_TIME 5.0

static SemaphoreHandle_t parachuteSemphr = xSemaphoreCreateMutex();
static ParachuteStatus *parachute = 0;

// 开伞请求时刻（invalid 表示无请求）
static TIME releaseRequestTime;
// 开伞请求确认标志（驱动已发出开伞指令）
static bool releaseSent = false;

static inline bool lock(double TIMEOUT = -1)
{
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(parachuteSemphr, TIMEOUT_Ticks))
        return true;
    return false;
}
static inline void unlock()
{
    xSemaphoreGive(parachuteSemphr);
}

/*
    注册降落伞

    返回值：
      非0:添加成功 返回key
      0:添加失败（已有降落伞或内存不足）
*/
uint32_t Parachute_register(double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (parachute != 0)
        { // 降落伞已存在
            unlock();
            return 0;
        }

        parachute = new ParachuteStatus;
        if (parachute)
        {
            parachute->updateTime.set_invalid();
            parachute->ready = false;
            parachute->deployed = false;
            parachute->manufacturer = 0;

            releaseRequestTime.set_invalid();
            releaseSent = false;
        }
        unlock();
        return (uint32_t) parachute;
    }
    return 0;
}
/*
    取消注册降落伞

    返回值：
    true:移除成功
    false:移除失败
*/
bool Parachute_unRegister(uint32_t key, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (parachute == 0)
        { // 降落伞不存在
            unlock();
            return false;
        }
        if ((uint32_t) parachute != key)
        { // key不正确
            unlock();
            return false;
        }

        delete parachute;
        parachute = 0;

        unlock();
        return true;
    }
    return false;
}

/*
    更新降落伞状态（由驱动调用）

    返回值：
    true:成功
    false:失败
*/
bool Parachute_updateStatus(uint32_t key, bool ready, bool deployed, uint8_t manufacturer, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (parachute == 0)
        { // 降落伞不存在
            unlock();
            return false;
        }
        if ((uint32_t) parachute != key)
        { // key不正确
            unlock();
            return false;
        }

        parachute->ready = ready;
        parachute->deployed = deployed;
        parachute->manufacturer = manufacturer;
        parachute->updateTime = TIME::now();

        unlock();
        return true;
    }
    return false;
}

/*
    获取降落伞是否存在且在线

    返回值：
    true:降落伞存在且数据在线
    false:降落伞不存在或数据超时
*/
bool Parachute_isPresent(double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (parachute == 0)
        { // 降落伞不存在
            unlock();
            return false;
        }

        bool online = parachute->updateTime.is_valid() && parachute->updateTime.get_pass_time() < PARACHUTE_ONLINE_TIMEOUT;
        unlock();
        return online;
    }
    return false;
}

/*
    获取降落伞状态（需先确认 Parachute_isPresent 为 true）

    返回值：
    true:成功
    false:失败
*/
bool Parachute_getStatus(ParachuteStatus *res_status, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (parachute == 0)
        { // 降落伞不存在
            unlock();
            return false;
        }

        *res_status = *parachute;

        unlock();
        return true;
    }
    return false;
}

/*
    请求开伞（立即发送开伞指令，降落伞就绪时生效）

    返回值：
    true:降落伞存在且开伞请求已提交
    false:降落伞不存在或请求失败
*/
bool Parachute_release(double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (parachute == 0)
        { // 降落伞不存在
            unlock();
            return false;
        }

        releaseRequestTime = TIME::now();
        releaseSent = false;

        unlock();
        return true;
    }
    return false;
}

/*
    撤销开伞请求（仅在降落伞尚未触发时有效）
*/
void Parachute_cancelRelease()
{
    if (lock(0.01))
    {
        releaseRequestTime.set_invalid();
        releaseSent = false;
        unlock();
    }
}

/*
    驱动内部接口：查询开伞请求（由降落伞驱动轮询调用）

      res_active: 是否处于开伞请求有效期内（需要发送开伞指令）

    返回值：
    true:成功
    false:失败（无降落伞）
*/
bool Parachute_getReleaseRequest(bool *res_active)
{
    if (lock(0.01))
    {
        if (parachute == 0)
        { // 降落伞不存在
            unlock();
            return false;
        }

        if (releaseRequestTime.is_valid() && releaseRequestTime.get_pass_time() < PARACHUTE_RELEASE_HOLD_TIME)
        {
            releaseSent = true;
            *res_active = true;
        }
        else
            *res_active = false;

        unlock();
        return true;
    }
    return false;
}
