#include "forwardDist.hpp"

#include "Parameters.hpp"
#include "semphr.h"

#define MAX_SENSORS MAX_FORWARDDIST_SENSORS
static SemaphoreHandle_t forwardDistSemphr = xSemaphoreCreateMutex();
static forwardDistSensor *sensors[MAX_SENSORS] = {0};

static inline bool lock(double TIMEOUT = -1)
{
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(forwardDistSemphr, TIMEOUT_Ticks))
        return true;
    return false;
}
static inline void unlock()
{
    xSemaphoreGive(forwardDistSemphr);
}

/*
    注册前向距离传感器
      id: 返回的传感器id指针
    返回值：
      非0:添加成功 返回key
      0:添加失败（无空位或内存不足）
*/
uint32_t forwardDistSensor_register(uint8_t *id, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        // 寻找空位
        int8_t emptySlot = -1;
        for (uint8_t i = 0; i < MAX_SENSORS; ++i)
        {
            if (sensors[i] == 0)
            {
                emptySlot = i;
                break;
            }
        }

        if (emptySlot < 0)
        { // 无空位
            unlock();
            return 0;
        }

        sensors[emptySlot] = new forwardDistSensor;
        forwardDistSensor *sensor = sensors[emptySlot];
        if (sensor == 0)
        { // 内存申请失败
            unlock();
            return 0;
        }

        sensor->id = emptySlot;
        sensor->distance = 0;
        sensor->updateTime.set_invalid();
        sensor->available = false;

        sensor->yaw = 0;
        sensor->yaw_available = false;

        // 返回传感器id
        if (id)
            *id = emptySlot;

        unlock();
        return (uint32_t) sensor;
    }
    return 0;
}

/*
    取消注册前向距离传感器
      id: 传感器id
      key: 传感器密钥

    返回值：
    true:移除成功
    false:移除失败
*/
bool forwardDistSensor_unRegister(uint8_t id, uint32_t key, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (id >= MAX_SENSORS)
        { // 传感器不存在
            unlock();
            return false;
        }
        if (sensors[id] == 0)
        { // 传感器不存在
            unlock();
            return false;
        }
        if ((uint32_t) sensors[id] != key)
        { // key不正确
            unlock();
            return false;
        }

        delete sensors[id];
        sensors[id] = 0;

        unlock();
        return true;
    }
    return false;
}

/*
    获取最小前向距离
      res_distance: 返回的最小距离(cm)

    返回值：
    true:成功
    false:失败（无可用传感器）
*/
bool forwardDistSensor_getMinDistance(double *res_distance, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        double minDist = -1;
        bool found = false;

        for (uint8_t i = 0; i < MAX_SENSORS; ++i)
        {
            if (sensors[i])
            {
                forwardDistSensor *sensor = sensors[i];
                // 检查是否超时
                if (sensor->updateTime.get_pass_time() > 2)
                    sensor->available = false;

                if (sensor->available)
                {
                    if (!found || sensor->distance < minDist)
                    {
                        minDist = sensor->distance;
                        found = true;
                    }
                }
            }
        }

        if (!found)
        { // 无可用传感器
            unlock();
            return false;
        }

        *res_distance = minDist;
        unlock();
        return true;
    }
    return false;
}

/*
    获取第一个可用的航向角
      res_yaw: 返回的航向角(rad)

    返回值：
    true:成功
    false:失败（无可用航向角）
*/
bool forwardDistSensor_getYaw(double *res_yaw, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        for (uint8_t i = 0; i < MAX_SENSORS; ++i)
        {
            if (sensors[i])
            {
                forwardDistSensor *sensor = sensors[i];
                // 检查是否超时
                if (sensor->updateTime.get_pass_time() > 2)
                    sensor->available = false;

                if (sensor->available && sensor->yaw_available)
                {
                    *res_yaw = sensor->yaw;
                    unlock();
                    return true;
                }
            }
        }

        // 无可用航向角
        unlock();
        return false;
    }
    return false;
}

/*
    更新前向距离传感器
      id: 传感器id
      key: 传感器密钥
      distance: 对墙距离(cm)
      yaw: 航向角(rad)
      available: 传感器是否可用
      yaw_available: 航向角是否可用

    返回值：
    true:成功
    false:失败
*/
bool forwardDistSensor_update(uint8_t id, uint32_t key, double distance, double yaw, bool available, bool yaw_available, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (id >= MAX_SENSORS)
        { // 传感器不存在
            unlock();
            return false;
        }
        if (sensors[id] == 0)
        { // 传感器不存在
            unlock();
            return false;
        }
        if ((uint32_t) sensors[id] != key)
        { // key不正确
            unlock();
            return false;
        }

        forwardDistSensor *sensor = sensors[id];
        sensor->distance = distance;
        sensor->yaw = yaw;
        sensor->updateTime = TIME::now();
        sensor->available = available;
        sensor->yaw_available = yaw_available;

        unlock();
        return true;
    }
    return false;
}

/*
    设置前向距离传感器不可用
      id: 传感器id
      key: 传感器密钥

    返回值：
    true:成功
    false:失败
*/
bool forwardDistSensor_setUnavailable(uint8_t id, uint32_t key, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (id >= MAX_SENSORS)
        { // 传感器不存在
            unlock();
            return false;
        }
        if (sensors[id] == 0)
        { // 传感器不存在
            unlock();
            return false;
        }
        if ((uint32_t) sensors[id] != key)
        { // key不正确
            unlock();
            return false;
        }

        forwardDistSensor *sensor = sensors[id];
        sensor->available = false;
        sensor->yaw_available = false;

        unlock();
        return true;
    }
    return false;
}
