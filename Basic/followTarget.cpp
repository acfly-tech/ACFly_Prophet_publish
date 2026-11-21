#include "followTarget.hpp"

#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "semphr.h"

static SemaphoreHandle_t followSemphr = xSemaphoreCreateMutex();
static followSensor *sensor = 0;
static uint16_t sensorTempInd = 0;

static inline bool Lock_follow(double TIMEOUT = -1)
{
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(followSemphr, TIMEOUT_Ticks))
        return true;
    return false;
}
static inline void UnLock_follow()
{
    xSemaphoreGive(followSemphr);
}

/*
        注册精准降落传感器

        返回值：
        非0:添加成功
        0:添加失败（已有传感器或内存不足）
*/
uint32_t followSensorRegister(followDataType dataType, double TIMEOUT)
{
    if (Lock_follow(TIMEOUT))
    {
        if (sensor != 0)
        { // 传感器已存在
            UnLock_follow();
            return 0;
        }

        sensor = new followSensor;
        if (sensor)
        {
            sensor->dataType = dataType;
            sensor->pos.zero();
            sensor->vel.zero();
            sensor->updateTime.set_invalid();
            sensor->available = false;
            sensor->globalXYPosAvailable = false;
            sensor->delay = 0;

            if (++sensorTempInd == 0)
                sensorTempInd = 1;
        }
        UnLock_follow();
        return (uint32_t) sensor;
    }
    return 0;
}
/*
        取消注册精准降落传感器

        返回值：
        true:移除成功
        false:移除失败
*/
bool followSensorUnRegister(double TIMEOUT)
{
    if (Lock_follow(TIMEOUT))
    {
        if (sensor == 0)
        { // 传感器不存在
            UnLock_follow();
            return false;
        }

        delete sensor;
        sensor = 0;

        UnLock_follow();
        return true;
    }
    return false;
}

/*
        获取精准降落传感器

        返回值：
        true:成功
        false:失败
*/
uint16_t get_followSensor(followSensor *res_sensor, double TIMEOUT)
{
    if (Lock_follow(TIMEOUT))
    {
        if (sensor == 0)
        { // 传感器不存在
            UnLock_follow();
            return false;
        }

        if (sensor->updateTime.get_pass_time() > 1.5)
        {
            sensor->vel.zero();
            sensor->available = false;
        }
        *res_sensor = *sensor;

        UnLock_follow();
        return sensorTempInd;
    }
    return 0;
}

/*
        更新精准降落传感器

        返回值：
        true:成功
        false:失败
*/
bool update_followSensor(uint32_t key, vector3<double> pos, vector3<double> vel, vector3<double> acc, double yaw, double yawRate, double comp_r, double comp_yaw, vector3<double> pos_offset,
                         bool available, bool followInLpw, double delay, double TIMEOUT)
{
    if (Lock_follow(TIMEOUT))
    {
        if (sensor == 0)
        { // 传感器不存在
            UnLock_follow();
            return false;
        }
        if (key != (uint32_t) sensor)
        { // 钥匙不正确
            UnLock_follow();
            return false;
        }

        switch (sensor->dataType)
        {
            case followDataType_Gsv_xyz:
            case followDataType_Gsv_xy:
            case followDataType_nRTL_Gsv_xyz:
            case followDataType_nRTL_Gsv_xy:
            case followDataType_track_Gs_xy:
            case followDataType_track_Gs_xyz:
            {
                sensor->lat = pos.x;
                sensor->lon = pos.y;
                sensor->pos.z = pos.z;

                PosSensorHealthInf2 global_inf;
                if (get_OptimalGlobal_XY(&global_inf))
                { // 获取指定经纬度平面坐标
                    map_projection_project(&global_inf.mp, sensor->lat, sensor->lon, &sensor->pos.x, &sensor->pos.y);
                    sensor->pos.x -= global_inf.HOffset.x;
                    sensor->pos.y -= global_inf.HOffset.y;

                    sensor->globalXYPosAvailable = true;
                }
                else
                    sensor->globalXYPosAvailable = available = false;

                break;
            }
            default:
            {
                sensor->pos = pos;
                sensor->globalXYPosAvailable = false;
                break;
            }
        }
        sensor->pos += pos_offset;
        sensor->vel = vel;
        sensor->acc = acc;
        sensor->yaw = yaw;
        sensor->yawRate = yawRate;
        if (delay >= 0)
            sensor->delay = delay;

        if (yaw < 50)
        {
            double sinYaw, cosYaw;
            fast_sin_cos(sensor->yaw - comp_yaw, &sinYaw, &cosYaw);
            sensor->pos.x += comp_r * cosYaw;
            sensor->pos.y += comp_r * sinYaw;
            sensor->vel.x += -yawRate * comp_r * sinYaw;
            sensor->vel.y += yawRate * comp_r * cosYaw;
            // sensor->acc.x += -sq(yawRate) * comp_r * cosYaw;
            // sensor->acc.y += -sq(yawRate) * comp_r * sinYaw;

            sensor->comp_r = comp_r;
            sensor->comp_yaw = comp_yaw;
        }
        else
        {
            sensor->comp_r = 0;
            sensor->comp_yaw = 0;
        }

        sensor->updateTime = TIME::now();
        sensor->available = available;
        sensor->followInLpw = followInLpw;

        UnLock_follow();
        return true;
    }
    return false;
}
/*
        是能精准降落传感器

        返回值：
        true:成功
        false:失败
*/
bool setInavailable_followSensor(uint32_t key, double TIMEOUT)
{
    if (Lock_follow(TIMEOUT))
    {
        if (sensor == 0)
        { // 传感器不存在
            UnLock_follow();
            return false;
        }
        if (key != (uint32_t) sensor)
        { // 钥匙不正确
            UnLock_follow();
            return false;
        }

        sensor->updateTime = TIME::now();
        sensor->available = false;

        UnLock_follow();
        return true;
    }
    return false;
}