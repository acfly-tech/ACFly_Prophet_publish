#include "precLand.hpp"

#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "semphr.h"

#define MAX_SENSORS MAX_PRECLAND_SENSORS
static SemaphoreHandle_t precLandSemphr = xSemaphoreCreateMutex();
static precLandSensor *sensors[MAX_SENSORS] = {0};
static int8_t currentSId = -1;

static inline bool lock(double TIMEOUT = -1)
{
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(precLandSemphr, TIMEOUT_Ticks))
        return true;
    return false;
}
static inline void unlock()
{
    xSemaphoreGive(precLandSemphr);
}

/*
    注册精准降落传感器
      id: 传感器id
    返回值：
      非0:添加成功 返回key
      0:添加失败（已有传感器或内存不足）
*/
uint32_t precLandSensor_register(uint8_t id, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (id >= MAX_SENSORS)
        { // 传感器不存在
            unlock();
            return false;
        }
        if (sensors[id] != 0)
        { // 传感器已存在
            unlock();
            return false;
        }

        sensors[id] = new precLandSensor;
        precLandSensor *sensor = sensors[id];
        if (sensor == 0)
        { // 内存申请失败
            unlock();
            return 0;
        }
        if (sensor)
        {
            sensor->id = id;

            sensor->pos.zero();
            sensor->delay = 0;
            sensor->updateTime.set_invalid();
            sensor->available = false;
            sensor->z_available = false;

            sensor->yaw = 0;
            sensor->yaw_available = false;
        }
        unlock();
        return (uint32_t) sensor;
    }
    return 0;
}
/*
    取消注册精准降落传感器
      id: 传感器id
      key: 传感器密钥

    返回值：
    true:移除成功
    false:移除失败
*/
bool precLandSensor_unRegister(uint8_t id, uint32_t key, double TIMEOUT)
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
    获取当前精准降落传感器
      res_sensor: 返回的传感器

    返回值：
    true:成功
    false:失败
*/
bool precLandSensor_get(precLandSensor *res_sensor, double TIMEOUT)
{
    if (lock(TIMEOUT))
    {
        if (currentSId >= 0)
        { // 检查当前传感器是否存在
            if (sensors[currentSId] == 0)
                currentSId = -1;
        }
        if (currentSId < 0)
        { // 当前传感器不存在
            // 寻找新传感器
            for (uint8_t i = 0; i < MAX_SENSORS; ++i)
            { // 选空位填充
                if (sensors[i])
                {
                    precLandSensor *sensor = sensors[i];
                    if (sensor->updateTime.get_pass_time() > 2)
                        sensor->available = false;

                    if (sensor->available)
                    {
                        currentSId = i;
                        break;
                    }
                    else
                        currentSId = i;
                    break;
                }
            }
        }
        else
        { // 当前传感器存在
            // 检查当前传感器是否可用
            precLandSensor *sensor = sensors[currentSId];
            if (sensor->updateTime.get_pass_time() > 2)
                sensor->available = false;

            if (!sensor->available)
            { // 当前传感器不可用
              // 寻找是否有可用传感器
                for (uint8_t i = 0; i < MAX_SENSORS; ++i)
                { // 选空位填充
                    if (sensors[i])
                    {
                        precLandSensor *sensor = sensors[i];
                        if (sensor->updateTime.get_pass_time() > 2)
                            sensor->available = false;

                        if (sensor->available)
                        {
                            currentSId = i;
                            break;
                        }
                    }
                }
            }
        }

        if (currentSId < 0)
        { // 传感器不存在
            unlock();
            return false;
        }
        uint8_t id = currentSId;

        precLandSensor *sensor = sensors[id];
        if (sensor->updateTime.get_pass_time() > 2)
            sensor->available = false;
        *res_sensor = *sensor;

        unlock();
        return true;
    }
    return false;
}

/*
    获取指定精准降落传感器
      id: 传感器id
      res_sensor: 返回的传感器

    返回值：
    true:成功
    false:失败
*/
bool precLandSensor_get(uint8_t id, precLandSensor *res_sensor, double TIMEOUT)
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

        precLandSensor *sensor = sensors[id];
        if (sensor->updateTime.get_pass_time() > 2)
            sensor->available = false;
        *res_sensor = *sensor;

        unlock();
        return true;
    }
    return false;
}

/*
    更新精准降落传感器
      id: 传感器id
      key: 传感器密钥
      pos: 降落点相对于当前位置的东北天距离(cm)
      yaw: 航向角(rad 东0 北pi/2)
      available: 传感器是否可用
      z_available: 传感器z轴是否可用
      yaw_available: 传感器yaw是否可用

    返回值：
    true:成功
    false:失败
*/
bool precLandSensor_updateENU(uint8_t id, uint32_t key, vector3<double> pos, double yaw, bool available, bool z_available, bool yaw_available, double TIMEOUT)
{
    Quaternion quat;
    get_AirframeY_quat(&quat);
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
        precLandSensor *sensor = sensors[id];

        __attribute__((aligned(8))) PrecLandInf precLandInf;
        if (ReadParamGroup("precLand", (uint64_t *) &precLandInf, 0) == PR_OK)
        {
            PrecLandSensorInf *sensorInf = &precLandInf.sensorInfs[id];

            sensor->yaw = yaw;
            sensor->yaw_available = available && yaw_available;

            // 获取延时补偿姿态
            Quaternion quat;
            get_history_AirframeQuatY(&quat, sensorInf->delay[0]);
            sensor->delay = sensorInf->delay[0];

            // 补偿传感器位置偏移
            vector3<double> offset_comp = quat.rotate(vector3<double>(sensorInf->pofs_x[0], sensorInf->pofs_y[0], sensorInf->pofs_z[0]));
            pos.x -= offset_comp.x;
            pos.y -= offset_comp.y;
            pos.z -= offset_comp.z;

            // 补偿位置偏移
            switch (sensorInf->ofsType[0])
            {
                case 0:
                { // 前左上 通过yaw信息计算前向方向
                    if (sensor->yaw_available)
                    {
                        double sinYaw, cosYaw;
                        fast_sin_cos(sensor->yaw, &sinYaw, &cosYaw);
                        pos.x += BodyHeading2ENU_x(sensorInf->ofs_x[0], sensorInf->ofs_y[0], sinYaw, cosYaw);
                        pos.y += BodyHeading2ENU_y(sensorInf->ofs_x[0], sensorInf->ofs_y[0], sinYaw, cosYaw);
                    }
                    break;
                }
                case 1:
                { // 东北天
                    pos.x += sensorInf->ofs_x[0];
                    pos.y += sensorInf->ofs_y[0];
                    break;
                }
            }
            pos.z += sensorInf->ofs_z[0];
        }
        sensor->yaw = yaw - quat.getYaw();
        sensor->pos = pos;
        sensor->updateTime = TIME::now();
        sensor->available = available;
        sensor->z_available = z_available;

        unlock();
        return true;
    }
    return false;
}