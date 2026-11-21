#include "Receiver.hpp"
#include "HardWareConfig.hpp"
#include "ReceiverBackend.hpp"

#include "AC_Math.hpp"
#include "FreeRTOS.h"
#include "Parameters.hpp"
#include "semphr.h"
#include <map>

using namespace std;

/*
        Receiver配置：
        uint64：8xint8_t 8通道映射
        4xfloat：8通道最小值(offset)
        4xfloat：8通道缩放(scale)
*/
struct ReceiverSync
{
    Receiver rc;
    SemaphoreHandle_t mutex;

    // 上次的切换通道值
    float last_sw_rc;
};
static map<SName, ReceiverSync> Receivers;

/**
 * @brief 注册接收机
 * @param name 接收机名称
 * @param rc_type 接收机类型
 * @param prio 初始优先级
 * @param initialCfg 预校准信息
 */
bool ReceiverRegister(SName name, RC_CLASS rc_type, uint8_t prio, const RC_CONFIG *initialCfg)
{
    LockInitializationStatus();
    if (getInitializationCompleted() == true)
    {
        UnLockInitializationStatus();
        return false;
    }

    // 注册接收机校准参数
    MAV_PARAM_TYPE param_types[] = {
        MAV_PARAM_TYPE_UINT64,                                                                      // 8通道映射
        MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, // 8通道最小值
        MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, // 8通道缩放
    };
    PR_RESULT res;
    if (initialCfg)
        res = ParamGroupRegister(SName("RC_") + name, 1, 9, param_types, 0, (uint64_t *) initialCfg);
    else
        res = ParamGroupRegister(SName("RC_") + name, 0, 9, param_types, 0, 0);
    if (res != PR_OK)
    { // 加入参数表失败
        UnLockInitializationStatus();
        return false;
    }

    // 定义接收机序号
    uint8_t id = Receivers.size();

    // 注册接收机个体参数
    RC_IND_CONFIG init_rc_ind_cfg = {0};
    init_rc_ind_cfg.prio[0] = prio;
    init_rc_ind_cfg.sw_chan[0] = 0;
    init_rc_ind_cfg.sw_rcInd[0] = 0;
    MAV_PARAM_TYPE ind_param_types[] = {
        MAV_PARAM_TYPE_UINT8, // prio
        MAV_PARAM_TYPE_UINT8, // sw_chan
        MAV_PARAM_TYPE_UINT8, // sw_rcInd
    };
    SName param_names[] = {
        SName("RC") + ('0' + id) + name + "_prio",
        SName("RC") + ('0' + id) + name + "_swc",
        SName("RC") + ('0' + id) + name + "_swi",
    };
    InitialParametersInit((uint64_t *) &init_rc_ind_cfg, ind_param_types, param_names, sizeof(init_rc_ind_cfg) / 8);
    ParamGroupRegister(SName("RCI_") + name, 1, sizeof(init_rc_ind_cfg) / 8, ind_param_types, param_names, (uint64_t *) &init_rc_ind_cfg);

    // 初始化新接收机
    ReceiverSync new_rc;
    new_rc.rc.available = new_rc.rc.connected = false;
    new_rc.rc.last_update_time = TIME::now();
    new_rc.mutex = xSemaphoreCreateMutex();
    new_rc.rc.preCalibrated = initialCfg ? true : false;
    new_rc.rc.rc_type = rc_type;
    new_rc.rc.id = id;

    // 加入接收机表
    vPortEnterCritical();
    Receivers.insert(std::pair<SName, ReceiverSync>(name, new_rc));
    vPortExitCritical();

    UnLockInitializationStatus();
    return true;
}

/**
 * @brief 更新接收机数据
 * @param name 接收机名称
 * @param raw_data 接收机原始通道数据
 * @param channel_count 接收机通道数
 */
bool ReceiverUpdate(SName name, bool connected, float raw_data[16], uint8_t channel_count, double TIMEOUT)
{
    // 初始化完成后才可以更新接收机
    if (getInitializationCompleted() == false)
        return false;
    if (channel_count < 4 || channel_count > 16)
        return false;

    map<SName, ReceiverSync>::iterator it = Receivers.find(name);
    if (it == Receivers.end())
        return false;

    // 获取接收机配置参数
    bool rc_initialized;
    uint64_t rc_config[9];
    ReadParamGroup(SName("RC_") + name, rc_config, &rc_initialized);
    rc_initialized = !rc_initialized;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(it->second.mutex, TIMEOUT_Ticks))
    {
        it->second.rc.connected = connected;
        if (connected)
        { // 已连接更新数据
            it->second.rc.update_time = it->second.rc.last_update_time.get_pass_time_st();
            it->second.rc.raw_available_channels = channel_count;
            memcpy(it->second.rc.raw_data, raw_data, channel_count * sizeof(float));
            if (rc_initialized || it->second.rc.preCalibrated)
            { // Receiver已校准更新通道数据
                uint8_t *ch_reflections = (uint8_t *) &rc_config[0];
                float *channels_Min_RC = (float *) &rc_config[1];
                float *channels_RC_Scale = (float *) &rc_config[5];
                it->second.rc.available_channels = 8;
                for (uint8_t i = 0; i < 8; ++i)
                {
                    uint8_t ch_reflection = ch_reflections[i];

                    if (ch_reflection < channel_count && raw_data[ch_reflection] > -20 && raw_data[ch_reflection] < 120.0f && raw_data[ch_reflection] >= -10 && raw_data[ch_reflection] <= 110)
                        it->second.rc.data[i] = constrain((raw_data[ch_reflection] - channels_Min_RC[i]) * channels_RC_Scale[i], 0.0f, 100.0f);
                    else if (i < 4)
                    { // 通道数目不足
                        it->second.rc.available_channels = i;
                        it->second.rc.available = false;
                        xSemaphoreGive(it->second.mutex);
                        return false;
                    }
                    else
                    {
                        it->second.rc.available_channels = i;
                        break;
                    }
                }
                it->second.rc.available = true;
            }
            else
                it->second.rc.available = false;
        }
        else
            it->second.rc.available = false;
        xSemaphoreGive(it->second.mutex);
        return true;
    }
    return false;
}

/*获取接收机*/
// 计算接收机主优先级（低5bit）
static inline uint8_t getRcPriority(const RC_IND_CONFIG &ind_cfg)
{
    return ind_cfg.prio[0] & 0x1F;
}

// 计算接收机次优先级（高3bit）
static inline uint8_t getRcSubPriority(const RC_IND_CONFIG &ind_cfg)
{
    return (ind_cfg.prio[0] >> 5) & 0x07;
}

static SemaphoreHandle_t rc_it_Mutex = xSemaphoreCreateMutex();
static map<SName, ReceiverSync>::iterator rc_it = Receivers.end();
/**
 * @brief 获取当前可用接收机
 * @param rc 获取的接收机
 * @param name 获取到的接收机名称
 * @param TIMEOUT 超时时间
 */
bool getReceiver(Receiver *rc, SName *name, double TIMEOUT)
{
    // 初始化完成后才可以获取接收机
    if (getInitializationCompleted() == false)
    {
        rc->available = rc->connected = false;
        return false;
    }

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    // 读取RC_COMMON_CONFIG配置
    RC_COMMON_CONFIG common_cfg = {0};
    bool common_cfg_new = false;
    ReadParamGroup(SName("RC_COMMON"), (uint64_t *) &common_cfg, &common_cfg_new);

    // 读取摇杆中位死区配置
    float neutralZ[2] = {0};
    ReadParam("MFunc_NeutralZ", 0, 0, (uint64_t *) &neutralZ[0], 0);

    // 获取当前接收机指针
    map<SName, ReceiverSync>::iterator cur_it;
    if (xSemaphoreTake(rc_it_Mutex, TIMEOUT_Ticks))
    {
        cur_it = rc_it;
        // xSemaphoreGive(rc_it_Mutex);
    }
    else
    {
        rc->available = rc->connected = false;
        return false;
    }

    // 当前接收机是否可用
    bool cur_rc_available = false;
    // 当前接收机优先级
    int8_t current_rc_prio = -1;
    // 当前接收机次优先级
    int8_t current_rc_sub_prio = -1;
    // 需要切换到的接收机序号
    int8_t need_sw_rc_ind = -1;
    // 当前接收机是否有动作
    bool cur_rc_has_action = false;
    // 检查当前接收机是否可用
    if (cur_it != Receivers.end())
    {
        if (xSemaphoreTake(cur_it->second.mutex, portMAX_DELAY))
        {
            // 判断连接状态
            if (cur_it->second.rc.last_update_time.get_pass_time() > 2.0f)
            { // 接收机超时未更新，认为已断开
                cur_it->second.rc.connected = cur_it->second.rc.available = false;
            }
            if (cur_it->second.rc.connected == false)
            {
                xSemaphoreGive(cur_it->second.mutex);
                rc_it = cur_it = Receivers.end();
            }
            else
            { // 接收机已连接

                // 获取当前接收机个体配置
                RC_IND_CONFIG cur_ind_cfg = {0};
                ReadParamGroup(SName("RCI_") + cur_it->first, (uint64_t *) &cur_ind_cfg, 0);

                // 获取当前接收机优先级
                current_rc_prio = getRcPriority(cur_ind_cfg);
                current_rc_sub_prio = getRcSubPriority(cur_ind_cfg);

                if (cur_it->second.rc.available)
                { // 当前接收机可用
                    cur_rc_available = true;

                    // 检测当前接收机动作
                    if (cur_it->second.rc.available)
                    {
                        cur_rc_has_action = !in_symmetry_range_mid(cur_it->second.rc.data[0], 50, neutralZ[0]) || !in_symmetry_range_mid(cur_it->second.rc.data[1], 50, neutralZ[0]) ||
                                            !in_symmetry_range_mid(cur_it->second.rc.data[2], 50, neutralZ[0]) || !in_symmetry_range_mid(cur_it->second.rc.data[3], 50, neutralZ[0]);
                    }

                    // 接收机通道变化切换接收机
                    if (common_cfg.configs[0] & RC_COMMON_CONFIG_SW_ACT)
                    {
                        if (cur_ind_cfg.sw_chan[0] > 0 && cur_ind_cfg.sw_chan[0] <= cur_it->second.rc.raw_available_channels)
                        {
                            uint8_t chan = cur_ind_cfg.sw_chan[0] - 1;
                            uint8_t target_id = cur_ind_cfg.sw_rcInd[0];
                            float cur_val = cur_it->second.rc.raw_data[chan];
                            // 跳变检测
                            if (cur_it->second.last_sw_rc < 120 && fabs(cur_val - cur_it->second.last_sw_rc) > 20.0f)
                            {
                                need_sw_rc_ind = target_id;
                            }
                            cur_it->second.last_sw_rc = cur_val;
                        }
                        else
                            cur_it->second.last_sw_rc = 200;
                    }
                    else
                        cur_it->second.last_sw_rc = 200;
                }
                else
                    cur_it->second.last_sw_rc = 200;

                xSemaphoreGive(cur_it->second.mutex);
            }
        }
    }

    // 检测接收机切换
    int8_t sw_rc_prio = -1;
    int8_t sw_rc_sub_prio = -1;
    if (cur_rc_available)
    {
        sw_rc_sub_prio = current_rc_sub_prio;
        sw_rc_prio = current_rc_prio;
    }
    map<SName, ReceiverSync>::iterator switch_it = Receivers.end();
    for (auto it = Receivers.begin(); it != Receivers.end(); ++it)
    {
        // 获取接收机个体配置
        RC_IND_CONFIG ind_cfg = {0};
        ReadParamGroup(SName("RCI_") + it->first, (uint64_t *) &ind_cfg, 0);

        if (xSemaphoreTake(it->second.mutex, portMAX_DELAY))
        {
            // 如果不是当前接收机 重置last_sw_rc
            if (cur_it == Receivers.end() || it != cur_it)
                it->second.last_sw_rc = 200;

            // 判断连接状态
            if (it->second.rc.last_update_time.get_pass_time() > 2.0f)
            { // 接收机超时未更新，认为已断开
                it->second.rc.connected = it->second.rc.available = false;
            }

            // 可用的接收机进行切换判断
            if (it->second.rc.available)
            {
                // 根据优先级切换接收机
                if ((common_cfg.configs[0] & RC_COMMON_CONFIG_SW_PRIO) || !cur_rc_available || current_rc_prio < 0)
                {
                    uint8_t prio = getRcPriority(ind_cfg);
                    if (prio > sw_rc_prio)
                    {
                        sw_rc_prio = prio;
                        sw_rc_sub_prio = getRcSubPriority(ind_cfg);
                        switch_it = it;
                    }
                }

                // 根据need_sw_rc_ind切换接收机
                if (need_sw_rc_ind >= 0 && it->second.rc.id == need_sw_rc_ind)
                {
                    uint8_t tgt_prio = getRcPriority(ind_cfg);
                    if (!(common_cfg.configs[0] & RC_COMMON_CONFIG_SW_PRIO) || tgt_prio >= sw_rc_prio)
                    {
                        sw_rc_prio = tgt_prio;
                        sw_rc_sub_prio = getRcSubPriority(ind_cfg);
                        switch_it = it;
                    }
                }

                // 摇杆动作切换接收机
                if (common_cfg.configs[0] & RC_COMMON_CONFIG_SW_ACT)
                {
                    bool rc_has_action = !in_symmetry_range_mid(it->second.rc.data[0], 50, neutralZ[0]) || !in_symmetry_range_mid(it->second.rc.data[1], 50, neutralZ[0]) ||
                                         !in_symmetry_range_mid(it->second.rc.data[2], 50, neutralZ[0]) || !in_symmetry_range_mid(it->second.rc.data[3], 50, neutralZ[0]);
                    if (rc_has_action)
                    {
                        uint8_t tgt_prio = getRcPriority(ind_cfg);
                        if (!cur_rc_has_action)
                        {
                            // 当前接收机无动作，其他有动作 → 切换
                            if (!(common_cfg.configs[0] & RC_COMMON_CONFIG_SW_PRIO) || tgt_prio >= sw_rc_prio)
                            {
                                sw_rc_prio = tgt_prio;
                                sw_rc_sub_prio = getRcSubPriority(ind_cfg);
                                switch_it = it;
                            }
                        }
                        else
                        {
                            // 当前接收机有动作，其他也有动作 → 优先级高者切换
                            if (tgt_prio > (uint8_t) sw_rc_prio)
                            {
                                sw_rc_prio = tgt_prio;
                                sw_rc_sub_prio = getRcSubPriority(ind_cfg);
                                switch_it = it;
                            }
                            else if (tgt_prio == (uint8_t) current_rc_prio)
                            { // 双方都有动作且主优先级相同 → 用次优先级比较，高者优先，相同则不切换
                                uint8_t tgt_sub_prio = getRcSubPriority(ind_cfg);
                                if (tgt_sub_prio > (uint8_t) sw_rc_sub_prio)
                                {
                                    sw_rc_prio = tgt_prio;
                                    sw_rc_sub_prio = tgt_sub_prio;
                                    switch_it = it;
                                }
                            }
                        }
                    }
                }
            }

            xSemaphoreGive(it->second.mutex);
        }
    }

    // 如果当前接收机不可用且无可切换的可用接收机
    // 寻找优先级最高的已连接接收机
    if (!cur_rc_available && switch_it == Receivers.end())
    {
        // 无当前接收机或需要根据优先级切换时才重新寻找，否则保持当前接收机
        if (cur_it == Receivers.end() || (common_cfg.configs[0] & RC_COMMON_CONFIG_SW_PRIO))
        {
            sw_rc_prio = current_rc_prio;
            for (auto it = Receivers.begin(); it != Receivers.end(); ++it)
            {
                // 获取接收机个体配置
                RC_IND_CONFIG ind_cfg = {0};
                ReadParamGroup(SName("RCI_") + it->first, (uint64_t *) &ind_cfg, 0);

                if (xSemaphoreTake(it->second.mutex, portMAX_DELAY))
                {
                    if (it->second.rc.connected)
                    {
                        if (!(common_cfg.configs[0] & RC_COMMON_CONFIG_SW_PRIO))
                        {
                            sw_rc_prio = getRcPriority(ind_cfg);
                            switch_it = it;
                            xSemaphoreGive(it->second.mutex);
                            break;
                        }
                        else if (getRcPriority(ind_cfg) > sw_rc_prio)
                        {
                            sw_rc_prio = getRcPriority(ind_cfg);
                            switch_it = it;
                        }
                    }
                    xSemaphoreGive(it->second.mutex);
                }
            }
        }
    }

    // 根据switch_it进行接收机切换
    if (switch_it != Receivers.end() && switch_it != cur_it)
    {
        rc_it = switch_it;
        cur_it = rc_it;
    }
    xSemaphoreGive(rc_it_Mutex);

    // 返回结果
    if (cur_it != Receivers.end())
    {
        if (xSemaphoreTake(cur_it->second.mutex, portMAX_DELAY))
        {
            if (rc)
                *rc = cur_it->second.rc;
            if (name)
                *name = cur_it->first;
            xSemaphoreGive(cur_it->second.mutex);
            return true;
        }
    }
    if (rc)
        rc->connected = rc->available = false;
    return false;
}

// 判断是否为远程接收机类型
static inline bool isRemoteReceiverType(RC_CLASS rc_type)
{
    return rc_type == RC_CLASS::MainRemoteRc || rc_type == RC_CLASS::SubRemoteRc;
}

static SemaphoreHandle_t rmrc_it_Mutex = xSemaphoreCreateMutex();
static map<SName, ReceiverSync>::iterator rmrc_it = Receivers.end();
/**
 * @brief 获取当前可用远程接收机（仅通过优先级切换）
 * @param rc 获取的接收机
 * @param name 获取到的接收机名称
 * @param TIMEOUT 超时时间
 */
bool getRemoteReceiver(Receiver *rc, SName *name, double TIMEOUT)
{
    // 初始化完成后才可以获取接收机
    if (getInitializationCompleted() == false)
    {
        rc->available = rc->connected = false;
        return false;
    }

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    // 读取RC_COMMON_CONFIG配置
    RC_COMMON_CONFIG common_cfg = {0};
    bool common_cfg_new = false;
    ReadParamGroup(SName("RC_COMMON"), (uint64_t *) &common_cfg, &common_cfg_new);

    // 获取当前接收机指针
    map<SName, ReceiverSync>::iterator cur_it;
    if (xSemaphoreTake(rmrc_it_Mutex, TIMEOUT_Ticks))
    {
        cur_it = rmrc_it;
    }
    else
    {
        rc->available = rc->connected = false;
        return false;
    }

    // 当前接收机是否可用
    bool cur_rc_available = false;
    // 当前接收机优先级
    int8_t current_rc_prio = -1;
    // 检查当前接收机是否可用
    if (cur_it != Receivers.end())
    {
        if (xSemaphoreTake(cur_it->second.mutex, portMAX_DELAY))
        {
            // 判断连接状态
            if (cur_it->second.rc.last_update_time.get_pass_time() > 2.0f)
            { // 接收机超时未更新，认为已断开
                cur_it->second.rc.connected = cur_it->second.rc.available = false;
            }
            if (cur_it->second.rc.connected == false || !isRemoteReceiverType(cur_it->second.rc.rc_type))
            {
                xSemaphoreGive(cur_it->second.mutex);
                rmrc_it = cur_it = Receivers.end();
            }
            else
            { // 接收机已连接
                // 获取当前接收机个体配置
                RC_IND_CONFIG cur_ind_cfg = {0};
                ReadParamGroup(SName("RCI_") + cur_it->first, (uint64_t *) &cur_ind_cfg, 0);

                // 获取当前接收机优先级
                current_rc_prio = getRcPriority(cur_ind_cfg);

                if (cur_it->second.rc.available)
                { // 当前接收机可用
                    cur_rc_available = true;
                }

                xSemaphoreGive(cur_it->second.mutex);
            }
        }
    }

    // 检测接收机切换（仅通过优先级）
    int8_t sw_rc_prio = -1;
    if (cur_rc_available)
        sw_rc_prio = current_rc_prio;
    map<SName, ReceiverSync>::iterator switch_it = Receivers.end();

    // 仅当RMRC_COMMON_CONFIG_SW_PRIO开启或当前接收机不可用时才寻找更高优先级接收机
    if ((common_cfg.configs[0] & RMRC_COMMON_CONFIG_SW_PRIO) || !cur_rc_available)
    {
        for (auto it = Receivers.begin(); it != Receivers.end(); ++it)
        {
            // 获取接收机个体配置
            RC_IND_CONFIG ind_cfg = {0};
            ReadParamGroup(SName("RCI_") + it->first, (uint64_t *) &ind_cfg, 0);

            if (xSemaphoreTake(it->second.mutex, portMAX_DELAY))
            {
                // 判断连接状态
                if (it->second.rc.last_update_time.get_pass_time() > 2.0f)
                { // 接收机超时未更新，认为已断开
                    it->second.rc.connected = it->second.rc.available = false;
                }

                // 仅考虑远程接收机类型
                if (!isRemoteReceiverType(it->second.rc.rc_type))
                {
                    xSemaphoreGive(it->second.mutex);
                    continue;
                }

                // 可用的接收机进行切换判断
                if (it->second.rc.available)
                {
                    uint8_t prio = getRcPriority(ind_cfg);
                    if (prio > sw_rc_prio)
                    {
                        sw_rc_prio = prio;
                        switch_it = it;
                    }
                }

                xSemaphoreGive(it->second.mutex);
            }
        }
    }

    // 如果当前接收机不可用且无可切换的可用接收机
    // 寻找优先级最高的已连接远程接收机
    if (!cur_rc_available && switch_it == Receivers.end())
    {
        sw_rc_prio = -1;
        for (auto it = Receivers.begin(); it != Receivers.end(); ++it)
        {
            // 获取接收机个体配置
            RC_IND_CONFIG ind_cfg = {0};
            ReadParamGroup(SName("RCI_") + it->first, (uint64_t *) &ind_cfg, 0);

            if (xSemaphoreTake(it->second.mutex, portMAX_DELAY))
            {
                // 仅考虑远程接收机类型
                if (!isRemoteReceiverType(it->second.rc.rc_type))
                {
                    xSemaphoreGive(it->second.mutex);
                    continue;
                }

                if (it->second.rc.connected)
                {
                    uint8_t prio = getRcPriority(ind_cfg);
                    if (prio > sw_rc_prio)
                    {
                        sw_rc_prio = prio;
                        switch_it = it;
                    }
                }
                xSemaphoreGive(it->second.mutex);
            }
        }
    }

    // 根据switch_it进行接收机切换
    if (switch_it != Receivers.end() && switch_it != cur_it)
    {
        rmrc_it = switch_it;
        cur_it = rmrc_it;
    }
    xSemaphoreGive(rmrc_it_Mutex);

    // 返回结果
    if (cur_it != Receivers.end())
    {
        if (xSemaphoreTake(cur_it->second.mutex, portMAX_DELAY))
        {
            if (rc)
                *rc = cur_it->second.rc;
            if (name)
                *name = cur_it->first;
            xSemaphoreGive(cur_it->second.mutex);
            return true;
        }
    }
    if (rc)
        rc->connected = rc->available = false;
    return false;
}

/**
 * @brief 获取指定接收机
 * @param name 接收机名称
 * @param rc 获取的接收机
 * @param TIMEOUT 超时时间
 */
bool getReceiver(SName name, Receiver *rc, double TIMEOUT)
{
    // 初始化完成后才可以获取接收机
    if (getInitializationCompleted() == false)
        return false;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    map<SName, ReceiverSync>::iterator it = Receivers.find(name);
    if (it != Receivers.end())
    {
        if (xSemaphoreTake(it->second.mutex, TIMEOUT_Ticks))
        {
            if (it->second.rc.last_update_time.get_pass_time() > 2.0f)
            {
                it->second.rc.connected = false;
                xSemaphoreGive(it->second.mutex);
                return false;
            }

            *rc = it->second.rc;
            xSemaphoreGive(it->second.mutex);
            return true;
        }
        ++it;
    }
    return false;
}
/*获取接收机*/