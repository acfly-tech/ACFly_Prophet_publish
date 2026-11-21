#include "drv_extIO_Expansion_C.hpp"

#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "MavlinkCMDProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "StorageSystem.hpp"
#include "drv_ExtIO.hpp"
#include "drv_PWMOut.hpp"
#include "drv_can.hpp"
#include "task.h"
#include <string.h>

struct DriverInfo
{
    CanMailBox *mail_box;
    uint32_t sensor_key;
};

#define window_size 25
typedef struct
{                              // 滑动窗口滤波器结构体定义
    float buffer[window_size]; // 存储窗口内的采样值
    int index;                 // 当前写入位置索引
    int count;                 // 当前窗口内有效值的个数
    float sum;                 // 当前窗口内值的总和，用于快速计算平均值
} SlidingWindowFilter;

static void init_slidingwindow_filter(SlidingWindowFilter *filter, float initial_value)
{
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0f;

    for (int i = 0; i < window_size; i++)
    {
        filter->buffer[i] = initial_value;
        filter->sum += initial_value;
    }
    filter->count = window_size;
}

static float update_slidingwindow_filter(SlidingWindowFilter *filter, float new_value)
{
    filter->sum -= filter->buffer[filter->index];
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;

    filter->index = (filter->index + 1) % window_size;

    if (filter->count < window_size)
    {
        filter->count++;
    }

    if (filter->count <= 0)
        return 0.0f;

    return filter->sum / filter->count;
}

static void extIO_Expansion_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    float anglog_ch[4] = {0};
    float tmr_ch[5] = {0};

    // 获取校准参数
    FLOWMETER_CONFIG FlowMeterConfig = {0};
    ReadParamGroup("Exp_board", (uint64_t *) &FlowMeterConfig, 0);

    SlidingWindowFilter WeightFilter;
    init_slidingwindow_filter(&WeightFilter, 0);

    float Flowmeter = 0;
    float Capacity = 0;
    float Weight_Raw = 0;
    float Weight_Filter = 0;

    TIME heartBeat_TIME(false);

    TIME Log_TIME(false);
    while (1)
    {
        if (heartBeat_TIME.get_pass_time() > 1)
        { // 获取校准参数
            heartBeat_TIME = TIME::now();
            ReadParamGroup("Exp_board", (uint64_t *) &FlowMeterConfig, 0);
        }

        CanPacket mail;
        if (driver_info.mail_box->receiveMail(&mail, 0.01))
        {
            switch (mail.Identifier)
            {
                case 0xaacc120:
                {
                    if (mail.DataLength < 8)
                        break;
                    anglog_ch[0] = (*(float *) &mail.data[0] + FlowMeterConfig.anglog1_offset[0]) * FlowMeterConfig.anglog1_kp[0];
                    anglog_ch[1] = (*(float *) &mail.data[4] + FlowMeterConfig.anglog2_offset[0]) * FlowMeterConfig.anglog2_kp[0];
                    break;
                }
                case 0xaacc121:
                {
                    if (mail.DataLength < 8)
                        break;
                    anglog_ch[2] = (*(float *) &mail.data[0] + FlowMeterConfig.anglog3_offset[0]) * FlowMeterConfig.anglog3_kp[0];
                    anglog_ch[3] = (*(float *) &mail.data[4] + FlowMeterConfig.anglog4_offset[0]) * FlowMeterConfig.anglog4_kp[0];
                    break;
                }
                case 0xaacc122:
                {
                    if (mail.DataLength < 8)
                        break;
                    tmr_ch[0] = (*(float *) &mail.data[0] + FlowMeterConfig.tmr_ch1_offset[0]) * FlowMeterConfig.tmr_ch1_kp[0];
                    tmr_ch[1] = (*(float *) &mail.data[4] + FlowMeterConfig.tmr_ch2_offset[0]) * FlowMeterConfig.tmr_ch2_kp[0];
                    break;
                }
                case 0xaacc123:
                {
                    if (mail.DataLength < 8)
                        break;
                    tmr_ch[2] = (*(float *) &mail.data[0] + FlowMeterConfig.tmr_ch3_offset[0]) * FlowMeterConfig.tmr_ch3_kp[0];
                    tmr_ch[3] = (*(float *) &mail.data[4] + FlowMeterConfig.tmr_ch4_offset[0]) * FlowMeterConfig.tmr_ch4_kp[0];
                    break;
                }
                case 0xaacc124:
                {
                    if (mail.DataLength < 8)
                        break;
                    tmr_ch[4] = (*(float *) &mail.data[0] + FlowMeterConfig.tmr_ch5_offset[0]) * FlowMeterConfig.tmr_ch5_kp[0];
                    Weight_Raw = (*(float *) &mail.data[4] + FlowMeterConfig.weight_offset[0]) * FlowMeterConfig.weight_kp[0];
                    Weight_Filter = update_slidingwindow_filter(&WeightFilter, -Weight_Raw / 1000.0f);
                    break;
                }
            }

            if (mail.Identifier <= 0xaacc124 && mail.Identifier >= 0xaacc120 && Log_TIME.get_pass_time() > 0.5)
            {
                Log_TIME = TIME::now();

                Flowmeter = tmr_ch[0];
                Capacity = anglog_ch[0];

                bool unLocked;
                is_Attitude_Control_Enabled(&unLocked);
                if (unLocked)
                {
                    double exps[3];
                    exps[0] = Flowmeter;
                    exps[1] = Capacity;
                    exps[2] = Weight_Filter;
                    SDLog_Msg_DebugVect("exps", exps, 3);
                }

                uint8_t payload[43];
                memset(payload, 0, 43);
                uint16_t message_type = 65000; // need >=65000
                uint8_t availbale = true;
                char nameChar[13] = {"称重计"};
                char uint1[6] = {"L/Min"};
                char uint2[6] = {"L"};
                char uint3[6] = {"kg"};
                float value1 = 0, value2 = 0, value3 = 0;

                value1 = Flowmeter;
                value2 = Capacity;
                value3 = Weight_Filter;
                memcpy(&payload[0], &nameChar[0], 13); // name
                memcpy(&payload[13], &availbale, 1);   // availbale
                memcpy(&payload[14], &value1, 4);      // value1
                memcpy(&payload[18], &uint1, 5);       // uint1
                memcpy(&payload[23], &value2, 4);      // value2
                memcpy(&payload[27], &uint2, 5);       // uint2
                memcpy(&payload[32], &value3, 4);      // value3
                memcpy(&payload[36], &uint3, 5);       // uint3

                for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                {
                    const Port *port = get_CommuPort(i);
                    if (port && port->write != 0)
                    {
                        if (mavlink_lock_chan(i, 0.01))
                        {
                            mavlink_message_t msg_sd;
                            mavlink_msg_component_extension43_pack_chan(get_CommulinkSysId(),  // system id
                                                                        get_CommulinkCompId(), // component id
                                                                        i,                     // chan
                                                                        &msg_sd,
                                                                        0,                    // target_network
                                                                        get_CommulinkSysId(), // target system
                                                                        MAV_COMP_ID_USER6,    // target component
                                                                        message_type,         // message_type
                                                                        payload);
                            mavlink_send_msg(port, &msg_sd, 0.01, 0.01);
                            mavlink_unlock_chan(i);
                        }
                    }
                }
            }
        }
    }
}

static bool extIO_Expansion_DriverInit()
{
    return true;
}
static bool extIO_Expansion_DriverRun(CanMailBox *mailBox)
{
    CanId Ids[5];

    // 发送实时信息包
    Ids[0].Identifier = 0xaacc120;
    Ids[0].IdType = 1;
    Ids[1].Identifier = 0xaacc121;
    Ids[1].IdType = 1;
    Ids[2].Identifier = 0xaacc122;
    Ids[2].IdType = 1;
    Ids[3].Identifier = 0xaacc123;
    Ids[3].IdType = 1;
    Ids[4].Identifier = 0xaacc124;
    Ids[4].IdType = 1;
    mailBox->init(10, Ids, 5);

    DriverInfo *driver_info = new DriverInfo;
    driver_info->mail_box = mailBox;
    xTaskCreate(extIO_Expansion_Server, "extIO_Expansion", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_extIO_Expansion_C()
{
    FLOWMETER_CONFIG init_cfg;
    init_cfg.anglog1_offset[0] = 0;
    init_cfg.anglog1_kp[0] = 1.0;
    init_cfg.anglog2_offset[0] = 0;
    init_cfg.anglog2_kp[0] = 1.0;
    init_cfg.anglog3_offset[0] = 0;
    init_cfg.anglog3_kp[0] = 1.0;
    init_cfg.anglog4_offset[0] = 0;
    init_cfg.anglog4_kp[0] = 1.0;
    init_cfg.tmr_ch1_offset[0] = 0;
    init_cfg.tmr_ch1_kp[0] = 1.0;
    init_cfg.tmr_ch2_offset[0] = 0;
    init_cfg.tmr_ch2_kp[0] = 1.0;
    init_cfg.tmr_ch3_offset[0] = 0;
    init_cfg.tmr_ch3_kp[0] = 1.0;
    init_cfg.tmr_ch4_offset[0] = 0;
    init_cfg.tmr_ch4_kp[0] = 1.0;
    init_cfg.tmr_ch5_offset[0] = 0;
    init_cfg.tmr_ch5_kp[0] = 1.0;
    init_cfg.weight_offset[0] = 0;
    init_cfg.weight_kp[0] = 1.0;

    MAV_PARAM_TYPE param_types[] = {
        MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32,
    };
    SName param_names[] = {
        "Exp_a1_offset", "Exp_a1_kp", "Exp_a2_offset", "Exp_a2_kp", "Exp_a3_offset", "Exp_a3_kp", "Exp_a4_offset", "Exp_a4_kp", "Exp_t1_offset",     "Exp_t1_kp",
        "Exp_t2_offset", "Exp_t2_kp", "Exp_t3_offset", "Exp_t3_kp", "Exp_t4_offset", "Exp_t4_kp", "Exp_t5_offset", "Exp_t5_kp", "Exp_weight_offset", "Exp_weight_kp",
    };
    InitialParametersInit((uint64_t *) &init_cfg, param_types, param_names, sizeof(init_cfg) / 8);
    ParamGroupRegister("Exp_board", 3, sizeof(init_cfg) / 8, param_types, param_names, (uint64_t *) &init_cfg);

    CanFunc_Register(11, extIO_Expansion_DriverInit, extIO_Expansion_DriverRun);
}