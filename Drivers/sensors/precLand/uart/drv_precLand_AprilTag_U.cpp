#include "Basic.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "task.h"

#include "Filters_LP.hpp"
#include "StorageSystem.hpp"
#include "followTarget.hpp"
#include "precLand.hpp"

struct DriverInfo
{
    uint32_t param;
    Port port;
};

typedef struct
{
    float ID;
    float Tx;
    float Ty;
    float Tz;
    float Rz;
    //	float Rx;
    //	float Ry;
} __AC_PACKED _AprilTag;
double apriltag_x = 0;
double apriltag_y = 0;

static void precLand_AprilTag_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    /*状态机变量*/
    static uint8_t rc_step = 0;
    static uint8_t sumA;
    static uint8_t sumB;

    _AprilTag AprilTag;
    /*状态机变量*/

    // 注册精准降落传感器
    uint8_t prec_ind = 0;
    uint8_t param_prec_ind = driver_info.param;
    if (param_prec_ind >= 0 && param_prec_ind < MAX_PRECLAND_SENSORS)
        prec_ind = param_prec_ind;
    uint32_t precKey = 0;
    do
    {
        precKey = precLandSensor_register(prec_ind);
        os_delay(1.0);
    } while (precKey == 0);

    TIME lastGcsSdTIME(false);

    while (1)
    {
        uint8_t r_data;
        extern float debug_test[30];
        debug_test[13] += 1;
        if (driver_info.port.read(&r_data, 1, 2, 0.5))
        {
            if (rc_step == 0)
            {
                if (r_data == 0xFE)
                {
                    ++rc_step;
                }
                else
                {
                    rc_step = 0;
                }
            }
            else if (rc_step == 1)
            {
                if (r_data == 0x04)
                {
                    ++rc_step;
                    sumA = sumB = 0;
                }
                else
                {
                    rc_step = 0;
                }
            }
            else if (rc_step < 22)
            {
                ((unsigned char *) &AprilTag)[rc_step - 2] = r_data;
                sumA += r_data;
                sumB += sumA;
                ++rc_step;
            }
            else if (rc_step == 22)
            {
                if (r_data == sumA)
                {
                    ++rc_step;
                }
                else
                {
                    rc_step = 0;
                }
            }
            else
            {
                if (r_data == sumB)
                {
                    debug_test[14] += 1;
                    if (AprilTag.ID == 4 || AprilTag.ID == 14 || AprilTag.ID == 96)
                    {
                        debug_test[15] += 1;

                        PrecLandInf precLandInf;
                        ReadParamGroup("precLand", (uint64_t *) &precLandInf, 0);

                        Quaternion quat;
                        get_history_AirframeQuatY(&quat, 0.05);

                        vector3<double> position;
                        get_Position_Ctrl(&position);
                        double roll = quat.getRoll();
                        double pitch = quat.getPitch();
                        double height = AprilTag.Tz * 1.0;
                        // double height = position.z * 1.0;

                        //						vector3<double> position;
                        //						get_Position_Ctrl(&position);
                        //						if(position.z > 500){
                        //							height = position.z;
                        //						}else{
                        //							height = AprilTag.Tz * 1.0;
                        //						}

                        double fix_y = height * sin(pitch - degree2rad(precLandInf.pitch_offset[0])) * 1.0;
                        double fix_x = height * sin(roll - degree2rad(precLandInf.roll_offset[0])) * 1.0;
                        //						fix_y = 0;
                        //						fix_x = 0;
                        vector3<double> bodypos;
                        vector3<double> pos;
                        bodypos.x = (AprilTag.Ty - fix_y);
                        bodypos.y = -(AprilTag.Tx - fix_x);
                        bodypos.z = AprilTag.Tz;

                        //						static Filter_LP_IIR_1 apriltag_position_lp[2];
                        //						apriltag_position_lp[0].set_cutoff_frequency(25, 1);
                        //						apriltag_position_lp[1].set_cutoff_frequency(25, 1);
                        ////						apriltag_position_lp[0].set_value(bodypos.x);
                        ////						apriltag_position_lp[1].set_value(bodypos.y);
                        //
                        //						apriltag_position_lp[0].run(bodypos.x);
                        //						apriltag_position_lp[1].run(bodypos.y);
                        //
                        //						bodypos.x = apriltag_position_lp[0].get_result();
                        //						bodypos.y = apriltag_position_lp[1].get_result();

                        get_AirframeY_quat(&quat);
                        double yaw = quat.getYaw();
                        pos.x = BodyHeading2ENU_x(bodypos.x, bodypos.y, sin(yaw), cos(yaw));
                        pos.y = BodyHeading2ENU_y(bodypos.x, bodypos.y, sin(yaw), cos(yaw));
                        pos.z = -AprilTag.Tz;

                        precLandSensor_updateENU(prec_ind, precKey, pos, yaw + degree2rad(-AprilTag.Rz), true, true, true);

                        //						extern float debug_test[30];
                        //						debug_test[21] = bodypos.x;
                        //						debug_test[22] = bodypos.y;
                        //						debug_test[24] = height;

                        apriltag_x = bodypos.x;
                        apriltag_y = bodypos.y;

                        //						debug_test[16] = AprilTag.Tx;
                        //						debug_test[17] = AprilTag.Ty;
                        //						debug_test[18] = AprilTag.Ry;

                        double vvv[12];
                        vvv[0] = pos.x;
                        vvv[1] = pos.y;
                        vvv[2] = degree2rad(AprilTag.Rz);
                        vvv[3] = AprilTag.Tz;

                        vvv[4] = bodypos.x;
                        vvv[5] = bodypos.y;
                        vvv[6] = height;
                        vvv[7] = fix_y;
                        vvv[8] = fix_x;
                        vvv[9] = AprilTag.ID;
                        vvv[10] = AprilTag.Tx;
                        vvv[11] = AprilTag.Ty;
                        SDLog_Msg_DebugVect("apriltag", vvv, 12);

                        if (lastGcsSdTIME.get_pass_time() > 1.5)
                        {
                            lastGcsSdTIME = TIME::now();

                            uint8_t payload[43];
                            memset(payload, 0, 43);
                            uint16_t message_type = 65000; // need >=65000
                            uint8_t availbale = true;
                            char nameChar[13] = {"二维码"};
                            char uint1[6] = {"前"};
                            char uint2[6] = {"左"};
                            char uint3[6] = {"下"};
                            float value1 = 0, value2 = 0, value3 = 0;
                            value1 = bodypos.x;
                            value2 = bodypos.y;
                            value3 = height;
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
                    else
                    {
                        if (lastGcsSdTIME.get_pass_time() > 1.5)
                        {
                            lastGcsSdTIME = TIME::now();

                            uint8_t payload[43];
                            memset(payload, 0, 43);
                            uint16_t message_type = 65000; // need >=65000
                            uint8_t availbale = true;
                            char nameChar[13] = {"二维码"};
                            char uint1[6] = {"前"};
                            char uint2[6] = {"左"};
                            char uint3[6] = {"下"};
                            float value1 = 0, value2 = 0, value3 = 0;
                            value1 = 0;
                            value2 = 0;
                            value3 = 0;
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

                rc_step = 0;
            }
        }
    }
}

static bool precLand_AprilTag_DriverInit(Port port, uint32_t param)
{
    // 波特率115200
    port.SetBaudRate(115200, 2, 2);
    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(precLand_AprilTag_Server, "precLand_AprilTag", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_precLand_AprilTag_U()
{
    PortFunc_Register(122, precLand_AprilTag_DriverInit);
}
