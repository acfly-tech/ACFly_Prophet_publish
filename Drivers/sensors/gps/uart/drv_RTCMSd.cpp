#include "drv_RTCMSd.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "StorageSystem.hpp"
#include "task.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

struct DriverInfo
{
    uint32_t param;
    Port port;
};

static inline void send_rtcm_msg(mavlink_gps_rtcm_data_t *mavlinkRtcmData)
{
    if (mavlinkRtcmData == 0)
        return;

    // if( mavlink_lock_chan( chan, 1 ) )
    //{
    //	mavlink_message_t msg_sd;
    //	mavlink_msg_gps_rtcm_data_encode_chan(
    //		get_CommulinkSysId() , //system id
    //		get_CommulinkCompId() , //component id
    //		chan , //chan
    //		&msg_sd,
    //		mavlinkRtcmData);

    //	mavlink_msg_to_send_buffer(
    //		port->write,
    //		port->lock,
    //		port->unlock,
    //		&msg_sd, 1, 1);

    //	mavlink_unlock_chan(chan);
    //}

    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    {
        const Port *port = get_CommuPort(i);
        if (port && port->write != 0)
        {
            if (mavlink_lock_chan(i, 0.01))
            {
                mavlink_message_t msg_sd;
                mavlink_msg_gps_rtcm_data_encode_chan(get_CommulinkSysId(),  // system id
                                                      get_CommulinkCompId(), // component id
                                                      i,                     // chan
                                                      &msg_sd, mavlinkRtcmData);

                mavlink_send_msg(port, &msg_sd, 1, 1);

                mavlink_unlock_chan(i);
            }
        }
    }
}

static void RTK_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    driver_info.port.SetBaudRate(460800, 3, 0.1);

    uint32_t rtcm_start = 0;
    uint32_t rtcm_state = 0;
    uint32_t rtcm_msg_length = 0;
    uint32_t rtcm_sequenceId = 0;
    __attribute__((aligned(4))) uint8_t rtcm_datas[2048];

    while (1)
    {
        uint8_t r_data;
        if (driver_info.port.read(&r_data, 1, 2, 0.1))
        {
            uint8_t *cDatas = &rtcm_datas[rtcm_start];
            uint32_t validSize = sizeof(rtcm_datas) - rtcm_start;
            if (rtcm_state > validSize - 1)
                rtcm_state = 0;
            cDatas[rtcm_state++] = r_data;

            if (rtcm_state == 1)
            { // 每帧 RTCM 数据的数据头固定为 1101 0011 0000 00
                if (r_data != 0xD3)
                    rtcm_state = 0;
            }
            else if (rtcm_state < 4)
            {
                if (rtcm_state == 3)
                {
                    if ((cDatas[1] & 0b11111100) != 0)
                        rtcm_state = 0;
                    else
                    {
                        // 有效数据长度
                        rtcm_msg_length = (uint16_t) (((cDatas[1] & 3) << 8) | (cDatas[2]));
                        if (rtcm_msg_length + 6 > validSize)
                        { //  包大小超过rtcm_datas大小
                            rtcm_state = 0;
                        }
                    }
                }
            }
            else if (rtcm_state < rtcm_msg_length + 7)
            {
                if (rtcm_state == rtcm_msg_length + 6)
                { // 接受到完整的一帧数据，开始转发
                    uint16_t msg_id = (cDatas[3] << 4) | (cDatas[4] >> 4);

                    uint32_t rtcmTotal = rtcm_start + rtcm_state;
                    if (1)
                    // if( (rtcmTotal>MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*3) || (rtcmTotal%MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN>100) )
                    {
                        mavlink_gps_rtcm_data_t mavlinkRtcmData;
                        if (rtcmTotal <= MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN)
                        {
                            mavlinkRtcmData.len = rtcmTotal;
                            mavlinkRtcmData.flags = (rtcm_sequenceId & 0x1F) << 3;
                            memcpy(&mavlinkRtcmData.data, rtcm_datas, rtcmTotal);
                            if (MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN > rtcmTotal)
                                memset(&mavlinkRtcmData.data[rtcmTotal], 0, MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN - rtcmTotal);
                            send_rtcm_msg(&mavlinkRtcmData);
                        }
                        else
                        {
                            uint8_t fragmentId = 0;
                            int start = 0;
                            while (start < rtcmTotal)
                            {
                                int length = (rtcmTotal - start > MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) ? MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN : rtcmTotal - start;
                                mavlinkRtcmData.flags = 1;                              // LSB set indicates message is fragmented
                                mavlinkRtcmData.flags |= (fragmentId++) << 1;           // Next 2 bits are fragment id
                                mavlinkRtcmData.flags |= (rtcm_sequenceId & 0x1F) << 3; // Next 5 bits are sequence id
                                mavlinkRtcmData.len = length;
                                memcpy(&mavlinkRtcmData.data, &rtcm_datas[start], length);
                                if (MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN > length)
                                    memset(&mavlinkRtcmData.data[length], 0, MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN - length);
                                send_rtcm_msg(&mavlinkRtcmData);
                                start += length;
                            }
                        }
                        rtcm_start = 0;
                    }
                    else
                        rtcm_start += rtcm_state;

                    rtcm_sequenceId++;
                    if (rtcm_sequenceId == (1 << 5))
                        rtcm_sequenceId = 0;
                    rtcm_state = 0;
                }
            }
        }
    }
}

static bool RTK_DriverInit(Port port, uint32_t param)
{
    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(RTK_Server, "RTCM", 4000, driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_RTCMSd()
{
    PortFunc_Register(23, RTK_DriverInit);
}
