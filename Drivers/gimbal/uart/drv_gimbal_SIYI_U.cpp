#include "drv_gimbal_SIYI_U.hpp"
#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "MavlinkCMDProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "drv_CRC.hpp"
#include "drv_PWMOut.hpp"
#include "task.h"

static uint16_t componentID = 0;

struct DriverInfo
{
    uint32_t param;
    Port port;
};

// 发送 MAV_COMP_ID_CAMERA 心跳包
static inline void send_heartbeat()
{
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    {
        const Port *port = get_CommuPort(i);
        mavlink_message_t msg_sd;
        if (port && port->write)
        {
            if (mavlink_lock_chan(i, 0.01))
            {
                mavlink_msg_heartbeat_pack_chan(get_CommulinkSysId(), // system id
                                                componentID,          // component id
                                                i,                    // chan
                                                &msg_sd,
                                                MAV_TYPE_CAMERA,   // type
                                                MAV_AUTOPILOT_PX4, // autopilot
                                                0,                 // base mode
                                                0,                 // custom mode
                                                0                  // sys status
                );
                mavlink_send_msg(port, &msg_sd);
                mavlink_unlock_chan(i);
            }
        }
    }
}

// 发送 camera_information
static inline void send_camera_information()
{
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    {
        const Port *port = get_CommuPort(i);
        mavlink_message_t msg_sd;
        if (port && port->write)
        {
            if (mavlink_lock_chan(i, 0.01))
            {
                uint8_t vendor_name[32] = {"SIYI"};
                uint8_t model_name[32] = {"SIYI"};
                uint32_t flags = CAMERA_CAP_FLAGS_HAS_GIMBAL_PITCH | CAMERA_CAP_FLAGS_HAS_GIMBAL_YAW | CAMERA_CAP_FLAGS_HAS_GIMBAL_CENTER | CAMERA_CAP_FLAGS_HAS_GIMBAL_DOWN |
                                 // CAMERA_CAP_FLAGS_GIMBAL_CTRL_SEND_WHEN_CHANGED|
                                 CAMERA_CAP_FLAGS_CAPTURE_IMAGE | CAMERA_CAP_FLAGS_CAPTURE_VIDEO | CAMERA_CAP_FLAGS_HAS_DOUBLE_VIDEO | CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM | CAMERA_CAP_FLAGS_HAS_MODES |
                                 CAMERA_CAP_FLAGS_HAS_TRACKING_POINT |
                                 // CAMERA_CAP_FLAGS_TARGET_DETECTION|
                                 // CAMERA_CAP_FLAGS_SHOW_CAMERA_AREA|
                                 // CAMERA_CAP_FLAGS_TARGET_SELECTION_RECTANGLE|
                                 CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE | CAMERA_CAP_FLAGS_HAS_LASER | CAMERA_CAP_FLAGS_HAS_OSD | CAMERA_CAP_FLAGS_HAS_ACK;

                mavlink_msg_camera_information_pack_chan(get_CommulinkSysId(), // system id
                                                         componentID,          // component id
                                                         i,                    // chan
                                                         &msg_sd,
                                                         TIME::get_System_Run_Time() * 1e6, // time_boot_ms
                                                         vendor_name,                       // vendor_name
                                                         model_name,                        // model_name
                                                         0,                                 // firmware_version
                                                         0,                                 // focal_length [mm]
                                                         0,                                 // sensor_size_h [mm]
                                                         0,                                 // sensor_size_v [mm]
                                                         0,                                 // resolution_h [pix]
                                                         0,                                 // resolution_v [pix]
                                                         0,                                 // lens_id
                                                         flags,                             // flags
                                                         0,                                 // cam_definition_version
                                                         0,                                 // cam_definition_uri
                                                         0,                                 // gimbal_device_id
                                                         0                                  // camera_device_id
                );
                mavlink_send_msg(port, &msg_sd, 0.01, 0.01);
                mavlink_unlock_chan(i);
            }
        }
    }
}

static uint32_t target_system = 0;
static uint32_t target_component = 0;
static inline void send_ack(uint32_t command, bool res)
{
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    {
        const Port *port = get_CommuPort(i);
        mavlink_message_t msg_sd;
        if (port && port->write)
        {
            if (mavlink_lock_chan(i, 0.01))
            {
                mavlink_msg_command_ack_pack_chan(get_CommulinkSysId(), // system id
                                                  target_component,     // component id
                                                  i, &msg_sd,
                                                  command, // command
                                                  // res ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED ,	//result
                                                  MAV_RESULT_ACCEPTED,
                                                  100,             // progress
                                                  0,               // param2
                                                  target_system,   // target system
                                                  target_component // target component
                );
                mavlink_send_msg(port, &msg_sd, 1, 1);
                mavlink_unlock_chan(i);
            }
        }
    }
}

// 发送无人机状态(camera_status报文)到MAVLINK端口
static inline void send_camera_status_to_mavlink_ports(mavlink_camera_status_t *status)
{
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    {
        const Port *port = get_CommuPort(i);
        mavlink_message_t msg_sd;
        if (port && port->write)
        {
            if (mavlink_lock_chan(i, 0.2))
            {
                mavlink_msg_camera_status_encode_chan(get_CommulinkSysId(), componentID, i, &msg_sd, status);
                mavlink_send_msg(port, &msg_sd, 1, 1);
                mavlink_unlock_chan(i);
            }
        }
    }
}

// 发送空回复(camera_status报文)到MAVLINK端口
static inline void send_camera_status_to_mavlink_ports(uint8_t event_id)
{
    mavlink_camera_status_t camera_status;
    camera_status.target_system = 0;
    camera_status.cam_idx = componentID;
    camera_status.img_idx = 0;
    camera_status.event_id = event_id;
    camera_status.p1 = 0;
    camera_status.p2 = 0;
    camera_status.p3 = 0;
    camera_status.p4 = 0;
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    {
        const Port *port = get_CommuPort(i);
        mavlink_message_t msg_sd;
        if (port && port->write)
        {
            if (mavlink_lock_chan(i, 0.2))
            {
                mavlink_msg_camera_status_encode_chan(get_CommulinkSysId(), componentID, i, &msg_sd, &camera_status);
                mavlink_send_msg(port, &msg_sd, 1, 1);
                mavlink_unlock_chan(i);
            }
        }
    }
}
// 发送云台的状态字符串信息(statustext报文)给MAVLINK端口
static void sendStatusTextMsgToMavlinkAllPorts(char *text)
{
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    {
        const Port *port = get_CommuPort(i);
        if (port && port->write)
        {
            mavlink_message_t msg_sd;
            if (mavlink_lock_chan(i, 0.01))
            {
                mavlink_msg_statustext_pack_chan(get_CommulinkSysId(),  // system id
                                                 get_CommulinkCompId(), // component id
                                                 i,                     // chan
                                                 &msg_sd, MAV_SEVERITY_EMERGENCY, text, 0, 0);
                mavlink_send_msg(port, &msg_sd);
                mavlink_unlock_chan(i);
            }
        }
    }
}

// 请求云台状态
static inline void request_camera_status_from_pod(DriverInfo *driver_info, uint8_t event_id)
{
    mavlink_camera_status_t camera_status;
    camera_status.target_system = 0;
    camera_status.cam_idx = componentID;
    camera_status.img_idx = 0;
    camera_status.event_id = event_id;
    camera_status.p1 = 0;
    camera_status.p2 = 0;
    camera_status.p3 = 0;
    camera_status.p4 = 0;

    mavlink_message_t msg_sd;
    mavlink_msg_camera_status_encode_chan(get_CommulinkSysId(), componentID, MAVLINK_COMM_NUM_BUFFERS - 1, &msg_sd, &camera_status);
    mavlink_send_msg(&driver_info->port, &msg_sd, 1, 1);
}

// 发送云台控制流
static inline void sendCtrlMsgToPod(DriverInfo *driver_info, uint8_t *data, uint16_t dataLen, uint8_t cmd, bool need_ack)
{
    static uint16_t seq = 0;
    __attribute__((aligned(32))) uint8_t sdata[64];
    *(uint16_t *) &sdata[0] = 0x6655;
    *(uint8_t *) &sdata[2] = need_ack ? 1 : 0;
    *(uint16_t *) &sdata[3] = dataLen;
    *(uint16_t *) &sdata[5] = seq++;
    *(uint8_t *) &sdata[7] = cmd;
    memcpy(&sdata[8], data, dataLen);
    uint32_t res = 0;
    CRC_Calc(cg_CRC_CONFIG_CRC16_XMODEM, (uint8_t *) &sdata[0], dataLen + 8, &res, 0);
    *(uint16_t *) &sdata[dataLen + 8] = res;
    driver_info->port.write((uint8_t *) &sdata, dataLen + 8 + 2, 0.1, 0.1);
}
// 发送GPS定位数据给云台
static void sendGPSToPod(DriverInfo *driver_info)
{
    /*发送姿态数据*/
    typedef struct
    {
        uint32_t time_boot_ms;
        float rol;
        float pit;
        float yaw;
        float rolRate;
        float pitRate;
        float yawRate;
    } __AC_PACKED _attPack;
    _attPack attPack;

    attPack.time_boot_ms = TIME::get_System_Run_Time() * 1000;
    Quaternion quat;
    get_AirframeY_quat(&quat);
    attPack.rol = +quat.getRoll();
    attPack.pit = -quat.getPitch();
    attPack.yaw = -quat.getYaw();
    if (attPack.yaw < 0)
        attPack.yaw += 2 * Pi;
    vector3<double> angularRate;
    get_AngularRate_Ctrl(&angularRate);
    attPack.rolRate = +angularRate.x;
    attPack.pitRate = -angularRate.y;
    attPack.yawRate = -angularRate.z;

    sendCtrlMsgToPod(driver_info, (uint8_t *) &attPack, sizeof(_attPack), 0x22, false);
    /*发送姿态数据*/
}

static void YT_SIYI_Read(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    // 状态信息
    TIME targetAvTIME(false);

    uint16_t zooms[5];

    // 数据状态机
    uint8_t rc_counter = 0;
    uint8_t rdata = 0;
    uint16_t crc = 0;

    TIME statusSendTime(1);
    double gimBal_yaw_angle = INT16_MAX, gimBal_yaw_rate = INT16_MAX;
    double gimBal_pitch_angle = INT16_MAX, gimBal_pitch_rate = INT16_MAX;
    double gimBal_roll_angle = INT16_MAX, gimBal_roll_rate = INT16_MAX;
    int16_t temperature_highest = 0;
    int16_t temperature_lowest = 0;
    int16_t temperature_average = 0;
    int16_t temperature_center = 0;
    uint8_t thermal_zoomLevel = 0;

    double laser_distance = UINT16_MAX;
    uint16_t image_ind = 0;
    uint8_t photoType = 255;

    uint8_t encodeType = 0;
    uint8_t resolutionVideo = 0;
    uint8_t modeVideo = 0;
    double remainingSdCardCapcity = 0;
    double visibleSdCardCapcityUsed = 0;
    double thermalSdCardCapcityUsed = 0;
    uint8_t photoOrVideoMode = 0;
    uint8_t sdCardStatus = 0;
    uint8_t trackStatus = 0;
    uint16_t visible_zoomLevel = 0;

    mavlink_camera_status_t camera_status;

    // uint32_t followKey = followSensorRegister(followDataType_track_s_xy);
    TIME feedbackTIME_100(false);
    TIME feedbackTIME_101(false);
    TIME feedbackTIME_102(false);
    TIME feedbackTIME_103(false);

    uint8_t payload[300];
    uint8_t cmd = 0;
    uint16_t length = 0;
    uint16_t seq = 0;
    while (1)
    {
        uint8_t readLen = driver_info.port.read(&rdata, 1, 0.05, 0.05);
        if (readLen > 0)
        {
            if (rc_counter > 255)
                rc_counter = 0;
            payload[rc_counter++] = rdata;
            if (rc_counter == 1) // 帧头
            {
                if (rdata != 0x55)
                {
                    rc_counter = 0;
                }
            }
            else if (rc_counter == 2) // 帧头
            {
                if (rdata != 0x66)
                {
                    rc_counter = 0;
                }
                crc = 0;
            }
            else if (rc_counter == 3)
            {
                length = 0;
            }
            else if (rc_counter == 5)
            {
                length = *(uint16_t *) &payload[3];
            }
            else if (rc_counter == 7)
            {
                seq = *(uint16_t *) &payload[5];
            }
            else if (rc_counter == 8)
            {
                cmd = payload[7];
            }
            else if (rc_counter >= length + 10)
            {
                uint32_t res = 0;
                if (CRC_Calc(cg_CRC_CONFIG_CRC16_XMODEM, &payload[0], length + 8, &res, 0))
                {
                    uint16_t crc = *(uint16_t *) &payload[length + 8];
                    extern float debug_test[30];
                    if (res == crc)
                    {
                        debug_test[19]++;
                        if (cmd == 0x0D)
                        { // attitude
                            gimBal_yaw_angle = (*(int16_t *) &payload[8]) * -0.1 * 50;
                            gimBal_pitch_angle = *(int16_t *) &payload[10] * 0.1 * 50;
                            gimBal_roll_angle = *(int16_t *) &payload[12] * 0.1 * 50;

                            gimBal_yaw_rate = *(int16_t *) &payload[14] * 0.1 * 50;
                            gimBal_pitch_rate = *(int16_t *) &payload[16] * 0.1 * 50;
                            gimBal_roll_rate = *(int16_t *) &payload[18] * 0.1 * 50;
                        }
                        else if (cmd == 0x18)
                        {
                            visible_zoomLevel = (payload[8] + payload[9] * 0.1) * 10;
                        }
                        else if (cmd == 0x48) // 格式化
                        {
                            camera_status.target_system = 0;
                            camera_status.cam_idx = componentID;
                            camera_status.img_idx = image_ind;
                            camera_status.event_id = 12;
                            ((uint8_t *) &camera_status.p1)[0] = payload[0];
                            ((uint32_t *) &camera_status.p2)[0] = 0;
                            ((uint32_t *) &camera_status.p3)[0] = 0;
                            ((uint32_t *) &camera_status.p4)[0] = 0;
                            static TIME event_sendTime;
                            if (event_sendTime.get_pass_time_st() > 0.1 || event_sendTime.t == 0)
                            {
                                send_camera_status_to_mavlink_ports(&camera_status);
                                event_sendTime = TIME::now();
                            }
                        }
                        else if (cmd == 0x0B) // 拍照反馈
                        {
                            Quaternion airframe_quat;
                            get_AirframeY_quat(&airframe_quat);
                            airframe_quat.Enu2Ned();
                            double roll = rad2degree(airframe_quat.getRoll());
                            double pitch = rad2degree(airframe_quat.getPitch());
                            double yaw = rad2degree(airframe_quat.getYaw());

                            mavlink_camera_status_t camera_status;
                            camera_status.target_system = 0;
                            camera_status.cam_idx = componentID;
                            camera_status.img_idx = image_ind;
                            camera_status.event_id = 1;
                            camera_status.p1 = 0;
                            camera_status.p2 = 0;
                            camera_status.p3 = 0;
                            camera_status.p4 = 0;

                            uint8_t takePhotoStatus = 0;
                            if (payload[8] == 0)
                            {
                                takePhotoStatus = 0; // 拍照完成
                                image_ind++;
                                camera_status.img_idx = image_ind;
                            }
                            else if (payload[8] == 1)
                            {
                                takePhotoStatus = 6; // 拍照失败
                            }
                            if (payload[8] == 0 || payload[8] == 1)
                            {
                                camera_status.event_id = 1;
                                ((uint8_t *) &camera_status.p1)[0] = takePhotoStatus;
                                ((uint8_t *) &camera_status.p1)[1] = photoType;
                                ((int16_t *) &camera_status.p1)[1] = gimBal_pitch_angle * 50;
                                ((int16_t *) &camera_status.p2)[0] = gimBal_yaw_angle * 50;
                                ((int16_t *) &camera_status.p2)[1] = INT16_MAX;  // 水平视场角度
                                ((int16_t *) &camera_status.p3)[0] = INT16_MAX;  // 垂直视场角度
                                ((int16_t *) &camera_status.p3)[1] = pitch * 50; // 无人机俯仰角度
                                ((int16_t *) &camera_status.p4)[0] = roll * 50;  // 无人机俯仰角度
                                ((int16_t *) &camera_status.p4)[1] = yaw * 50;   // 无人机偏航角度
                                static TIME sendTime;
                                if (sendTime.t == 0 || sendTime.get_pass_time() > 0.5 || takePhotoStatus == 0)
                                {
                                    sendTime = TIME::now();
                                    send_camera_status_to_mavlink_ports(&camera_status);
                                }
                            }
                        }
                        else if (cmd == 0x0A) // 录像反馈
                        {
                            camera_status.target_system = 0;
                            camera_status.cam_idx = componentID;
                            camera_status.img_idx = image_ind;
                            camera_status.event_id = 3;
                            camera_status.p1 = 0;
                            camera_status.p2 = 0;
                            camera_status.p3 = 0;
                            camera_status.p4 = 0;

                            uint8_t record_type = 0;
                            uint8_t record_status = 0;
                            if (payload[11] == 0 || payload[11] == 1)
                                record_status = payload[8];
                            else if (payload[11] == 2)
                                record_status = 4;
                            else if (payload[11] == 3)
                                record_status = 6;

                            ((uint16_t *) &camera_status.p1)[0] = record_type;
                            ((uint16_t *) &camera_status.p1)[1] = record_status;
                            ((uint32_t *) &camera_status.p2)[0] = UINT32_MAX;
                            ((uint32_t *) &camera_status.p3)[0] = UINT32_MAX;
                            static TIME event_sendTime;
                            if (event_sendTime.get_pass_time_st() > 0.1 || event_sendTime.t == 0)
                            {
                                send_camera_status_to_mavlink_ports(&camera_status);
                            }
                        }
                    }
                }
                rc_counter = 0;
            }
        }

        if (feedbackTIME_100.get_pass_time() >= 0.3)
        { // 云台姿态和激光测距反馈（反馈类型=100）
            feedbackTIME_100 = TIME::now();
            mavlink_camera_status_t camera_status;
            camera_status.target_system = 0;
            camera_status.cam_idx = componentID;
            camera_status.img_idx = image_ind;
            camera_status.event_id = 100;
            camera_status.p1 = 0;
            camera_status.p2 = 0;
            camera_status.p3 = 0;
            camera_status.p4 = 0;

            ((int16_t *) &camera_status.p1)[0] = gimBal_pitch_angle;
            ((int16_t *) &camera_status.p1)[1] = gimBal_roll_angle;
            ((int16_t *) &camera_status.p2)[0] = gimBal_yaw_angle;
            ((int16_t *) &camera_status.p2)[1] = gimBal_pitch_rate;
            ((int16_t *) &camera_status.p3)[0] = gimBal_roll_rate;
            ((int16_t *) &camera_status.p3)[1] = gimBal_yaw_rate;
            ((uint16_t *) &camera_status.p4)[0] = laser_distance;

            send_camera_status_to_mavlink_ports(&camera_status);
        }

        if (feedbackTIME_103.get_pass_time() >= 1)
        { // 可见光相机状态反馈（反馈类型=103）
            feedbackTIME_103 = TIME::now();
            mavlink_camera_status_t camera_status;
            camera_status.target_system = 0;
            camera_status.cam_idx = componentID;
            camera_status.img_idx = image_ind;
            camera_status.event_id = 103;
            camera_status.p1 = 0;
            camera_status.p2 = 0;
            camera_status.p3 = 0;
            camera_status.p4 = 0;

            ((uint16_t *) &camera_status.p1)[0] = visible_zoomLevel; // 混合变倍倍率
            ((uint16_t *) &camera_status.p1)[1] = 0;                 // 可见光焦距

            send_camera_status_to_mavlink_ports(&camera_status);
        }
    }
}
static void YT_SIYI_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    CmdMsg msg;
    TIME attSendTime, heartBeatSendTime;

    while (1)
    {
        msg.cmd = 0;
        bool msgAvailable = ReceiveCmdMsgFromTask(&msg, componentID, 0.01);
        if (msgAvailable)
        {
            target_system = msg.target_system;
            target_component = msg.target_component;
            if (msg.cmd == MAV_CMD_REQUEST_CAMERA_INFORMATION)
            { // 相机信息发送
                send_camera_information();
                send_ack(msg.cmd, 1);
            }
            else if (msg.cmd == MAV_CMD_REQUEST_MESSAGE)
            { // 相机信息发送
                if (msg.params[0] == MAVLINK_MSG_ID_CAMERA_INFORMATION)
                {
                    send_camera_information();
                    send_ack(msg.cmd, 1);
                }
            }
            else if (msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
            { // 俯仰，向上为正;  航向，向左为正  FRD
                if (msg.params[4] == 0)
                { // 速度控制
                    int8_t payload[10];
                    payload[0] = msg.params[3] * 0.5; // 方位运动速度
                    payload[1] = msg.params[2] * 0.5; // 俯仰运动速度

                    sendCtrlMsgToPod(&driver_info, (uint8_t *) &payload, 2, 0x07, false);
                }
                else
                { // 角度控制
                    uint8_t payload[10];
                    *(int16_t *) &payload[0] = msg.params[1] * -10; // 云台俯仰角度 -90°至 30°
                    *(int16_t *) &payload[2] = msg.params[0] * 10;  // 云台方位角度
                    sendCtrlMsgToPod(&driver_info, payload, 4, 0x0E, true);
                }
            }
            else if (msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_CENTER)
            { // 一键回中
                uint8_t payload[10];
                payload[0] = 1;
                sendCtrlMsgToPod(&driver_info, payload, 1, 0x08, false);
            }
            else if (msg.cmd == MAV_CMD_DO_GIMBAL_MANAGER_DOWN)
            { // 一键向下
                uint8_t payload[10];
                *(int16_t *) &payload[0] = 0;
                *(int16_t *) &payload[2] = -90 * 10;
                sendCtrlMsgToPod(&driver_info, payload, 4, 0x0E, true);
            }
            else if (msg.cmd == MAV_CMD_SET_CAMERA_MODE)
            { // 模式切换
            }
            else if (msg.cmd == MAV_CMD_SET_CAMERA_ZOOM)
            { // 缩放
                if (msg.params[0] == ZOOM_TYPE_RANGE)
                {
                    uint8_t payload[10];
                    payload[0] = floor(msg.params[1]); // 可见光指定光学变倍倍率
                    payload[1] = msg.params[1] - floor(msg.params[1]);
                    sendCtrlMsgToPod(&driver_info, payload, 2, 0x0F, true);
                }
                else if (msg.params[0] == ZOOM_TYPE_CONTINUOUS)
                {
                    static bool stopZoomNeeded = false;
                    if (msg.params[1] == -1)
                    { // 放大
                        if (!stopZoomNeeded)
                        {
                            stopZoomNeeded = true;
                            uint8_t payload[10];
                            payload[0] = 1;
                            sendCtrlMsgToPod(&driver_info, payload, 1, 0x05, true);
                        }
                    }
                    else if (msg.params[1] == 0)
                    { // 停止缩放
                        uint8_t payload[10];
                        payload[0] = 0;
                        sendCtrlMsgToPod(&driver_info, payload, 1, 0x05, true);
                        stopZoomNeeded = false;
                    }
                    else if (msg.params[1] == 1)
                    { // 缩小
                        if (!stopZoomNeeded)
                        {
                            stopZoomNeeded = true;
                            uint8_t payload[10];
                            payload[0] = -1;
                            sendCtrlMsgToPod(&driver_info, payload, 1, 0x05, true);
                        }
                    }
                }
            }
            else if (msg.cmd == MAV_CMD_IMAGE_START_CAPTURE)
            { // 拍照
                uint8_t data = 0;
                sendCtrlMsgToPod(&driver_info, &data, 1, 0x0C, true);
                // 反馈
                SendCmdMsgToTask(msg, CAMERA_FEEDBACK_QUEUE_ID, 0.005);
                // sendLedSignal(LEDSignal_Success1);
                os_delay(0.2);
                sendCtrlMsgToPod(&driver_info, &data, 0, 0x0B, true);
            }
            else if (msg.cmd == MAV_CMD_VIDEO_START_CAPTURE)
            { // 开始录像
                uint8_t data = 2;
                sendCtrlMsgToPod(&driver_info, &data, 1, 0x0C, true);
                os_delay(1);
                sendCtrlMsgToPod(&driver_info, &data, 0, 0x0A, true);
            }
            else if (msg.cmd == MAV_CMD_VIDEO_STOP_CAPTURE)
            { // 停止录像
                uint8_t data = 2;
                sendCtrlMsgToPod(&driver_info, &data, 1, 0x0C, true);
                os_delay(0.2);
                sendCtrlMsgToPod(&driver_info, &data, 0, 0x0A, true);
            }
            else if (msg.cmd == MAV_CMD_SET_CAMERA_FOCUS)
            { // 聚焦
                if (msg.params[1] == 0)
                {
                    uint8_t payload[10];
                    payload[0] = 1;
                    payload[1] = 1920 * 0.5;
                    payload[2] = 1080 * 0.5;
                    sendCtrlMsgToPod(&driver_info, payload, 1, 0x04, true);
                }
            }
            else if (msg.cmd == MAV_CMD_STORAGE_FORMAT)
            { // 格式化SD卡
                uint8_t payload[10];
                sendCtrlMsgToPod(&driver_info, payload, 0, 0x48, true);
            }
            else if (msg.cmd == 1026)
            { // 授时
            }
            else if (msg.cmd == 1027)
            { // 重启复位
                if (msg.params[0] == 1)
                {
                    uint8_t payload[10];
                    payload[0] = 1;
                    payload[1] = 1;
                    sendCtrlMsgToPod(&driver_info, payload, 2, 0x80, true);
                }
            }
        }

        if (heartBeatSendTime.get_pass_time() > 0.99)
        {
            // 发送heartBeat
            send_heartbeat();
            heartBeatSendTime = TIME::now();

            static int attCnt = 5;
            if (attCnt++ >= 5)
            {
                attCnt = 0;
                uint8_t payload[10];
                payload[0] = 0;
                payload[1] = 4;
                sendCtrlMsgToPod(&driver_info, payload, 2, 0x25, true);

                payload[0] = 1;
                payload[1] = 2;
                sendCtrlMsgToPod(&driver_info, payload, 2, 0x25, true);
            }
            else
            {
                // 请求当前变倍值
                uint8_t payload[10];
                payload[0] = 0;
                sendCtrlMsgToPod(&driver_info, payload, 0, 0x18, true);
            }
        }

        if (attSendTime.get_pass_time() >= 0.02)
        {
            attSendTime = TIME::now();
            sendGPSToPod(&driver_info);
        }
    }
}

static bool YT_SIYI_DriverInit(Port port, uint32_t param)
{
    // 注册获取ID
    if (!registerMavlinkGimbalCompID(componentID, SName("GIMBAL_SIYI")))
        return false;

    // 注册消息队列
    TaskQueueRegister(componentID, 30);

    // 波特率19200
    port.SetBaudRate(115200, 2, 2);

    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;

    DriverInfo *driver_info2 = new DriverInfo;
    driver_info2->param = param;
    driver_info2->port = port;

    xTaskCreate(YT_SIYI_Server, "YT_SIYI", 2048, (void *) driver_info, SysPriority_ExtSensor, NULL);
    xTaskCreate(YT_SIYI_Read, "YT_SIYI_Read", 2048, (void *) driver_info2, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_gimbal_SIYI()
{
    PortFunc_Register(107, YT_SIYI_DriverInit);
}