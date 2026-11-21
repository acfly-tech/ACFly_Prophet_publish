#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "HardWareConfig.hpp"
#include "MavlinkCMDProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Missions.hpp"
#include "Modes.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "StorageSystem.hpp"
#include "drv_PWMOut.hpp"
#include "drv_USB.hpp"
#include "event_groups.h"
#include "semphr.h"

// 保存之前通道值
static double last_Channel_values[MAX_PWM_COUNT];
// 通道临时变量
static uint32_t channelTemp1[MAX_PWM_COUNT] = {0};
static float channelTemp2[MAX_PWM_COUNT] = {0};
static float channelTemp3[MAX_PWM_COUNT] = {0};
// 云台是否自动控制
static float GimbalCtrl_LockedAtt[MAX_PWM_COUNT];

// Aux通道参数全局缓冲区
// 布局: [AuxFuncsConfig固定部分] + [AuxFunc[aux_count][4]] + [AuxParam1[aux_count][2]] + [AuxParam2[aux_count][2]]
static uint8_t g_aux_ch_count = 0;
static uint8_t *g_aux_cfg_buf = nullptr;

static void readAuxConfig()
{
    if (g_aux_cfg_buf)
        ReadParamGroup("AuxCfg", (uint64_t *) g_aux_cfg_buf, 0);
}

static inline AuxFuncsConfig *getAuxFixedCfg()
{
    return (AuxFuncsConfig *) g_aux_cfg_buf;
}

// aux_idx: 辅助通道相对索引（0 = 第1条辅助通道）
static inline uint16_t getAuxChanFunc(uint8_t aux_idx)
{
    return ((uint16_t *) (g_aux_cfg_buf + sizeof(AuxFuncsConfig)))[aux_idx * 4];
}

static inline float getAuxChanParam1(uint8_t aux_idx)
{
    return ((float *) (g_aux_cfg_buf + sizeof(AuxFuncsConfig) + (uint32_t) g_aux_ch_count * 8))[aux_idx * 2];
}

static inline float getAuxChanParam2(uint8_t aux_idx)
{
    return ((float *) (g_aux_cfg_buf + sizeof(AuxFuncsConfig) + (uint32_t) g_aux_ch_count * 8 * 2))[aux_idx * 2];
}

/*热靴触发*/
// 拍照次数
static uint16_t PhotoCnt = 0;
// 相片序号
static uint16_t PhotoIndex = 1;

// 开始等待触发标志
static bool waiting_IOTrig = false;
// 发送完成标志
static EventGroupHandle_t IO_events = xEventGroupCreate();

static SemaphoreHandle_t PosLogMutex = xSemaphoreCreateMutex();
static bool SD_Pos_Record()
{
    // 获取时间
    RTC_TimeStruct RTC_Time;
    RTC_Time = Get_RTC_Time();
    // 获取速度
    vector3<double> vel;
    get_VelocityENU_Ctrl(&vel);

    // 获取姿态
    Quaternion airframe_quat;
    get_Attitude_quat(&airframe_quat);
    airframe_quat.Enu2Ned();

    static int8_t global_pos_ind = -1;
    PosSensorHealthInf3 posInf;
    if (global_pos_ind < 0)
    { // 第一次获取最优位置传感器
        if (get_OptimalGlobal_XYZ(&posInf))
            global_pos_ind = posInf.sensor_ind;
    }
    else
        get_PosSensorHealth_XYZ(&posInf, global_pos_ind);

    double lat = 0;
    double lon = 0;
    double alt = 0;
    double accN = 999999;
    double accE = 999999;
    double accD = 999999;
    uint8_t fix_type = 0;
    uint16_t week = 0;
    double TOW = 0;

    if (global_pos_ind >= 0)
    {
        Position_Sensor gps_sensor;
        if (GetPositionSensor(global_pos_ind, &gps_sensor))
        {
            if (gps_sensor.data.available && gps_sensor.data.sensor_type == Position_Sensor_Type_GlobalPositioning)
            {
                // 计算经纬度
                map_projection_reproject(&posInf.mp, posInf.PositionENU.x + posInf.HOffset.x, posInf.PositionENU.y + posInf.HOffset.y, &lat, &lon);
                // 高度
                alt = posInf.PositionENU.z + posInf.HOffset.z;
                alt *= 0.01;
                // 精度
                accN = gps_sensor.inf.addition_inf[4] * 0.01;
                accE = gps_sensor.inf.addition_inf[4] * 0.01;
                accD = gps_sensor.inf.addition_inf[5] * 0.01;
                // week
                week = gps_sensor.inf.addition_inf[2];
                // TOW
                TOW = gps_sensor.inf.addition_inf[3];
                // fix
                if (gps_sensor.inf.addition_inf[1] == 1)
                    fix_type = 0;
                else if (gps_sensor.inf.addition_inf[1] == 5)
                    fix_type = 34;
                else if (gps_sensor.inf.addition_inf[1] == 6)
                    fix_type = 50;
                else
                    fix_type = 16;
            }
        }
    }

    xSemaphoreTake(PosLogMutex, portMAX_DELAY);
    uint16_t photo_ind = PhotoIndex;
    char pos_txt_buf[200];
    int n = sprintf(pos_txt_buf, "%4d\t%6.6f\t[%4d]\t%4d,N\t%4d,E\t%4d,V \t%11.8f,Lat \t%11.8f,Lon \t%6.3f,Ellh \t%9.6f,    %9.6f,    %9.6f,    %2d,Q\r\n", PhotoIndex++, TOW, week, 0, 0, 0, lat, lon,
                    alt, accN, accE, accD, fix_type);

    xSemaphoreGive(PosLogMutex);

    if (!SDLog_Txt1(pos_txt_buf, n))
        return false;

    // 发送拍照信息到地面??
    double Altitude_Local = 0;
    vector3<double> Position;
    get_Position_Ctrl(&Position);
    double homeZ;
    double heightAboveGround = 0;
    if (getHomeLocalZ(&homeZ, 0, 0.01))
        heightAboveGround = Position.z - homeZ;

    mavlink_message_t msg_sd;
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
    { // 遍历所有端口
        if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
        {
            mavlink_msg_camera_feedback_pack_chan(get_CommulinkSysId(),  // system id
                                                  get_CommulinkCompId(), // component id
                                                  i, &msg_sd,
                                                  TIME::get_System_Run_Time() * 1e3, // boot ms
                                                  1,                                 // target_system  System ID
                                                  global_pos_ind,                    // camera_id
                                                  photo_ind,                         // Image index
                                                  lat * 1e7,                         // lat [degE7]
                                                  lon * 1e7,                         // lon [degE7]
                                                  alt,                               // alt_msl [m] Altitude (MSL).
                                                  heightAboveGround * 0.01,          // alt_rel [m] Altitude (Relative to HOME location).
                                                  0,                                 // Camera Roll
                                                  0,                                 // Camera Pitch
                                                  0,                                 // Camera Yaw
                                                  30,                                // foc_len [mm] Focal Length
                                                  0,                                 // 0:Shooting photos, not video
                                                  photo_ind                          // Completed image captures
            );
            const Port *port = get_CommuPort(i);
            if (port && port->write)
            {
                mavlink_send_msg(port, &msg_sd, 0, 2 / configTICK_RATE_HZ);
            }
            mavlink_unlock_chan(i);
        }
    }
    return true;
}

static void IOTrigTCB(void *pvParameter1, uint32_t ulParameter2)
{
    uint8_t trig_ena[8];
    if (ReadParam("Aux_CamTrigEna", 0, 0, (uint64_t *) trig_ena, 0) == PR_OK)
    {
        if (trig_ena[0])
        {
            if (SD_Pos_Record())
            {
                xEventGroupSetBits(IO_events, 1 << 0);
                if (waiting_IOTrig == false)
                    sendLedSignal(LEDSignal_Success1);
            }
        }
    }
}

void wait_IOTrig_start()
{
    waiting_IOTrig = true;
    xEventGroupClearBits(IO_events, 1 << 0);
}

bool wait_IOTrig(double TIME)
{
    uint32_t revents = xEventGroupWaitBits(IO_events, 1 << 0, pdTRUE, pdFALSE, TIME * configTICK_RATE_HZ);
    waiting_IOTrig = false;
    if ((revents & (1 << 0)) == 0)
        return false;
    else
        return true;
}

// IO翻转计数??
static void camTrigCallback()
{
    static TIME last_trig_TIME(false);
    double pass_t = last_trig_TIME.get_pass_time_st();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pass_t > 0.05)
    {
        xLPDaemonTaskCallFromISR(IOTrigTCB, 0, 0, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*热靴触发*/

/*记录Pos*/
bool recordPos(bool acTrig)
{
    if (acTrig)
    {
        xSemaphoreTake(PosLogMutex, portMAX_DELAY);
        ++PhotoCnt;
        xSemaphoreGive(PosLogMutex);

        mavlink_message_t msg_sd;
        for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
        { // 遍历所有端口
            if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
            {
                mavlink_msg_camera_status_pack_chan(get_CommulinkSysId(),  // system id
                                                    get_CommulinkCompId(), // component id
                                                    i, &msg_sd,
                                                    TIME::get_System_Run_Time() * 1e6, // boot us
                                                    255,                               // target_system
                                                    1,                                 // cam index
                                                    PhotoCnt,                          // image index
                                                    1,                                 // event id
                                                    0,                                 // p1
                                                    0,                                 // p2
                                                    0,                                 // p3
                                                    0                                  // p4
                );
                const Port *port = get_CommuPort(i);
                if (port && port->write)
                {
                    mavlink_send_msg(port, &msg_sd, 0, 2 / configTICK_RATE_HZ);
                }
                mavlink_unlock_chan(i);
            }
        }
    }
    return SD_Pos_Record();
}
/*记录Pos*/

/*拍照功能*/
static SemaphoreHandle_t CamMutex = xSemaphoreCreateRecursiveMutex();
bool AuxCamTakePhoto()
{
    if (getInitializationCompleted() == false)
        return false;

    uint8_t cam_chans = 0;
    readAuxConfig();
    AuxFuncsConfig *cfg = getAuxFixedCfg();

    if (get_is_usb_connected())
    {
        if (cfg->Aux_YTSurveyPic[0] > 0)
        { // 数字云台需要拍照
            CmdMsg cmd_msg;
            cmd_msg.cmd = MAV_CMD_IMAGE_START_CAPTURE;
            cmd_msg.cmdSourceType = 0;
            cmd_msg.params[0] = 0;
            cmd_msg.params[1] = 0;
            cmd_msg.params[2] = 1;
            SendCmdMsgToAllCamera(cmd_msg, 0.01);
            recordPos(true);
        }
        return false;
    }

    if (xSemaphoreTakeRecursive(CamMutex, 0.1 * configTICK_RATE_HZ))
    {
        uint8_t MainMotorCount = get_MainMotorCount();
        if (cfg->Aux_CamTrigEna[0])
            wait_IOTrig_start();

        // 拉低（拉高）相机PWM
        for (uint8_t i = MainMotorCount; i < MAX_PWM_COUNT; ++i)
        {
            uint16_t aux_cfg = getAuxChanFunc(i);
            if ((aux_cfg >= 25 && aux_cfg <= 48) || (aux_cfg >= 1025 && aux_cfg <= 1048))
            {
                Aux_PWM_Out(cfg->Aux_CamOnPwm[0] * 0.1 - 100, i);
                ++cam_chans;
            }
        }

        if (cam_chans == 0)
        { // 无相机返回
            uint8_t res = 0;

            if (cfg->Aux_YTSurveyPic[0] > 0)
            { // 数字云台需要拍照
                if (!cfg->Aux_CamTrigEna[0])
                    wait_IOTrig_start();

                CmdMsg cmd_msg;
                cmd_msg.cmd = MAV_CMD_IMAGE_START_CAPTURE;
                cmd_msg.cmdSourceType = 0;
                cmd_msg.params[0] = 0;
                cmd_msg.params[1] = 0;
                cmd_msg.params[2] = 1;
                SendCmdMsgToAllCamera(cmd_msg, 0.01);

                if (cfg->Aux_CamTrigEna[0] > 0)
                { // 数字云台需要等待trig引脚触发
                    if (wait_IOTrig(cfg->Aux_CamShTime[0] > 1.5 ? 1.5 : cfg->Aux_CamShTime[0]))
                        res = 10;
                }
                else
                { // 数字云台等待指令回馈
                    wait_IOTrig(0);

                    double waitT = 0;
                    while (1)
                    {
                        CmdMsg msg;
                        bool msg_rd = ReceiveCmdMsgFromTask(&msg, CAMERA_FEEDBACK_QUEUE_ID, 0.002);
                        if (msg_rd)
                        { // 相机反馈成功
                            res = msg.cmd;
                            break;
                        }

                        if (waitT > cfg->Aux_CamShTime[0] || waitT > 1.5)
                            break;
                        waitT += 0.01;
                        os_delay(0.01);
                    }
                }
            }

            xSemaphoreGiveRecursive(CamMutex);
            if (res)
            {
                xSemaphoreTake(PosLogMutex, portMAX_DELAY);
                ++PhotoCnt;
                xSemaphoreGive(PosLogMutex);

                mavlink_message_t msg_sd;
                for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                { // 遍历所有端口
                    if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
                    {
                        mavlink_msg_camera_status_pack_chan(get_CommulinkSysId(),  // system id
                                                            get_CommulinkCompId(), // component id
                                                            i, &msg_sd,
                                                            TIME::get_System_Run_Time() * 1e6, // boot us
                                                            255,                               // target_system
                                                            1,                                 // cam index
                                                            PhotoCnt,                          // image index
                                                            1,                                 // event id
                                                            0,                                 // p1
                                                            0,                                 // p2
                                                            0,                                 // p3
                                                            0                                  // p4
                        );
                        const Port *port = get_CommuPort(i);
                        if (port && port->write)
                        {
                            mavlink_send_msg(port, &msg_sd, 0, 2 / configTICK_RATE_HZ);
                        }
                        mavlink_unlock_chan(i);
                    }
                }

                // 如果无需等待内部触发则立刻记录POS
                if (res != 3)
                    SD_Pos_Record();

                return true;
            }
            else
                return false;
        }

        // 拉高（拉低）相机PWM
        os_delay(cfg->Aux_CamShTime[0]);
        for (uint8_t i = MainMotorCount; i < MAX_PWM_COUNT; ++i)
        {
            uint16_t aux_cfg = getAuxChanFunc(i);
            if ((aux_cfg >= 25 && aux_cfg <= 48) || (aux_cfg >= 1025 && aux_cfg <= 1048))
                Aux_PWM_Out(cfg->Aux_CamOffPwm[0] * 0.1 - 100, i);
        }

        xSemaphoreTake(PosLogMutex, portMAX_DELAY);
        ++PhotoCnt;
        xSemaphoreGive(PosLogMutex);

        mavlink_message_t msg_sd;
        for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
        { // 遍历所有端口
            if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
            {
                mavlink_msg_camera_status_pack_chan(get_CommulinkSysId(),  // system id
                                                    get_CommulinkCompId(), // component id
                                                    i, &msg_sd,
                                                    TIME::get_System_Run_Time() * 1e6, // boot us
                                                    255,                               // target_system
                                                    1,                                 // cam index
                                                    PhotoCnt,                          // image index
                                                    1,                                 // event id
                                                    0,                                 // p1
                                                    0,                                 // p2
                                                    0,                                 // p3
                                                    0                                  // p4
                );
                const Port *port = get_CommuPort(i);
                if (port && port->write)
                {
                    mavlink_send_msg(port, &msg_sd, 0, 2 / configTICK_RATE_HZ);
                }
                mavlink_unlock_chan(i);
            }
        }

        if (cfg->Aux_CamTrigEna[0])
        { // 等待相机热靴
            if (wait_IOTrig(cfg->Aux_CamShTime[0] > 1.5 ? 1.5 : cfg->Aux_CamShTime[0]) == false)
            {
                xSemaphoreGiveRecursive(CamMutex);
                return false;
            }
        }

        // 如果热靴没有使能则立刻记录POS
        if (!cfg->Aux_CamTrigEna[0])
            SD_Pos_Record();

        xSemaphoreGiveRecursive(CamMutex);
        return true;
    }
    else
        return false;
}

static uint8_t camPhotoAsyncStep = 0;
static float camPhotoAsyncT = 0;
static uint16_t lastCamPhotoAsyncPhotoIndex = 0;
#define resetCamPhotoAsyncStep (camPhotoAsyncStep = camPhotoAsyncT = 0)
bool AuxCamTakePhotoAsync(double TIMEOUT)
{
    if (xSemaphoreTakeRecursive(CamMutex, TIMEOUT * configTICK_RATE_HZ))
    {
        resetCamPhotoAsyncStep;
        camPhotoAsyncStep = 1;
        camPhotoAsyncT = 0;
        xSemaphoreGiveRecursive(CamMutex);
        return true;
    }
    return false;
}
static void AuxCamTakePhotoAsyncServer(float h)
{
    AuxFuncsConfig *cfg = getAuxFixedCfg();
    if (getInitializationCompleted() == false || get_is_usb_connected())
    {
        if (get_is_usb_connected() && cfg->Aux_YTSurveyPic[0] > 0 && camPhotoAsyncStep > 0)
        {
            CmdMsg cmd_msg;
            cmd_msg.cmd = MAV_CMD_IMAGE_START_CAPTURE;
            cmd_msg.cmdSourceType = 111;
            cmd_msg.params[0] = 0;
            cmd_msg.params[1] = 0;
            cmd_msg.params[2] = 1;
            SendCmdMsgToAllCamera(cmd_msg, 0.01);

            recordPos(true);
        }
        if (camPhotoAsyncStep > 1)
            xSemaphoreGiveRecursive(CamMutex);
        camPhotoAsyncStep = camPhotoAsyncT = 0;
        // 清空数字云台反馈缓冲区
        CmdMsg msg;
        ReceiveCmdMsgFromTask(&msg, CAMERA_FEEDBACK_QUEUE_ID, 0);

        return;
    }

    if (camPhotoAsyncStep == 0)
    { // 空闲时处理数字云台拍照反馈
        CmdMsg msg;
        bool msg_rd = ReceiveCmdMsgFromTask(&msg, CAMERA_FEEDBACK_QUEUE_ID, 0);
        if (msg_rd && msg.cmd == 1)
        {
            xSemaphoreTake(PosLogMutex, portMAX_DELAY);
            ++PhotoCnt;
            xSemaphoreGive(PosLogMutex);
            SD_Pos_Record();
        }
    }

    if (camPhotoAsyncStep > 1)
    {
        camPhotoAsyncT += h;
        if (camPhotoAsyncT > 10)
            resetCamPhotoAsyncStep;
    }

    switch (camPhotoAsyncStep)
    {
        case 1:
        { // 准备启动触发
            if (xSemaphoreTakeRecursive(CamMutex, 0.1 * configTICK_RATE_HZ))
            {
                uint8_t cam_chans = 0;
                lastCamPhotoAsyncPhotoIndex = PhotoIndex;

                uint8_t MainMotorCount = get_MainMotorCount();
                // 使能相机PWM
                for (uint8_t i = MainMotorCount; i < MAX_PWM_COUNT; ++i)
                {
                    uint16_t aux_cfg = getAuxChanFunc(i);
                    if ((aux_cfg >= 25 && aux_cfg <= 48) || (aux_cfg >= 1025 && aux_cfg <= 1048))
                    {
                        Aux_PWM_Out(cfg->Aux_CamOnPwm[0] * 0.1 - 100, i);
                        ++cam_chans;
                    }
                }
                camPhotoAsyncT = 0;

                if (cam_chans == 0)
                { // 无相机尝试数字云台
                    uint8_t res = 0;

                    if (cfg->Aux_YTSurveyPic[0] > 0)
                    { // 数字云台需要拍照
                        CmdMsg cmd_msg;
                        cmd_msg.cmd = MAV_CMD_IMAGE_START_CAPTURE;
                        cmd_msg.cmdSourceType = 111;
                        cmd_msg.params[0] = 0;
                        cmd_msg.params[1] = 0;
                        cmd_msg.params[2] = 1;
                        SendCmdMsgToAllCamera(cmd_msg, 0.01);

                        if (cfg->Aux_CamTrigEna[0] > 0)
                        { // 数字云台需要等待trig引脚触发
                            camPhotoAsyncStep = 20;
                        }
                        else
                        { // 进入数字云台等待指令反馈步骤
                            camPhotoAsyncStep = 30;
                        }
                    }
                    else
                    {
                        resetCamPhotoAsyncStep;
                        xSemaphoreGiveRecursive(CamMutex);
                    }
                }
                else
                { // 相机引脚触发完成
                    camPhotoAsyncStep = 10;
                }
            }
            else
                resetCamPhotoAsyncStep;
            break;
        }

        case 10:
        { // 相机引脚触发完成
            // 准备失效相机引脚

            if (camPhotoAsyncT > cfg->Aux_CamShTime[0])
            {
                // 失效相机PWM
                uint8_t MainMotorCount = get_MainMotorCount();
                for (uint8_t i = MainMotorCount; i < MAX_PWM_COUNT; ++i)
                {
                    uint16_t aux_cfg = getAuxChanFunc(i);
                    if ((aux_cfg >= 25 && aux_cfg <= 48) || (aux_cfg >= 1025 && aux_cfg <= 1048))
                        Aux_PWM_Out(cfg->Aux_CamOffPwm[0] * 0.1 - 100, i);
                }

                xSemaphoreTake(PosLogMutex, portMAX_DELAY);
                ++PhotoCnt;
                xSemaphoreGive(PosLogMutex);

                mavlink_message_t msg_sd;
                for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                { // 遍历所有端口
                    if (mavlink_lock_chan(i, 2 / configTICK_RATE_HZ))
                    {
                        mavlink_msg_camera_status_pack_chan(get_CommulinkSysId(),  // system id
                                                            get_CommulinkCompId(), // component id
                                                            i, &msg_sd,
                                                            TIME::get_System_Run_Time() * 1e6, // boot us
                                                            255,                               // target_system
                                                            1,                                 // cam index
                                                            PhotoCnt,                          // image index
                                                            1,                                 // event id
                                                            0,                                 // p1
                                                            0,                                 // p2
                                                            0,                                 // p3
                                                            0                                  // p4
                        );
                        const Port *port = get_CommuPort(i);
                        if (port && port->write)
                        {
                            mavlink_send_msg(port, &msg_sd, 0, 2 / configTICK_RATE_HZ);
                        }
                        mavlink_unlock_chan(i);
                    }
                }

                if (cfg->Aux_CamTrigEna[0])
                { // 等待相机热靴
                    camPhotoAsyncStep = 20;
                }
                else
                { // 如果热靴没有使能则立刻记录POS
                    SD_Pos_Record();
                    camPhotoAsyncStep = 11;
                    camPhotoAsyncT = 0;
                    //						resetCamPhotoAsyncStep;
                    //						xSemaphoreGiveRecursive(CamMutex);
                }
            }
            break;
        }
        case 11:
        { // 延时等待sh时间
            if (camPhotoAsyncT > cfg->Aux_CamShTime[0])
            {
                resetCamPhotoAsyncStep;
                xSemaphoreGiveRecursive(CamMutex);
            }
            break;
        }

        case 20:
        { // 等待热靴触发
            if (camPhotoAsyncT > cfg->Aux_CamShTime[0] && (lastCamPhotoAsyncPhotoIndex != PhotoIndex || camPhotoAsyncT > 1.5))
            {
                resetCamPhotoAsyncStep;
                xSemaphoreGiveRecursive(CamMutex);
            }
            break;
        }
        case 30:
        { // 数字云台等待触发
            CmdMsg msg;
            bool msg_rd = ReceiveCmdMsgFromTask(&msg, CAMERA_FEEDBACK_QUEUE_ID, 0);
            if (msg_rd)
                SD_Pos_Record();
            if (msg_rd || camPhotoAsyncT > cfg->Aux_CamShTime[0] || camPhotoAsyncT > 1.5)
            {
                resetCamPhotoAsyncStep;
                xSemaphoreGiveRecursive(CamMutex);
            }
        }
    }
}
/*拍照功能*/

/*喷洒功能*/
static float TankRMPercent = 100;
float getTankRMPercent()
{
    return TankRMPercent;
}

bool setPump1(float percent)
{
    if (getInitializationCompleted() == false)
        return false;
    if (!isvalid(percent))
        return false;

    readAuxConfig();

    uint8_t chCount = 0;
    uint8_t MainMotorCount = get_MainMotorCount();
    for (uint8_t i = MainMotorCount; i < MAX_PWM_COUNT; ++i)
    {
        uint16_t aux_cfg = getAuxChanFunc(i);
        if (aux_cfg >= 525 && aux_cfg <= 548)
        {
            GimbalCtrl_LockedAtt[i] = percent;
            ++chCount;
        }
    }
    return chCount != 0;
}

bool setPump1bySpeed(float sp)
{
    if (getInitializationCompleted() == false)
        return false;
    if (!isvalid(sp))
        return false;

    readAuxConfig();
    AuxFuncsConfig *cfg = getAuxFixedCfg();

    uint8_t chCount = 0;
    uint8_t MainMotorCount = get_MainMotorCount();
    for (uint8_t i = MainMotorCount; i < MAX_PWM_COUNT; ++i)
    {
        uint16_t aux_cfg = getAuxChanFunc(i);
        if (aux_cfg >= 525 && aux_cfg <= 548)
        {
            float percent = constrain(sp * cfg->Aux_Pump1Sp[0], 0.0f, 100.0f);
            GimbalCtrl_LockedAtt[i] = percent;
            ++chCount;
        }
    }
    return chCount != 0;
}
/*喷洒功能*/

bool setAuxPWM(float PWMus, uint8_t ind)
{
    if (getInitializationCompleted() == false)
        return false;
    if (PWMus < 0 || PWMus > 100000)
        return false;

    readAuxConfig();
    uint8_t MainMotorCount = get_MainMotorCount();
    if (ind < MainMotorCount)
        return false;
    if (ind >= MAX_PWM_COUNT)
        return false;
    uint16_t aux_cfg = getAuxChanFunc(ind);
    if ((aux_cfg >= 0 && aux_cfg <= 16) || (aux_cfg >= 1001 && aux_cfg <= 1016))
    {
        GimbalCtrl_LockedAtt[ind] = PWMus;
        return true;
    }
    return false;
}

void init_process_AuxFuncs()
{
    //	Receiver rc;
    //	if( getReceiver(&rc) )
    //	{
    //		//复位保存之前通道
    //		for( uint8_t i = 0; i < rc.raw_available_channels; ++i )
    //			last_Channel_values[i] = rc.raw_data[i];
    //		for( uint8_t i = rc.raw_available_channels; i < 16; ++i )
    //			last_Channel_values[i] = -200;
    //	}
    //	else
    //	{
    // 复位保存之前通道
    for (uint8_t i = 0; i < MAX_PWM_COUNT; ++i)
        last_Channel_values[i] = -200;
    //	}
    // 复位运动自动控制标志
    for (uint8_t i = 0; i < MAX_PWM_COUNT; ++i)
    {
        channelTemp1[i] = 0;
        channelTemp2[i] = 0;
        channelTemp3[i] = 0;
        GimbalCtrl_LockedAtt[i] = 200000;
    }
}

void process_AuxFuncs(const Receiver *rc, double h)
{
    readAuxConfig();
    AuxFuncsConfig *cfg = getAuxFixedCfg();

    AuxCamTakePhotoAsyncServer(h);

    double freq = 1.0 / h;
    uint8_t MainMotorCount = get_MainMotorCount();
    for (uint8_t i = MainMotorCount; i < MAX_PWM_COUNT; ++i)
    {
        uint16_t aux_cfg = getAuxChanFunc(i);
        float aux_param1 = getAuxChanParam1(i);
        float aux_param2 = getAuxChanParam2(i);
        if (aux_cfg == 0)
        { // 无功能(可通过指令控制PWM输出)
            if (GimbalCtrl_LockedAtt[i] < 100000)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 1000) * 0.1, i);
        }
        if (aux_cfg >= 1 && aux_cfg <= 16)
        { // 映射遥控器通道
            bool stLocked = false;
            uint8_t stLock_chan = cfg->stLockParam[0] >> 24;
            if (stLock_chan)
            {
                if (rc->raw_available_channels >= stLock_chan)
                {
                    float chan = rc->raw_data[stLock_chan - 1];
                    if (chan > 68)
                        stLocked = true;
                }
            }

            if (stLocked)
            { // 舵机已锁定
                if (cfg->auxCfg[0] & AUX_CFG_STLOCK_FORCEVALUE_BIT)
                {
                    uint32_t value = cfg->stLockParam[0] & 0xff;
                    Aux_PWM_Out((value - 1000) * 0.1, i);
                }
            }
            else
            {
                if (GimbalCtrl_LockedAtt[i] < 100000)
                    Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 1000) * 0.1, i);
                uint8_t ref_chan = aux_cfg - 1;
                if (rc->connected && rc->raw_available_channels > ref_chan)
                {
                    channelTemp1[i] = 0;

                    if (fabs(GimbalCtrl_LockedAtt[i]) < 100000 && last_Channel_values[i] > -100)
                    { // Aux自动控制
                        // 通道变化大于阈值才调整Aux
                        if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
                        {
                            Aux_PWM_Out((rc->raw_data[ref_chan] - 50.0) * aux_param1 + 50 + aux_param2 * 0.1, i);
                            last_Channel_values[i] = rc->raw_data[ref_chan];
                            GimbalCtrl_LockedAtt[i] = 200000;
                        }
                    }
                    else
                    { // Aux手动控制
                        Aux_PWM_Out((rc->raw_data[ref_chan] - 50.0) * aux_param1 + 50 + aux_param2 * 0.1, i);
                        last_Channel_values[i] = rc->raw_data[ref_chan];
                    }
                }
                else
                {
                    last_Channel_values[i] = -200;

                    //					if( channelTemp1[i] >= 2*freq )
                    //					{
                    //						GimbalCtrl_LockedAtt[i] = 1000;
                    //					}
                    //					else
                    //						++channelTemp1[i];
                }
            }
        }
        else if (aux_cfg >= 501 && aux_cfg <= 516)
        { // 映射遥控器通道（开关量）
            uint8_t ref_chan = aux_cfg - 501;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                if (rc->raw_data[ref_chan] > 50)
                {
                    if (aux_param1 > 0)
                        Aux_PWM_Out(6000, i);
                    else
                        Aux_PWM_Out(-100, i);
                }
                else
                {
                    if (aux_param1 > 0)
                        Aux_PWM_Out(-100, i);
                    else
                        Aux_PWM_Out(6000, i);
                }
            }
        }
        if (aux_cfg >= 525 && aux_cfg <= 548)
        { // 控制水泵
            double max = (cfg->Aux_Pump1Max[0] - 1000) * 0.1;
            double min = (cfg->Aux_Pump1Min[0] - 1000) * 0.1;
            double st = (cfg->Aux_Pump1St[0] - 1000) * 0.1;
            double scale = (max - st) / 100.0;
            if (GimbalCtrl_LockedAtt[i] < 200)
            { // 自动控制水泵
                if (GimbalCtrl_LockedAtt[i] > -20 && GimbalCtrl_LockedAtt[i] < 120)
                {
                    float out = constrain(GimbalCtrl_LockedAtt[i], 0.0f, 100.0f);
                    Aux_PWM_Out(out * scale + st, i);
                }
                else
                { // 关闭水泵
                    Aux_PWM_Out(min, i);
                }
            }

            uint8_t ref_chan = aux_cfg - 525;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 水泵手动控制
                    float out = (rc->raw_data[ref_chan] - 50.0) * aux_param1 + aux_param2;
                    out = constrain(out, 0.0f, 100.0f);
                    float deadZone = 15;
                    if (out < deadZone)
                        Aux_PWM_Out(min, i);
                    else
                        Aux_PWM_Out((out - deadZone) * (100 - st) / (100 - deadZone) + st, i);
                }
            }
            else
            { // 无遥控信号
                last_Channel_values[i] = -200;
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 非自动控制且无遥控信号
                    // 关闭水泵
                    Aux_PWM_Out(min, i);
                }
            }
        }
        else if (aux_cfg >= 25 && aux_cfg <= 48)
        { // 用遥控器对应通道进行相机快门触发(raw_data)
            if (xSemaphoreTakeRecursive(CamMutex, 0))
            {
                if (camPhotoAsyncStep == 0)
                {
                    uint8_t ref_chan = aux_cfg - 25;
                    if (rc->connected && rc->raw_available_channels > ref_chan)
                    {
                        if (last_Channel_values[i] > -100)
                        {
                            if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 15)
                            { // 触发相机
                                if (last_Channel_values[i] > -100)
                                {
                                    AuxCamTakePhoto();
                                }
                                last_Channel_values[i] = rc->raw_data[ref_chan];
                            }
                            else
                                Aux_PWM_Out(cfg->Aux_CamOffPwm[0] * 0.1 - 100, i);
                        }
                        else
                        { // 旧通道数据不可用
                            Aux_PWM_Out(cfg->Aux_CamOffPwm[0] * 0.1 - 100, i);
                            last_Channel_values[i] = rc->raw_data[ref_chan];
                        }
                    }
                    else
                    { // 无遥控器
                        Aux_PWM_Out(cfg->Aux_CamOffPwm[0] * 0.1 - 100, i);
                        last_Channel_values[i] = -200;
                    }
                }
                xSemaphoreGiveRecursive(CamMutex);
            }
        }
        else if (aux_cfg >= 49 && aux_cfg <= 72)
        { // 用遥控器对应通道进行无刷云台俯仰控制(raw_data)
            double angle90 = (cfg->Aux_BsYTPit90[0] - 1000) * 0.1;
            double angle0 = (cfg->Aux_BsYTPit0[0] - 1000) * 0.1;
            double scale = (angle90 - angle0) / 90.0;
            if (GimbalCtrl_LockedAtt[i] < 200)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 0) * scale + angle0, i);

            uint8_t ref_chan = aux_cfg - 49;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
                { // 云台自动控制
                    // 通道变化大于阈值才调整云台
                    if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
                    {
                        float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                        angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                        Aux_PWM_Out((angle - 0) * scale + angle0, i);
                        last_Channel_values[i] = rc->raw_data[ref_chan];
                        GimbalCtrl_LockedAtt[i] = 200000;
                    }
                }
                else
                { // 云台手动控制
                    float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                    angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                    last_Channel_values[i] = rc->raw_data[ref_chan];
                }
            }
            else
            { // 无遥控信号
                last_Channel_values[i] = -200;
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 非自动控制且无遥控信号
                    // 锁定0角度
                    float angle = 0;
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                }
            }
        }
        else if (aux_cfg >= 73 && aux_cfg <= 96)
        { // 用遥控器对应通道进行舵机云台俯仰控制(raw_data)
            double angle90 = (cfg->Aux_StYTPit90[0] - 1000) * 0.1;
            double angle0 = (cfg->Aux_StYTPit0[0] - 1000) * 0.1;
            double scale = (angle90 - angle0) / 90.0;
            Quaternion quat;
            get_Airframe_quat(&quat);
            double pitch = rad2degree(quat.getPitch());
            if (GimbalCtrl_LockedAtt[i] < 200)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - pitch - 0) * scale + angle0, i);

            uint8_t ref_chan = aux_cfg - 73;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
                { // 云台自动控制
                    // 通道变化大于阈值才调整云台
                    if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
                    {
                        float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                        angle -= pitch;
                        angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                        Aux_PWM_Out((angle - 0) * scale + angle0, i);
                        last_Channel_values[i] = rc->raw_data[ref_chan];
                        GimbalCtrl_LockedAtt[i] = 20000;
                    }
                }
                else
                { // 云台手动控制
                    float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                    angle -= pitch;
                    angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                    last_Channel_values[i] = rc->raw_data[ref_chan];
                }
            }
            else
            { // 无遥控信号
                last_Channel_values[i] = -200;
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 非自动控制且无遥控信号
                    // 锁定0角度
                    float angle = 0;
                    Aux_PWM_Out((angle - pitch - 0) * scale + angle0, i);
                }
            }
        }
        else if (aux_cfg >= 273 && aux_cfg <= 296)
        { // 用遥控器对应通道进行舵机云台2俯仰控制(raw_data)
            double angle90 = (cfg->Aux_StYT2Pit90[0] - 1000) * 0.1;
            double angle0 = (cfg->Aux_StYT2Pit0[0] - 1000) * 0.1;
            double scale = (angle90 - angle0) / 90.0;
            Quaternion quat;
            get_Airframe_quat(&quat);
            double pitch = rad2degree(quat.getPitch());
            if (GimbalCtrl_LockedAtt[i] < 200)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - pitch - 0) * scale + angle0, i);

            uint8_t ref_chan = aux_cfg - 273;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
                { // 云台自动控制
                    // 通道变化大于阈值才调整云台
                    if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
                    {
                        float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                        angle -= pitch;
                        angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                        Aux_PWM_Out((angle - 0) * scale + angle0, i);
                        last_Channel_values[i] = rc->raw_data[ref_chan];
                        GimbalCtrl_LockedAtt[i] = 20000;
                    }
                }
                else
                { // 云台手动控制
                    float angle = (rc->raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                    angle -= pitch;
                    angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                    last_Channel_values[i] = rc->raw_data[ref_chan];
                }
            }
            else
            { // 无遥控信号
                last_Channel_values[i] = -200;
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 非自动控制且无遥控信号
                    // 锁定0角度
                    float angle = 0;
                    Aux_PWM_Out((angle - pitch - 0) * scale + angle0, i);
                }
            }
        }
        else if (aux_cfg >= 97 && aux_cfg <= 120)
        { // 用遥控器对应通道进行舵机云台横滚控制(raw_data)
            double angleN45 = (cfg->Aux_StYTRolN45[0] - 1000) * 0.1;
            double angleP45 = (cfg->Aux_StYTRolP45[0] - 1000) * 0.1;
            double angle0 = (angleN45 + angleP45) / 2;
            double scale = (angleP45 - angle0) / 45.0;
            Quaternion quat;
            get_Airframe_quat(&quat);
            double roll = rad2degree(quat.getRoll());
            if (GimbalCtrl_LockedAtt[i] < 200)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - roll - 0) * scale + angleN45, i);

            uint8_t ref_chan = aux_cfg - 97;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
                { // 云台自动控制
                    // 通道变化大于阈值才调整云台
                    if (fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10)
                    {
                        float angle = (rc->raw_data[ref_chan] - 50.0) * (45.0 / 50.0) * aux_param1 + aux_param2;
                        angle -= roll;
                        angle = constrain(angle, cfg->Aux_YTRollMax[0]);
                        Aux_PWM_Out((angle - 0) * scale + angle0, i);
                        last_Channel_values[i] = rc->raw_data[ref_chan];
                        GimbalCtrl_LockedAtt[i] = 20000;
                    }
                }
                else
                { // 云台手动控制
                    float angle = (rc->raw_data[ref_chan] - 50.0) * (45.0 / 50.0) * aux_param1 + aux_param2;
                    angle -= roll;
                    angle = constrain(angle, cfg->Aux_YTRollMax[0]);
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                    last_Channel_values[i] = rc->raw_data[ref_chan];
                }
            }
            else
            { // 无遥控器信号
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 非自动控制且无遥控信号
                    // 锁定0角度
                    float angle = 0;
                    Aux_PWM_Out((angle - roll - 0) * scale + angle0, i);
                }
            }
        }
        else if (aux_cfg >= 121 && aux_cfg <= 137)
        { // 数字云台控制，在线程中处理，请勿删除
        }
        else if (aux_cfg >= 173 && aux_cfg <= 196)
        { // 用遥控器对应通道进行舵机云台俯仰往复控制(raw_data)
            double angle90 = (cfg->Aux_StYTPit90[0] - 1000) * 0.1;
            double angle0 = (cfg->Aux_StYTPit0[0] - 1000) * 0.1;
            double scale = (angle90 - angle0) / 90.0;

            uint32_t *speedSign = &channelTemp1[i];
            float *speed = &channelTemp2[i];
            float *currentAngle = &channelTemp3[i];

            uint8_t ref_chan = aux_cfg - 173;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                *speed = rc->raw_data[ref_chan] * aux_param1 + aux_param2;
                if (*speed < 0)
                    *speed = 0;
            }

            if (*speedSign)
                *currentAngle += (*speed) * h;
            else
                *currentAngle -= (*speed) * h;
            if (*currentAngle >= 90)
            {
                *currentAngle = 90;
                *speedSign = 0;
            }
            else if (*currentAngle <= 0)
            {
                *currentAngle = 0;
                *speedSign = 1;
            }
            Aux_PWM_Out((*currentAngle) * scale + angle0, i);
        }
        else if (aux_cfg >= 197 && aux_cfg <= 220)
        { // 用遥控器对应通道进行舵机云台横滚往复控制(raw_data)
            double angleN45 = (cfg->Aux_StYTRolN45[0] - 1000) * 0.1;
            double angleP45 = (cfg->Aux_StYTRolP45[0] - 1000) * 0.1;
            double angle0 = (angleN45 + angleP45) / 2;
            double scale = (angleP45 - angle0) / 45.0;

            uint32_t *speedSign = &channelTemp1[i];
            float *speed = &channelTemp2[i];
            float *currentAngle = &channelTemp3[i];

            uint8_t ref_chan = aux_cfg - 197;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                *speed = rc->raw_data[ref_chan] * aux_param1 + aux_param2;
                if (*speed < 0)
                    *speed = 0;
            }

            if (*speedSign)
                *currentAngle += (*speed) * h;
            else
                *currentAngle -= (*speed) * h;
            if (*currentAngle >= 45)
            {
                *currentAngle = 45;
                *speedSign = 0;
            }
            else if (*currentAngle <= -45)
            {
                *currentAngle = -45;
                *speedSign = 1;
            }

            Aux_PWM_Out((*currentAngle) * scale + angle0, i);
        }
        else if (aux_cfg >= 221 && aux_cfg <= 244)
        { // 用遥控器对应通道进行摇摆喷头控制(raw_data)
            double angle90 = (cfg->Aux_StYTPit90[0] - 1000) * 0.1;
            double angle0 = (cfg->Aux_StYTPit0[0] - 1000) * 0.1;
            double scale = (angle90 - angle0) / 90.0;

            uint32_t *swingSign = &channelTemp1[i];
            float *swingSpeed = &channelTemp2[i];
            float *swingAngle = &channelTemp3[i];

            uint8_t ref_chan = aux_cfg - 221;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                if (rc->raw_data[ref_chan] > 50)
                { // 开启摇摆
                    *swingSpeed = (rc->raw_data[ref_chan] - 50.0) * aux_param1;
                    if (*swingSpeed < 0)
                        *swingSpeed = 0;

                    // 摆动范围由 Aux_YTPitMin 和 Aux_YTPitMax 控制
                    float centerAngle = aux_param2;
                    float swingRange = (cfg->Aux_YTPitMax[0] - cfg->Aux_YTPitMin[0]) * 0.5;
                    if (swingRange > 45)
                        swingRange = 45;
                    float swingMin = centerAngle - swingRange;
                    float swingMax = centerAngle + swingRange;
                    // 限制在允许范围内
                    if (swingMin < cfg->Aux_YTPitMin[0])
                        swingMin = cfg->Aux_YTPitMin[0];
                    if (swingMax > cfg->Aux_YTPitMax[0])
                        swingMax = cfg->Aux_YTPitMax[0];

                    if (*swingSign)
                        *swingAngle += (*swingSpeed) * h;
                    else
                        *swingAngle -= (*swingSpeed) * h;
                    if (*swingAngle >= swingMax)
                    {
                        *swingAngle = swingMax;
                        *swingSign = 0;
                    }
                    else if (*swingAngle <= swingMin)
                    {
                        *swingAngle = swingMin;
                        *swingSign = 1;
                    }
                }
                else
                { // 关闭摇摆，回到中心角度
                    float centerAngle = aux_param2;
                    if (*swingAngle > centerAngle)
                    {
                        *swingAngle -= 60.0f * h;
                        if (*swingAngle < centerAngle)
                            *swingAngle = centerAngle;
                    }
                    else if (*swingAngle < centerAngle)
                    {
                        *swingAngle += 60.0f * h;
                        if (*swingAngle > centerAngle)
                            *swingAngle = centerAngle;
                    }
                    *swingSpeed = 0;
                }
            }
            else
            { // 无遥控信号，回中心
                float centerAngle = aux_param2;
                if (*swingAngle > centerAngle)
                {
                    *swingAngle -= 60.0f * h;
                    if (*swingAngle < centerAngle)
                        *swingAngle = centerAngle;
                }
                else if (*swingAngle < centerAngle)
                {
                    *swingAngle += 60.0f * h;
                    if (*swingAngle > centerAngle)
                        *swingAngle = centerAngle;
                }
            }

            Aux_PWM_Out((*swingAngle) * scale + angle0, i);
        }

        else if (aux_cfg >= 301 && aux_cfg <= 316)
        { // 用遥控器对应通道进行起落架控制(raw_data)
            // param1: 离地自动打开高度(m) 下降到低于该高度时自动放下起落架
            // param2: 离地自动关闭高度(m) 爬升到高于该高度时自动收起起落架
            // 建议param1 < param2形成迟滞区间; 两参数为0时仅手动控制
            // 遥控通道>60强制打开 <40强制关闭 中间位按高度自动控制
            uint32_t *gearState = &channelTemp1[i]; // 0-关闭(收起) 1-打开(放下)

            uint8_t ref_chan = aux_cfg - 301;
            bool manual_valid = false;
            bool manual_open = false;
            if (rc->connected && rc->raw_available_channels > ref_chan)
            {
                if (rc->raw_data[ref_chan] > 60)
                {
                    manual_valid = true;
                    manual_open = true;
                }
                else if (rc->raw_data[ref_chan] < 40)
                {
                    manual_valid = true;
                    manual_open = false;
                }
            }

            if (manual_valid)
            { // 手动控制起落架
                *gearState = manual_open ? 1 : 0;
            }
            else
            { // 高度自动控制(迟滞区间内保持当前状态)
                if (aux_param1 > 0 || aux_param2 > 0)
                {
                    vector3<double> pos;
                    double homeZ = 0;
                    if (get_Position_Ctrl(&pos, 0.01) && getHomeLocalZ(&homeZ, 0, 0.01))
                    {
                        double agl = (pos.z - homeZ) * 0.01; // 离地高度(m)
                        if (aux_param1 > 0 && agl <= aux_param1)
                            *gearState = 1; // 低于打开高度 自动放下
                        else if (aux_param2 > 0 && agl >= aux_param2)
                            *gearState = 0; // 高于关闭高度 自动收起
                    }
                }
            }

            // 输出PWM
            if (*gearState)
                Aux_PWM_Out((cfg->Aux_GearOpenPwm[0] - 1000) * 0.1, i);
            else
                Aux_PWM_Out((cfg->Aux_GearClosePwm[0] - 1000) * 0.1, i);
        }

        else if (aux_cfg == 950)
        { // 断药开关
            bool chanValue;
            Aux_ChannelRead(i, &chanValue);
            if (aux_param1 < 0)
                chanValue = !chanValue;
            if (chanValue && last_Channel_values[i] >= 0)
            { // 断药开关触发
                if (last_Channel_values[i] > 50)
                    TankRMPercent = -20;
                else
                    last_Channel_values[i] += 1;
            }
            else if (chanValue == false && last_Channel_values[i] <= 0)
            { // 断药开关未触发
                if (last_Channel_values[i] < -50)
                {
                    if (TankRMPercent <= 0)
                        TankRMPercent = 100;
                }
                else
                    last_Channel_values[i] -= 1;
            }
            else
                last_Channel_values[i] = 0;
        }

        else if (aux_cfg >= 1001 && aux_cfg <= 1016)
        { // 映射虚拟摇杆通道
            Receiver jrc;
            getRemoteReceiver(&jrc, 0);

            if (GimbalCtrl_LockedAtt[i] < 100000)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 1000) * 0.1, i);
            uint8_t ref_chan = aux_cfg - 1001;
            if (jrc.connected && jrc.raw_available_channels > ref_chan)
            {
                if (fabs(GimbalCtrl_LockedAtt[i]) < 100000 && last_Channel_values[i] > -100)
                { // Aux自动控制
                    // 通道变化大于阈值才调整Aux
                    if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10)
                    {
                        Aux_PWM_Out((jrc.raw_data[ref_chan] - 50.0) * aux_param1 + 50 + aux_param2 * 0.1, i);
                        last_Channel_values[i] = jrc.raw_data[ref_chan];
                        GimbalCtrl_LockedAtt[i] = 200000;
                    }
                }
                else
                { // Aux手动控制
                    Aux_PWM_Out((jrc.raw_data[ref_chan] - 50.0) * aux_param1 + 50 + aux_param2 * 0.1, i);
                    last_Channel_values[i] = jrc.raw_data[ref_chan];
                }
            }
            else
                last_Channel_values[i] = -200;
        }
        else if (aux_cfg >= 1025 && aux_cfg <= 1048)
        { // 用虚拟摇杆对应通道进行相机快门触发(raw_data)
            if (xSemaphoreTakeRecursive(CamMutex, 0))
            {
                if (camPhotoAsyncStep == 0)
                {
                    Receiver jrc;
                    getRemoteReceiver(&jrc, 0);

                    uint8_t ref_chan = aux_cfg - 1025;
                    if (jrc.connected && jrc.raw_available_channels > ref_chan)
                    {
                        if (last_Channel_values[i] > -100)
                        {
                            if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 15)
                            { // 触发相机
                                if (last_Channel_values[i] > -100)
                                {
                                    AuxCamTakePhoto();
                                }
                                last_Channel_values[i] = jrc.raw_data[ref_chan];
                            }
                            else
                                Aux_PWM_Out(cfg->Aux_CamOffPwm[0] * 0.1 - 100, i);
                        }
                        else
                        { // 旧通道数据不可用
                            Aux_PWM_Out(cfg->Aux_CamOffPwm[0] * 0.1 - 100, i);
                            last_Channel_values[i] = jrc.raw_data[ref_chan];
                        }
                    }
                    else
                    { // 无遥控器
                        Aux_PWM_Out(cfg->Aux_CamOffPwm[0] * 0.1 - 100, i);
                        last_Channel_values[i] = -200;
                    }
                }
                xSemaphoreGiveRecursive(CamMutex);
            }
        }
        else if (aux_cfg >= 1049 && aux_cfg <= 1072)
        { // 用虚拟摇杆对应通道进行无刷云台俯仰控制(raw_data)
            Receiver jrc;
            getRemoteReceiver(&jrc, 0);

            double angle90 = (cfg->Aux_BsYTPit90[0] - 1000) * 0.1;
            double angle0 = (cfg->Aux_BsYTPit0[0] - 1000) * 0.1;
            double scale = (angle90 - angle0) / 90.0;
            if (GimbalCtrl_LockedAtt[i] < 200)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - 0) * scale + angle0, i);

            uint8_t ref_chan = aux_cfg - 1049;
            if (jrc.connected && jrc.raw_available_channels > ref_chan)
            {
                if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
                { // 云台自动控制
                    // 通道变化大于阈值才调整云台
                    if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10)
                    {
                        float angle = (jrc.raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                        angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                        Aux_PWM_Out((angle - 0) * scale + angle0, i);
                        last_Channel_values[i] = jrc.raw_data[ref_chan];
                        GimbalCtrl_LockedAtt[i] = 200000;
                    }
                }
                else
                { // 云台手动控制
                    float angle = (jrc.raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                    angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                    last_Channel_values[i] = jrc.raw_data[ref_chan];
                }
            }
            else
            { // 无遥控信号
                last_Channel_values[i] = -200;
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 非自动控制且无遥控信叿
                    // 锁定0角度
                    float angle = 0;
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                }
            }
        }
        else if (aux_cfg >= 1073 && aux_cfg <= 1096)
        { // 用虚拟摇杆对应通道进行舵机云台俯仰控制(raw_data)
            Receiver jrc;
            getRemoteReceiver(&jrc, 0);

            double angle90 = (cfg->Aux_StYTPit90[0] - 1000) * 0.1;
            double angle0 = (cfg->Aux_StYTPit0[0] - 1000) * 0.1;
            double scale = (angle90 - angle0) / 90.0;
            Quaternion quat;
            get_Airframe_quat(&quat);
            double pitch = rad2degree(quat.getPitch());
            if (GimbalCtrl_LockedAtt[i] < 200)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - pitch - 0) * scale + angle0, i);

            uint8_t ref_chan = aux_cfg - 1073;
            if (jrc.connected && jrc.raw_available_channels > ref_chan)
            {
                if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
                { // 云台自动控制
                    // 通道变化大于阈值才调整云台
                    if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10)
                    {
                        float angle = (jrc.raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                        angle -= pitch;
                        angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                        Aux_PWM_Out((angle - 0) * scale + angle0, i);
                        last_Channel_values[i] = jrc.raw_data[ref_chan];
                        GimbalCtrl_LockedAtt[i] = 20000;
                    }
                }
                else
                { // 云台手动控制
                    float angle = (jrc.raw_data[ref_chan] - 50.0) * (60.0 / 50.0) * aux_param1 + 45 + aux_param2;
                    angle -= pitch;
                    angle = constrain(angle, cfg->Aux_YTPitMin[0], cfg->Aux_YTPitMax[0]);
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                    last_Channel_values[i] = jrc.raw_data[ref_chan];
                }
            }
            else
            { // 无遥控信号
                last_Channel_values[i] = -200;
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 非自动控制且无遥控信叿
                    // 锁定0角度
                    float angle = 0;
                    Aux_PWM_Out((angle - pitch - 0) * scale + angle0, i);
                }
            }
        }
        else if (aux_cfg >= 1097 && aux_cfg <= 1120)
        { // 用虚拟摇杆对应通道进行舵机云台横滚控制(raw_data)
            Receiver jrc;
            getRemoteReceiver(&jrc, 0);

            double angleN45 = (cfg->Aux_StYTRolN45[0] - 1000) * 0.1;
            double angleP45 = (cfg->Aux_StYTRolP45[0] - 1000) * 0.1;
            double angle0 = (angleN45 + angleP45) / 2;
            double scale = (angleP45 - angle0) / 45.0;
            Quaternion quat;
            get_Airframe_quat(&quat);
            double roll = rad2degree(quat.getRoll());
            if (GimbalCtrl_LockedAtt[i] < 200)
                Aux_PWM_Out((GimbalCtrl_LockedAtt[i] - roll - 0) * scale + angleN45, i);

            uint8_t ref_chan = aux_cfg - 97;
            if (jrc.connected && jrc.raw_available_channels > ref_chan)
            {
                if (fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i] > -100)
                { // 云台自动控制
                    // 通道变化大于阈值才调整云台
                    if (fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10)
                    {
                        float angle = (jrc.raw_data[ref_chan] - 50.0) * (45.0 / 50.0) * aux_param1 + aux_param2;
                        angle -= roll;
                        angle = constrain(angle, cfg->Aux_YTRollMax[0]);
                        Aux_PWM_Out((angle - 0) * scale + angle0, i);
                        last_Channel_values[i] = jrc.raw_data[ref_chan];
                        GimbalCtrl_LockedAtt[i] = 20000;
                    }
                }
                else
                { // 云台手动控制
                    float angle = (jrc.raw_data[ref_chan] - 50.0) * (45.0 / 50.0) * aux_param1 + aux_param2;
                    angle -= roll;
                    angle = constrain(angle, cfg->Aux_YTRollMax[0]);
                    Aux_PWM_Out((angle - 0) * scale + angle0, i);
                    last_Channel_values[i] = jrc.raw_data[ref_chan];
                }
            }
            else
            { // 无遥控器信号
                if (GimbalCtrl_LockedAtt[i] > 200)
                { // 非自动控制且无遥控信叿
                    // 锁定0角度
                    float angle = 0;
                    Aux_PWM_Out((angle - roll - 0) * scale + angle0, i);
                }
            }
        }
        else if (aux_cfg >= 5001 && aux_cfg <= 5003)
        { // 物理按键一键功能（低电平触发，弹起执行，超时0.7s无效）
            // aux_param1 取整: 触发所需点击次数(0和1=单击, 2=双击, 3=三击...)
            // channelTemp1[i]: 按键按下标志(0=空闲, 1=按下中)
            // channelTemp2[i]: 计时器(按下时=持续时间, 弹起时=等待下次点击倒计时)
            // channelTemp3[i]: 已累计的有效点击次数
            uint32_t *isPressed = &channelTemp1[i];
            float *timer = &channelTemp2[i];
            float *clickCount = &channelTemp3[i];
            uint32_t targetClicks = (uint32_t) (aux_param1 + 0.5f);
            if (targetClicks <= 1)
                targetClicks = 1; // 0和1都是单击

            // 读取物理按键（低电平=按下）
            bool pinLow = false;
            bool readOk = Aux_ChannelRead(i, &pinLow);
            pinLow = !pinLow;

            if (readOk && pinLow)
            { // 按键按下（低电平）
                if (*isPressed == 0)
                { // 首次检测到按下
                    *isPressed = 1;
                    *timer = 0;
                }
                else
                { // 持续按下，累计时间
                    *timer += h;
                    if (*timer > 0.7)
                    { // 按住超过0.7s，整组点击作废
                        *isPressed = 0;
                        *timer = 0;
                        *clickCount = 0;
                    }
                }
            }
            else
            { // 按键弹起（高电平或读取失败）
                if (*isPressed)
                {
                    if (*timer > 0.15 && *timer <= 0.7)
                    { // 有效短按弹起
                        *clickCount += 1;
                        if (*clickCount == targetClicks)
                        { // 达到目标点击次数 → 执行
                            *clickCount = 0;

                            ModeMsg mode_msg;
                            mode_msg.cmd_type = CMD_TYPE_RC_AUX;
                            mode_msg.sd_sysid = 0;
                            mode_msg.sd_compid = 0;
                            mode_msg.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
                            mode_msg.params[0] = 0;
                            mode_msg.params[1] = 0;
                            mode_msg.params[2] = 0;
                            mode_msg.params[3] = 0;
                            mode_msg.params[4] = 0;
                            mode_msg.params[5] = 0;
                            mode_msg.params[6] = 0;

                            if (aux_cfg == 5001)
                            { // 一键起飞
                                mode_msg.cmd = 24;
                                mode_msg.params[6] = aux_param2; // 起飞高度(米)
                                SendMsgToMode(mode_msg, 0.01);
                            }
                            else if (aux_cfg == 5002)
                            { // 一键返航
                                mode_msg.cmd = 176;
                                mode_msg.params[0] = (uint32_t) MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                                mode_msg.params[1] = PX4_CUSTOM_MAIN_MODE_AUTO;
                                mode_msg.params[2] = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
                                SendMsgToMode(mode_msg, 0.01);
                            }
                            else if (aux_cfg == 5003)
                            { // 一键降落
                                mode_msg.cmd = 176;
                                mode_msg.params[0] = (uint32_t) MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                                mode_msg.params[1] = PX4_CUSTOM_MAIN_MODE_AUTO;
                                mode_msg.params[2] = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
                                SendMsgToMode(mode_msg, 0.01);
                            }
                        }
                    }
                    else
                    { // 无效点击（太短），作废整组
                        *clickCount = 0;
                    }
                    *isPressed = 0;
                    *timer = 0; // 弹起后开始等待下一次点击的倒计时由下面处理
                }
                else
                { // 空闲中（未按下），如果有未完成的点击组，等待超时
                    if (*clickCount > 0)
                    {
                        *timer += h;
                        if (*timer > 0.6)
                        { // 0.6秒内没有新点击，整组作废
                            *clickCount = 0;
                            *timer = 0;
                        }
                    }
                }
            }
        }
        else if (aux_cfg == 5032)
        { // 延迟切入任务模式（短按启动延迟，延迟期间长按撤销）
            // channelTemp1[i]: 状态机(0=空闲, 1=按下中, 2=延迟倒计时中, 3=撤销长按中, 4=等待下次点击, 5=松开等待)
            // channelTemp2[i]: 通用计时器(按下时长/弹起等待超时/报警间隔/撤销长按时长)
            // channelTemp3[i]: case0/1/4=连击计数; case2=剩余延迟秒数; case3=未使用
            // aux_param1: 触发所需点击次数(0和1=单击, 2=双击...)
            // aux_param2: 任务起点模式 0=继续当前, 1=从头, 2/3/4=跳到第(param2-1)次降落/返航后
            uint32_t *state = &channelTemp1[i];
            float *timer = &channelTemp2[i];
            float *val = &channelTemp3[i]; // case0/1/4: 连击计数; case2: 剩余延迟秒数
            uint32_t targetClicks = (uint32_t) (aux_param1 + 0.5f);
            if (targetClicks <= 1)
                targetClicks = 1;

            // 读取物理按键（低电平=按下）
            bool pinLow = false;
            bool readOk = Aux_ChannelRead(i, &pinLow);
            pinLow = !pinLow;

            switch (*state)
            {
                case 0:       // 空闲
                    *val = 0; // 复位连击计数
                    if (readOk && pinLow)
                    { // 首次检测到按下
                        *state = 1;
                        *timer = 0;
                    }
                    break;
                case 1: // 按下中（与5001-5003完全一致的防抖逻辑）
                    if (readOk && pinLow)
                    { // 持续按下，累计时间
                        *timer += h;
                        if (*timer > 0.7)
                        { // 按住超过0.7s，整组点击作废
                            *state = 0;
                            *timer = 0;
                            *val = 0;
                        }
                    }
                    else
                    { // 按键弹起
                        if (*timer > 0.15 && *timer <= 0.7)
                        {              // 有效短按弹起
                            *val += 1; // 连击计数
                            if ((uint32_t) (*val) >= targetClicks)
                            { // 达到点击次数，启动延迟倒计时
                                *state = 2;
                                *val = 45.0; // 剩余延迟秒数
                                *timer = 0;
                                sendLedSignal(LEDSignal_Start1);
                            }
                            else
                            { // 还需要更多点击，等待下一次
                                *state = 4;
                                *timer = 0;
                            }
                        }
                        else
                        { // 无效点击（太短），作废整组
                            *state = 0;
                            *timer = 0;
                            *val = 0;
                        }
                    }
                    break;
                case 4: // 等待下次点击（空闲中，等待新一轮按下）
                    *timer += h;
                    if (*timer > 0.6)
                    { // 0.6秒无新点击，整组作废
                        *state = 0;
                        *timer = 0;
                        *val = 0;
                    }
                    else if (readOk && pinLow)
                    { // 再次按下
                        *state = 1;
                        *timer = 0;
                    }
                    break;
                case 2:        // 延迟倒计时中
                    *val -= h; // 剩余延迟秒数递减
                    // 每隔几秒报警一次
                    *timer += h;
                    if (*timer >= 3.0)
                    {
                        *timer = 0;
                        sendLedSignal(LEDSignal_Continue1);
                    }
                    if (*val <= 0)
                    {               // 延迟结束，设置起点并切入任务模式
                        *state = 5; // 进入松开等待，防止误触发
                        *val = 0;
                        *timer = 0;

                        // aux_param2: 任务起点模式 0=继续当前, 1=从头, 2/3/4=跳到第(param2-1)次降落/返航后的第一个航点
                        uint32_t startMode = (uint32_t) (aux_param2 + 0.5f);
                        if (startMode == 1)
                        { // 从头开始
                            setCurrentMission(0);
                            reset_RestoreWpInf(restoreWpInf);
                            storeRAM_restoreWpInf();
                        }
                        else if (startMode >= 2)
                        { // 跳到第(startMode-1)个降落后的第一个航点
                            uint32_t targetLandCount = startMode - 1;
                            uint32_t landCount = 0;
                            uint16_t wpCount = getMissionsCount();
                            uint16_t jumpTo = 0;
                            bool foundLand = false;
                            for (uint16_t mi = 0; mi < wpCount; ++mi)
                            {
                                MissionInf rwp;
                                if (ReadMission(mi, &rwp))
                                {
                                    if (rwp.cmd == MAV_CMD_NAV_LAND || rwp.cmd == MAV_CMD_NAV_RETURN_TO_LAUNCH || rwp.cmd == MAV_CMD_NAV_LAND_LOCAL)
                                    {
                                        ++landCount;
                                        if (landCount == targetLandCount)
                                            foundLand = true;
                                    }
                                    else if (foundLand && rwp.cmd == MAV_CMD_NAV_WAYPOINT)
                                    { // 降落后的第一个航点
                                        jumpTo = mi;
                                        break;
                                    }
                                }
                            }
                            if (jumpTo > 0)
                            {
                                setCurrentMission(jumpTo);
                                reset_RestoreWpInf(restoreWpInf);
                                storeRAM_restoreWpInf();
                            }
                            else
                            { // 未找到匹配航点，从头开始
                                setCurrentMission(0);
                                reset_RestoreWpInf(restoreWpInf);
                                storeRAM_restoreWpInf();
                            }
                        }
                        // startMode == 0: 继续，不改航点，Mission模式自动恢复

                        // 发切任务模式指令
                        {
                            ModeMsg mode_msg;
                            mode_msg.cmd_type = CMD_TYPE_RC_AUX;
                            mode_msg.sd_sysid = 0;
                            mode_msg.sd_compid = 0;
                            mode_msg.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
                            mode_msg.cmd = 176;
                            mode_msg.params[0] = (uint32_t) MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                            mode_msg.params[1] = PX4_CUSTOM_MAIN_MODE_AUTO;
                            mode_msg.params[2] = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
                            mode_msg.params[3] = 0;
                            mode_msg.params[4] = 0;
                            mode_msg.params[5] = 0;
                            mode_msg.params[6] = 0;
                            SendMsgToMode(mode_msg, 0.01);
                        }
                        // 发解锁指令（如果尚未解锁）
                        {
                            ModeMsg arm_msg;
                            arm_msg.cmd_type = CMD_TYPE_RC_AUX;
                            arm_msg.sd_sysid = 0;
                            arm_msg.sd_compid = 0;
                            arm_msg.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
                            arm_msg.cmd = MAV_CMD_COMPONENT_ARM_DISARM;
                            arm_msg.params[0] = 1; // arm
                            arm_msg.params[1] = 0;
                            arm_msg.params[2] = 0;
                            arm_msg.params[3] = 0;
                            arm_msg.params[4] = 0;
                            arm_msg.params[5] = 0;
                            arm_msg.params[6] = 0;
                            SendMsgToMode(arm_msg, 0.01);
                        }
                        break;
                    }
                    // 检测撤销长按
                    if (readOk && pinLow)
                    { // 倒计时期间按下 → 进入撤销长按检测
                        *state = 3;
                        *timer = 0;
                    }
                    break;
                case 3: // 撤销长按中
                    if (readOk && pinLow)
                    {
                        *timer += h;
                        if (*timer > 0.7)
                        {               // 长按超过0.7s，撤销
                            *state = 5; // 进入松开等待，防止松开后立刻重新触发
                            *timer = 0;
                            *val = 0;
                            sendLedSignal(LEDSignal_Err1);
                        }
                    }
                    else
                    { // 太短不算撤销，回到延迟倒计时
                        *state = 2;
                        *timer = 0;
                    }
                    break;
                case 5: // 撤销后等待按键释放
                    if (readOk && !pinLow)
                    { // 按键已释放，回空闲
                        *state = 0;
                        *timer = 0;
                    }
                    break;
            }
        }
    }

    //	/*数字云台接口*/
    //		for( uint8_t i = 0; i < 8; ++i )
    //		{	//横滚 俯仰 偏航 变??
    //			//模式 聚焦 拍照录像 归位
    //			uint16_t aux_cfg = ((uint16_t*)&cfg->DgYTRFunc)[i*4];
    //			float aux_param1 = ((float*)&cfg->DgYTRParam1)[i*2];
    //			float aux_param2 = ((float*)&cfg->DgYTRParam2)[i*2];
    //
    //			if( aux_cfg>=1 && aux_cfg<=16 )
    //			{	//遥控器通道控制角度
    //				if( rc->connected )
    //				{
    //					uint8_t ref_chan = aux_cfg - 1;
    //					if( rc->raw_available_channels > ref_chan )
    //					{
    //						float value;
    //						if( i == DgYTZoom )	//缩放通道特殊处理
    //							value = rc->raw_data[ref_chan]*0.3*aux_param1+aux_param2*0.1;
    //						else
    //							value = (rc->raw_data[ref_chan]-50.0)*aux_param1+aux_param2*0.1;
    //
    //						set_DgYTCtrl( i,
    //							0, //0-角度模式 1-角速度模式
    //							value,
    //							0.1 );
    //					}
    //				}
    //			}
    //			else if( aux_cfg>=25 && aux_cfg<=48 )
    //			{	//遥控器通道控制角度
    //				if( rc->connected )
    //				{
    //					uint8_t ref_chan = aux_cfg - 25;
    //					if( rc->raw_available_channels > ref_chan )
    //					{
    //						set_DgYTCtrl( i,
    //							1, //0-角度模式 1-角速度模式
    //							(rc->raw_data[ref_chan]-50.0)*aux_param1+aux_param2*0.1,
    //							0.1 );
    //					}
    //				}
    //			}
    //		}
    //	/*数字云台接口*/
}

bool AuxGimbalSetAngle(double angle)
{
    readAuxConfig();
    AuxFuncsConfig *cfg = getAuxFixedCfg();

    uint8_t MainMotorsCount = get_MainMotorCount();
    for (uint8_t i = MainMotorsCount; i < MAX_PWM_COUNT; ++i)
    {
        uint8_t aux_cfg = (uint8_t) getAuxChanFunc(i);
        if (aux_cfg >= 49 && aux_cfg <= 72)
        { // 无刷云台俯仰
            angle = constrain(angle, (double) cfg->Aux_YTPitMin[0], (double) cfg->Aux_YTPitMax[0]);
            GimbalCtrl_LockedAtt[i] = angle;
        }
        else if (aux_cfg >= 73 && aux_cfg <= 96)
        { // 舵机云台俯仰
            angle = constrain(angle, (double) cfg->Aux_YTPitMin[0], (double) cfg->Aux_YTPitMax[0]);
            GimbalCtrl_LockedAtt[i] = angle;
        }
        else if (aux_cfg >= 97 && aux_cfg <= 120)
        { // 舵机云台横滚
            angle = constrain(angle, (double) cfg->Aux_YTRollMax[0]);
            GimbalCtrl_LockedAtt[i] = angle;
        }
    }
    return true;
}

void init_AuxFuncs()
{
    // 注册相机热靴捕获中断回调
    register_InputTriggerCallback(camTrigCallback);

    // 注册消息队列
    TaskQueueRegister(CAMERA_FEEDBACK_QUEUE_ID, 30);

    // 根据MAX_PWM_COUNT动态注册参数（物理通道+CAN扩展虚拟通道）
    // 缓冲区布局: [AuxFuncsConfig固定部分] + [AuxFunc[n][4]] + [AuxParam1[n][2]] + [AuxParam2[n][2]]
    uint8_t aux_count = MAX_PWM_COUNT;
    g_aux_ch_count = aux_count;
    uint32_t fixed_count = sizeof(AuxFuncsConfig) / 8; // 固定部分参数个数
    uint32_t total_count = fixed_count + (uint32_t) aux_count * 3;

    g_aux_cfg_buf = new uint8_t[sizeof(AuxFuncsConfig) + (uint32_t) aux_count * 3 * 8]();
    AuxFuncsConfig *initial_cfg = (AuxFuncsConfig *) g_aux_cfg_buf;

    // 初始化固定部分默认值
    initial_cfg->auxCfg[0] = 0;
    initial_cfg->stLockParam[0] = 1000;
    initial_cfg->Aux_YTSurveyPic[0] = 0;
    initial_cfg->Aux_CamOnPwm[0] = 2000;
    initial_cfg->Aux_CamOffPwm[0] = 1000;
    initial_cfg->Aux_CamShTime[0] = 0.1f;
    initial_cfg->Aux_CamTrigEna[0] = 0;
    initial_cfg->Aux_BsYTPit0[0] = 1000;
    initial_cfg->Aux_BsYTPit90[0] = 2000;
    initial_cfg->Aux_StYTPit0[0] = 1250;
    initial_cfg->Aux_StYTPit90[0] = 1750;
    initial_cfg->Aux_StYT2Pit0[0] = 1250;
    initial_cfg->Aux_StYT2Pit90[0] = 1750;
    initial_cfg->Aux_StYTRolN45[0] = 1000;
    initial_cfg->Aux_StYTRolP45[0] = 2000;
    initial_cfg->Aux_YTPitMin[0] = -20.0f;
    initial_cfg->Aux_YTPitMax[0] = 120.0f;
    initial_cfg->Aux_YTRollMax[0] = 45.0f;
    initial_cfg->Aux_Pump1Min[0] = 1000.0f;
    initial_cfg->Aux_Pump1St[0] = 1100.0f;
    initial_cfg->Aux_Pump1Max[0] = 1950.0f;
    initial_cfg->Aux_Pump1Sp[0] = 0.15f;
    initial_cfg->Aux_GearOpenPwm[0] = 2000;
    initial_cfg->Aux_GearClosePwm[0] = 1000;
    /* 遥控通道1-16对应的数字云台功能与参数初始值 */
    for (uint8_t j = 0; j < 16; ++j)
    {
        initial_cfg->RcYTFunc[j][0] = 0;
        initial_cfg->RcYTParam1[j][0] = 1.0f;
        initial_cfg->RcYTParam2[j][0] = 1.0f;
    }

    // 初始化动态Aux通道部分默认值
    // Func区: offset = sizeof(AuxFuncsConfig), aux_count * 8字节
    // Param1区: offset = sizeof(AuxFuncsConfig) + aux_count*8
    // Param2区: offset = sizeof(AuxFuncsConfig) + aux_count*8*2
    uint16_t *aux_func_base = (uint16_t *) (g_aux_cfg_buf + sizeof(AuxFuncsConfig));
    float *aux_param1_base = (float *) (g_aux_cfg_buf + sizeof(AuxFuncsConfig) + (uint32_t) aux_count * 8);
    float *aux_param2_base = (float *) (g_aux_cfg_buf + sizeof(AuxFuncsConfig) + (uint32_t) aux_count * 8 * 2);
    for (uint8_t j = 0; j < aux_count; ++j)
    {
        aux_func_base[j * 4] = 0;      // Func默认无功能
        aux_param1_base[j * 2] = 1.0f; // Param1默认1
        aux_param2_base[j * 2] = 0.0f; // Param2默认0
    }

    // 构建参数类型数组
    MAV_PARAM_TYPE *param_types = new MAV_PARAM_TYPE[total_count];
    SName *param_names = new SName[total_count];

    // 固定部分（与AuxFuncsConfig结构体成员顺序完全对应）
    uint32_t p = 0;
    param_types[p] = MAV_PARAM_TYPE_UINT32;
    param_names[p++] = "Aux_cfg"; // auxCfg
    param_types[p] = MAV_PARAM_TYPE_UINT32;
    param_names[p++] = "Aux_stLockParam"; // stLockParam
    param_types[p] = MAV_PARAM_TYPE_UINT8;
    param_names[p++] = "Aux_YTSurveyPic";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_CamOnPwm";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_CamOffPwm";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "Aux_CamShTime";
    param_types[p] = MAV_PARAM_TYPE_UINT8;
    param_names[p++] = "Aux_CamTrigEna";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_BsYTPit0";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_BsYTPit90";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_StYTPit0";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_StYTPit90";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_StYT2Pit0";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_StYT2Pit90";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_StYTRolN45";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_StYTRolP45";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "Aux_StPitMin";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "Aux_StPitMax";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "Aux_StRolMax";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "Aux_Pump1Min";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "Aux_Pump1St";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "Aux_Pump1Max";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "Aux_Pump1Sp";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_GearOpenPwm";
    param_types[p] = MAV_PARAM_TYPE_UINT16;
    param_names[p++] = "Aux_GearClosePwm";
    // RcYTFunc[16]
    for (uint8_t j = 0; j < 16; ++j)
    {
        char buf[16];
        sprintf(buf, "Aux_RcYT%dFunc", j + 1);
        param_types[p] = MAV_PARAM_TYPE_UINT16;
        param_names[p++] = buf;
    }
    // RcYTParam1[16]
    for (uint8_t j = 0; j < 16; ++j)
    {
        char buf[17];
        sprintf(buf, "Aux_RcYT%dParam1", j + 1);
        param_types[p] = MAV_PARAM_TYPE_REAL32;
        param_names[p++] = buf;
    }
    // RcYTParam2[16]
    for (uint8_t j = 0; j < 16; ++j)
    {
        char buf[17];
        sprintf(buf, "Aux_RcYT%dParam2", j + 1);
        param_types[p] = MAV_PARAM_TYPE_REAL32;
        param_names[p++] = buf;
    }
    // 动态Aux通道部分: 所有Func, 再所有Param1, 再所有Param2
    for (uint8_t j = 0; j < aux_count; ++j)
    {
        char buf[16];
        sprintf(buf, "Aux_%dFunc", j + 1);
        param_types[p] = MAV_PARAM_TYPE_UINT16;
        param_names[p++] = buf;
    }
    for (uint8_t j = 0; j < aux_count; ++j)
    {
        char buf[16];
        sprintf(buf, "Aux_%dParam1", j + 1);
        param_types[p] = MAV_PARAM_TYPE_REAL32;
        param_names[p++] = buf;
    }
    for (uint8_t j = 0; j < aux_count; ++j)
    {
        char buf[16];
        sprintf(buf, "Aux_%dParam2", j + 1);
        param_types[p] = MAV_PARAM_TYPE_REAL32;
        param_names[p++] = buf;
    }

    InitialParametersInit((uint64_t *) g_aux_cfg_buf, param_types, param_names, total_count);
    ParamGroupRegister("AuxCfg", 1, total_count, param_types, param_names, (uint64_t *) g_aux_cfg_buf);
    delete[] param_types;
    delete[] param_names;
}
