#include "drv_msp_DisplayPort.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "StorageSystem.hpp"
#include "drv_msp.hpp"
#include "task.h"

struct DriverInfo
{
    uint32_t param;
    Port port;
};

/**
 * @brief DisplayPort֡����
 */
typedef enum
{
    DISPLAYPORT_HEARTBEAT = 0,    // ����
    DISPLAYPORT_CLEAR_SCREEN = 2, // ����
    DISPLAYPORT_WRITE_STRING = 3, // д���ַ���
    DISPLAYPORT_DRAW_SCREEN = 4,  // ˢ����ʾ
    DISPLAYPORT_SET_OPTIONS = 5,  // ����
} displayport_cmd_e;

/**
 * @brief ����DisplayPort��������
 * @param port MSP�˿�
 */
static void displayport_clear_screen(msp_port_t *port, DriverInfo *driver_info)
{
    uint8_t data[1] = {DISPLAYPORT_CLEAR_SCREEN};
    msp_packet_t packet;
    packet.cmd = MSP_DISPLAYPORT;
    packet.flags = 0;
    packet.result = MSP_RESULT_ACK;
    packet.data = data;
    packet.len = sizeof(data);
    msp_send_response(port, &packet);
    // ��ȡ����õ����ݣ����͵�����
    const uint8_t *resp = msp_get_out_buf(port);
    uint16_t len = msp_get_out_len(port);
    uint8_t wbuff[64];
    memcpy(wbuff, resp, len);
    driver_info->port.write(wbuff, len, portMAX_DELAY, portMAX_DELAY);
}

/**
 * @brief ��OSDָ��λ����ʾ�ַ���
 * @param port MSP�˿�
 * @param row �к� (0-19, ȡ����OSD�ֱ���)
 * @param col �к� (0-31, ȡ����OSD�ֱ���)
 * @param text Ҫ��ʾ���ַ���
 */
static void displayport_write_string(msp_port_t *port, DriverInfo *driver_info, uint8_t row, uint8_t col, const char *text)
{
    uint8_t data[64];
    uint8_t len = 0;
    // ����DisplayPort���ݰ�
    data[len++] = DISPLAYPORT_WRITE_STRING;
    data[len++] = row;
    data[len++] = col;
    data[len++] = 0; // ���ԣ���ͨ�ı���

    // �����ַ���
    while (*text && len < sizeof(data) - 5)
    {
        data[len++] = *text++;
    }

    msp_packet_t packet;
    packet.cmd = MSP_DISPLAYPORT;
    packet.flags = 0;
    packet.result = MSP_RESULT_ACK;
    packet.data = data;
    packet.len = len;
    msp_send_response(port, &packet);
    // ��ȡ����õ�����
    const uint8_t *resp = msp_get_out_buf(port);
    uint16_t resp_len = msp_get_out_len(port);
    uint8_t wbuff[64];
    memcpy(wbuff, resp, resp_len);
    driver_info->port.write(wbuff, resp_len, portMAX_DELAY, portMAX_DELAY);
}

/**
 * @brief ˢ����ʾ���ύ���и��ģ�
 * @param port MSP�˿�
 */
static void displayport_draw_screen(msp_port_t *port, DriverInfo *driver_info)
{
    uint8_t data[1] = {DISPLAYPORT_DRAW_SCREEN};
    msp_packet_t packet;
    packet.cmd = MSP_DISPLAYPORT;
    packet.flags = 0;
    packet.result = MSP_RESULT_ACK;
    packet.data = data;
    packet.len = sizeof(data);
    msp_send_response(port, &packet);
    const uint8_t *resp = msp_get_out_buf(port);
    uint16_t len = msp_get_out_len(port);
    uint8_t wbuff[64];
    memcpy(wbuff, resp, len);
    driver_info->port.write(wbuff, len, portMAX_DELAY, portMAX_DELAY);
}

/**
 * @brief ����DisplayPort������
 * @param port MSP�˿�
 *
 * ˵����
 * - ���������ڱ���DisplayPort���ӻ�Ծ
 * - ����ÿ100-500ms����һ��
 * - ĳЩOSD�豸�����ʱ��δ�յ��������˳�DisplayPortģʽ
 */
static void displayport_heartbeat(msp_port_t *port, DriverInfo *driver_info)
{
    uint8_t data[1] = {DISPLAYPORT_HEARTBEAT};
    msp_packet_t packet;
    packet.cmd = MSP_DISPLAYPORT;
    packet.flags = 0;
    packet.result = MSP_RESULT_ACK;
    packet.data = data;
    packet.len = sizeof(data);
    msp_send_response(port, &packet);
    const uint8_t *resp = msp_get_out_buf(port);
    uint16_t len = msp_get_out_len(port);
    uint8_t wbuff[64];
    memcpy(wbuff, resp, len);
    driver_info->port.write(wbuff, len, portMAX_DELAY, portMAX_DELAY);
}

// ��OSD����ʾ�����ַ�ӳ���
void display_char_map(msp_port_t *port, DriverInfo *driver_info)
{
    uint8_t wbuff[128];
    uint16_t len;

    // ����
    displayport_clear_screen(port, driver_info);
    os_delay(0.05);

    // ��ʾÿ���ַ���16x16����
    for (int row = 0; row < 20; row++)
    {
        for (int col = 0; col < 19; col++)
        {
            int code = row * 16 + col;
            char buf[2];
            buf[0] = 'A';
            buf[1] = '\0';

            displayport_write_string(port, driver_info, row, col * 2, buf);
        }
    }

    // ˢ��
    displayport_draw_screen(port, driver_info);
}

static void MSP_DisplayPort_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    // �ȴ���ʼ�����
    while (get_Altitude_MSStatus() != MS_Ready)
        os_delay(1);

    msp_port_t port;
    msp_init(&port);
    TIME heartbeat_time = TIME::now();
    uint8_t buff[24];

    while (1)
    { // 20*53Ļ��
        if (heartbeat_time.get_pass_time() >= 0.5)
        {
            // ������
            heartbeat_time = TIME::now();
            displayport_heartbeat(&port, &driver_info);
        }

        // ����
        displayport_clear_screen(&port, &driver_info);

        // ��̬��ȡ
        Quaternion quat;
        get_history_AirframeQuatY(&quat, 0.1);
        int roll = rad2degree(quat.getRoll());
        int pitch = rad2degree(quat.getPitch());
        int yaw = rad2degree(quat.getYaw());
        // ����
        sprintf((char *) buff, "%c%-4.1d", 21, pitch);
        displayport_write_string(&port, &driver_info, 0, 22, (char *) buff);
        // ���
        sprintf((char *) buff, "%c%-4.1d", 20, roll);
        displayport_write_string(&port, &driver_info, 0, 28, (char *) buff);

        // ģʽ��ʾ
        AFunc mode = getCurrentFlyMode();
        switch (mode)
        {
            case AFunc_AltHold:
            {
                sprintf((char *) buff, "ALTHOLD");
                break;
            }
            case AFunc_PosHold:
            {
                sprintf((char *) buff, "POSHOLD");
                break;
            }
            case AFunc_Mission:
            {
                sprintf((char *) buff, "MISSION");
                break;
            }
            case AFunc_RTL:
            {
                sprintf((char *) buff, "RTL ");
                break;
            }
            case AFunc_Land:
            {
                sprintf((char *) buff, "LAND");
                break;
            }
            case AFunc_Manual:
            { // �ֶ�
                sprintf((char *) buff, "MANUAL");
                break;
            }
            case AFunc_Stabilize:
            {
                // ����
                sprintf((char *) buff, "STABILIZE");
                break;
            }
            case AFunc_Rattitude:
            { // ������
                sprintf((char *) buff, "RATTITUDE");
                break;
            }
            default:
            {
                sprintf((char *) buff, "NULL");
                break;
            }
        }
        displayport_write_string(&port, &driver_info, 18, 22, (char *) buff);

        // ��Ҹ߶�
        static bool last_inFlight = false;
        bool inFlight;
        get_is_inFlight(&inFlight);
        static double homeZ = 0;
        if (inFlight != last_inFlight)
        {
            getHomeLocalZ(&homeZ, 0, 0.01);
            last_inFlight = inFlight;
        }
        vector3<double> position;
        get_Position_Ctrl(&position);
        double heightAboveGround = position.z - homeZ;

        sprintf((char *) buff, "%c%-.0f%c", 127, heightAboveGround * 0.01, 12);
        displayport_write_string(&port, &driver_info, 11, 0, (char *) buff);

        // �����ٶ�
        vector3<double> vel;
        get_VelocityENU_Ctrl(&vel);
        double Gs = safe_sqrt(vel.x * vel.x + vel.y * vel.y) * 0.01;

        sprintf((char *) buff, "%c%-.0f%c", 112, Gs * 3.6, 158);
        displayport_write_string(&port, &driver_info, 12, 0, (char *) buff);

        // ����λ��
        Receiver rc;
        getReceiver(&rc, 0, 0.02);
        if (rc.available)
        {
            sprintf((char *) buff, "%c%-.0f", 4, rc.data[0]);
            displayport_write_string(&port, &driver_info, 19, 0, (char *) buff);
        }

        // ���
        BatteryInfo batInfo;
        getCurrentBatteryInfo(&batInfo);
        uint8_t bat_chat;
        if (batInfo.totalPercent >= 95.0)
            bat_chat = 144;
        else if (batInfo.totalPercent >= 75.0)
            bat_chat = 145;
        else if (batInfo.totalPercent >= 55.0)
            bat_chat = 146;
        else if (batInfo.totalPercent >= 35.0)
            bat_chat = 147;
        else if (batInfo.totalPercent >= 15.0)
            bat_chat = 148;
        else if (batInfo.totalPercent >= 5.0)
            bat_chat = 149;
        else
            bat_chat = 150;

        sprintf((char *) buff, "%c%-.1f%c", bat_chat, batInfo.totalVoltRawFilted, 6);
        displayport_write_string(&port, &driver_info, 19, 5, (char *) buff);
        sprintf((char *) buff, "%-.1f%c", batInfo.totalCurrent, 154);
        displayport_write_string(&port, &driver_info, 19, 15, (char *) buff);

        // ����¶�
        extern float esc_temperature;
        if (esc_temperature != -1)
        {
            sprintf((char *) buff, "%c%.0f%c", 122, esc_temperature, 14);
            displayport_write_string(&port, &driver_info, 10, 0, (char *) buff);
        }

        // ����״̬
        uint16_t mav_mode, mav_main_mode, mav_sub_mode;
        get_mav_modes(mav_mode, mav_main_mode, mav_sub_mode);
        if (mav_mode & 0x80)
        { // ����ͼ��
          //			sprintf((char *)buff, "UNLOCK");
          //			displayport_write_string(&port, &driver_info, 17, 22, (char *)buff);
        }
        else
        { // ����ͼ��
            sprintf((char *) buff, "LOCK");
            displayport_write_string(&port, &driver_info, 10, 22, (char *) buff);
        }

        // ����ʱ��
        static TIME inFlight_TIME(false);
        get_is_inFlight(&inFlight);

        static uint32_t inFlight_time = 0;
        if (inFlight && inFlight_TIME.get_pass_time() > 1)
        {
            inFlight_TIME = TIME::now();
            inFlight_time++;
        }
        sprintf((char *) buff, "%.1d:%2.2d", inFlight_time / 60, inFlight_time % 60);
        displayport_write_string(&port, &driver_info, 1, 0, (char *) buff);

        // ��ȡ����ȫ��λ��������Ϣ
        PosSensorHealthInf3 global_inf;
        if (get_OptimalGlobal_XYZ(&global_inf) == true)
        {
            Position_Sensor gps_sensor;
            if (GetPositionSensor(global_inf.sensor_ind, &gps_sensor))
            { // ��ʾGPS��Ϣ

                // ������
                sprintf((char *) buff, "%c%.0f", 31, gps_sensor.inf.addition_inf[0]);
                displayport_write_string(&port, &driver_info, 1, 49, (char *) buff);

                // ���о���
                bool inFlight;
                get_is_inFlight(&inFlight);
                static double last_global_x = 0, last_global_y = 0;
                static bool last_global_init = false;
                static double flight_distance = 0;
                double x, y;
                map_projection_project(&global_inf.mp, gps_sensor.data.position_Global.x, gps_sensor.data.position_Global.y, &x, &y);
                if (last_global_init == false)
                { // ��ʼ��
                    last_global_init = true;

                    last_global_x = x;
                    last_global_y = y;

                    uint8_t pLen = sprintf((char *) buff, "%c%.0f%c", 113, 0.0f, 126); // �ܾ���
                    displayport_write_string(&port, &driver_info, 0, 0, (char *) buff);
                }
                else
                {
                    double error_global_x = fabs(x - last_global_x);
                    double error_global_y = fabs(y - last_global_y);
                    double distance = sqrt((error_global_x * error_global_x) + (error_global_y * error_global_y));

                    flight_distance += (distance / 100);

                    last_global_x = x;
                    last_global_y = y;

                    if (flight_distance < 1000)
                    {                                                                  // ��ʾ��λ��
                        sprintf((char *) buff, "%c%.0f%c", 113, flight_distance, 126); // �ܾ���
                        displayport_write_string(&port, &driver_info, 0, 0, (char *) buff);
                    }
                    else
                    {                                                                         // ��ʾ��λǧ��
                        sprintf((char *) buff, "%c%.0f%c", 113, flight_distance / 1000, 125); // �ܾ���
                        displayport_write_string(&port, &driver_info, 0, 0, (char *) buff);
                    }
                }

                vector2<double> homePoint;
                if (getHomePoint(&homePoint))
                {
                    vector3<double> Position;
                    get_Position_Ctrl(&Position);

                    vector2<double> Point;
                    Point.x = Position.x - homePoint.x;
                    Point.y = Position.y - homePoint.y;

                    //					//��Ҿ���
                    //					double Gs = safe_sqrt( Point.x*Point.x + Point.y*Point.y ) * 0.01;
                    //					sprintf((char *)buff, "%5.3f", Gs * 0.001);		//��Ҿ���
                    //					displayport_write_string(&port, &driver_info, 1, 23, (char *)buff);

                    // ��ҽǶ�
                    double angle;
                    get_is_inFlight(&inFlight);
                    if (inFlight)
                    {
                        vector2<double> Point;
                        Point.x = Position.x - homePoint.x;
                        Point.y = Position.y - homePoint.y;

                        // ��ҽǶ�
                        Point.x = Position.x - homePoint.x;
                        Point.y = Position.y - homePoint.y;

                        angle = rad2degree(atan2(Point.y, Point.x));
                        angle = -angle + 90; // enuתned
                        if (angle < 0)
                            angle += 360;
                    }
                    else
                    {
                        angle = 0;
                    }
                    sprintf((char *) buff, "%c%.0f", 5, angle); // ��ҽǶ�
                    displayport_write_string(&port, &driver_info, 0, 49, (char *) buff);
                }
            }
        }

        // ˢ����Ļ
        displayport_draw_screen(&port, &driver_info);
        os_delay(0.1);
    }
}

static bool MSP_DisplayPort_DriverInit(Port port, uint32_t param)
{
    port.SetBaudRate(115200, 2, 2);
    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(MSP_DisplayPort_Server, "MSP_DisplayPort", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_msp_DisplayPort()
{
    PortFunc_Register(171, MSP_DisplayPort_DriverInit);
}