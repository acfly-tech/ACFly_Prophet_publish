#include "drv_GPS1_NMEA.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "StorageSystem.hpp"
#include "gpsConfig.hpp"
#include "task.h"

struct DriverInfo
{
    uint32_t param;
    Port port;
};

enum GPS_Scan_Operation
{
    // 在指定波特率下发送初始化信息
    GPS_Scan_Baud9600 = 9600,
    GPS_Scan_Baud38400 = 38400,
    GPS_Scan_Baud460800 = 460800,
    GPS_Scan_Baud115200 = 115200,

    // 检查是否设置成功
    GPS_Check_Baud,
    // 在当前波特率下再次发送配置
    GPS_ResendConfig,
    // GPS已检测存在
    GPS_Present,
};

enum FrameName
{
    FrameNone = 0,
    FrameGGA,
    FrameGSV,
    FrameGPRMC,
};

struct NMEA_GPS_State_Machine
{
    FrameName frame_name = FrameNone;
    uint32_t frame_datas_ind = 0;
    uint16_t frame_datas_length;
    uint8_t read_state = 0;
    uint16_t crc = 0;

    uint16_t dataP[120];
    uint16_t dataPInd;
    uint16_t dataPCount;
};

static uint8_t AsciiToByte(uint8_t b)
{
    uint8_t ret = 0;
    if (b >= '0' && b <= '9')
        ret = b - '0';
    else if (b >= 'A' && b <= 'F')
        ret = b - 'A' + 10;
    else if (b >= 'a' && b <= 'f')
        ret = b - 'a' + 10;
    else
        ret = 0;
    return ret;
}

static bool NMEA_Str2num(uint8_t *str, double &result)
{
    uint8_t *p = str;
    uint8_t ilen = 0, flen = 0, i;
    uint8_t mask = 0;
    int64_t integerNum = 0;
    double floatNum = 0;
    int readCnt = 0;
    while (1)
    {
        readCnt++;
        if (readCnt > 100)
            return false;

        if (*p == '-')
        {
            mask |= 0x02;
            p++;
        }

        if (*p == ',' || (*p == '*'))
            break;

        if (*p == '.')
        {
            mask |= 0X01;
            p++;
        }
        else if (*p > '9' || (*p < '0'))
        {
            ilen = 0;
            flen = 0;
            break;
        }
        if (mask & 0X01)
            flen++;
        else
            ilen++;
        p++;
    }

    if (mask & 0X02)
        str++;

    if (ilen > 16)
        return false;

    if (flen > 16)
        return false;

    for (i = 0; i < ilen; i++)
    {
        integerNum *= 10;
        integerNum += (str[i] - '0');
    }

    double frac = 0;
    for (i = flen; i > 0; i--)
        frac = (frac + (str[ilen + i] - '0')) * 0.1;

    double res = integerNum + frac;

    if (mask & 0X02)
        res = -res;

    result = res;
    return true;
}

static double convert_from_ddmm(double val)
{
    int degrees = (int) val / 100;
    double minutes = val - degrees * 100.0;
    return degrees + minutes / 60.0;
}

static inline void NMEA_ResetRxStateMachine(NMEA_GPS_State_Machine *state_machine)
{
    state_machine->dataPInd = state_machine->dataPCount = 0;
    state_machine->read_state = state_machine->frame_datas_ind = 0;
}

static inline bool NEMA_Parse(NMEA_GPS_State_Machine *state_machine, uint8_t *frame_datas, uint8_t r_data)
{
    if (state_machine->frame_datas_ind > 1000)
        NMEA_ResetRxStateMachine(state_machine);

    frame_datas[state_machine->frame_datas_ind++] = r_data;
    switch (state_machine->read_state)
    {
        case 0:
        {
            if (state_machine->frame_datas_ind == 1)
            {
                if (r_data != '$')
                    state_machine->frame_datas_ind = 0;
                else
                    state_machine->read_state = 1;
            }
            break;
        }
        case 1:
        {
            if (state_machine->frame_datas_ind == 6)
            {
                if (strncmp((char *) &frame_datas[3], "GGA", 3) == 0)
                {
                    state_machine->frame_name = FrameGGA;
                    state_machine->read_state = 2;
                }
                else if (strncmp((char *) &frame_datas[3], "RMC", 3) == 0)
                {
                    state_machine->frame_name = FrameGPRMC;
                    state_machine->read_state = 2;
                }
                else if (strncmp((char *) &frame_datas[3], "GSV", 3) == 0)
                {
                    state_machine->frame_name = FrameGSV;
                    state_machine->read_state = 2;
                }
                else
                    NMEA_ResetRxStateMachine(state_machine);
            }
            break;
        }
        case 2:
        {
            if (r_data == ',')
            {
                state_machine->dataP[state_machine->dataPInd++] = state_machine->frame_datas_ind;
                if (state_machine->dataPInd > 118)
                    NMEA_ResetRxStateMachine(state_machine);
            }
            else if (r_data == '*')
            {
                state_machine->read_state = 3;
                state_machine->crc = 0;
            }
            break;
        }
        case 3:
        {
            state_machine->crc = AsciiToByte(r_data) * 16;
            state_machine->read_state = 4;
            break;
        }
        case 4:
        {
            state_machine->crc += AsciiToByte(r_data);
            state_machine->read_state = 5;
            break;
        }
        case 5:
        {
            if (r_data == 0x0D)
            {
                state_machine->read_state = 6;
            }
            else
                NMEA_ResetRxStateMachine(state_machine);
            break;
        }
        case 6:
        {
            uint16_t data_len = state_machine->frame_datas_ind;
            uint16_t dataPCount = state_machine->dataPInd;
            NMEA_ResetRxStateMachine(state_machine);
            if (r_data == 0x0A)
            {
                uint8_t result = frame_datas[1];
                for (int i = 2; frame_datas[i] != '*'; i++)
                    result ^= frame_datas[i];
                if (result == state_machine->crc)
                {
                    state_machine->frame_datas_length = data_len;
                    state_machine->dataPCount = dataPCount;
                    return true;
                }
                else
                    return false;
            }
            break;
        }
    }
    return false;
}

static void GPS_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    // GPS识别状态
    GPS_Scan_Operation current_GPS_Operation = GPS_Scan_Baud9600;
    // 数据读取状态机
    __attribute__((aligned(4))) uint8_t NMEA_frame_datas[1024];
    NMEA_GPS_State_Machine nmea_state;
    // 上次更新时间
    TIME last_update_time;

    // 注册Rtk端口
    RtkPort rtk_port;
    rtk_port.ena = false;
    rtk_port.write = driver_info.port.write;
    rtk_port.lock = driver_info.port.lock;
    rtk_port.unlock = driver_info.port.unlock;
    int8_t rtk_port_ind = RtkPortRegister(rtk_port);

    // 等待初始化完成
    while (getInitializationCompleted() == false)
        os_delay(0.1);

    os_delay(3.0);

GPS_CheckBaud:
    while (1)
    {
        // 更改指定波特率
        driver_info.port.SetBaudRate(current_GPS_Operation, 3, 0.1);
        switch (current_GPS_Operation)
        {
            case GPS_Scan_Baud9600:
                current_GPS_Operation = GPS_Scan_Baud38400;
                break;
            case GPS_Scan_Baud38400:
                current_GPS_Operation = GPS_Scan_Baud460800;
                break;
            case GPS_Scan_Baud460800:
                current_GPS_Operation = GPS_Scan_Baud115200;
                break;
            default:
            case GPS_Scan_Baud115200:
                current_GPS_Operation = GPS_Scan_Baud9600;
                break;
        }

        // 清空接收缓冲区准备接收数据
        driver_info.port.reset_rx(0.1);
        NMEA_ResetRxStateMachine(&nmea_state);
        TIME RxChkStartTime = TIME::now();
        while (RxChkStartTime.get_pass_time() < 2)
        {
            uint8_t r_data;
            if (driver_info.port.read(&r_data, 1, 0.5, 0.1))
            {
                bool nmeaParsed = NEMA_Parse(&nmea_state, NMEA_frame_datas, r_data);
                if (nmeaParsed && nmea_state.frame_name == FrameGPRMC)
                    goto GPS_Present;
            }
        }
    }

GPS_Present:
    uint32_t sensor_key = 0;
    uint8_t gps_ind = 1;
    if (driver_info.param >= 1 && driver_info.param <= MAX_GPS_COUNT)
        gps_ind = driver_info.param;
    uint8_t gps_sensor_index = default_gps1_sensor_index + gps_ind - 1;

    GpsConfig gps_cfg;
    char gpsName[] = "GPS1";
    gpsName[3] = '0' + gps_ind;
    char gpsCfgName[] = "GPS1Cfg";
    gpsCfgName[3] = '0' + gps_ind;
    if (ReadParamGroup(gpsCfgName, (uint64_t *) &gps_cfg, 0) == PR_OK)
    {
        sensor_key = PositionSensorRegister(gps_sensor_index, SName(gpsName) + SName("_NMEA"), Position_Sensor_Type_GlobalPositioning, Position_Sensor_DataType_sv_xy, Position_Sensor_frame_ENU,
                                            gps_cfg.delay[0], 30, 30);
    }
    else
    {
        sensor_key =
            PositionSensorRegister(gps_sensor_index, SName(gpsName) + SName("_NMEA"), Position_Sensor_Type_GlobalPositioning, Position_Sensor_DataType_sv_xy, Position_Sensor_frame_ENU, 0.1, 30, 30);
    }

    // 开启Rtk注入
    RtkPort_setEna(rtk_port_ind, true);

    // 清空接收缓冲区准备接收数据
    driver_info.port.reset_rx(0.1);
    NMEA_ResetRxStateMachine(&nmea_state);
    last_update_time = TIME::now();

    // 卫星计数
    uint16_t gsvSatsTotal = 0;
    uint8_t gsvFirstPrefix[2] = {0, 0};
    uint8_t gsvConstellations = 0;

    // 附加数据
    double addition_inf[8] = {0};

    while (1)
    {
        uint8_t r_data;
        if (driver_info.port.read(&r_data, 1, 2, 0.1))
        {
            bool nmeaParsed = NEMA_Parse(&nmea_state, NMEA_frame_datas, r_data);
            if (nmeaParsed)
            {
                switch (nmea_state.frame_name)
                {
                    case FrameGPRMC:
                    {
                        bool available = false;

                        char nmeaAV = NMEA_frame_datas[nmea_state.dataP[1]];
                        if (nmeaAV == 'A')
                            available = true;

                        double lat, lon;
                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[2]], lat) == false)
                        {
                            available = false;
                        }
                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[4]], lon) == false)
                        {
                            available = false;
                        }

                        char nmeaNS = NMEA_frame_datas[nmea_state.dataP[3]];
                        if (nmeaNS != 'N' && nmeaNS != 'S')
                        {
                            available = false;
                        }
                        char nmeaEW = NMEA_frame_datas[nmea_state.dataP[5]];
                        if (nmeaEW != 'E' && nmeaEW != 'W')
                        {
                            available = false;
                        }

                        double nmeaVel, nmeaVelDir;
                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[6]], nmeaVel) == false)
                        {
                            available = false;
                        }
                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[7]], nmeaVelDir) == false)
                        {
                            available = false;
                        }
                        nmeaVel *= 51.44444;

                        lat = convert_from_ddmm(lat);
                        if (nmeaNS == 'S')
                            lat = -lat;

                        lon = convert_from_ddmm(lon);
                        if (nmeaEW == 'W')
                            lon = -lon;

                        vector3<double> position_Global;
                        position_Global.x = lat;
                        position_Global.y = lon;
                        position_Global.z = 0;

                        vector3<double> velocity;
                        velocity.y = nmeaVel * cos(degree2rad(nmeaVelDir)); // North
                        velocity.x = nmeaVel * sin(degree2rad(nmeaVelDir)); // East

                        PositionSensorChangeDataType(gps_sensor_index, sensor_key, Position_Sensor_DataType_sv_xy);
                        if (available)
                        {
                            PositionSensorUpdatePositionGlobalVel(gps_sensor_index, sensor_key, position_Global, velocity, available, -1, 100, 100, addition_inf, 100, 100);
                        }
                        else
                            PositionSensorSetInavailable(gps_sensor_index, sensor_key);

                        break;
                    }

                    case FrameGGA:
                    {
                        double fix;
                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[5]], fix))
                        {
                            switch ((int) fix)
                            {
                                case 0:
                                    addition_inf[1] = 0;
                                    break;
                                case 1:
                                    addition_inf[1] = 3;
                                    break;
                                case 2:
                                    addition_inf[1] = 6;
                                    break;
                                default:
                                    addition_inf[1] = 0;
                                    break;
                            }
                        }
                        else
                            addition_inf[1] = 0;

                        NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[6]], addition_inf[0]);

                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[7]], addition_inf[5]))
                        {
                            addition_inf[5] *= 100;
                            addition_inf[4] = addition_inf[5];
                        }
                        break;
                    }

                        //                    case FrameGSV:
                        //                    {
                        //                        /*
                        //                                GSV格式: $xxGSV,numMsg,msgNum,numSV,...*cs
                        //                                numMsg: 该星座的GSV消息总数 (dataP[0])
                        //                                msgNum: 当前消息编号1-based (dataP[1])
                        //                                numSV:  该星座可见卫星总数 (dataP[2])

                        //                                多星座(GP/GL/GA/BD等)各自有独立的GSV消息组,
                        //                                需要把各星座的numSV累加得到总卫星数.
                        //                                只在每组的第一条(msgNum==1)取numSV,避免重复累加.
                        //                                当检测到新一轮周期(同一星座前缀再次出现)时上报总数.
                        //                        */
                        //                        double count, ind, sats;
                        //                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[0]], count) == false)
                        //                            break;
                        //                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[1]], ind) == false)
                        //                            break;
                        //                        if (NMEA_Str2num(&NMEA_frame_datas[nmea_state.dataP[2]], sats) == false)
                        //                            break;

                        //                        // 只处理每组的第一条消息(ind==1),因为numSV在同一组的所有消息中相同
                        //                        if ((uint8_t) ind == 1)
                        //                        {
                        //                            uint8_t p0 = NMEA_frame_datas[1];
                        //                            uint8_t p1 = NMEA_frame_datas[2];

                        //                            // 检测新周期: 同一星座前缀再次出现
                        //                            if (gsvConstellations > 0 && p0 == gsvFirstPrefix[0] && p1 == gsvFirstPrefix[1])
                        //                            {
                        //                                addition_inf[0] = gsvSatsTotal;
                        //                                gsvSatsTotal = 0;
                        //                                gsvConstellations = 0;
                        //                            }

                        //                            if (gsvConstellations == 0)
                        //                            {
                        //                                gsvFirstPrefix[0] = p0;
                        //                                gsvFirstPrefix[1] = p1;
                        //                            }

                        //                            gsvSatsTotal += (uint16_t) sats;
                        //                            ++gsvConstellations;
                        //                        }

                        //                        break;
                        //                    }

                    default:
                        break;
                }
            }
        }
        else
        { // 接收不到数据
            PositionSensorUnRegister(gps_sensor_index, sensor_key);
            // 关闭Rtk注入
            RtkPort_setEna(rtk_port_ind, false);
            goto GPS_CheckBaud;
        }
    }
}

static bool GPS_DriverInit(Port port, uint32_t param)
{
    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(GPS_Server, "GPS_NMEA", 3000, driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_GPS1_NMEA()
{
    PortFunc_Register(11, GPS_DriverInit);
}
