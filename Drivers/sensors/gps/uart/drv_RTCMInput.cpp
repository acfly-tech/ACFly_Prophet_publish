#include "drv_RTCMInput.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "SensorsBackend.hpp"
#include "task.h"

#define SensorInd 2

struct DriverInfo
{
    uint32_t param;
    Port port;
};

uint8_t calcNmeaChecksum(char *buf, uint16_t len)
{
    uint8_t sum = buf[0];
    for (int i = 1; i < len; i++)
    {
        sum ^= buf[i];
    }
    return sum;
}
void sendGGA(DriverInfo &driverInfo)
{
    // 获取UTC时间
    RTC_TimeStruct UTC_Time;
    UTC_Time.Year = 0;
    UTC_Time = Get_RTC_Time();
    if (UTC_Time.Year == 0)
        return;
    UTC_Time.Hours -= 8;
    if (UTC_Time.Hours < 0)
        UTC_Time.Hours += 24;

    double lat = 0, lon = 0, alt = 0;
    PosSensorHealthInf2 global_posInf;
    if (get_OptimalGlobal_XY(&global_posInf))
    {
        map_projection_reproject(&global_posInf.mp, global_posInf.PositionENU.x + global_posInf.HOffset.x, global_posInf.PositionENU.y + global_posInf.HOffset.y, &lat, &lon);
    }

    //		lat = 23.0804194;
    //		lon = 112.7349876;
    //		alt = 100;

    char latNS;
    if (lat >= 0)
        latNS = 'N';
    else
    {
        lat *= -1;
        latNS = 'S';
    }
    lat = floor(lat) * 100 + (lat - floor(lat)) * 60.0;

    char lonEW;
    if (lon >= 0)
        lonEW = 'E';
    else
    {
        lon *= -1;
        lonEW = 'W';
    }
    lon = floor(lon) * 100 + (lon - floor(lon)) * 60.0;

    static char currentTimeChar[12];
    //"hhmmss.zzz"
    sprintf(currentTimeChar, "%02d%02d%02d.%04d", UTC_Time.Hours, UTC_Time.Minutes, UTC_Time.Seconds, UTC_Time.SubSeconds);

    static char buf[256];
    // $GPGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>,<14>＊hh<CR><LF>
    int len = sprintf(buf, "$GPGGA,%s,%011.6f,%c,%012.6f,%c,1,12,0.5,680,M,48,M,,", currentTimeChar, lat, latNS, lon, lonEW);

    sprintf(&buf[len], "*%02X\r\n", calcNmeaChecksum(&buf[1], len - 1));
    int lenSend = strlen(buf);

    if (driverInfo.port.write)
        driverInfo.port.write((uint8_t *) &buf[0], lenSend, 1, 1);
}

static void RTCMInput_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

// 状态机
#define BUFFER_LEN 1024
    uint8_t _buffer[BUFFER_LEN + 2];
    uint16_t _pos = 0;
    uint8_t parse_state = 0;
    uint16_t _message_length = 0;
    TIME sendGGATime(1);
    extern float debug_test[30];
    uint32_t bytePerSec = 0;
    while (1)
    {
        uint8_t rdata;
        if (driver_info.port.read(&rdata, 1, 1, 0.1))
        {
            bytePerSec++;
            _buffer[_pos++] = rdata;
            if (parse_state == 0)
            {
                if (rdata == 0xD3)
                    parse_state = 1;
                else
                {
                    _pos = 0;
                    _message_length = 0;
                }
            }
            else
            {
                if (_pos == 3)
                {
                    _message_length = static_cast<uint16_t>((((_buffer[1] & 3) << 8) | (_buffer[2])));
                    if (_message_length + 6 > BUFFER_LEN)
                    {
                        parse_state = 0;
                        _pos = 0;
                    }
                }

                if (_pos >= _message_length + 6)
                { // 接收完成
                    inject_RtkPorts(_buffer, _pos);
                    _pos = 0;
                    parse_state = 0;

                    debug_test[13]++;
                }
            }
        }

        if (sendGGATime.get_pass_time() > 1)
        {
            debug_test[14] = bytePerSec;
            bytePerSec = 0;
            sendGGA(driver_info);
            sendGGATime = TIME::now();
        }
    }
}

static bool RTCMInput_DriverInit(Port port, uint32_t param)
{
    // 波特率115200
    if (param < 1000)
        port.SetBaudRate(115200, 2, 2);
    else
        port.SetBaudRate(param, 2, 2);

    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(RTCMInput_Server, "RTCMIn", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_RTCMInput()
{
    PortFunc_Register(8, RTCMInput_DriverInit);
}