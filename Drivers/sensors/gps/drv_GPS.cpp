#include "drv_GPS.hpp"
#include "Basic.hpp"
#include "HardWareConfig.hpp"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "gpsConfig.hpp"

// ublox
#include "drv_GPS1_Ublox.hpp"
#include "drv_GPS1_UbloxBase.hpp"
#include "drv_GPS1_UbloxM10.hpp"
#include "drv_GPS2_UbloxMobile.hpp"
// 和芯星通
#include "drv_GPS1_UM482.hpp"
#include "drv_GPS1_UM982.hpp"
#include "drv_GPS1_UM982Base.hpp"
#include "drv_GPS1_UM982Track.hpp"
// Septentrio
#include "drv_GPS1_MosaicG5.hpp"
#include "drv_GPS1_MosaicX5.hpp"
#include "drv_GPS1_MosaicX6.hpp"
// NMEA
#include "drv_GPS1_NMEA.hpp"
// RTCM
#include "drv_RTCMInput.hpp"
#include "drv_RTCMSd.hpp"

static bool rtc_updated = false;
bool gps_rtcUpdated()
{
    return rtc_updated;
}
bool gps_setRtcOnce(double lat, double lon, uint16_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_minute, uint8_t utc_second)
{
    if (!rtc_updated)
    {
        int8_t TimeZone = GetTimeZone(lat, lon);
        RTC_TimeStruct rtc = Get_RTC_Time();
        UTC2LocalTime(&rtc, utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_second, TimeZone, 0);
        // 仅在 RTC 实际写入成功后才置位标志, 保证 gps_rtcUpdated()==true
        // 时新时间一定已生效。若 50ms 内仍未获取到 RTC 锁(返回 false),
        // 保持标志为 false, 等下一条 GPS 时间包再重试。
        if (Set_RTC_Time(&rtc, 0.05))
        {
            rtc_updated = true;
            return true;
        }
        return false;
    }
    return false;
}

uint64_t gps_packUtc(uint16_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_minute, double utc_second)
{
    return ((uint64_t) (utc_year - 1990) << 56) | ((uint64_t) utc_month << 52) | ((uint64_t) utc_day << 46) | ((uint64_t) utc_hour << 40) | ((uint64_t) utc_minute << 32) |
           ((uint64_t) utc_second << 24) | ((uint64_t) (utc_second * 1e6) % (uint64_t) 1e6);
}

void init_drv_GPS()
{
    // 注册 GPS 配置参数
    GpsGenralConfig gpsGenralCfg;
    // GNSS_Cfg
    gpsGenralCfg.GNSS_Cfg[0] = GPS_CONFIGS_HEIGHT_TYPE_ASML_BIT;
    MAV_PARAM_TYPE gpsGenralCfg_types[] = {// GNSS_Cfg
                                           MAV_PARAM_TYPE_UINT32};
    SName gpsGenralCfg_names[] = {// 搜星GNSS设置
                                  "GPS_Configs"};
    ParamGroupRegister("GPSConfigs", 1, sizeof(gpsGenralCfg) / 8, gpsGenralCfg_types, gpsGenralCfg_names, (uint64_t *) &gpsGenralCfg);

    // 注册GPS参数
    GpsConfig initial_cfg;
    // 搜星GNSS设置
    initial_cfg.GNSS_Mode[0] = 0;
    // 延时时间
    initial_cfg.delay[0] = 0.1;
    // 双天线参数
    initial_cfg.daoVecX[0] = 0;
    initial_cfg.daoVecY[0] = 0;
    initial_cfg.daoVecZ[0] = 0;

    MAV_PARAM_TYPE param_types[] = {
        // 搜星GNSS设置
        MAV_PARAM_TYPE_UINT32,
        // 延时时间
        MAV_PARAM_TYPE_REAL32,
        // 双天线参数
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
        MAV_PARAM_TYPE_REAL32,
    };

    SName param_names1[] = {
        // 搜星GNSS设置
        "GPS1_GNSS",
        // 延时时间
        "GPS1_delay",
        // 双天线参数
        "GPS1_daoVecX",
        "GPS1_daoVecY",
        "GPS1_daoVecZ",
    };
    InitialParametersInit((uint64_t *) &initial_cfg, param_types, param_names1, sizeof(initial_cfg) / 8);
    ParamGroupRegister("GPS1Cfg", 2, sizeof(initial_cfg) / 8, param_types, param_names1, (uint64_t *) &initial_cfg);

    SName param_names2[] = {
        // 搜星GNSS设置
        "GPS2_GNSS",
        // 延时时间
        "GPS2_delay",
        // 双天线参数
        "GPS2_daoVecX",
        "GPS2_daoVecY",
        "GPS2_daoVecZ",
    };
    InitialParametersInit((uint64_t *) &initial_cfg, param_types, param_names2, sizeof(initial_cfg) / 8);
    ParamGroupRegister("GPS2Cfg", 2, sizeof(initial_cfg) / 8, param_types, param_names2, (uint64_t *) &initial_cfg);

    SName param_names3[] = {
        // 搜星GNSS设置
        "GPS3_GNSS",
        // 延时时间
        "GPS3_delay",
        // 双天线参数
        "GPS3_daoVecX",
        "GPS3_daoVecY",
        "GPS3_daoVecZ",
    };
    InitialParametersInit((uint64_t *) &initial_cfg, param_types, param_names3, sizeof(initial_cfg) / 8);
    ParamGroupRegister("GPS3Cfg", 2, sizeof(initial_cfg) / 8, param_types, param_names3, (uint64_t *) &initial_cfg);

    SName param_names4[] = {
        // 搜星GNSS设置
        "GPS4_GNSS",
        // 延时时间
        "GPS4_delay",
        // 双天线参数
        "GPS4_daoVecX",
        "GPS4_daoVecY",
        "GPS4_daoVecZ",
    };
    InitialParametersInit((uint64_t *) &initial_cfg, param_types, param_names4, sizeof(initial_cfg) / 8);
    ParamGroupRegister("GPS4Cfg", 2, sizeof(initial_cfg) / 8, param_types, param_names4, (uint64_t *) &initial_cfg);

    // 初始化外设驱动
    if (HTL_isEnabled() == false)
    {
        // ublox
        init_drv_GPS1_Ublox();
        init_drv_GPS1_UbloxM10();
        init_drv_GPS1_UbloxBase();
        init_drv_GPS2_UbloxMobile();
        // 和芯星通
        init_drv_GPS1_UM982();
        init_drv_GPS1_UM982Base();
        init_drv_GPS1_UM982Track();
        init_drv_GPS1_UM482();
        // Septentrio
        init_drv_GPS1_MosaicX5();
        init_drv_GPS1_MosaicX6();
        init_drv_GPS1_MosaicG5();
        // NMEA
        init_drv_GPS1_NMEA();
        // RTCM
        init_drv_RTCMSd();
        init_drv_RTCMInput();
    }
}