#include "Basic.hpp"
#include "Parameters.hpp"
extern "C" float getCPULoad();
#include "StorageSystem.hpp"
#include "drv_SDMMC.hpp"
#include "fatfs.h"
#include <stdlib.h>

#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "HardWareConfig.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "Receiver.hpp"
#include "Sensors.hpp"
#include "drv_ADC.hpp"
#include "drv_BootLoader.hpp"
#include "drv_USB.hpp"
#include "message_buffer.h"
#include "semphr.h"
#include "stream_buffer.h"

// GPS RTC 校准状态（实现在 drv_GPS.cpp）
extern bool gps_rtcUpdated();

// 挂载路径
static const char log_mountPath[4] = {DEV_SD_EXT + '0', ':', '/', 0};
static const char usr_mountPath[4] = {DEV_SD_USR + '0', ':', '/', 0};

// SD卡是否初始化完成
static bool SD_Init_Complete = false;
static bool SD_Init_Success = false;
extern "C" bool Get_SD_Init_Complete(void)
{
    return SD_Init_Complete;
}
static inline void Set_SD_Init_Complete(void)
{
    SD_Init_Complete = true;
    /* Get_SD_Init_Success原误返回SD_Init_Complete且Success从未置位(死变量);
     * 在此同步置位保持FTP等外部可见行为不变(挂载成功即可用) */
    SD_Init_Success = true;
}
static inline void Clear_SD_Init_Complete(void)
{
    SD_Init_Success = false;
    SD_Init_Complete = false;
}

extern "C" bool Get_SD_Init_Success(void)
{
    return SD_Init_Success;
}
static inline void Set_SD_Init_Success(void)
{
    SD_Init_Success = true;
}
static inline void Clear_SD_Init_Success(void)
{
    SD_Init_Success = false;
}

// SD卡更改标志
extern "C"
{
    uint8_t get_MSC_DEV_DISKS_CNT(void);
    const uint8_t *get_MSC_DEV_DISKS(void);
    const bool *get_MSC_DEV_DISKS_READONLY(void);
}
static bool mscExtWritten = false;
static bool mscUsrWritten = false;
extern "C" void mscDiskWritten(uint8_t lun)
{
    if (lun < get_MSC_DEV_DISKS_CNT())
    {
        switch (get_MSC_DEV_DISKS()[lun])
        {
            case DEV_SD_EXT:
            {
                mscExtWritten = true;
                break;
            }
            case DEV_SD_USR:
            {
                mscUsrWritten = true;
                break;
            }
        }
    }
}

/*SD卡信息*/
static bool SdAvailable = false;
static uint64_t tot_size = 0;
static uint64_t fre_size = 0;
static float totGB = 0;
static float freGB = 0;
static uint8_t sdSaveFailCounter = 0;
static inline void reset_SdInfo()
{
    SdAvailable = false;
    tot_size = fre_size = 0;
    totGB = freGB = 0;
    sdSaveFailCounter = 0;
}
static inline void update_SdSize(uint64_t tSize, uint64_t writtenSize)
{
    if (writtenSize < fre_size)
        fre_size -= writtenSize;
    else
        fre_size = 0;
    freGB = (fre_size >> 20) / 1024.0f;

    if (tSize != writtenSize)
    {
        if (sdSaveFailCounter < 250)
            ++sdSaveFailCounter;
    }
    else if (sdSaveFailCounter > 0)
        --sdSaveFailCounter;
}
float getSdTotalSizeGB()
{
    return totGB;
}
float getSdFreeSizeGB()
{
    return freGB;
}
/*SD卡信息*/

/*SD Log*/
// Log缓冲区
#define BufSize 20480
Static_DTCMBuf uint8_t LogStreamBufferStorage[BufSize + 1];
Static_DTCMBuf StaticMessageBuffer_t LogStreamBufferStruct;
Static_DTCMBuf MessageBufferHandle_t LogStreamBuffer = xMessageBufferCreateStatic(BufSize, LogStreamBufferStorage, &LogStreamBufferStruct);
;
__attribute__((aligned(4))) Static_AXIDMABuf char buf[BufSize];
static SemaphoreHandle_t LogSemphr = xSemaphoreCreateMutex();

static bool Lock_SDLog(double TIMEOUT = -1)
{
    // if (get_is_usb_connected())
    //     return false;
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(LogSemphr, TIMEOUT_Ticks))
        return true;
    return false;
}
static void UnLock_SDLog()
{
    xSemaphoreGive(LogSemphr);
}

/*Log数据*/
bool SDLog_Msg_SysState(double SyncTIMEOUT)
{
    // 获取接收机
    Receiver rc;
    SName rcName;
    getReceiver(&rc, &rcName);
    // 获取控制器状态
    bool att_ctrl_ena, alt_ctrl_ena, pos_ctrl_ena;
    is_Attitude_Control_Enabled(&att_ctrl_ena);
    is_Altitude_Control_Enabled(&alt_ctrl_ena);
    is_Position_Control_Enabled(&pos_ctrl_ena);
    // 获取电量
    BatteryInfo batInfo;
    int8_t batInd = -1;
    getCurrentBatteryInfo(&batInfo, &batInd);

    struct Msg
    {
        uint8_t msg_type;
        uint8_t length;
        /*flags
                bit0-姿态控制器开启
                bit1-高度控制器开启
                bit2-位置控制器开启
        */
        uint16_t flags;
        uint32_t Time;
        uint8_t cpuLoad;
        int8_t currentRc;
        uint8_t MainBatRMPercent;
        int8_t batId;
        float MainBatVoltage;
        float MainBatCurrent;
        float RcChannels[8];
        int8_t cpuTemperature;
        uint8_t flyMode; // bit7:是否为安全模式
        uint8_t rsv2[6];
    } __AC_PACKED;
    Msg msg = {0};

    msg.msg_type = LogMsg_SysState;
    msg.length = sizeof(msg);
    // flags
    uint16_t flags = 0;
    if (att_ctrl_ena)
        flags |= (1 << 0);
    if (alt_ctrl_ena)
        flags |= (1 << 1);
    if (pos_ctrl_ena)
        flags |= (1 << 2);
    msg.flags = flags;

    msg.Time = TIME::get_System_Run_Time() * 1e+4;
    // CPU load
    msg.cpuLoad = getCPULoad();
    // 接收机状态
    if (rc.available == false)
        msg.currentRc = -1;
    else
    {
        msg.currentRc = rc.id;
        memcpy(msg.RcChannels, rc.data, 8 * sizeof(float));
    }
    // 电池状态
    msg.batId = batInd;
    msg.MainBatVoltage = batInfo.totalVoltRaw;
    msg.MainBatCurrent = batInfo.totalCurrent;
    msg.MainBatRMPercent = batInfo.totalPercent;
    msg.cpuTemperature = adcGet_CPUTemperature();
    msg.flyMode = (is_MSafeCtrl() ? (1 << 7) : 0) | getCurrentFlyMode();
    if (Lock_SDLog(SyncTIMEOUT))
    {
        xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);
        UnLock_SDLog();
        return true;
    }
    return false;
}

bool SDLog_Msg_IMUSensor(uint8_t ind, IMUType imu_type, IMU_Sensor sensor, double SyncTIMEOUT)
{
    if (imu_type != IMUType_Accelerometer && imu_type != IMUType_Gyroscope && imu_type != IMUType_Magnetometer)
        return false;

    struct Msg
    {
        uint8_t msg_type;
        uint8_t length;
        // 类别 1-加速度 2-陀螺仪 3-磁罗盘 4-双天线
        uint8_t type;
        // 序号
        uint8_t ind;
        uint32_t Time;
        // 原始值(双天线传感器为双天线向量)
        float rawValue[3];
        // 校准值(双天线传感器为基准向量)
        float value[3];
        // 更新周期
        float updateT;
        // 温度(双天线传感器为延时时间)
        float temperature;
    } __AC_PACKED;
    Msg msg;

    msg.msg_type = LogMsg_IMUSensor;
    msg.length = sizeof(msg);
    msg.type = imu_type;
    msg.ind = ind;

    msg.Time = TIME::get_System_Run_Time() * 1e+4;

    msg.rawValue[0] = sensor.data_raw.x;
    msg.rawValue[1] = sensor.data_raw.y;
    msg.rawValue[2] = sensor.data_raw.z;

    msg.value[0] = sensor.data.x;
    msg.value[1] = sensor.data.y;
    msg.value[2] = sensor.data.z;

    msg.updateT = sensor.sample_time;
    msg.temperature = sensor.temperature;

    if (Lock_SDLog(SyncTIMEOUT))
    {
        xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);
        UnLock_SDLog();
        return true;
    }
    return false;
}
bool SDLog_Msg_DFSensor(uint8_t ind, DAO_Sensor sensor, double SyncTIMEOUT)
{
    struct Msg
    {
        uint8_t msg_type;
        uint8_t length;
        // 类别 1-加速度 2-陀螺仪 3-磁罗盘 4-双天线
        uint8_t type;
        // 序号
        uint8_t ind;
        uint32_t Time;
        // 原始值(双天线传感器为双天线向量)
        float rawValue[3];
        // 校准值(双天线传感器为基准向量)
        float value[3];
        // 更新周期
        float updateT;
        // 温度(双天线传感器为延时时间)
        float temperature;
    } __AC_PACKED;
    Msg msg;

    msg.msg_type = LogMsg_IMUSensor;
    msg.length = sizeof(msg);
    msg.type = IMUType_DFAntenna;
    msg.ind = ind;

    msg.Time = TIME::get_System_Run_Time() * 1e+4;

    msg.rawValue[0] = sensor.relPos.x;
    msg.rawValue[1] = sensor.relPos.y;
    msg.rawValue[2] = sensor.relPos.z;

    msg.value[0] = sensor.st_relPos.x;
    msg.value[1] = sensor.st_relPos.y;
    msg.value[2] = sensor.st_relPos.z;

    msg.updateT = sensor.sample_time;
    msg.temperature = sensor.delay;

    if (Lock_SDLog(SyncTIMEOUT))
    {
        xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);
        UnLock_SDLog();
        return true;
    }
    return false;
}

bool SDLog_Msg_PosSensor(uint8_t ind, Position_Sensor sensor, double SyncTIMEOUT)
{
    struct MsgS
    {
        uint8_t msg_type;
        uint8_t length;
        uint8_t SensorType; // 为Position_Sensor_Type
        uint8_t DataType;   // Position_Sensor_DataType
        uint32_t Time;
        uint8_t sensor; // 传感器编号（bit7为是否可用）
        uint8_t frame;  // Position_Sensor_frame
        uint8_t rsv1[2];
        uint16_t eph; // 水平精度cm
        uint16_t epv; // 垂直精度cm
        double posx;
        double posy;
        double posz;
    } __AC_PACKED;
    struct MsgV
    {
        uint8_t msg_type;
        uint8_t length;
        uint8_t SensorType; // 为Position_Sensor_Type
        uint8_t DataType;   // Position_Sensor_DataType
        uint32_t Time;
        uint8_t sensor; // 传感器编号（bit7为是否可用）
        uint8_t frame;  // Position_Sensor_frame
        uint8_t rsv1[2];
        uint16_t eph; // 水平精度cm
        uint16_t epv; // 垂直精度cm
        double velx;
        double vely;
        double velz;
    } __AC_PACKED;
    struct MsgSV
    {
        uint8_t msg_type;
        uint8_t length;
        uint8_t SensorType; // 为Position_Sensor_Type
        uint8_t DataType;   // Position_Sensor_DataType
        uint32_t Time;
        uint8_t sensor; // 传感器编号（bit7为是否可用）
        uint8_t frame;  // Position_Sensor_frame
        uint8_t rsv1[2];
        uint16_t eph; // 水平精度cm
        uint16_t epv; // 垂直精度cm
        double posx;
        double posy;
        double posz;
        double velx;
        double vely;
        double velz;
    } __AC_PACKED;
    struct MsgGS
    {
        uint8_t msg_type;
        uint8_t length;
        uint8_t SensorType; // 为Position_Sensor_Type
        uint8_t DataType;   // Position_Sensor_DataType
        uint32_t Time;
        uint8_t sensor; // 传感器编号（bit7为是否可用）
        uint8_t frame;  // Position_Sensor_frame
        uint8_t numSV;  // 星数
        uint8_t fix;    // fix_type
        uint16_t eph;   // 水平精度cm
        uint16_t epv;   // 垂直精度cm
        double OriginLat;
        double OriginLon;
        double Lat;
        double Lon;
        double posx;
        double posy;
        double posz;
    } __AC_PACKED;
    struct MsgGSV
    {
        uint8_t msg_type;
        uint8_t length;
        uint8_t SensorType; // 为Position_Sensor_Type
        uint8_t DataType;   // Position_Sensor_DataType
        uint32_t Time;
        uint8_t sensor; // 传感器编号（bit7为是否可用）
        uint8_t frame;  // Position_Sensor_frame
        uint8_t numSV;  // 星数
        uint8_t fix;    // fix_type
        uint16_t eph;   // 水平精度cm
        uint16_t epv;   // 垂直精度cm
        double OriginLat;
        double OriginLon;
        double Lat;
        double Lon;
        double posx;
        double posy;
        double posz;
        double velx;
        double vely;
        double velz;
    } __AC_PACKED;

    uint8_t dataType = sensor.data.sensor_DataType;
    if (dataType > 100)
        dataType -= 100;
    if (dataType < 8)
    { // s传感器
        if (sensor.data.sensor_type == Position_Sensor_Type_GlobalPositioning)
        { // GS
            MsgGS msg;
            msg.msg_type = LogMsg_PosSensor;
            msg.length = sizeof(msg);
            msg.SensorType = sensor.data.sensor_type;
            msg.DataType = sensor.data.sensor_DataType;
            msg.Time = TIME::get_System_Run_Time() * 1e+4;
            msg.sensor = sensor.data.available ? (1 << 7) | ind : ind;
            msg.frame = sensor.data.data_frame;
            msg.OriginLat = rad2degree(sensor.data.mp.lat0_rad);
            msg.OriginLon = rad2degree(sensor.data.mp.lon0_rad);
            msg.Lat = sensor.data.position_Global.x;
            msg.Lon = sensor.data.position_Global.y;
            msg.posx = sensor.data.position.y * 0.01;
            msg.posy = sensor.data.position.x * 0.01;
            msg.posz = -sensor.data.position.z * 0.01;
            msg.numSV = sensor.inf.addition_inf[0];
            msg.fix = (sensor.inf.addition_inf[1] > 1) ? sensor.inf.addition_inf[1] : 1;
            float eph = sensor.inf.addition_inf[4];
            float epv = sensor.inf.addition_inf[5];
            msg.eph = eph < 65535 ? eph : 65535;
            msg.epv = epv < 65535 ? epv : 65535;
            if (Lock_SDLog(SyncTIMEOUT))
            {
                xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);

                UnLock_SDLog();
                return true;
            }
        }
        else
        { // S
            MsgS msg;
            msg.msg_type = LogMsg_PosSensor;
            msg.length = sizeof(msg);
            msg.SensorType = sensor.data.sensor_type;
            msg.DataType = sensor.data.sensor_DataType;
            msg.Time = TIME::get_System_Run_Time() * 1e+4;
            msg.sensor = sensor.data.available ? (1 << 7) | ind : ind;
            msg.frame = sensor.data.data_frame;
            msg.posx = sensor.data.position.y * 0.01;
            msg.posy = sensor.data.position.x * 0.01;
            msg.posz = -sensor.data.position.z * 0.01;
            msg.eph = sensor.data.xy_trustD < 65535 ? sensor.data.xy_trustD : 65535;
            msg.epv = sensor.data.z_trustD < 65535 ? sensor.data.z_trustD : 65535;
            if (Lock_SDLog(SyncTIMEOUT))
            {
                xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);

                UnLock_SDLog();
                return true;
            }
        }
    }
    else if (dataType < 16)
    { // v传感器
        MsgV msg;
        msg.msg_type = LogMsg_PosSensor;
        msg.length = sizeof(msg);
        msg.SensorType = sensor.data.sensor_type;
        msg.DataType = sensor.data.sensor_DataType;
        msg.Time = TIME::get_System_Run_Time() * 1e+4;
        msg.sensor = sensor.data.available ? (1 << 7) | ind : ind;
        msg.frame = sensor.data.data_frame;
        msg.velx = sensor.data.velocity.y * 0.01;
        msg.vely = sensor.data.velocity.x * 0.01;
        msg.velz = -sensor.data.velocity.z * 0.01;
        msg.eph = sensor.data.xy_trustD < 65535 ? sensor.data.xy_trustD : 65535;
        msg.epv = sensor.data.z_trustD < 65535 ? sensor.data.z_trustD : 65535;
        if (Lock_SDLog(SyncTIMEOUT))
        {
            xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);

            UnLock_SDLog();
            return true;
        }
    }
    else if (dataType < 24)
    { // sv传感器
        if (sensor.data.sensor_type == Position_Sensor_Type_GlobalPositioning)
        { // GSV
            MsgGSV msg;
            msg.msg_type = LogMsg_PosSensor;
            msg.length = sizeof(msg);
            msg.SensorType = sensor.data.sensor_type;
            msg.DataType = sensor.data.sensor_DataType;
            msg.Time = TIME::get_System_Run_Time() * 1e+4;
            msg.sensor = sensor.data.available ? (1 << 7) | ind : ind;
            msg.frame = sensor.data.data_frame;
            msg.OriginLat = rad2degree(sensor.data.mp.lat0_rad);
            msg.OriginLon = rad2degree(sensor.data.mp.lon0_rad);
            msg.Lat = sensor.data.position_Global.x;
            msg.Lon = sensor.data.position_Global.y;
            msg.posx = sensor.data.position.y * 0.01;
            msg.posy = sensor.data.position.x * 0.01;
            msg.posz = -sensor.data.position.z * 0.01;
            msg.velx = sensor.data.velocity.y * 0.01;
            msg.vely = sensor.data.velocity.x * 0.01;
            msg.velz = -sensor.data.velocity.z * 0.01;
            msg.numSV = sensor.inf.addition_inf[0];
            msg.fix = (sensor.inf.addition_inf[1] > 1) ? sensor.inf.addition_inf[1] : 1;
            float eph = sensor.inf.addition_inf[4];
            float epv = sensor.inf.addition_inf[5];
            msg.eph = eph < 65535 ? eph : 65535;
            msg.epv = epv < 65535 ? epv : 65535;
            if (Lock_SDLog(SyncTIMEOUT))
            {
                xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);

                UnLock_SDLog();
                return true;
            }
        }
        else
        { // S
            MsgSV msg;
            msg.msg_type = LogMsg_PosSensor;
            msg.length = sizeof(msg);
            msg.SensorType = sensor.data.sensor_type;
            msg.DataType = sensor.data.sensor_DataType;
            msg.Time = TIME::get_System_Run_Time() * 1e+4;
            msg.sensor = sensor.data.available ? (1 << 7) | ind : ind;
            msg.frame = sensor.data.data_frame;
            msg.posx = sensor.data.position.y * 0.01;
            msg.posy = sensor.data.position.x * 0.01;
            msg.posz = -sensor.data.position.z * 0.01;
            msg.velx = sensor.data.velocity.y * 0.01;
            msg.vely = sensor.data.velocity.x * 0.01;
            msg.velz = -sensor.data.velocity.z * 0.01;
            msg.eph = sensor.data.xy_trustD < 65535 ? sensor.data.xy_trustD : 65535;
            msg.epv = sensor.data.z_trustD < 65535 ? sensor.data.z_trustD : 65535;
            if (Lock_SDLog(SyncTIMEOUT))
            {
                xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);

                UnLock_SDLog();
                return true;
            }
        }
    }

    return false;
}
bool SDLog_Msg_Attitude(double SyncTIMEOUT)
{
    Quaternion airframe_quat;
    if (get_AirframeY_quat(&airframe_quat, SyncTIMEOUT) == false)
        return false;
    vector3<double> angular_rate;
    if (get_AngularRate_Ctrl(&angular_rate, SyncTIMEOUT) == false)
        return false;
    struct Msg
    {
        uint8_t msg_type;
        uint8_t length;
        uint16_t rsv;
        uint32_t Time;
        double roll;
        double pitch;
        double yaw;
        double rollspeed;
        double pitchspeed;
        double yawspeed;
    } __AC_PACKED;
    Msg msg;

    msg.msg_type = LogMsg_Attitude;
    msg.length = sizeof(msg);
    msg.Time = TIME::get_System_Run_Time() * 1e+4;
    msg.roll = airframe_quat.getRoll();
    msg.pitch = airframe_quat.getPitch();
    msg.yaw = airframe_quat.getYaw();
    msg.rollspeed = angular_rate.x;
    msg.pitchspeed = angular_rate.y;
    msg.yawspeed = angular_rate.z;
    if (Lock_SDLog(SyncTIMEOUT))
    {
        xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);

        UnLock_SDLog();
        return true;
    }
    return false;
}
bool SDLog_Msg_AttitudeQuaternion(double SyncTIMEOUT)
{
    Quaternion airframe_quat;
    if (get_AirframeY_quat(&airframe_quat, SyncTIMEOUT) == false)
        return false;
    vector3<double> angular_rate;
    if (get_AngularRate_Ctrl(&angular_rate, SyncTIMEOUT) == false)
        return false;
    struct Msg
    {
        uint8_t msg_type;
        uint8_t length;
        uint16_t rsv;
        uint32_t Time;
        double q0;
        double q1;
        double q2;
        double q3;
        double rollspeed;
        double pitchspeed;
        double yawspeed;
    } __AC_PACKED;
    Msg msg;

    msg.msg_type = LogMsg_AttitudeQuaternion;
    msg.length = sizeof(msg);
    msg.Time = TIME::get_System_Run_Time() * 1e+4;
    msg.q0 = airframe_quat.get_qw();
    msg.q1 = airframe_quat.get_qx();
    msg.q2 = airframe_quat.get_qy();
    msg.q3 = airframe_quat.get_qz();
    msg.rollspeed = angular_rate.x;
    msg.pitchspeed = angular_rate.y;
    msg.yawspeed = angular_rate.z;
    if (Lock_SDLog(SyncTIMEOUT))
    {
        xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);

        UnLock_SDLog();
        return true;
    }
    return false;
}
bool SDLog_Msg_LocalPositionNed(double SyncTIMEOUT)
{
    vector3<double> pos;
    vector3<double> vel;
    vector3<double> acc;
    if (get_Position_Ctrl(&pos, SyncTIMEOUT) == false)
        return false;
    if (get_VelocityENU_Ctrl(&vel, SyncTIMEOUT) == false)
        return false;
    if (get_AccelerationENU_Ctrl(&acc, SyncTIMEOUT) == false)
        return false;
    struct Msg
    {
        uint8_t msg_type;
        uint8_t length;
        int8_t XYSensor;
        int8_t ZSensor;
        uint32_t Time;
        double posx;
        double posy;
        double posz;
        double xspeed;
        double yspeed;
        double zspeed;
        double accx;
        double accy;
        double accz;
    } __AC_PACKED;
    Msg msg;

    msg.msg_type = LogMsg_LocalPositionNed;
    msg.length = sizeof(msg);
    msg.XYSensor = get_Current_XYSensor();
    msg.ZSensor = get_Current_ZSensor();
    msg.Time = TIME::get_System_Run_Time() * 1e+4;
    msg.posx = pos.y * 0.01;
    msg.posy = pos.x * 0.01;
    msg.posz = -pos.z * 0.01;
    msg.xspeed = vel.y * 0.01;
    msg.yspeed = vel.x * 0.01;
    msg.zspeed = -vel.z * 0.01;
    msg.accx = acc.y * 0.01;
    msg.accy = acc.x * 0.01;
    msg.accz = -acc.z * 0.01;
    if (Lock_SDLog(SyncTIMEOUT))
    {
        xMessageBufferSend(LogStreamBuffer, &msg, sizeof(msg), 0);

        UnLock_SDLog();
        return true;
    }
    return false;
}
bool SDLog_Msg_DebugVect(const char *name, double vect[], uint8_t length, double SyncTIMEOUT)
{
    if (length > 20)
        return false;
    struct Msg
    {
        uint8_t msg_type;
        uint8_t length;
        uint16_t rsv;
        uint32_t Time;
        char name[10];
    } __AC_PACKED;

    uint8_t *buf = new uint8_t[sizeof(Msg) + length * 8];
    Msg *msg = (Msg *) buf;
    msg->msg_type = LogMsg_DebugVect;
    msg->length = sizeof(Msg) + length * 8;
    msg->Time = TIME::get_System_Run_Time() * 1e+4;
    for (uint8_t i = 0; i < 10; ++i)
    {
        msg->name[i] = name[i];
        if (name[i] == 0)
            break;
    }
    memcpy(&buf[sizeof(Msg)], vect, length * 8);

    if (Lock_SDLog(SyncTIMEOUT))
    {
        xMessageBufferSend(LogStreamBuffer, buf, msg->length, 0);

        UnLock_SDLog();
        delete[] buf;
        return true;
    }
    delete[] buf;
    return false;
}
/*Log数据*/
/*SD Log*/

/*Txt1 Log*/
// Log缓冲区
#define Txt1BufSize 4096
Static_DTCMBuf uint8_t Txt1LogStreamBufferStorage[Txt1BufSize + 1];
Static_DTCMBuf StaticMessageBuffer_t Txt1LogStreamBufferStruct;
Static_DTCMBuf MessageBufferHandle_t Txt1LogStreamBuffer = xMessageBufferCreateStatic(Txt1BufSize, Txt1LogStreamBufferStorage, &Txt1LogStreamBufferStruct);
;
static SemaphoreHandle_t Txt1LogSemphr = xSemaphoreCreateMutex();
static bool Lock_Txt1Log(double TIMEOUT = -1)
{
    //    if (get_is_usb_connected())
    //        return false;
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(Txt1LogSemphr, TIMEOUT_Ticks))
        return true;
    return false;
}
static void UnLock_Txt1Log()
{
    xSemaphoreGive(Txt1LogSemphr);
}

bool SDLog_Txt1(const char *txt, uint16_t length, double SyncTIMEOUT)
{
    if (Lock_Txt1Log(SyncTIMEOUT))
    {
        xMessageBufferSend(Txt1LogStreamBuffer, txt, length, 0);

        UnLock_Txt1Log();
        return true;
    }
    return false;
}
/*Txt1 Log*/

/*ubx文件记录 Log*/
// ubx文件记录缓冲区
#define UbxBufSize 84 * 1024
__attribute__((aligned(4))) Static_SRAM2Buf uint8_t UbxLogStreamBufferStorage[UbxBufSize + 1];
Static_DTCMBuf StaticMessageBuffer_t UbxLogStreamBufferStruct;
Static_DTCMBuf MessageBufferHandle_t UbxLogStreamBuffer = xMessageBufferCreateStatic(UbxBufSize, UbxLogStreamBufferStorage, &UbxLogStreamBufferStruct);
;
static SemaphoreHandle_t UbxLogSemphr = xSemaphoreCreateMutex();
static bool Lock_UbxLog(double TIMEOUT = -1)
{
    //    if (get_is_usb_connected())
    //        return false;
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(UbxLogSemphr, TIMEOUT_Ticks))
        return true;
    return false;
}
static void UnLock_UbxLog()
{
    xSemaphoreGive(UbxLogSemphr);
}

bool SDLog_Ubx(const char *txt, uint16_t length, double SyncTIMEOUT)
{
    if (Lock_UbxLog(SyncTIMEOUT))
    {
        xMessageBufferSend(UbxLogStreamBuffer, txt, length, 0);

        UnLock_UbxLog();
        return true;
    }
    return false;
}
/*ubx文件记录 Log*/

/*sd卡清理*/
/**
 * @brief 解析数字日期字符串为整数值。
 * @param str 要解析的字符串（目录名 YYYYMMDD 或文件名前缀 YYYYMMDDHHMM）。
 * @param digits 需解析的前导数字位数，默认 8（YYYYMMDD）。
 * @param strict true(默认)：字符串总长度必须等于 digits（用于目录名严格匹配，
 *               避免 "ACFly"、".aclog" 等被误判为日期）；
 *               false：只要求长度 >= digits，解析前 digits 位（用于文件名前缀）。
 * @return 由前 digits 位拼成的整数值；无效格式返回 0。
 */
static uint32_t parse_date_dir(const char *str, int digits = 8, bool strict = true)
{
    size_t len = strlen(str);
    if (strict)
    {
        if ((int) len != digits)
            return 0;
    }
    else
    {
        if ((int) len < digits)
            return 0;
    }
    uint32_t v = 0;
    for (int i = 0; i < digits; i++)
    {
        if (str[i] < '0' || str[i] > '9')
            return 0;
        v = v * 10 + (str[i] - '0');
    }
    return v;
}

/**
 * @brief 将字节大小转换为人类可读的字符串格式 (如 1.2 MB)
 * @param bytes 文件大小，单位为字节 (FSIZE_t 类型)
 * @param output 存储格式化结果的缓冲区
 * @param max_len 缓冲区最大长度
 */
void format_file_size(FSIZE_t bytes, char *output, size_t max_len)
{
    double size = (double) bytes;
    const char *units[] = {"B", "KB", "MB", "GB", "TB"};
    int unit_index = 0;
    const int base = 1024;

    // 处理 0 字节的情况
    if (bytes == 0)
    {
        snprintf(output, max_len, "0 B");
        return;
    }

    // 循环直到找到合适的单位
    while (size >= base && unit_index < (sizeof(units) / sizeof(units[0]) - 1))
    {
        size /= base;
        unit_index++;
    }

    // 对于小于 1KB 的大小，避免显示 0.x KB，直接显示 B
    if (unit_index == 0)
    {
        snprintf(output, max_len, "%lu B", (unsigned long) bytes);
        return;
    }

    // 格式化输出：保留一位小数
    snprintf(output, max_len, "%.1f %s", size, units[unit_index]);
}

/**
 * @brief 递归删除目录及其内容，并记录日志
 * @param fs 文件系统对象
 * @param path 目录路径（如 "0:/20250101"）
 * @param log_file 日志文件对象（NULL 如果不记录）
 * @param files_deleted 文件删除计数
 * @param dirs_deleted 文件夹删除计数
 * @return FRESULT 操作结果
 */
static FRESULT delete_directory(FATFS *fs, const char *path, FIL *log_file, uint32_t *files_deleted, uint32_t *dirs_deleted, uint8_t hierarchy)
{
    DIR dir;
    FILINFO fno;
    FRESULT res;
    char subpath[64];
    char log_txt[128];
    UINT bw;

    res = f_opendir(&dir, path);
    if (res != FR_OK)
        return res;

    bool log_txt_del_first = true;
    uint8_t log_txt_f = hierarchy + 1;
    for (uint8_t i = 0; i < hierarchy + 1; ++i)
    {
        log_txt[i] = '\t';
    }

    while (1)
    {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0)
            break;
        if (fno.fname[0] == '.')
            continue;

        snprintf(subpath, sizeof(subpath), "%s/%s", path, fno.fname);

        if (fno.fattrib & AM_DIR)
        {
            // 递归删除子目录
            if (hierarchy < 2)
            {
                res = delete_directory(fs, subpath, log_file, files_deleted, dirs_deleted, ++hierarchy);
                // if (res != FR_OK) {
                //     f_closedir(&dir);
                //     return res;
                // }
                if (res == FR_OK)
                    (*dirs_deleted)++;
            }
        }
        else
        {
            // 删除文件
            FSIZE_t file_size = fno.fsize;
            res = f_unlink(subpath);
            if (res != FR_OK)
            {
                f_closedir(&dir);
                return res;
            }
            (*files_deleted)++;
            // 记录日志
            if (log_file)
            {
                if (log_txt_del_first)
                {
                    log_txt_del_first = false;

                    snprintf(&log_txt[log_txt_f], sizeof(log_txt) - log_txt_f, "Deleting directory: %s\r\n", path);
                    f_write(log_file, log_txt, strlen(log_txt), &bw);
                }

                char fileSiz_txt[12];
                format_file_size(file_size, fileSiz_txt, sizeof(fileSiz_txt));
                snprintf(&log_txt[log_txt_f], sizeof(log_txt) - log_txt_f, "\tDeleted file: %s FileSize:%s\r\n", subpath, fileSiz_txt);
                f_write(log_file, log_txt, strlen(log_txt), &bw);
                f_sync(log_file);
            }
        }
    }

    f_closedir(&dir);
    // 删除空目录
    res = f_unlink(path);
    if (res == FR_OK && log_file)
    {
        (*dirs_deleted)++;
        snprintf(&log_txt[log_txt_f], sizeof(log_txt) - log_txt_f, "Deleted directory: %s\r\n", path);
        f_write(log_file, log_txt, strlen(log_txt), &bw);
        f_sync(log_file);
    }
    return res;
}

/**
 * @brief 清理旧日期文件夹（排除当前日期）
 * @param fs 文件系统对象
 * @param root_dir 根目录（如 "0:/"）
 * @param target_free_bytes 目标剩余空间
 * @param current_year 当前年份
 * @param current_mon 当前月份
 * @param current_mday 当前日期
 * @param log_file 日志文件对象（NULL 如果不记录）
 * @param files_deleted 文件删除计数
 * @param dirs_deleted 文件夹删除计数
 * @return FRESULT 操作结果
 */
static FRESULT cleanup_oldest_dirs(FATFS *fs, const char *root_dir, uint64_t target_free_bytes, int current_year, int current_mon, int current_mday, FIL *log_file, uint32_t *files_deleted,
                                   uint32_t *dirs_deleted)
{
    DIR dir;
    FILINFO fno;
    FRESULT res;
    uint32_t oldest_date = 0xFFFFFFFF;
    char oldest_dir[16] = "";
    char path[64];
    DWORD fre_clust;
    uint64_t free_bytes;
    uint32_t current_date = current_year * 10000 + current_mon * 100 + current_mday;

    while (1)
    {
        res = f_getfree(root_dir, &fre_clust, &fs);
        if (res != FR_OK)
            return res;
        free_bytes = (uint64_t) fre_clust * fs->csize * 512;
        if (free_bytes >= target_free_bytes)
            return FR_OK;

        res = f_opendir(&dir, root_dir);
        if (res != FR_OK)
            return res;

        oldest_date = 0xFFFFFFFF;
        oldest_dir[0] = 0;

        while (1)
        {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
                break;
            if (fno.fname[0] == '.' || !(fno.fattrib & AM_DIR))
                continue;

            uint32_t date = parse_date_dir(fno.fname);
            if (date != 0 && date < oldest_date && date != current_date)
            {
                oldest_date = date;
                strncpy(oldest_dir, fno.fname, sizeof(oldest_dir) - 1);
                oldest_dir[sizeof(oldest_dir) - 1] = 0;
            }
        }
        f_closedir(&dir);

        if (oldest_date == 0xFFFFFFFF)
            return FR_NO_FILE;

        snprintf(path, sizeof(path), "%s%s", root_dir, oldest_dir);
        res = delete_directory(fs, path, log_file, files_deleted, dirs_deleted, 0);
        if (res != FR_OK)
            return res;
    }

    return FR_OK;
}

/**
 * @brief 解析日志文件名，提取基时间戳、Index 和 logFileDiffInd
 * @param filename 文件名（如 "202510121001(2)_3(1).aclog"）
 * @param base_timestamp 输出基时间戳（如 202510121001）
 * @param index 输出 Index（0 如果无）
 * @param logFileDiffInd 输出 logFileDiffInd（0 如果无）
 * @return 1 如果有效，0 否则
 */
static int parse_log_file(const char *filename, uint32_t *base_timestamp, int *index, int *logFileDiffInd)
{
    size_t len = strlen(filename);
    if (len < 13 || strcmp(&filename[len - 6], ".aclog") != 0)
        return 0;

    *base_timestamp = 0;
    *index = 0;
    *logFileDiffInd = 0;

    char base[16] = {0};
    size_t base_copy_len = len - 6; // 到 .aclog 前的长度
    if (base_copy_len >= sizeof(base))
        return 0; // 溢出风险

    // 复制完整 base（包含可能的 DiffInd 和 Index）
    strncpy(base, filename, base_copy_len);

    // 查找最后一个 _ 获取 Index
    char *underscore = strrchr(base, '_');
    if (underscore)
    {
        // 计算 _ 到末尾的长度
        size_t index_len = base_copy_len - (underscore - base);
        if (index_len == 0)
            return 0;

        char index_str[4] = {0};
        size_t copy_len = index_len < sizeof(index_str) - 1 ? index_len : sizeof(index_str) - 1;
        strncpy(index_str, underscore + 1, copy_len);

        // 截断到第一个非数字
        for (size_t i = 0; i < copy_len && index_str[i]; i++)
        {
            if (index_str[i] < '0' || index_str[i] > '9')
            {
                index_str[i] = 0;
                break;
            }
        }
        if (index_str[0] == 0)
            return 0;

        *index = atoi(index_str);

        // 截断 base 到 _ 前
        *underscore = 0;
    }

    // 查找 DiffInd：YYYYMMDDHHMM(Ind)
    char *open_paren = strrchr(base, '(');
    if (open_paren && open_paren > base + 12)
    { // 确保在 YYYYMMDDHHMM 后
        char *close_paren = strrchr(open_paren, ')');
        if (close_paren && close_paren == open_paren + 2)
        { // (X) 格式，3 个字符
            // 提取 DiffInd：(2) -> 2
            if (open_paren[1] >= '0' && open_paren[1] <= '9')
            {
                *logFileDiffInd = open_paren[1] - '0';
                // 截断 base 到 ( 前，提取时间戳
                *open_paren = 0;
            }
        }
    }

    // 解析时间戳：YYYYMMDDHHMM
    size_t base_len = strlen(base);
    if (base_len != 12)
        return 0;

    // base 为 12 位 YYYYMMDDHHMM。以宽松模式解析前 8 位年月日。
    // 早期版本误用 parse_date_dir(base) —— 该函数 strict=true 且 digits=8，
    // 对 12 位 base 直接返回 0，导致当前会话的所有日志 base_timestamp 被算成
    // 极小值，被 cleanup_current_dir 把当前正在写入的日志误判为旧文件删除。
    *base_timestamp = parse_date_dir(base, 8, false) * 10000 + (base[8] - '0') * 1000 + (base[9] - '0') * 100 + (base[10] - '0') * 10 + (base[11] - '0');

    return 1;
}

/**
 * @brief 清理当前日期文件夹内文件（保护当前日志系列）
 * @param fs 文件系统对象
 * @param current_dir 当前日期文件夹路径（如 "0:/20251012/"）
 * @param target_free_bytes 目标剩余空间
 * @param current_timestamp 当前日志基时间戳（如 202510121001）
 * @param logFileDiffInd 当前 DiffInd
 * @param log_file 日志文件对象（NULL 如果不记录）
 * @param files_deleted 文件删除计数
 * @return FRESULT 操作结果
 */
static FRESULT cleanup_current_dir(FATFS *fs, const char *root_dir, const char *current_dir, uint64_t target_free_bytes, uint32_t current_timestamp, int logFileDiffInd, int logFileInd, FIL *log_file,
                                   uint32_t *files_deleted)
{
    DIR dir;
    FILINFO fno;
    FRESULT res;
    uint32_t oldest_timestamp = 0xFFFFFFFF;
    int oldest_index = 0x7FFFFFFF;
    char oldest_file[32] = "";
    char path[64];
    char log_txt[128];
    UINT bw;
    DWORD fre_clust;
    uint64_t free_bytes;

    while (1)
    {
        res = f_getfree(root_dir, &fre_clust, &fs);
        if (res != FR_OK)
            return res;
        free_bytes = (uint64_t) fre_clust * fs->csize * 512;
        if (free_bytes >= target_free_bytes)
            return FR_OK;

        res = f_opendir(&dir, current_dir);
        if (res != FR_OK)
            return res;

        oldest_timestamp = 0xFFFFFFFF;
        oldest_index = 0x7FFFFFFF;
        oldest_file[0] = 0;
        FSIZE_t oldest_file_size = 0;

        while (1)
        {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
                break;
            if (fno.fname[0] == '.' || (fno.fattrib & AM_DIR))
                continue;

            uint32_t timestamp;
            int index;
            int diff_ind;
            if (!parse_log_file(fno.fname, &timestamp, &index, &diff_ind))
                continue;

            // 保护当前日志系列（基时间戳 == current_timestamp，index > 0 或主文件）
            if (timestamp == current_timestamp && diff_ind == logFileDiffInd)
            {
                if (index >= logFileInd)
                    continue;
                if (index < oldest_index)
                {
                    oldest_index = index;
                    oldest_file_size = fno.fsize;
                    strncpy(oldest_file, fno.fname, sizeof(oldest_file) - 1);
                }
            }
            else
            {
                if (timestamp < oldest_timestamp)
                {
                    oldest_timestamp = timestamp;
                    oldest_file_size = fno.fsize;
                    strncpy(oldest_file, fno.fname, sizeof(oldest_file) - 1);
                }
            }
        }
        f_closedir(&dir);

        if (oldest_file[0] == 0)
            return FR_NO_FILE;

        snprintf(path, sizeof(path), "%s%s", current_dir, oldest_file);
        res = f_unlink(path);
        if (res != FR_OK)
            return res;

        (*files_deleted)++;
        if (log_file)
        {
            char fileSiz_txt[12];
            format_file_size(oldest_file_size, fileSiz_txt, sizeof(fileSiz_txt));
            snprintf(log_txt, sizeof(log_txt), "\tDeleted file: %s FileSize:%s\r\n", path, fileSiz_txt);
            f_write(log_file, log_txt, strlen(log_txt), &bw);
            f_sync(log_file);
        }
    }

    return FR_OK;
}

/**
 * @brief 主清理函数：先清理旧文件夹，再清理当前文件夹，并记录日志
 * @param fs 文件系统对象
 * @param root_dir 根目录（如 "0:/"）
 * @param log_mount_path 日志挂载路径（如 "0:/"）
 * @param target_free_bytes 目标剩余空间
 * @param current_year 当前年份
 * @param current_mon 当前月份
 * @param current_mday 当前日期
 * @param current_hour 当前小时
 * @param current_min 当前分钟
 * @param current_sec 当前秒
 * @param logFileDiffInd 当前 DiffInd
 * @return FRESULT 操作结果
 */
FRESULT cleanup_logs(FATFS *fs, const char *root_dir, uint64_t target_free_bytes, int current_year, int current_mon, int current_mday, int current_hour, int current_min, int logFileDiffInd,
                     int logFileInd)
{
    FRESULT res;
    DWORD fre_clust;
    uint64_t free_bytes;
    uint32_t files_deleted = 0;
    uint32_t dirs_deleted = 0;
    FIL log_file;
    char filename[64];
    char log_txt[128];
    UINT bw;

    // 检查初始空间
    res = f_getfree(root_dir, &fre_clust, &fs);
    if (res != FR_OK)
        return res;
    free_bytes = (uint64_t) fre_clust * fs->csize * 512;
    if (free_bytes >= target_free_bytes)
        return FR_OK; // 无需清理

    // 打开日志文件
    filename[0] = 0;
    strncat(filename, log_mountPath, sizeof(filename) - 1);
    strncat(filename, "ACFly/sysLog.txt", sizeof(filename) - strlen(filename) - 1);
    res = f_open(&log_file, filename, FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK)
        return res;

    // 记录清理开始
    tm current_tm = Get_RTC_Time_tm();
    current_tm.tm_year += 1900;
    current_tm.tm_mon += 1;
    snprintf(log_txt, sizeof(log_txt), "\r\nSD Cleaner\r\n%d/%02d/%02d-%02d:%02d:%02d\r\n", current_tm.tm_year, current_tm.tm_mon, current_tm.tm_mday, current_tm.tm_hour, current_tm.tm_min,
             current_tm.tm_sec);
    res = f_write(&log_file, log_txt, strlen(log_txt), &bw);
    if (res != FR_OK)
    {
        f_close(&log_file);
        return res;
    }
    f_sync(&log_file);

    // 记录总空间和剩余空间
    DWORD tot_sectors;
    res = f_getfree(root_dir, &fre_clust, &fs);
    if (res != FR_OK)
    {
        f_close(&log_file);
        return res;
    }
    tot_sectors = (fs->n_fatent - 2) * fs->csize; // 总扇区数
    float totGB = (float) tot_sectors * 512 / (1024 * 1024 * 1024);
    float freGB = (float) fre_clust * fs->csize * 512 / (1024 * 1024 * 1024);
    snprintf(log_txt, sizeof(log_txt), "Total Space:%.2fGB\r\nFree Space:%.2fGB\r\n", totGB, freGB);
    res = f_write(&log_file, log_txt, strlen(log_txt), &bw);
    snprintf(log_txt, sizeof(log_txt), "Requested space:%.2fGB\r\n", (float) target_free_bytes / (1024 * 1024 * 1024));
    res = f_write(&log_file, log_txt, strlen(log_txt), &bw);
    if (res != FR_OK)
    {
        f_close(&log_file);
        return res;
    }
    f_sync(&log_file);

    // 清理旧日期文件夹
    res = cleanup_oldest_dirs(fs, root_dir, target_free_bytes, current_year, current_mon, current_mday, &log_file, &files_deleted, &dirs_deleted);
    if (res != FR_OK && res != FR_NO_FILE)
    {
        f_close(&log_file);
        return res;
    }

    // 检查空间
    res = f_getfree(root_dir, &fre_clust, &fs);
    if (res != FR_OK)
    {
        f_close(&log_file);
        return res;
    }
    if ((uint64_t) fre_clust * fs->csize * 512 >= target_free_bytes)
    {
        // 记录完成信息
        tot_sectors = (fs->n_fatent - 2) * fs->csize;
        totGB = (float) tot_sectors * 512 / (1024 * 1024 * 1024);
        freGB = (float) fre_clust * fs->csize * 512 / (1024 * 1024 * 1024);
        snprintf(log_txt, sizeof(log_txt),
                 "%d Files and %d Directories deleted.\r\nTotal Space:%.1fGB\r\nFree "
                 "Space:%.1fGB\r\nSD Cleanup Completed.\r\n",
                 files_deleted, dirs_deleted, totGB, freGB);
        f_write(&log_file, log_txt, strlen(log_txt), &bw);
        f_sync(&log_file);
        f_close(&log_file);
        return FR_OK;
    }

    // 清理当前日期文件夹
    char current_dir_path[64];
    snprintf(current_dir_path, sizeof(current_dir_path), "%s%d%02d%02d/", root_dir, current_year, current_mon, current_mday);
    uint32_t current_timestamp = (uint32_t) current_year * 100000000 + current_mon * 1000000 + current_mday * 10000 + current_hour * 100 + current_min;
    res = cleanup_current_dir(fs, root_dir, current_dir_path, target_free_bytes, current_timestamp, logFileDiffInd, logFileInd, &log_file, &files_deleted);
    if (res != FR_OK && res != FR_NO_FILE)
    {
        f_close(&log_file);
        return res;
    }

    // 记录完成信息
    res = f_getfree(root_dir, &fre_clust, &fs);
    if (res != FR_OK)
    {
        f_close(&log_file);
        return res;
    }
    tot_sectors = (fs->n_fatent - 2) * fs->csize;
    totGB = (float) tot_sectors * 512 / (1024 * 1024 * 1024);
    freGB = (float) fre_clust * fs->csize * 512 / (1024 * 1024 * 1024);
    snprintf(log_txt, sizeof(log_txt),
             "%d Files and %d Directories deleted.\r\nTotal Space:%.2fGB\r\nFree "
             "Space:%.2fGB\r\nSD Cleanup Completed.\r\n",
             files_deleted, dirs_deleted, totGB, freGB);
    res = f_write(&log_file, log_txt, strlen(log_txt), &bw);
    f_sync(&log_file);
    f_close(&log_file);

    return res;
}
/*sd卡清理*/

/*SD卡更新参数*/
// 最大行字节数
#define maxLineLen 100
// 最大参数名字节数
#define maxParamNameLen 16

// ASCALL码转char
static inline char ConvertHexChar(char ch)
{
    if ((ch >= '0') && (ch <= '9'))
        return (ch - '0');
    else if ((ch >= 'A') && (ch <= 'F'))
        return ((ch - 'A') + 10);
    else if ((ch >= 'a') && (ch <= 'f'))
        return ((ch - 'a') + 10);
    else
        return (-1);
}
static inline bool runParamCheck(FIL *fil, bool doUpdateParam, bool &logFilOpened, FIL *logFil, char *logTxt, uint16_t *updatedParamsCount = 0)
{
    uint8_t illegal_char_counter = 0;
    uint16_t lineNumber = 0;
    while (1)
    {
        // 读取一行字符
        char txt[maxLineLen];
        txt[maxLineLen - 1] = 0;
        char *readRes = f_gets(txt, maxLineLen - 1, fil);
        if (readRes == txt)
        { // 读取成功

            ++lineNumber;
            // 获取行长度
            size_t lineSize = strlen(txt);
            // 行结束符必须为\n
            if (txt[lineSize - 1] != '\n' && !f_eof(fil))
            {
                if (logFilOpened)
                {
                    sprintf(logTxt, "\tParam file data error. No CRLF detected or line too long.\tLine: %d\r\n", lineNumber);
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
                return false;
            }
            // 注释符直接开始读下一行
            if (txt[0] == '#')
                continue;
            // 校验和
            uint8_t checkSum = 0;

            // 跳过空格
            uint8_t paramName_start = 0;
            for (uint8_t i = 0; i < lineSize; ++i)
            {
                if (lineNumber == 1 && illegal_char_counter <= 4 && i == illegal_char_counter && txt[i] > 126)
                { // 文件头编码
                    // 跳过
                    ++illegal_char_counter;
                }
                else if (txt[i] == ' ' || txt[i] == '\t')
                { // 空格
                  // 跳过
                }
                else if (txt[i] == '\r' || txt[i] == '\n')
                    // 换行符读下一行
                    goto CONTINE_READLINE;
                else
                {
                    paramName_start = i;
                    break;
                }
            }

            // 获取参数名称
            uint8_t paramName_len = 0;
            for (uint8_t i = paramName_start; i < lineSize; ++i)
            {
                if (txt[i] == ',' || txt[i] == ' ' || txt[i] == '\t')
                { // 参数分隔符
                    txt[i] = 0;
                    if (paramName_len == 0 || paramName_len > maxParamNameLen)
                    { // 参数名称不正确
                        if (logFilOpened)
                        {
                            sprintf(logTxt, "\tParam name error. Param name empty or too long.\tLine: %d\r\n", lineNumber);
                            f_puts(logTxt, logFil);
                            f_sync(logFil);
                        }
                        return false;
                    }
                    break;
                }
                else if (txt[i] >= 33 && txt[i] <= 126)
                { // 参数字符
                    checkSum += txt[i];
                    ++paramName_len;
                }
                else
                { // 参数名含有不合法字符
                    if (logFilOpened)
                    {
                        sprintf(logTxt,
                                "\tParam name error. Param name contains illegal charactor.\tLine: "
                                "%d\r\n",
                                lineNumber);
                        f_puts(logTxt, logFil);
                        f_sync(logFil);
                    }
                    return false;
                }
            }

            // 获取参数数值
            uint8_t paramValue_start = paramName_start + paramName_len + 1;
            uint8_t paramValue_len = 0;
            float paramValue_f = std::nanf("");
            for (uint8_t i = paramValue_start; i < lineSize; ++i)
            {
                if (txt[i] == ',' || txt[i] == ' ' || txt[i] == '\t')
                { // 参数分隔符
                    txt[i] = 0;
                    break;
                }
                else if (is_number(txt[i]) || txt[i] == '.' || txt[i] == '-' || txt[i] == '+')
                { // 参数数值字符
                    checkSum += txt[i];
                    ++paramValue_len;
                }
                else
                { // 参数数值含有不合法字符
                    if (logFilOpened)
                    {
                        sprintf(logTxt,
                                "\tParam value error. Param value contains illegal "
                                "charactor.\tLine: %d\r\n",
                                lineNumber);
                        f_puts(logTxt, logFil);
                        f_sync(logFil);
                    }
                    return false;
                }
            }
            paramValue_f = atof(&txt[paramValue_start]);
            if (!isvalid(paramValue_f))
            {
                if (logFilOpened)
                {
                    sprintf(logTxt, "\tParam value error. Param value data illegal.\tLine: %d\r\n", lineNumber);
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
                // 参数数值错误
                return false;
            }

            // 获取参数类型
            uint8_t paramType_start = paramValue_start + paramValue_len + 1;
            uint8_t paramType_len = 0;
            uint8_t paramType = 0;
            for (uint8_t i = paramType_start; i < lineSize; ++i)
            {
                if (txt[i] == ',' || txt[i] == ' ' || txt[i] == '\t')
                { // 参数分隔符
                    txt[i] = 0;
                    break;
                }
                else if (is_number(txt[i]))
                { // 参数类别字符
                    checkSum += txt[i];
                    ++paramType_len;
                }
                else
                { // 参数类别含有不合法字符
                    if (logFilOpened)
                    {
                        sprintf(logTxt,
                                "\tParam type error. Param type contains illegal charactor.\tLine: "
                                "%d\r\n",
                                lineNumber);
                        f_puts(logTxt, logFil);
                        f_sync(logFil);
                    }
                    return false;
                }
            }
            if (paramType_len == 0 || paramType_len > 2)
            {
                if (logFilOpened)
                {
                    sprintf(logTxt, "\tParam type error. Param type data illegal.\tLine: %d\r\n", lineNumber);
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
                // 参数类别错误
                return false;
            }
            paramType = atoi(&txt[paramType_start]);
            if (paramType == 0 || paramType > 10)
            {
                if (logFilOpened)
                {
                    sprintf(logTxt, "\tParam type error. Param type data illegal.\tLine: %d\r\n", lineNumber);
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
                // 参数类别错误
                return false;
            }

            // 获取校验
            uint8_t paramChecksum_start = paramType_start + paramType_len + 1;
            uint8_t paramChecksum_len = 0;
            uint8_t paramChecksum = 0;
            for (uint8_t i = paramChecksum_start; i < lineSize; ++i)
            {
                if (txt[i] == ',' || txt[i] == ' ' || txt[i] == '\t' || txt[i] == '\r')
                { // 参数分隔符
                    txt[i] = 0;
                    break;
                }
                else if (is_number(txt[i]) || is_lowercase_letter(txt[i]) || is_capital_letter(txt[i]))
                { // 参数校验字符
                    ++paramChecksum_len;
                }
                else
                { // 参数校验含有不合法字符
                    if (logFilOpened)
                    {
                        sprintf(logTxt,
                                "\tParam checksum error. Param checksum contains illegal "
                                "charactor.\tLine: %d\r\n",
                                lineNumber);
                        f_puts(logTxt, logFil);
                        f_sync(logFil);
                    }
                    return false;
                }
            }
            if (paramChecksum_len == 0 || paramChecksum_len > 2)
            {
                if (logFilOpened)
                {
                    sprintf(logTxt, "\tParam checksum error. Param checksum data illegal.\tLine: %d\r\n", lineNumber);
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
                // 参数校验错误
                return false;
            }
            uint8_t rdChecksum = 0;
            for (uint8_t i = paramChecksum_start; i < paramChecksum_start + paramChecksum_len; ++i)
            { // 求校验和
                rdChecksum = (rdChecksum << 4) | ConvertHexChar(txt[i]);
            }
            if (checkSum != rdChecksum)
            {
                if (logFilOpened)
                {
                    sprintf(logTxt, "\tParam checksum error. Param checksum error.\tLine: %d\r\n", lineNumber);
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
                // 参数校验错误
                return false;
            }
            else
            { // 校验成功
                SName param_name(&txt[paramName_start]);
                MAV_PARAM_TYPE param_type;
                __attribute__((aligned(8))) uint64_t paramValue_o;
                if (ReadParam(param_name, 0, &param_type, &paramValue_o, 0) == PR_OK)
                { // 读取参数成功
                    if (param_type != paramType)
                    {
                        if (logFilOpened)
                        {
                            sprintf(logTxt,
                                    "\tParam type error. Param type doesn't match.\tLine: "
                                    "%d\tParam name:%s\r\n",
                                    lineNumber, &txt[paramName_start]);
                            f_puts(logTxt, logFil);
                            f_sync(logFil);
                        }
                        // 参数类型不符合
                        return false;
                    }
                    if (doUpdateParam)
                    { // 需要更新参数
                        uint64_t paramValue;
                        float paramValue_of = std::nanf("");
                        switch (param_type)
                        {
                            case MAV_PARAM_TYPE_INT8:
                                paramValue = (int8_t) paramValue_f;
                                paramValue_of = *(int8_t *) &paramValue_o;
                                break;
                            case MAV_PARAM_TYPE_UINT8:
                                paramValue = (uint8_t) paramValue_f;
                                paramValue_of = *(uint8_t *) &paramValue_o;
                                ;
                                break;
                            case MAV_PARAM_TYPE_INT16:
                                paramValue = (int16_t) paramValue_f;
                                paramValue_of = *(int16_t *) &paramValue_o;
                                break;
                            case MAV_PARAM_TYPE_UINT16:
                                paramValue = (uint16_t) paramValue_f;
                                paramValue_of = *(uint16_t *) &paramValue_o;
                                break;
                            case MAV_PARAM_TYPE_INT32:
                                paramValue = (int32_t) paramValue_f;
                                paramValue_of = *(int32_t *) &paramValue_o;
                                break;
                            case MAV_PARAM_TYPE_UINT32:
                                paramValue = (uint32_t) paramValue_f;
                                paramValue_of = *(uint32_t *) &paramValue_o;
                                break;
                            case MAV_PARAM_TYPE_REAL32:
                                paramValue = *(uint32_t *) &paramValue_f;
                                paramValue_of = *(float *) &paramValue_o;
                                break;
                            default:
                                continue;
                                break;
                        }
                        if (isvalid(paramValue_of))
                        {
                            char valueStr1[30];
                            char valueStr2[30];
                            sprintf(valueStr1, "%.3f", paramValue_f);
                            sprintf(valueStr2, "%.3f", paramValue_of);
                            if (strcmp(valueStr1, valueStr2) != 0)
                            { // 参数不一致需要保存
                                if (strcmp(&txt[paramName_start], "Init_Boot_Count") && strcmp(&txt[paramName_start], "Init_Firmware_V"))
                                {
                                    UpdateParam(param_name, paramValue);
                                    if (updatedParamsCount)
                                        ++(*updatedParamsCount);
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (logFilOpened)
                    {
                        sprintf(logTxt,
                                "\tParam name error. No parameter named \"%s\" in the "
                                "controller.\tLine: %d\r\n",
                                &txt[paramName_start], lineNumber);
                        f_puts(logTxt, logFil);
                        f_sync(logFil);
                    }
                    // 无此参数
                    return false;
                }
            }
        }
        else
            // 读取结束
            break;

    CONTINE_READLINE:
        continue;
    }
    return true;
}
/*用sd卡参数文件更新参数*/
enum UDPARAM_RS
{
    UDPARAM_RS_NOFILE = 0,
    UDPARAM_RS_SUCCESS = 1,
    UDPARAM_RS_FAIL = -1,
};
static inline UDPARAM_RS updateParams(FIL *fil, FIL *logFil)
{
    // 执行结果
    UDPARAM_RS res = UDPARAM_RS_NOFILE;
    // 文件指针
    DIR dir;
    char filename[80];
    // 文件操作状态
    FRESULT fres;
    // 文件（夹）状态
    FILINFO finfo;
    // 是否查找到参数文件
    bool file_found = false;
    // 是否有ACFly目录
    filename[0] = 0;
    strcat(filename, usr_mountPath);
    strcat(filename, "ACFlySetup");
    fres = f_stat(filename, &finfo);
    if (fres == FR_OK)
    { // 查找acParams后缀文件
        fres = f_findfirst(&dir, &finfo, filename, "*.acParams");
        if (fres == FR_OK && finfo.fname[0])
            file_found = true;
        // 关闭文件夹
        f_closedir(&dir);
    }
    // 文件未找到尝试备选1
    if (!file_found)
    {
        filename[0] = 0;
        strcat(filename, log_mountPath);
        strcat(filename, "ACFly");
        fres = f_stat(filename, &finfo);
        if (fres == FR_OK)
        { // 查找acParams后缀文件
            fres = f_findfirst(&dir, &finfo, filename, "*.acParams");
            if (fres == FR_OK && finfo.fname[0])
                file_found = true;
            // 关闭文件夹
            f_closedir(&dir);
        }
    }

    // 开始更新参数
    if (file_found)
    { // acParams后缀文件存在
        strcat(filename, "/");
        strcat(filename, finfo.fname);
        f_chmod(filename, 0, AM_RDO);
        if (!f_open(fil, filename, FA_OPEN_EXISTING | FA_READ))
        { // 文件打开成功
            res = UDPARAM_RS_FAIL;

            // 打开log文件
            bool logFilOpened = false;
            char log_filename[30];
            char logTxt[150];
            log_filename[0] = 0;
            strcat(log_filename, log_mountPath);
            strcat(log_filename, "ACFly/sysLog.txt");
            fres = f_open(logFil, log_filename, FA_OPEN_APPEND | FA_WRITE);
            if (fres == FR_OK)
            { // log打开成功
                logFilOpened = true;

                // 记录清理开始
                tm current_tm = Get_RTC_Time_tm();
                current_tm.tm_year += 1900;
                current_tm.tm_mon += 1;
                sprintf(logTxt, "\r\nParam Updater\r\n%d/%02d/%02d-%02d:%02d:%02d\r\n", current_tm.tm_year, current_tm.tm_mon, current_tm.tm_mday, current_tm.tm_hour, current_tm.tm_min,
                        current_tm.tm_sec);
                f_puts(logTxt, logFil);

                sprintf(logTxt, "\tParam file detected: %s\r\n", filename);
                f_puts(logTxt, logFil);
                f_sync(logFil);
            }

            uint16_t updatedParamsCount = 0;
            // 跳过文件头BOM
            uint8_t BOMlen = 0;
            while (1)
            {
                char txt;
                UINT bytes_read = 0;
                fres = f_read(fil, &txt, 1, &bytes_read);
                if (fres != FR_OK || bytes_read != 1)
                {
                    if (logFilOpened)
                    {
                        sprintf(logTxt, "\tParam file empty or open failed!\r\n");
                        f_puts(logTxt, logFil);
                        f_sync(logFil);
                    }
                    // 读取文件错误
                    goto READFIN;
                }

                if (txt > 0)
                { // 读取到字符
                    break;
                }
                else
                { // 读取到非字符
                    ++BOMlen;
                    if (BOMlen > 5)
                    {
                        if (logFilOpened)
                        {
                            sprintf(logTxt, "\tParam file header data check failed!\r\n");
                            f_puts(logTxt, logFil);
                            f_sync(logFil);
                        }
                        // 文件开头格式错误
                        goto READFIN;
                    }
                }
            }

            // 校验参数
            f_lseek(fil, BOMlen);
            if (runParamCheck(fil, false, logFilOpened, logFil, logTxt) == false)
            {
                if (logFilOpened)
                {
                    sprintf(logTxt, "Param update failed.\r\n");
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
                // 参数文件校验失败
                goto READFIN;
            }

            // 更新参数
            f_lseek(fil, BOMlen);
            if (runParamCheck(fil, true, logFilOpened, logFil, logTxt, &updatedParamsCount))
            {
                if (logFilOpened)
                {
                    sprintf(logTxt, "%d parameters have been updated.\r\nParam update success.\r\n", updatedParamsCount);
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
                res = UDPARAM_RS_SUCCESS;
            }
            else
            {
                if (logFilOpened)
                {
                    sprintf(logTxt,
                            "Param update failed during the writing process.\r\n%d "
                            "parameters have been updated.\r\n",
                            updatedParamsCount);
                    f_puts(logTxt, logFil);
                    sprintf(logTxt, "Please check your SD card and re-update parameters!\r\n");
                    f_puts(logTxt, logFil);
                    f_sync(logFil);
                }
            }

        READFIN:
            // 关闭文件
            f_close(fil);
            // 删除文件
            f_unlink(filename);
            // log
            if (logFilOpened)
                f_close(logFil);
        }
    }
    return res;
}

/*SD卡更新参数*/

// 文件系统
static FATFS log_fs;
static FATFS usr_fs;
// 文件
static FIL LogFile;
static FIL Txt1File;
static FIL UbxFile;
// log文件信息
static tm logFileCreateTime;
static uint16_t logFileDiffInd = 0;

static void SDLog_Task(void *pvParameters)
{
    Clear_SD_Init_Complete();
    /* f_mount 不受 FatFs FF_FS_REENTRANT 保护，重挂载时需串行化 */
    Lock_SDVolume(-1);
    FRESULT fres = f_mount(&log_fs, log_mountPath, 1);
    fres = f_mount(&usr_fs, usr_mountPath, 1);
    UnLock_SDVolume();

    // 首先执行参数更新
    while (getInitializationCompleted() == false)
        os_delay(0.1);
    if (updateParams(&LogFile, &Txt1File) == UDPARAM_RS_SUCCESS)
        sendLedSignal(LEDSignal_Success1);

reload_SD:
    Clear_SD_Init_Complete();
    // 重挂载后旧缓冲区中可能残留针对旧文件(被 PC 修改/删除/改名)的数据，
    // 必须清空，避免写入新文件造成数据错乱。
    xMessageBufferReset(LogStreamBuffer);
    xMessageBufferReset(Txt1LogStreamBuffer);
    xMessageBufferReset(UbxLogStreamBuffer);
    /* 重挂载时获取卷级锁，避免与 FTP 等其它任务的 FatFs 操作竞态 */
    /* FatFs 的 f_mount 不受 FF_FS_REENTRANT 保护 */
    Lock_SDVolume(-1);
    fres = f_mount(&log_fs, log_mountPath, 1);
    UnLock_SDVolume();
    if (fres == FR_OK)
    {
        Set_SD_Init_Complete();

        // 计算容量
        FATFS *fs;
        DWORD fre_clust;
        /* Get volume information and free clusters of drive 1 */
        fres = f_getfree((TCHAR const *) log_mountPath, &fre_clust, &fs);
        /* Get total sectors and free sectors */
        tot_size = (uint64_t) ((fs->n_fatent - 2) * fs->csize) * (uint64_t) 512;
        fre_size = (uint64_t) fre_clust * (uint64_t) fs->csize * (uint64_t) 512;
        totGB = (tot_size >> 20) / 1024.0f;
        freGB = (fre_size >> 20) / 1024.0f;

        // 文件（夹）名
        char filename[80];
        uint16_t filename_len;
        // 文件（夹）状态
        FILINFO finfo;
        // 写文件临时变量
        UINT length;

        // 读取最大日志文件大小参数
        uint32_t maxLogFilSize[2];
        maxLogFilSize[0] = 800;
        ReadParam("SDLog_MaxFilSiz", 0, 0, (uint64_t *) &maxLogFilSize, 0);
        maxLogFilSize[0] = constrain(maxLogFilSize[0], (uint32_t) 50, (uint32_t) 800);

        // 计算保留空间大小
        uint16_t reserve_space_mb = (tot_size >> 20) * 0.3;
        if (reserve_space_mb < maxLogFilSize[0] + 200)
            reserve_space_mb = maxLogFilSize[0] + 200;

        if (pvParameters)
        { // 已预先生成则直接使用传递来的文件名
            strcpy(filename, (char *) pvParameters);
            delete[] ((char *) pvParameters);
            pvParameters = 0;

            // 自动清理SD卡
            if (fre_size < (uint64_t) reserve_space_mb * 1024 * 1024)
            {
                // log目录（用独立变量，避免覆盖已保存的log文件路径）
                char cleanup_dir[32];
                cleanup_dir[0] = 0;
                strcat(cleanup_dir, log_mountPath);
                strcat(cleanup_dir, "ACFly/Log/");
                cleanup_logs(fs, cleanup_dir, (uint64_t) reserve_space_mb * 1024 * 1024, logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour,
                             logFileCreateTime.tm_min, logFileDiffInd, 0);
            }
        }
        else
        {
            // 创建ACFly目录
            filename[0] = 0;
            strcat(filename, log_mountPath);
            strcat(filename, "ACFly");
            fres = f_stat(filename, &finfo);
            if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
            { // 删除同名文件
                if ((finfo.fattrib & AM_RDO) != 0)
                    f_chmod(filename, 0, AM_RDO);
                f_unlink(filename);
            }
            fres = f_mkdir(filename);
            if (fres != FR_OK && fres != FR_EXIST)
                goto sdErr;

            //// 创建BootLoaderUpdate目录
            // filename[0] = 0;
            // strcat(filename, log_mountPath);
            // strcat(filename, "ACFly/BootLoaderUpdate");
            // fres = f_stat(filename, &finfo);
            // if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
            //{ // 删除同名文件
            //     if ((finfo.fattrib & AM_RDO) != 0)
            //         f_chmod(filename, 0, AM_RDO);
            //     f_unlink(filename);
            // }
            // fres = f_mkdir(filename);
            // if (fres != FR_OK && fres != FR_EXIST)
            //     goto sdErr;

            // 创建log目录
            filename[0] = 0;
            strcat(filename, log_mountPath);
            strcat(filename, "ACFly/Log");
            fres = f_stat(filename, &finfo);
            if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
            { // 删除同名文件
                if ((finfo.fattrib & AM_RDO) != 0)
                    f_chmod(filename, 0, AM_RDO);
                f_unlink(filename);
            }
            fres = f_mkdir(filename);
            if (fres != FR_OK && fres != FR_EXIST)
                goto sdErr;

            // 获取日期时间
            logFileCreateTime = Get_RTC_Time_tm();
            logFileCreateTime.tm_year += 1900;
            logFileCreateTime.tm_mon += 1;

            // 自动清理SD卡
            if (fre_size < (uint64_t) reserve_space_mb * 1024 * 1024)
            {
                // log目录（用独立变量，避免破坏filename中的"ACFly/Log"路径导致后续双斜杠）
                char cleanup_dir[32];
                cleanup_dir[0] = 0;
                strcat(cleanup_dir, log_mountPath);
                strcat(cleanup_dir, "ACFly/Log/");
                cleanup_logs(fs, cleanup_dir, (uint64_t) reserve_space_mb * 1024 * 1024, logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour,
                             logFileCreateTime.tm_min, logFileDiffInd, 0);
            }

            // 创建log日期目录
            filename_len = strlen(filename);
            sprintf(&filename[filename_len], "/%d%02d%02d", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday);
            fres = f_stat(filename, &finfo);
            if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
            { // 删除同名文件
                if ((finfo.fattrib & AM_RDO) != 0)
                    f_chmod(filename, 0, AM_RDO);
                f_unlink(filename);
            }
            fres = f_mkdir(filename);
            if (fres != FR_OK && fres != FR_EXIST)
                goto sdErr;

            // 创建Log文件
            filename_len = strlen(filename);
            sprintf(&filename[filename_len], "/%d%02d%02d%02d%02d.aclog", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour,
                    logFileCreateTime.tm_min);
            fres = f_stat(filename, &finfo);
            logFileDiffInd = 0;
            while (fres != FR_NO_FILE)
            { // 删除同名文件
                if (fres != FR_OK)
                    goto sdErr;
                sprintf(&filename[filename_len], "/%d%02d%02d%02d%02d(%d).aclog", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour,
                        logFileCreateTime.tm_min, ++logFileDiffInd);
                fres = f_stat(filename, &finfo);
            }

            fres = f_open(&LogFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
            // 写入数据头和版本信息
            buf[0] = 'A';
            buf[1] = 'C';
            buf[2] = 1;
            buf[3] = (1 << 1) | 1;
            f_write(&LogFile, buf, 4, &length);
            // 写描述字符
            sprintf(buf, "ACFlyK9 %d%02d%02d%02d%02d%02d", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour, logFileCreateTime.tm_min,
                    logFileCreateTime.tm_sec);
            f_write(&LogFile, buf, 26, &length);
            f_close(&LogFile);
        }
        // 打开log文件
        fres = f_open(&LogFile, filename, FA_OPEN_APPEND | FA_WRITE);
        uint16_t logFileIndex = 0;

        // Txt1文件名
        char Txt1Filename[80];
        strcpy(Txt1Filename, filename);
        filename_len = strlen(filename);
        sprintf(&Txt1Filename[filename_len - 6], "_1.txt");
        bool Txt1FileCreated = false;

        // Ubx文件名
        char UbxFilename[80];
        strcpy(UbxFilename, filename);
        filename_len = strlen(filename);
        sprintf(&UbxFilename[filename_len - 6], ".ubx");
        bool UbxFileCreated = false;

        // 只读状态
        bool diskReadOnly = true;
        for (uint8_t i = 0; i < get_MSC_DEV_DISKS_CNT(); ++i)
        {
            if (get_MSC_DEV_DISKS()[i] == DEV_SD_EXT)
            {
                diskReadOnly = get_MSC_DEV_DISKS_READONLY()[i];
                break;
            }
        }

        TIME last_cleanupTIME(false);
        bool last_usb_connected = false;
        bool logRenamed = false; // GPS 校准 RTC 后是否已把日志文件名从 1970 改成真实日期
        TickType_t xLastWakeTime = xTaskGetTickCount();
        TIME last_flush_TIME;
        bool SDLog_wait_to_sync = false;
        bool Txt1_wait_to_sync = false;
        bool Ubx_wait_to_sync = false;
        uint8_t sync_ind = 0;
        while (1)
        {
            bool usb_connected = get_is_usb_connected();

            /* PC改写过SD卡(msc写入)时必须最先处理: 重挂载后再进入改名/写入逻辑,
             * 否则改名块对PC已删除/改动的文件执行f_sync/f_close/f_rename,
             * 操作过期句柄可能对已释放簇写FAT表造成文件系统损坏 */
            if (usb_connected == false)
            {
                // 电脑修改过sd卡
                // 重新挂载
                if (mscExtWritten)
                {
                    mscExtWritten = false;
                    goto reload_SD;
                }
            }

            // GPS 校准 RTC 后，把开机时用未校准时间(1970)命名的日志文件原地改名为真实日期。
            // 仅在以下条件全部满足时执行一次:
            //   1. GPS 已校准 RTC;
            //   2. 姿态控制器未开启(电机未解锁)，避免飞行中动文件;
            //   3. USB 未连接(SD 卡未被 PC 占用);
            //   4. 本会话尚未执行过改名;
            //   5. 日志创建时间还是无效年份(reload_SD重挂载后新建的文件已是真实日期, 无需再改名,
            //      否则会把已是真实日期的文件再加(1)后缀重复改名)。
            // 改名只重命名三个文件,不创建新日志内容,后续写操作继续追加到改名后的文件。
            if (logRenamed == false && gps_rtcUpdated() && usb_connected == false && logFileCreateTime.tm_year < 2024)
            {
                bool att_ctrl_ena = true;
                is_Attitude_Control_Enabled(&att_ctrl_ena);
                if (att_ctrl_ena == false)
                {
                    // 在动任何文件之前先读取并校验新日期。
                    // Get_RTC_Time_tm 在 RTC 影子寄存器未同步时返回上一次
                    // 的有效时间, 仅在首次同步之前返回 tm_year=-1 哨兵值。
                    // 若日期不合法则跳过本周期(保持 logRenamed 为 false),
                    // 下一次循环再重试。
                    tm newCreateTime = Get_RTC_Time_tm();
                    newCreateTime.tm_year += 1900;
                    newCreateTime.tm_mon += 1;

                    bool date_valid = newCreateTime.tm_year >= 2024 && newCreateTime.tm_year <= 2100 && newCreateTime.tm_mon >= 1 && newCreateTime.tm_mon <= 12 && newCreateTime.tm_mday >= 1 &&
                                      newCreateTime.tm_mday <= 31;
                    if (date_valid)
                    {
                        // 先创建并校验目标目录: 磁盘错误等导致目录不可用时跳过本周期
                        // (不动任何文件句柄, 循环末尾正常执行延时), 保持logRenamed为false下周期重试;
                        // 避免半途失败导致LogFile句柄失效、本会话日志静默丢失
                        char newdir[80];
                        snprintf(newdir, sizeof(newdir), "%sACFly/Log/%d%02d%02d", log_mountPath, newCreateTime.tm_year, newCreateTime.tm_mon, newCreateTime.tm_mday);
                        f_mkdir(newdir); // 不存在则创建,已存在返回 FR_EXIST,都无碍
                        if (f_stat(newdir, &finfo) == FR_OK && (finfo.fattrib & AM_DIR))
                        {
                            // 关闭已打开的文件(若有)，确保 FAT 表更新
                            f_sync(&LogFile);
                            f_close(&LogFile);
                            if (Txt1FileCreated)
                            {
                                f_sync(&Txt1File);
                                f_close(&Txt1File);
                                Txt1FileCreated = false;
                            }
                            if (UbxFileCreated)
                            {
                                f_sync(&UbxFile);
                                f_close(&UbxFile);
                                UbxFileCreated = false;
                            }

                            // 生成 .aclog 目标文件名(避开同名)
                            char newpath[80];
                            uint16_t diffInd = 0;
                            snprintf(newpath, sizeof(newpath), "%s/%d%02d%02d%02d%02d.aclog", newdir, newCreateTime.tm_year, newCreateTime.tm_mon, newCreateTime.tm_mday, newCreateTime.tm_hour,
                                     newCreateTime.tm_min);
                            while (f_stat(newpath, &finfo) == FR_OK)
                                snprintf(newpath, sizeof(newpath), "%s/%d%02d%02d%02d%02d(%d).aclog", newdir, newCreateTime.tm_year, newCreateTime.tm_mon, newCreateTime.tm_mday, newCreateTime.tm_hour,
                                         newCreateTime.tm_min, ++diffInd);

                            // 重命名三个文件(主文件 + _1.txt + .ubx)。三个文件共用同一前缀。
                            char oldTxt[80];
                            char oldUbx[80];
                            strcpy(oldTxt, Txt1Filename);
                            strcpy(oldUbx, UbxFilename);

                            // 主日志改名
                            // 注: 主日志必存在(启动时已创建), 无需检查返回值
                            f_rename(filename, newpath);

                            // txt / ubx 改名 (替换后缀)
                            // 旧文件可能从未创建过(无该类数据), 此时f_rename返回FR_NO_FILE且无副作用,
                            // 不会创建文件; 检查返回值而非事后f_stat:
                            // 目标撞名(FR_EXIST, 避让循环只查了.aclog未查派生名)时f_stat会误开
                            // 别的会话的同名文件导致数据混淆, rename失败则一律按未创建处理
                            char newTxt[80];
                            char newUbx[80];
                            uint16_t nlen = strlen(newpath);
                            strcpy(newTxt, newpath);
                            strcpy(newUbx, newpath);
                            sprintf(&newTxt[nlen - 6], "_1.txt");
                            sprintf(&newUbx[nlen - 6], ".ubx");
                            bool txtRenamed = (f_rename(oldTxt, newTxt) == FR_OK);
                            bool ubxRenamed = (f_rename(oldUbx, newUbx) == FR_OK);

                            // 更新本会话使用的文件名和时间基准
                            strcpy(filename, newpath);
                            filename_len = strlen(filename);
                            strcpy(Txt1Filename, newTxt);
                            strcpy(UbxFilename, newUbx);
                            logFileCreateTime = newCreateTime;
                            // 必须直接采用新目录下的diffInd(而非取max):
                            // 改名后主文件新名字使用diffInd(0=无后缀), 后续分卷要与主文件同diff_ind
                            // 才能被cleanup_current_dir识别为同一系列受保护;
                            // 若保持旧值(如开机时1970目录避让得到的1), 主文件(0)与分卷(1)
                            // 会被判为不同系列, 正在写入的主文件失去删除保护
                            logFileDiffInd = diffInd;
                            logFileIndex = 0;

                            // 以追加模式重新打开主日志(不创建新内容)
                            f_open(&LogFile, filename, FA_OPEN_APPEND | FA_WRITE);
                            // Txt1/Ubx: 仅当改名成功(文件确实随主日志改到新名)才追加模式重开保持内容,
                            // 否则按未创建处理, 后续首次写入时再创建新文件;
                            // (rename成功保证文件存在, FA_OPEN_APPEND不会额外建文件;
                            // rename失败时不能开: 撞名场景会追加到别的会话文件, 源不存在场景无文件可开)
                            if (txtRenamed && f_open(&Txt1File, Txt1Filename, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
                                Txt1FileCreated = true;
                            if (ubxRenamed && f_open(&UbxFile, UbxFilename, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
                                UbxFileCreated = true;
                            SDLog_wait_to_sync = true;

                            logRenamed = true;
                        }
                    }
                }
            }

            if (usb_connected == false)
            {
                // 电脑修改过sd卡
                // 重新挂载(mscExtWritten已在循环顶部优先处理, 此处保留结构性兄余保护)
                if (mscExtWritten)
                {
                    mscExtWritten = false;
                    goto reload_SD;
                }
            }

            // if (sdCeanupCmd != -2)
            //{ // 清理SD卡
            //     if (diskReadOnly || usb_connected == false)
            //         cleanupOutdatedLogs(sdCeanupCmd);
            //     sdCeanupCmd = -2;
            // }

            if (diskReadOnly || usb_connected == false)
            {
                if (last_cleanupTIME.get_pass_time() > 10 * 60)
                {
                    last_cleanupTIME = TIME::now();

                    fres = f_getfree((TCHAR const *) log_mountPath, &fre_clust, &fs);
                    fre_size = (uint64_t) fre_clust * (uint64_t) fs->csize * (uint64_t) 512;
                    freGB = (float) fre_size / (1024 * 1024 * 1024);
                    if (fre_size < (uint64_t) (reserve_space_mb / 2) * 1024 * 1024)
                    {
                        // log目录
                        filename[0] = 0;
                        strcat(filename, log_mountPath);
                        strcat(filename, "ACFly/Log");
                        cleanup_logs(fs, filename, (uint64_t) (reserve_space_mb) * 1024 * 1024, logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday,
                                     logFileCreateTime.tm_hour, logFileCreateTime.tm_min, logFileDiffInd, logFileIndex);
                    }
                }

                /*写入Log*/
                // 从缓冲区获取要写入SD卡的数据
                uint32_t t_length = 0;
                while (BufSize > t_length + 2)
                {
                    buf[t_length + 0] = 'A';
                    buf[t_length + 1] = 'C';
                    length = xMessageBufferReceive(LogStreamBuffer, &buf[t_length + 2], BufSize - t_length - 2, 0);
                    if (length > 0)
                    {
                        t_length += 2 + length;
                    }
                    else
                        break;
                }

                // 数据写入文件
                if (t_length > 0)
                {
                    f_write(&LogFile, buf, t_length, &length);
                    update_SdSize(t_length, length);

                    if (f_size(&LogFile) > maxLogFilSize[0] * 1024 * 1024)
                    {
                        f_close(&LogFile);

                        // 创建Log文件
                        ++logFileIndex;
                        filename[0] = 0;
                        strcat(filename, log_mountPath);
                        strcat(filename, "ACFly/Log");
                        filename_len = strlen(filename);
                        sprintf(&filename[filename_len], "/%d%02d%02d", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday);
                        filename_len = strlen(filename);
                        if (logFileDiffInd == 0)
                        {
                            sprintf(&filename[filename_len], "/%d%02d%02d%02d%02d_%d.aclog", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour,
                                    logFileCreateTime.tm_min, logFileIndex);
                        }
                        else
                        {
                            sprintf(&filename[filename_len], "/%d%02d%02d%02d%02d(%d)_%d.aclog", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday,
                                    logFileCreateTime.tm_hour, logFileCreateTime.tm_min, logFileDiffInd, logFileIndex);
                        }
                        fres = f_stat(filename, &finfo);
                        // 删除同名文件
                        if (fres != FR_NO_FILE)
                            f_unlink(filename);
                        fres = f_stat(filename, &finfo);
                        uint16_t ind = 0;
                        while (fres != FR_NO_FILE)
                        {
                            if (fres != FR_OK)
                                goto sdErr;
                            sprintf(&filename[filename_len], "/%d%02d%02d%02d%02d(%d)_%d(%d).aclog", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday,
                                    logFileCreateTime.tm_hour, logFileCreateTime.tm_min, logFileDiffInd, logFileIndex, ++ind);
                            fres = f_stat(filename, &finfo);
                        }

                        fres = f_open(&LogFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
                        // 写入数据头和版本信息
                        buf[0] = 'A';
                        buf[1] = 'C';
                        buf[2] = 1;
                        buf[3] = (1 << 1) | 1;
                        f_write(&LogFile, buf, 4, &length);
                        // 写描述字符
                        sprintf(buf, "ACFlyK9 %d%02d%02d%02d%02d%02d", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour,
                                logFileCreateTime.tm_min, logFileCreateTime.tm_sec);
                        f_write(&LogFile, buf, 26, &length);

                        // 清理SD卡
                        last_cleanupTIME = TIME::now();
                        // log目录
                        filename[0] = 0;
                        strcat(filename, log_mountPath);
                        strcat(filename, "ACFly/Log/");
                        cleanup_logs(fs, filename, (uint64_t) (reserve_space_mb) * 1024 * 1024, logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday,
                                     logFileCreateTime.tm_hour, logFileCreateTime.tm_min, logFileDiffInd, logFileIndex);
                    }
                    SDLog_wait_to_sync = true;
                }
                /*写入Log*/

                /*写入Txt1*/
                // 从缓冲区获取要写入SD卡的数据
                t_length = 0;
                while (BufSize > t_length)
                {
                    length = xMessageBufferReceive(Txt1LogStreamBuffer, &buf[t_length], BufSize - t_length, 0);
                    if (length > 0)
                    {
                        t_length += length;
                    }
                    else
                        break;
                }

                // 数据写入文件
                if (t_length > 0)
                {
                    if (Txt1FileCreated == false)
                    { // 创建txt1文件
                        fres = f_open(&Txt1File, Txt1Filename, FA_CREATE_ALWAYS | FA_WRITE);
                        Txt1FileCreated = true;
                    }
                    f_write(&Txt1File, buf, t_length, &length);
                    update_SdSize(t_length, length);
                    Txt1_wait_to_sync = true;
                }
                /*写入Txt1*/

                /*写入Ubx*/
                // 从缓冲区获取要写入SD卡的数据
                t_length = 0;
                while (BufSize > t_length)
                {
                    length = xMessageBufferReceive(UbxLogStreamBuffer, &buf[t_length], BufSize - t_length, 0);
                    if (length > 0)
                    {
                        t_length += length;
                    }
                    else
                        break;
                }

                // 数据写入文件
                if (t_length > 0)
                {
                    if (UbxFileCreated == false)
                    { // 创建Ubx文件
                        fres = f_open(&UbxFile, UbxFilename, FA_CREATE_ALWAYS | FA_WRITE);
                        f_sync(&UbxFile);
                        UbxFileCreated = true;
                    }
                    fres = f_write(&UbxFile, buf, t_length, &length);
                    update_SdSize(t_length, length);
                    Ubx_wait_to_sync = true;
                }
                /*写入Ubx*/

                // 每隔一段时间刷新数据
                if (last_flush_TIME.get_pass_time() > 1)
                {
                    switch (sync_ind)
                    {
                        case 0:
                        {
                            if (SDLog_wait_to_sync)
                            {
                                f_sync(&LogFile);
                                SDLog_wait_to_sync = false;
                            }
                            break;
                        }

                        case 1:
                        {
                            if (Txt1FileCreated && Txt1_wait_to_sync)
                            {
                                f_sync(&Txt1File);
                                Txt1_wait_to_sync = false;
                            }
                            break;
                        }

                        default:
                        {
                            if (UbxFileCreated && Ubx_wait_to_sync)
                            {
                                f_sync(&UbxFile);
                                Ubx_wait_to_sync = false;
                            }
                            break;
                        }
                    }

                    if (++sync_ind >= 3)
                        sync_ind = 0;
                    last_flush_TIME = TIME::now();
                }
            }
            else
            { // usb已连接不写入数据
                // 清空缓冲区
                if (last_usb_connected != usb_connected)
                {
                    xMessageBufferReset(LogStreamBuffer);
                    xMessageBufferReset(Txt1LogStreamBuffer);
                    xMessageBufferReset(UbxLogStreamBuffer);
                }
            }
            last_usb_connected = usb_connected;
            vTaskDelayUntil(&xLastWakeTime, 1);
        }
    }
    else
    { // SD卡挂载失败
    sdErr:
        while (1)
        {
            bool usb_connected = get_is_usb_connected();
            if (usb_connected == false)
            {
                // 电脑修改过sd卡
                // 重新挂载
                if (mscExtWritten)
                {
                    mscExtWritten = false;
                    goto reload_SD;
                }
                else
                    os_delay(0.01);
            }
            else
                os_delay(0.01);
        }
    }
}

#define SDLOG_StackSize 4500
Static_AXIDMABuf StackType_t SDLOG_Stack[SDLOG_StackSize];
Static_AXIDMABuf StaticTask_t SDLOG_TaskBuffer;
void init_SDStorage()
{
    // 更新bootloader
    SD_BL_Update(&usr_fs, &log_fs, &LogFile, &Txt1File);

    // 挂载文件系统
    FRESULT fres = f_mount(&log_fs, log_mountPath, 1);
    f_mount(&usr_fs, usr_mountPath, 1);

    // 读取硬件配置INI并写入InternalStorage
    HardWareInit(&LogFile, &Txt1File);

    // 日志文件名称传递到线程
    char *logFileName = 0;
    // 挂载并创建日志目录
    if (fres == FR_OK)
    {
        // 文件（夹）名
        char filename[80];
        uint16_t filename_len;
        // 文件（夹）状态
        FILINFO finfo;
        // 写文件临时变量
        UINT length;

        // 创建ACFly目录
        filename[0] = 0;
        strcat(filename, usr_mountPath);
        strcat(filename, "ACFlySetup");
        fres = f_stat(filename, &finfo);
        if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
        { // 删除同名文件
            if ((finfo.fattrib & AM_RDO) != 0)
                f_chmod(filename, 0, AM_RDO);
            f_unlink(filename);
        }
        fres = f_mkdir(filename);
        //			if (fres != FR_OK && fres != FR_EXIST)
        //					goto sdErr;

        // 创建BootLoaderUpdate目录
        filename[0] = 0;
        strcat(filename, usr_mountPath);
        strcat(filename, "ACFlySetup/BootLoaderUpdate");
        fres = f_stat(filename, &finfo);
        if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
        { // 删除同名文件
            if ((finfo.fattrib & AM_RDO) != 0)
                f_chmod(filename, 0, AM_RDO);
            f_unlink(filename);
        }
        fres = f_mkdir(filename);
        //			if (fres != FR_OK && fres != FR_EXIST)
        //					goto sdErr;

        // 创建ACFly目录
        filename[0] = 0;
        strcat(filename, log_mountPath);
        strcat(filename, "ACFly");
        fres = f_stat(filename, &finfo);
        if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
        { // 删除同名文件
            if ((finfo.fattrib & AM_RDO) != 0)
                f_chmod(filename, 0, AM_RDO);
            f_unlink(filename);
        }
        fres = f_mkdir(filename);
        if (fres != FR_OK && fres != FR_EXIST)
            goto sdErr;

        //// 创建BootLoaderUpdate目录
        // filename[0] = 0;
        // strcat(filename, log_mountPath);
        // strcat(filename, "ACFly/BootLoaderUpdate");
        // fres = f_stat(filename, &finfo);
        // if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
        //{ // 删除同名文件
        //     if ((finfo.fattrib & AM_RDO) != 0)
        //         f_chmod(filename, 0, AM_RDO);
        //     f_unlink(filename);
        // }
        // fres = f_mkdir(filename);
        //  if (fres != FR_OK && fres != FR_EXIST)
        //      goto sdErr;

        // 创建log目录
        filename[0] = 0;
        strcat(filename, log_mountPath);
        strcat(filename, "ACFly/Log");
        fres = f_stat(filename, &finfo);
        if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
        { // 删除同名文件
            if ((finfo.fattrib & AM_RDO) != 0)
                f_chmod(filename, 0, AM_RDO);
            f_unlink(filename);
        }
        fres = f_mkdir(filename);
        if (fres != FR_OK && fres != FR_EXIST)
            goto sdErr;

        // 获取日期时间
        logFileCreateTime = Get_RTC_Time_tm();
        logFileCreateTime.tm_year += 1900;
        logFileCreateTime.tm_mon += 1;

        // 创建log日期目录
        filename_len = strlen(filename);
        sprintf(&filename[filename_len], "/%d%02d%02d", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday);
        fres = f_stat(filename, &finfo);
        if (fres != FR_NO_FILE && (finfo.fattrib & AM_DIR) == 0)
        { // 删除同名文件
            if ((finfo.fattrib & AM_RDO) != 0)
                f_chmod(filename, 0, AM_RDO);
            f_unlink(filename);
        }
        fres = f_mkdir(filename);
        if (fres != FR_OK && fres != FR_EXIST)
            goto sdErr;

        // 创建Log文件
        filename_len = strlen(filename);
        sprintf(&filename[filename_len], "/%d%02d%02d%02d%02d.aclog", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour,
                logFileCreateTime.tm_min);
        fres = f_stat(filename, &finfo);
        logFileDiffInd = 0;
        while (fres != FR_NO_FILE)
        { // 删除同名文件
            if (fres != FR_OK)
                goto sdErr;
            sprintf(&filename[filename_len], "/%d%02d%02d%02d%02d(%d).aclog", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour,
                    logFileCreateTime.tm_min, ++logFileDiffInd);
            fres = f_stat(filename, &finfo);
        }
        // 传递日志名称
        filename_len = strlen(filename);
        logFileName = new char[filename_len + 1];
        memcpy(logFileName, filename, filename_len);
        logFileName[filename_len] = 0;

        fres = f_open(&LogFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
        // 写入数据头和版本信息
        buf[0] = 'A';
        buf[1] = 'C';
        buf[2] = 1;
        buf[3] = (1 << 1) | 1;
        fres = f_write(&LogFile, buf, 4, &length);
        // 写描述字符
        sprintf(buf, "ACFlyK9 %d%02d%02d%02d%02d%02d", logFileCreateTime.tm_year, logFileCreateTime.tm_mon, logFileCreateTime.tm_mday, logFileCreateTime.tm_hour, logFileCreateTime.tm_min,
                logFileCreateTime.tm_sec);

        fres = f_write(&LogFile, buf, 26, &length);
        f_close(&LogFile);
    }
    else
    { // SD卡挂载失败
    }
sdErr:

    // 数据记录参数
    MAV_PARAM_TYPE param_types[] = {
        MAV_PARAM_TYPE_UINT32, // SD卡记录文件最大大小(Mb)

        MAV_PARAM_TYPE_UINT32, // 系统状态记录分频
        MAV_PARAM_TYPE_UINT32, // 姿态记录分频
        MAV_PARAM_TYPE_UINT32, // 位置速度记录分频
        MAV_PARAM_TYPE_UINT32, // 原始IMU记录分频
        MAV_PARAM_TYPE_UINT32, // 姿态控制器记录分频
        MAV_PARAM_TYPE_UINT32, // 位置控制器记录分频
        MAV_PARAM_TYPE_UINT32, // 位置传感器记录
        MAV_PARAM_TYPE_UINT32, // 接收机信号记录
        MAV_PARAM_TYPE_UINT32, // PPK记录 0-不记录 1-GPS 2-RTK
    };
    SName param_names[] = {
        "SDLog_MaxFilSiz", // SD卡记录文件最大大小(Mb)

        "SDLog_SysState",  // 系统状态记录分频
        "SDLog_Att",       // 姿态记录分频
        "SDLog_LocalNed",  // 位置速度记录分频
        "SDLog_RawIMU",    // 原始IMU记录分频
        "SDLog_AttCtrl",   // 姿态控制器记录分频
        "SDLog_PosCtrl",   // 位置控制器记录分频
        "SDLog_PosSensor", // 位置传感器记录
        "SDLog_Receiver",  // 接收机信号记录
        "SDLog_PPK",       // PPK记录 0-不记录 1-GPS 2-RTK
    };
    uint64_t initial_cfg[] = {
        500, // SD卡记录文件最大大小(Mb)

        20, // 系统状态记录分频
        5,  // 姿态记录分频
        5,  // 位置速度记录分频
        0,  // 原始IMU记录分频
        10, // 姿态控制器记录分频
        10, // 位置控制器记录分频
        1,  // 位置传感器记录
        1,  // 接收机信号记录
        0,  // PPK记录 0-不记录 1-GPS 2-RTK
    };
    InitialParametersInit((uint64_t *) &initial_cfg, param_types, param_names, sizeof(initial_cfg) / 8);
    ParamGroupRegister("SDLog", 2, sizeof(initial_cfg) / 8, param_types, param_names, (uint64_t *) &initial_cfg);

    // 创建日志记录线程
    xTaskCreateStatic(SDLog_Task, "SDLog", SDLOG_StackSize, logFileName, SysPriority_UserTask, SDLOG_Stack, &SDLOG_TaskBuffer);
}