#include "FlightModule_RecordTrajectory.hpp"

#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "StorageSystem.hpp"
#include "basic.hpp"
#include "drv_SDMMC.hpp"
#include "sd_diskio.h"

/* RAII 卷级锁守卫：构造时加锁，析构时解锁，保证所有 return 路径都正确解锁 */
class SDVolumeLockGuard
{
  private:
    bool locked;

  public:
    SDVolumeLockGuard(double TIMEOUT)
        : locked(false)
    {
        locked = Lock_SDVolume(TIMEOUT);
    }
    ~SDVolumeLockGuard()
    {
        if (locked)
            UnLock_SDVolume();
    }
    bool isLocked() const
    {
        return locked;
    }
};

FlightModule_RecordTrajectory::FlightModule_RecordTrajectory(uint8_t index, uint8_t precscale, double h)
{
    // 复位状态机
    step = 0;
    // 复位计数器
    counter = 0;

    // 复位文件打开状态
    file_opened = false;
    // 复位记录编号
    this->tra_ind = 1;
    // 复位分频系数
    this->precscale = 1;
    // 复位行数
    this->record_lines = 0;

    // 设置记录编号
    this->tra_ind = index;
    // 设置分频系数
    this->precscale = precscale;
    // 设置记录时间间隔
    this->h = h;
}

FlightModule_RecordTrajectory::~FlightModule_RecordTrajectory()
{
    stop_record();
}

// 计算帧校验
uint16_t FlightModule_RecordTrajectory::calc_frameCheckSum(const REC_FRAME &frame)
{
    uint16_t check_sum = 0;
    for (uint8_t i = 0; i < sizeof(REC_FRAME) - 4; i += 2)
    {
        check_sum += ((uint16_t *) &frame)[i / 2];
    }
    return check_sum;
}

bool FlightModule_RecordTrajectory::stop_record()
{
    /* 持卷级锁保护所有 f_write/f_lseek/f_close 操作，防止 reload_SD 的 f_mount 删除 sobj 崩溃 */
    SDVolumeLockGuard volGuard(-1);
    if (!volGuard.isLocked())
        return false;

    if (file_opened)
    {
        // 写入行数
        f_lseek(&fil, 12);
        UINT bw;
        f_write(&fil, &record_lines, 4, &bw);

        // 关闭文件
        f_close(&fil);
        step = 0;
        return true;
    }
    return false;
}

bool FlightModule_RecordTrajectory::record()
{
    const char log_mountPath[4] = {DEV_SD_EXT + '0', ':', '/', 0};

    /* 持卷级锁保护所有文件操作，防止 reload_SD 的 f_mount 删除 sobj 崩溃 */
    SDVolumeLockGuard volGuard(-1);
    if (!volGuard.isLocked())
        return false;

    // 获取是否已经起飞
    bool inFlight;
    get_is_inFlight(&inFlight);
    switch (step)
    {
        case 0:
        {
            // 未起飞不记录
            if (!inFlight)
                return false;

            // 获取当前经纬度
            double lat, lon;
            if (!get_LatLon_fromOptimalGlobal(&lat, &lon))
                return false;

            // 获取相对高度
            double homeZ = 0;
            getHomeLocalZ(&homeZ, 0, 0.01);
            vector3<double> position;
            get_Position_Ctrl(&position);
            double height = position.z - homeZ;

            // 打开日志文件
            char filename[100];
            filename[0] = 0;
            strncat(filename, log_mountPath, sizeof(filename) - 1);
            snprintf(&filename[strlen(filename)], sizeof(filename) - strlen(filename) - 1, "ACFly/%d.acRecTra", tra_ind);
            FRESULT fres = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
            if (fres == FR_OK)
            { // 打开文件成功
                file_opened = true;

                // 拼文件头
                __AC_ALIGNED(8) REC_HEADER header;
                header.header[0] = 'A';
                header.header[1] = 'C';
                header.header[2] = 'T';
                header.header[3] = 'r';
                header.header[4] = 'a';
                header.header[5] = 'R';
                header.header[6] = 'e';
                header.header[7] = 'c';
                // bit0-23:版本
                // bit24-27:水平原点定位方式 0=经纬度
                // bit28-31:高度原点定位方式 0=相对高 1=海拔高
                header.ver = (0 << 28) | (0 << 24) | 1;
                // 总航点条目数
                header.frames_cnt = 0;
                // 记录时间间隔ms
                header.gap_ms = h * 1000;
                // 保留1
                header.rsv1 = 0;
                // 水平位置
                header.pos_x = lat;
                header.pos_y = lon;
                // 高度
                header.alt = height;

                // 写入文件头
                UINT bw;
                f_write(&fil, &header, sizeof(REC_HEADER), &bw);

                ++step;
            }
            else
                return false;
            break;
        }
        case 1:
        {
            // 未起飞不记录
            if (!inFlight)
            {
                stop_record();
                return false;
            }
            if (++counter >= precscale)
            {
                counter = 0;

                // 获取主机位置
                vector3<double> pos;
                get_Position_Ctrl(&pos);
                // 获取主机速度
                vector3<double> vel;
                get_VelocityENU_Ctrl(&vel);
                // 获取主机输出加速度
                vector3<double> tAccOut;
                get_TargetAccOut(&tAccOut);
                // 获取航向角
                double yaw;
                Quaternion quat;
                get_AirframeY_quat(&quat);
                yaw = quat.getYaw();
                // 获取航向角速度
                vector3<double> angRate;
                get_AngularRate_Ctrl(&angRate);

                // 组包
                __AC_ALIGNED(8) REC_FRAME frame;
                frame.pos[0] = pos.x;
                frame.pos[1] = pos.y;
                frame.pos[2] = pos.z;
                frame.vel[0] = vel.x;
                frame.vel[1] = vel.y;
                frame.vel[2] = vel.z;
                frame.acc[0] = tAccOut.x;
                frame.acc[1] = tAccOut.y;
                frame.acc[2] = tAccOut.z;
                frame.yaw = yaw;
                frame.yaw_rate = angRate.z;

                // 校验
                frame.header[0] = 'A';
                frame.header[1] = 'C';
                frame.checksum = calc_frameCheckSum(frame);

                // 写入文件
                UINT bw;
                FRESULT fres = f_write(&fil, &frame, sizeof(REC_FRAME), &bw);
                if (fres != FR_OK)
                {
                    stop_record();
                    return false;
                }
                ++record_lines;
            }
            break;
        }
    }

    return true;
}