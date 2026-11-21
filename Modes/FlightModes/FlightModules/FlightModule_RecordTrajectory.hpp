#pragma once

#include "FlightModes.hpp"

#include "ff.h"

class FlightModule_RecordTrajectory
{
  private:
    // 文件
    FIL fil;
    bool file_opened;
    // 记录编号
    uint8_t tra_ind;
    // 分频系数
    uint8_t precscale;
    // 文件行数
    uint32_t record_lines;
    // 记录时间间隔s
    double h;

    // 状态机
    uint8_t step;
    // 计数器
    uint32_t counter;

  public:
    // 帧头
    struct REC_HEADER
    {
        // ACTraRec
        uint8_t header[8];
        // bit0-23:版本
        // bit24-27:水平原点定位方式 0=经纬度
        // bit28-31:高度原点定位方式 0=相对高 1=海拔高
        uint32_t ver;
        // 总航点条目数
        uint32_t frames_cnt;
        // 记录时间间隔ms
        uint32_t gap_ms;
        // 保留1
        uint32_t rsv1;
        // 水平位置
        double pos_x;
        double pos_y;
        // 高度
        double alt;
    } __AC_PACKED;
    // 记录帧
    struct REC_FRAME
    {
        double pos[3];
        float vel[3];
        float acc[3];
        float yaw;
        float yaw_rate;

        // AC
        uint8_t header[2];
        // 校验和
        uint16_t checksum;
    } __AC_PACKED;

    // 计算帧校验
    static uint16_t calc_frameCheckSum(const REC_FRAME &frame);

    /**
     * @brief 记录飞行轨迹
     * @param index 序号
     * @param precscale 分频系数
     * @param h 记录时间间隔s
     */
    FlightModule_RecordTrajectory(uint8_t index, uint8_t precscale, double h);

    ~FlightModule_RecordTrajectory();

    // 模式主函数
    bool record();

    bool stop_record();
};