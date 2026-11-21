#pragma once

#include "Basic.hpp"
#include "vector3.hpp"

// 跟随偏移信息
struct FollowInf
{
    /*跟随偏移坐标系
            0-前左上 通过yaw信息计算前向方向
            1-东北天
    */
    uint32_t ofsType[2];
    // 跟随偏移
    float ofs_x[2];
    float ofs_y[2];
    float ofs_z[2];
} __ALIGNED_PACKED;

enum followDataType
{
    // 经纬度全球坐标系
    followDataType_Gsv_xyz = 0,
    followDataType_Gsv_xy,
    followDataType_Gsv_z,

    followDataType_Gsva_xyz,
    followDataType_Gsva_xy,
    followDataType_Gsva_z,

    // 本地坐标系
    followDataType_sv_xyz,
    followDataType_sv_xy,
    followDataType_sv_z,

    followDataType_sva_xyz,
    followDataType_sva_xy,
    followDataType_sva_z,

    // 不成为返航目标

    // 经纬度全球坐标系
    followDataType_nRTL_Gsv_xyz = 50,
    followDataType_nRTL_Gsv_xy,
    followDataType_nRTL_Gsv_z,

    followDataType_nRTL_Gsva_xyz,
    followDataType_nRTL_Gsva_xy,
    followDataType_nRTL_Gsva_z,

    // 本地坐标系
    followDataType_nRTL_sv_xyz,
    followDataType_nRTL_sv_xy,
    followDataType_nRTL_sv_z,

    followDataType_nRTL_sva_xyz,
    followDataType_nRTL_sva_xy,
    followDataType_nRTL_sva_z,

    // 追踪目标(不会成为返航目标)
    // 控制上仅保持距离不保持相对位置

    // 经纬度全球坐标系
    followDataType_track_Gs_xy = 100,
    followDataType_track_Gs_xyz,

    // 本地坐标系
    followDataType_track_s_xy,
    followDataType_track_s_xyz,
};
#define isFollowDataType_GlobalXY(x)                                                                                                                                                                   \
    (x == followDataType_Gsv_xyz || x == followDataType_Gsv_xy || x == followDataType_Gsva_xyz || x == followDataType_Gsva_xy || x == followDataType_nRTL_Gsv_xyz ||                                   \
     x == followDataType_nRTL_Gsv_xy || x == followDataType_nRTL_Gsva_xyz || x == followDataType_nRTL_Gsva_xy || x == followDataType_track_Gs_xy || x == followDataType_track_Gs_xyz)
#define isFollowDataType_XY(x)                                                                                                                                                                         \
    (x == followDataType_Gsv_xyz || x == followDataType_Gsv_xy || x == followDataType_Gsva_xyz || x == followDataType_Gsva_xy || x == followDataType_sv_xyz || x == followDataType_sv_xy ||            \
     x == followDataType_sva_xyz || x == followDataType_sva_xy || x == followDataType_nRTL_Gsv_xyz || x == followDataType_nRTL_Gsv_xy || x == followDataType_nRTL_Gsva_xyz ||                          \
     x == followDataType_nRTL_Gsva_xy || x == followDataType_nRTL_sv_xyz || x == followDataType_nRTL_sv_xy || x == followDataType_nRTL_sva_xyz || x == followDataType_nRTL_sva_xy ||                   \
     x == followDataType_track_Gs_xy || x == followDataType_track_Gs_xyz || x == followDataType_track_s_xy || x == followDataType_track_s_xyz)
#define isFollowDataType_XYZ(x)                                                                                                                                                                        \
    (x == followDataType_Gsv_xyz || x == followDataType_sv_xyz || x == followDataType_Gsva_xyz || x == followDataType_sva_xyz || x == followDataType_nRTL_Gsv_xyz ||                                   \
     x == followDataType_nRTL_sv_xyz || x == followDataType_nRTL_Gsva_xyz || x == followDataType_nRTL_sva_xyz || x == followDataType_track_Gs_xyz || x == followDataType_track_s_xyz)
#define isFollowDataType_Z(x)                                                                                                                                                                          \
    (x == followDataType_Gsv_z || x == followDataType_sv_z || x == followDataType_Gsva_z || x == followDataType_sva_z || x == followDataType_nRTL_Gsv_z || x == followDataType_nRTL_sv_z ||            \
     x == followDataType_nRTL_Gsva_z || x == followDataType_nRTL_sva_z || x == followDataType_Gsv_xyz || x == followDataType_sv_xyz || x == followDataType_Gsva_xyz || x == followDataType_sva_xyz ||  \
     x == followDataType_nRTL_Gsv_xyz || x == followDataType_nRTL_sv_xyz || x == followDataType_nRTL_Gsva_xyz || x == followDataType_nRTL_sva_xyz)
#define isFollowDataType_HaveAcc(x)                                                                                                                                                                    \
    (x == followDataType_Gsva_xyz || x == followDataType_Gsva_xy || x == followDataType_Gsva_z || x == followDataType_nRTL_Gsva_xyz || x == followDataType_nRTL_Gsva_xy ||                             \
     x == followDataType_nRTL_Gsva_z || x == followDataType_sva_xyz || x == followDataType_sva_xy || x == followDataType_sva_z || x == followDataType_nRTL_sva_xyz ||                                  \
     x == followDataType_nRTL_sva_xy || x == followDataType_nRTL_sva_z)
#define isFollowDataType_Point(x) (x < 100)
#define isFollowDataType_Track(x) (x >= 100)
#define isFollowDataType_NeedRTL(x) (x < 50)

struct followSensor
{
    // 传感器可用
    bool available;
    // 经纬度是否可用
    bool globalXYPosAvailable;
    // 传感器类型
    followDataType dataType;

    // 跟随经纬度
    double lat, lon;
    // 跟随位置
    vector3<double> pos;
    // 跟随速度
    vector3<double> vel;
    // 跟随加速度
    vector3<double> acc;
    // 跟随偏航角
    double yaw;
    // 跟随偏航角速度
    double yawRate;
    // 传感器延迟
    double delay;
    // 获取偏航是否可用
    inline bool yawAvailable() const
    {
        return (yaw >= -2.0 * Pi && yaw <= 2.0 * Pi);
    }

    // 传感器更新时间
    TIME updateTime;

    // 位置补偿的半径
    double comp_r;
    // 位置补偿的偏航角偏置(在yaw基础上偏置)
    double comp_yaw;
    // 低电量是否需要跟随返航
    bool followInLpw;
};

/*
        注册精准降落传感器

        返回值：
        非0:添加成功
        0:添加失败（已有传感器或内存不足）
*/
uint32_t followSensorRegister(followDataType dataType, double TIMEOUT = -1);
/*
        取消注册精准降落传感器

        返回值：
        true:移除成功
        false:移除失败
*/
bool followSensorUnRegister(double TIMEOUT = -1);

/*
        获取精准降落传感器

        返回值
        0：失败
        >1：成功 每次注册传感器时自增
*/
uint16_t get_followSensor(followSensor *res_sensor, double TIMEOUT = -1);

/*
        更新精准降落传感器

        key: 注册时返回的密钥
        pos: 位置
        vel: 速度
        yaw: 偏航角 不可用给100
        available: 是否可用

        返回值：
        true:成功
        false:失败
*/
bool update_followSensor(uint32_t key, vector3<double> pos, vector3<double> vel, vector3<double> acc, double yaw, double yawRate, double comp_r, double comp_yaw, vector3<double> pos_offset,
                         bool available, bool followInLpw, double delay = 0, double TIMEOUT = -1);
/*
        失能精准降落传感器

        返回值：
        true:成功
        false:失败
*/
bool setInavailable_followSensor(uint32_t key, double TIMEOUT = -1);