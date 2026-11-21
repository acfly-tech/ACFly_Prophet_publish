#include "FlightMode_TrajectoryReplay.hpp"

#include "ControlSystem.hpp"
#include "FlightModes.hpp"
#include "FlightModule_RecordTrajectory.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "StorageSystem.hpp"
#include "basic.hpp"
#include "drv_SDMMC.hpp"
#include "followTarget.hpp"
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

FlightModeClass FlightMode_TrajectoryReplay::get_fmClass()
{
    if (step == 3)
        return FMC_SemiAutomatic;
    else
        return FMC_Automatic;
}

bool FlightMode_TrajectoryReplay::set_params(double params[], uint16_t params_cnt)
{
    if (params[0] <= 0)
        return false;
    if (!is_valid_replay_type((int) params[1]))
        return false;

    // 初始化轨迹复演记录编号
    tra_ind = params[0];
    // 初始化轨迹复演类型
    replay_type = static_cast<REPLAY_TYPE>((int) params[1]);
    // 初始化正反向
    if (params[2] == 1)
    { // 轨迹反向 偏航反向
        reverse = true;
        yaw_reverse = true;
    }
    else if (params[2] == 2)
    { // 轨迹反向 偏航正向
        reverse = true;
        yaw_reverse = false;
    }
    else
    {
        reverse = false;
        yaw_reverse = false;
    }

    return true;
}
// 反向复演调整起点经纬度
bool FlightMode_TrajectoryReplay::calc_reversePoint(FlightModule_RecordTrajectory::REC_HEADER &header)
{
    // 复位文件指针到第一点
    FRESULT fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER));
    // 经纬度转平面坐标系
    double x, y;
    uint8_t sInd;
    if (!get_localXY_fromOptimalGlobal(header.pos_x, header.pos_y, &x, &y, &sInd))
        return false;
    // 读取首个点坐标
    __AC_ALIGNED(8)
    FlightModule_RecordTrajectory::REC_FRAME first_frame;
    __AC_ALIGNED(8)
    FlightModule_RecordTrajectory::REC_FRAME frame;
    UINT br;
    if (f_read(&fil, &frame, sizeof(frame), &br) == FR_OK && br == sizeof(frame) && frame.header[0] == 'A' && frame.header[1] == 'C' &&
        frame.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(frame))
    {
        first_frame = frame;
    }
    else
        return false;
    // 读取最后一个点坐标
    fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
    if (fres == FR_OK && f_read(&fil, &frame, sizeof(frame), &br) == FR_OK && br == sizeof(frame) && frame.header[0] == 'A' && frame.header[1] == 'C' &&
        frame.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(frame))
    {
        // 计算最后一个点经纬度位置
        x += frame.pos[0] - first_frame.pos[0];
        y += frame.pos[1] - first_frame.pos[1];
        double new_pos_x, new_pos_y;
        if (!get_LatLon_fromGlobal(sInd, &new_pos_x, &new_pos_y))
            return false;
        header.pos_x = new_pos_x;
        header.pos_y = new_pos_y;
        header.alt += frame.pos[2] - first_frame.pos[2];

        return true;
    }
    else
        return false;
}

FlightModeCheckResult FlightMode_TrajectoryReplay::check(const Receiver &rc, const FlightModeFlag &flag, const FlightMode_CommonVars &comVars, const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg)
{
    bool inFlight;
    get_is_inFlight(&inFlight);

    if (!inFlight)
        return FMCR_Err_NotTakeOff;
    if (get_Position_MSStatus() != MS_Ready)
        return FMCR_Err_NoPositioning;

    return FMCR_OK;
}

FlightMode_TrajectoryReplay::FlightMode_TrajectoryReplay()
    : FlightMode_Base("TrajectoryReplay", AFunc_TrajectoryReplay, 0)
{
    // 复位记录编号
    tra_ind = 1;
    // 复位文件打开状态
    file_opened = false;
    // 复位状态机
    step = 0;
    // 复位未来加速度指针
    future_accs = 0;
}

FlightMode_TrajectoryReplay::~FlightMode_TrajectoryReplay()
{
    if (file_opened)
    {
        f_close(&fil);
    }
    if (future_accs)
        delete[] future_accs;
}

bool FlightMode_TrajectoryReplay::get_all_future_points(const vector3<double> &current_tVel)
{
    // 复位所有未来点
    for (uint8_t i = 0; i < future_points_cnt; ++i)
        future_accs[i].zero();

    // 先读取最远的点
    // 即当前时刻 + future_points_cnt个点
    UINT br;
    bool next_frame2_available = false;
    FlightModule_RecordTrajectory::REC_FRAME next_frame2;
    if (total_points >= checked_points + future_points_cnt)
    {
        if (f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - checked_points - future_points_cnt) * sizeof(FlightModule_RecordTrajectory::REC_FRAME)) == FR_OK &&
            f_read(&fil, &next_frame2, sizeof(next_frame2), &br) == FR_OK && br == sizeof(next_frame2) && next_frame2.header[0] == 'A' && next_frame2.header[1] == 'C' &&
            next_frame2.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(next_frame2))
        {
            next_frame2_available = true;
        }
    }
    // 读取当前时刻 + future_points_cnt - 1个点
    // 到 当前时刻 + 1个点
    for (uint8_t i = 1; i < future_points_cnt; ++i)
    {
        FlightModule_RecordTrajectory::REC_FRAME next_frame;
        if (total_points >= checked_points + future_points_cnt - i)
        {
            if ((next_frame2_available || f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) +
                                                            (total_points - checked_points - future_points_cnt + i) * sizeof(FlightModule_RecordTrajectory::REC_FRAME)) == FR_OK) &&
                f_read(&fil, &next_frame, sizeof(next_frame), &br) == FR_OK && br == sizeof(next_frame) && next_frame.header[0] == 'A' && next_frame.header[1] == 'C' &&
                next_frame.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(next_frame))
            {
                if (next_frame2_available)
                {
                    // 未来一帧速度
                    vector3<double> frame2_vel;
                    frame2_vel.x = -next_frame2.vel[0];
                    frame2_vel.y = -next_frame2.vel[1];
                    frame2_vel.z = -next_frame2.vel[2];

                    // 之前一帧速度
                    vector3<double> frame_vel;
                    frame_vel.x = -next_frame.vel[0];
                    frame_vel.y = -next_frame.vel[1];
                    frame_vel.z = -next_frame.vel[2];

                    // 计算未来加速度
                    vector3<double> future_acc = (frame2_vel - frame_vel) / rec_h;
                    future_accs[future_points_cnt - i] = future_acc;
                }
                next_frame2 = next_frame;
                next_frame2_available = true;
            }
            else if (next_frame2_available)
                return false;
        }
    }
    // 求 当前帧 到 当前帧+1个点 加速度
    if (next_frame2_available)
    {
        // 未来一帧速度
        vector3<double> frame2_vel;
        frame2_vel.x = -next_frame2.vel[0];
        frame2_vel.y = -next_frame2.vel[1];
        frame2_vel.z = -next_frame2.vel[2];

        // 计算未来加速度
        vector3<double> future_acc = (frame2_vel - current_tVel) / rec_h;
        future_accs[0] = future_acc;
    }
    return true;
}

vector3<double> FlightMode_TrajectoryReplay::calc_future_acc(double future_t)
{
    double id_f = (future_t + cur_h) / rec_h;
    uint8_t dn_id = floor(id_f);
    uint8_t up_id = ceil(id_f);
    if (up_id >= future_points_cnt)
        return vector3<double>();

    return future_accs[dn_id] + (future_accs[up_id] - future_accs[dn_id]) * (id_f - dn_id);
}

FlightModeResult FlightMode_TrajectoryReplay::main_func(const Receiver &rc, FlightModeFlag &flag, FlightMode_CommonVars &comVars, const ModeMsg *msg, FlightModeMsgResult *msgRs,
                                                        const ModeFuncCfg *cfg, const AvoidanceCfg *avCfg, double h)
{
    // 打开位置控制器
    Altitude_Control_Enable();
    Position_Control_Enable();

    // 获取控制器是否打开
    bool pos_ena;
    is_Position_Control_Enabled(&pos_ena);
    if (!pos_ena)
        return FMR_Err_NoPositioning;

    /* 持卷级锁保护整个 main_func 期间的所有 FatFs 文件操作
     * 防止 SDLog_Task reload_SD 的 f_mount 调用 ff_del_syncobj(vQueueDelete)
     * 删除正在使用的信号量导致 Use-After-Free 崩溃
     * main_func 单次执行很快（毫秒级），不会长时间阻塞 reload_SD */
    SDVolumeLockGuard volGuard(-1);
    if (!volGuard.isLocked())
        return FMR_Err_ParamInvalid;

    switch (step)
    {
        case 0:
        { // 读取文件头信息

            // 上报模式状态
            setCurrentFlyMode(AFunc_TrajectoryReplay);

            // 打开日志文件
            const char log_mountPath[4] = {DEV_SD_EXT + '0', ':', '/', 0};
            char filename[100];
            filename[0] = 0;
            strncat(filename, log_mountPath, sizeof(filename) - 1);
            snprintf(&filename[strlen(filename)], sizeof(filename) - strlen(filename) - 1, "ACFly/%d.acRecTra", tra_ind);
            FRESULT fres = f_open(&fil, filename, FA_READ);
            if (fres == FR_OK)
            { // 打开文件成功
                file_opened = true;

                __AC_ALIGNED(8)
                FlightModule_RecordTrajectory::REC_HEADER header;
                UINT br;
                if (f_read(&fil, &header, sizeof(header), &br) == FR_OK && br == sizeof(header) && header.frames_cnt)
                {
                    // 获取记录时间间隔
                    rec_h = header.gap_ms * 1e-3;
                    // 复位当前积分时间
                    cur_h = 0;
                    // 获取总点数
                    total_points = header.frames_cnt;
                    // 复位运行点数
                    checked_points = 0;
                    // 复位接近距离
                    approaching_s.zero();
                    // 复位偏航偏移
                    yaw_offset = 0;

                    // 反向求解器
                    if (reverse)
                    {
                        // 初始化反向求解器
                        reverse_solver[0].reset_params(getAttCtrlCfg()->TD4_P1[0], getAttCtrlCfg()->TD4_P2[0], getAttCtrlCfg()->TD4_P3[0], getAttCtrlCfg()->TD4_P4[0]);
                        reverse_solver[1].reset_params(getAttCtrlCfg()->TD4_P1[2], getAttCtrlCfg()->TD4_P2[2], getAttCtrlCfg()->TD4_P3[2], getAttCtrlCfg()->TD4_P4[2]);
                        reverse_solver[2].reset_params(getPosCtrlCfg()->Z_TD4P3[0], getPosCtrlCfg()->Z_TD4P4[0], getPosCtrlCfg()->Z_TD4P4[0], getPosCtrlCfg()->Z_TD4P4[0]);
                        if (reverse_solver[0].get_h() <= 0 || reverse_solver[1].get_h() <= 0 || reverse_solver[2].get_h() <= 0)
                            return FMR_Err_ParamInvalid;
                        // 求需要读取的未来点数
                        double max_h = reverse_solver[0].get_h();
                        if (reverse_solver[1].get_h() > max_h)
                            max_h = reverse_solver[1].get_h();
                        if (reverse_solver[2].get_h() > max_h)
                            max_h = reverse_solver[2].get_h();
                        future_points_cnt = ceil((max_h * REVERSE_SOLVER_M + 1.1 * rec_h) / rec_h) + 1;
                        // 初始化未来点
                        future_accs = new vector3<double>[future_points_cnt];
                        if (!future_accs)
                            return FMR_Err_ParamInvalid;
                        // 复位td4
                        acc_td4[0].reset();
                        acc_td4[1].reset();
                        acc_td4[2].reset();
                        // 复位反向求解器时间累积
                        reverse_solver_cur_h[0] = 0;
                        reverse_solver_cur_h[1] = 0;
                        reverse_solver_cur_h[2] = 0;
                        // 计算第一帧未来点
                        FlightModule_RecordTrajectory::REC_FRAME frame;
                        UINT br;
                        if (f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME)) == FR_OK &&
                            f_read(&fil, &frame, sizeof(frame), &br) == FR_OK && br == sizeof(frame) && frame.header[0] == 'A' && frame.header[1] == 'C' &&
                            frame.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(frame))
                        {
                            vector3<double> current_tVel;
                            current_tVel.x = -frame.vel[0];
                            current_tVel.y = -frame.vel[1];
                            current_tVel.z = -frame.vel[2];
                            get_all_future_points(current_tVel);
                        }
                        else
                            return FMR_Err_ParamInvalid;
                    }

                    switch (replay_type)
                    {
                        case REPLAY_TYPE::FROM_START_POINT:
                        { // 从记录起点开始

                            // 反向复演计算
                            if (reverse)
                            {
                                // 反向复演调整起点经纬度
                                if (!calc_reversePoint(header))
                                    return FMR_Err_ParamInvalid;

                                // 复位文件指针
                                fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
                                if (fres != FR_OK)
                                    return FMR_Err_ParamInvalid;
                            }

                            // 获取起点任务信息
                            init_NavCmdInf(&navInf);
                            nav_params[0] = -1;
                            nav_params[1] = MAV_DO_REPOSITION_FLAGS_ALTITUDE_MODE;
                            nav_params[3] = std::nan("");
                            nav_params[4] = header.pos_x;
                            nav_params[5] = header.pos_y;
                            nav_params[6] = header.alt * 0.01;

                            step = 10;
                            break;
                        }
                        case REPLAY_TYPE::APPROACHING_POINT:
                        {
                            // 反向复演计算
                            if (reverse)
                            {
                                // 反向复演调整起点经纬度
                                if (!calc_reversePoint(header))
                                    return FMR_Err_ParamInvalid;

                                // 复位文件指针
                                fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
                                if (fres != FR_OK)
                                    return FMR_Err_ParamInvalid;
                            }

                            // 计算到起始点位置偏移
                            vector3<double> OC;
                            nav_params[0] = -1;
                            nav_params[1] = 0;
                            nav_params[3] = std::nan("");
                            nav_params[4] = header.pos_x;
                            nav_params[5] = header.pos_y;
                            nav_params[6] = header.alt * 0.01;
                            NavCmd16_WAYPOINT_GetInfo(MAV_FRAME_GLOBAL_RELATIVE_ALT, nav_params, 0, &OC, 0);
                            // 计算总复演时长
                            double total_t = total_points * rec_h;
                            // 校验复演偏移时间是否足够
                            if (OC.get_square() > sq(approaching_speed * total_t * 0.6) || sq(OC.x) + sq(OC.y) > sq(100 * 100) || fabs(OC.z) > 30 * 100)
                                return FMR_Err_ParamInvalid;
                            approaching_s = OC;
                        }
                        case REPLAY_TYPE::FROM_CURRENT_POINT:
                        { // 从当前点开始

                            // 读入第一个点旋转航向
                            if (reverse)
                            {
                                fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
                                if (fres != FR_OK)
                                    return FMR_Err_ParamInvalid;
                            }
                            __AC_ALIGNED(8)
                            FlightModule_RecordTrajectory::REC_FRAME frame;
                            UINT br;
                            if (f_read(&fil, &frame, sizeof(frame), &br) == FR_OK && br == sizeof(frame) && frame.header[0] == 'A' && frame.header[1] == 'C' &&
                                frame.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(frame))
                            {
                                if (!yaw_reverse)
                                { // 偏航正向
                                    Attitude_Control_set_Target_Yaw(frame.yaw);
                                }
                                else
                                { // 偏航反向
                                    double tYaw = fmod(frame.yaw + Pi, 2 * Pi);
                                    if (tYaw > Pi)
                                        tYaw -= 2 * Pi;
                                    else if (tYaw < -Pi)
                                        tYaw += 2 * Pi;
                                    Attitude_Control_set_Target_Yaw(tYaw);
                                }
                                // 恢复文件指针
                                if (!reverse)
                                    fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER));
                                else
                                    fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
                                if (fres != FR_OK)
                                    return FMR_Err_ParamInvalid;
                                step = 99;
                            }
                            else
                                return FMR_Err_ParamInvalid;

                            step = 99;
                            break;
                        }
                        case REPLAY_TYPE::FROM_CURRENT_POINT_YAW:
                        { // 从当前点和偏航开始

                            // 读入第一个点旋转航向
                            if (reverse)
                            {
                                fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
                                if (fres != FR_OK)
                                    return FMR_Err_ParamInvalid;
                            }
                            __AC_ALIGNED(8)
                            FlightModule_RecordTrajectory::REC_FRAME frame;
                            UINT br;
                            if (f_read(&fil, &frame, sizeof(frame), &br) == FR_OK && br == sizeof(frame) && frame.header[0] == 'A' && frame.header[1] == 'C' &&
                                frame.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(frame))
                            {
                                Attitude_Control_set_YawLock();
                                // 计算偏航偏移
                                Quaternion quat;
                                get_AirframeY_quat(&quat);
                                yaw_offset = fmod(quat.getYaw() - frame.yaw, 2 * Pi);
                                if (yaw_offset > Pi)
                                    yaw_offset -= 2 * Pi;
                                else if (yaw_offset < -Pi)
                                    yaw_offset += 2 * Pi;
                                // 恢复文件指针
                                if (!reverse)
                                    fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER));
                                else
                                    fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
                                if (fres != FR_OK)
                                    return FMR_Err_ParamInvalid;
                                step = 99;
                            }
                            else
                                return FMR_Err_ParamInvalid;

                            step = 99;
                            break;
                        }
                    }

                    break;
                }
            }
            return FMR_Err_NoMission;
            break;
        }

        // 先飞到起始点
        case 10:
        { // 飞到起始点
            NAV_CMD_RS res = Process_NavCmd(MAV_CMD_DO_REPOSITION, 1.0 / h, MAV_FRAME_GLOBAL_RELATIVE_ALT, nav_params, &navInf, rc, flag, comVars, cfg, avCfg);
            if (NavCmdRs_SuccessOrFault(res))
            { // NavCmd执行完成
                if (NavCmdRs_Success(res))
                {
                    // 读入第一个点旋转航向
                    __AC_ALIGNED(8)
                    FlightModule_RecordTrajectory::REC_FRAME frame;
                    UINT br;
                    if (f_read(&fil, &frame, sizeof(frame), &br) == FR_OK && br == sizeof(frame) && frame.header[0] == 'A' && frame.header[1] == 'C' &&
                        frame.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(frame))
                    {
                        FRESULT fres;
                        if (!yaw_reverse)
                        { // 偏航正向
                            Attitude_Control_set_Target_Yaw(frame.yaw);
                        }
                        else
                        { // 偏航反向
                            double tYaw = fmod(frame.yaw + Pi, 2 * Pi);
                            if (tYaw > Pi)
                                tYaw -= 2 * Pi;
                            else if (tYaw < -Pi)
                                tYaw += 2 * Pi;
                            Attitude_Control_set_Target_Yaw(tYaw);
                        }
                        // 恢复文件指针
                        if (!reverse)
                            fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER));
                        else
                            fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
                        if (fres != FR_OK)
                            return FMR_Err_ParamInvalid;
                        step = 99;
                    }
                    else
                        return FMR_Err_ParamInvalid;
                }
                else
                    return FMR_Err_ParamInvalid;
            }
            break;
        }

        // 复演
        case 99:
        { // 等待偏航旋转完成
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                // 复位跟随
                followTPos.zero();
                followTVel.zero();
                followTAcc.zero();
                last_followTPos.zero();
                followYawOffset = 0;
                // 复位偏航
                Attitude_Control_set_YawLock();
                // 复位首个点判断
                first_point = true;
                // 初始化刹车点
                vector3<double> pos;
                get_Position_Ctrl(&pos);
                followPos = pos;
                // 初始化接近距离叠加值
                approaching_addon.zero();

                ++step;
            }
            break;
        }
        case 100:
        {
            // 期望加速度
            cur_h += h;
            if (cur_h >= rec_h)
            {
                cur_h -= rec_h;

                __AC_ALIGNED(8)
                FlightModule_RecordTrajectory::REC_FRAME frame;
                UINT br;
                if (f_read(&fil, &frame, sizeof(frame), &br) == FR_OK && br == sizeof(frame) && frame.header[0] == 'A' && frame.header[1] == 'C' &&
                    frame.checksum == FlightModule_RecordTrajectory::calc_frameCheckSum(frame))
                {
                    // 首个点处理
                    if (first_point)
                    {
                        first_point = false;

                        // 初始化传感器跟随原点
                        followOrigin.x = frame.pos[0];
                        followOrigin.y = frame.pos[1];
                        followOrigin.z = frame.pos[2];
                    }
                    ++checked_points;

                    // 更新轨迹期望位置
                    followTPos.x = frame.pos[0];
                    followTPos.y = frame.pos[1];
                    followTPos.z = frame.pos[2];
                    last_followTPos = followTPos;
                    followTPos -= followOrigin;

                    if (!reverse)
                    { // 正向轨迹复演

                        // 更新轨迹期望速度
                        followTVel.x = frame.vel[0];
                        followTVel.y = frame.vel[1];
                        followTVel.z = frame.vel[2];

                        // 更新轨迹期望加速度
                        followTAcc.x = frame.acc[0];
                        followTAcc.y = frame.acc[1];
                        followTAcc.z = frame.acc[2];
                    }
                    else
                    { // 反向轨迹复演

                        // 更新轨迹期望速度
                        followTVel.x = -frame.vel[0];
                        followTVel.y = -frame.vel[1];
                        followTVel.z = -frame.vel[2];

                        // 获取未来点
                        get_all_future_points(followTVel);
                    }

                    // 偏航
                    if (!yaw_reverse)
                    {
                        followYaw = frame.yaw;
                    }
                    else
                    {
                        double tYaw = fmod(frame.yaw + Pi, 2 * Pi);
                        if (tYaw > Pi)
                            tYaw -= 2 * Pi;
                        else if (tYaw < -Pi)
                            tYaw += 2 * Pi;
                        followYaw = tYaw;
                    }
                    followYaw += yaw_offset;
                    followYawRate = frame.yaw_rate;

                    // 反向复演需要调整文件指针
                    if (reverse)
                    {
                        if (total_points > checked_points)
                        {
                            FRESULT fres = f_lseek(&fil, sizeof(FlightModule_RecordTrajectory::REC_HEADER) + (total_points - checked_points - 1) * sizeof(FlightModule_RecordTrajectory::REC_FRAME));
                            if (fres != FR_OK)
                                return FMR_Err_ParamInvalid;
                        }
                        else
                            return FMR_OK_RqSwitchMode;
                    }
                }
                else
                {
                    f_close(&fil);
                    if (checked_points == total_points)
                        return FMR_OK_RqSwitchMode;
                    else
                        return FMR_Err_ParamInvalid;
                }
            }

            // 反向td4求解器预估输出加速度
            if (reverse)
            {
                for (uint8_t i = 0; i < 3; ++i)
                {
                    reverse_solver_cur_h[i] += h;
                    if (reverse_solver_cur_h[i] >= reverse_solver[i].get_h())
                    {
                        reverse_solver_cur_h[i] -= reverse_solver[i].get_h();

                        // 求解需要输出的加速度
                        double future_acc[REVERSE_SOLVER_M];
                        for (uint8_t k = 0; k < REVERSE_SOLVER_M; ++k)
                            future_acc[k] = calc_future_acc(reverse_solver[i].get_h() * (k + 1))[i];
                        double solved_acc = reverse_solver[i].compute_expect(acc_td4[i], future_acc);
                        reverse_solver[i].run_td4(acc_td4[i], solved_acc);
                        // 输出
                        followTAcc[i] = solved_acc;
                    }
                }
            }

            // 叠加接近距离
            double approaching_step = approaching_speed * h;
            if (approaching_s.get_square() > sq(approaching_step))
            {
                vector3<double> addon = approaching_s * (approaching_step / safe_sqrt(approaching_s.get_square()));
                approaching_addon += addon;
                approaching_s -= addon;
            }
            else
            {
                approaching_s.zero();
                approaching_addon += approaching_s;
            }

            // 计算传感器数据滞后的时间
            // 速度*时间 插值 平滑传感器期望位置
            followTPos.x = last_followTPos.x + followTVel.x * cur_h + approaching_addon.x;
            followTPos.y = last_followTPos.y + followTVel.y * cur_h + approaching_addon.y;
            followTPos.z = last_followTPos.z + followTVel.z * cur_h + approaching_addon.z;
            followTPos -= followOrigin;

            // 补偿航向偏移
            double sin_yaw_ofs, cos_yaw_ofs;
            fast_sin_cos(yaw_offset, &sin_yaw_ofs, &cos_yaw_ofs);
            double followTPos_rt_x, followTPos_rt_y;
            followTPos_rt_x = BodyHeading2ENU_x(followTPos.x, followTPos.y, sin_yaw_ofs, cos_yaw_ofs);
            followTPos_rt_y = BodyHeading2ENU_y(followTPos.x, followTPos.y, sin_yaw_ofs, cos_yaw_ofs);
            double followTVel_rt_x, followTVel_rt_y;
            followTVel_rt_x = BodyHeading2ENU_x(followTVel.x, followTVel.y, sin_yaw_ofs, cos_yaw_ofs);
            followTVel_rt_y = BodyHeading2ENU_y(followTVel.x, followTVel.y, sin_yaw_ofs, cos_yaw_ofs);
            double followTAcc_rt_x, followTAcc_rt_y;
            followTAcc_rt_x = BodyHeading2ENU_x(followTAcc.x, followTAcc.y, sin_yaw_ofs, cos_yaw_ofs);
            followTAcc_rt_y = BodyHeading2ENU_y(followTAcc.x, followTAcc.y, sin_yaw_ofs, cos_yaw_ofs);

            // 控制偏航
            double YCtrlScale = degree2rad(getAttCtrlCfg()->maxYSp[0] / 50.0);
            if (rc.available)
            {
                followYawOffset += remove_deadband(50.0 - rc.data[1], (double) cfg->NeutralZone[0]) * YCtrlScale * h;
                if (followYawOffset > Pi)
                    followYawOffset -= 2 * Pi;
                else if (followYawOffset < -Pi)
                    followYawOffset += 2 * Pi;
            }
            Attitude_Control_set_Target_Yaw_Offboard(followYaw + followYawOffset, followYawRate);

            // XY速度前馈
            vector3<double> followTV;
            followTV.x += followTVel_rt_x;
            followTV.y += followTVel_rt_y;
            // Z速度前馈
            followTV.z += followTVel.z;

            // 计算期望位置速度
            double thr_stick_d = 0;
            double roll_sitck_d = 0;
            double pitch_sitck_d = 0;
            if (rc.available)
            {
                thr_stick_d = remove_deadband(rc.data[0] - 50.0, (double) cfg->NeutralZone[0]);
                roll_sitck_d = remove_deadband(rc.data[3] - 50.0, (double) cfg->NeutralZone[0]);
                pitch_sitck_d = remove_deadband(rc.data[2] - 50.0, (double) cfg->NeutralZone[0]);
            }
            vector3<double> TVel;
            rcCtrl_Pos_p(roll_sitck_d, pitch_sitck_d, followPos.x, followPos.y, TVel.x, TVel.y, followTV.x, followTV.y, rc, flag, comVars, msg, msgRs, cfg, avCfg, h);
            rcCtrl_Alt_p(thr_stick_d, followPos.z, TVel.z, followTV.z, rc, flag, comVars, msg, msgRs, cfg, avCfg, h);

            vector3<double> TPos;
            TPos.x = followPos.x;
            TPos.y = followPos.y;
            TPos.z = followPos.z;

            // XY位置跟随
            TPos.x += followTPos_rt_x;
            TPos.y += followTPos_rt_y;
            // Z位置跟随
            TPos.z += followTPos.z;

            // 水平控制
            Position_Control_set_TargetPosVelAccXY_OffBoard(TPos.x, TPos.y, TVel.x, TVel.y, followTAcc_rt_x, followTAcc_rt_y);
            // 高度控制
            Position_Control_set_TargetPosVelAccZ_OffBoard(TPos.z, TVel.z, followTAcc.z);
            break;
        }
    }
    return FMR_OK;
}