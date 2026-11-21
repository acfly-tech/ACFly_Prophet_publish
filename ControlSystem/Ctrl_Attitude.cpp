#include "ctrl_Attitude.hpp"
#include "ControlSystem.hpp"
#include "ESO_AngularRate.hpp"
#include "ESO_h.hpp"
#include "Filters_LP.hpp"
#include "HardWareConfig.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "TD3_3D.hpp"
#include "TD4.hpp"
#include "ctrl_Main.hpp"
#include "drv_ADC.hpp"
#include "drv_PWMOut.hpp"

#include "Parachute.hpp"
#include "StorageSystem.hpp"

/*参数*/
// 控制方式定义
struct UAVCtrlM
{
    // 初始化
    void (*Init)();
    // 上锁取消初始化
    void (*DeInit)();
    // 解锁前动作
    void (*PreArmControl)(Receiver rc);
    // 初始化动作
    void (*InitControl)(bool DbgSafe);
    // 动力分配
    void (*MotorControl)(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe);
    // eso类型 0-二阶 1-一阶(Heli)
    int8_t RP_eso_type;
    int8_t Y_eso_type;
    // 控制器类型 0-二阶推力导数控制 1-期望加速度P减扰动 2-期望加速度PD减扰动
    int8_t RP_ctrl_type;
    int8_t Y_ctrl_type;
    // 是否在初始化时执行控制程序
    bool ctrl_on_init;
    // 是否在离地（起飞）前不添加扰动
    bool RP_NoDisturbanceOG;
};
static bool CtrlM_available = false;
static UAVCtrlM CtrlM;
extern const UAVCtrlM *UAVCtrlMs[];

// 控制参数
// 缓冲区布局: [AttCtrlCfg 固定部分 (含开头8字节内参)] + [st_count*6 个 float（每组: min[2],mid[2],max[2]）]
static uint8_t g_att_st_count = 0;
static uint8_t *g_att_cfg_buf = nullptr;
#define cfg (*(AttCtrlCfg *) g_att_cfg_buf)

// 参数注册基址：跳过开头内参区（minThrottlePct + idleThrottlePct），从第一个注册参数开始
// 若内参字段有增减，只需在此处修改 offsetof 参数即可
static inline uint64_t *getAttCfgParamBase()
{
    return (uint64_t *) (g_att_cfg_buf + offsetof(AttCtrlCfg, UAVType));
}

// 舵机参数访问（按索引，直接返回 float 值）
static inline float *_stPtr(uint8_t i)
{
    return (float *) (g_att_cfg_buf + sizeof(AttCtrlCfg)) + (uint32_t) i * 6;
}
static inline float getStMin(uint8_t i)
{
    return _stPtr(i)[0];
}
static inline float getStMid(uint8_t i)
{
    return _stPtr(i)[2];
}
static inline float getStMax(uint8_t i)
{
    return _stPtr(i)[4];
}
float getAttCtrlStMin(uint8_t idx)
{
    return getStMin(idx);
}
float getAttCtrlStMid(uint8_t idx)
{
    return getStMid(idx);
}
float getAttCtrlStMax(uint8_t idx)
{
    return getStMax(idx);
}
uint8_t getAttCtrlStCount()
{
    return g_att_st_count;
}

// 电池掉压估计
struct VoltKpLS
{
  public:
    double sumTV;
    double sumT;
    double sumV;
    double sumT2;
    uint32_t n;
    double max_thr, min_thr;

    inline void reset()
    {
        n = sumTV = sumT = sumV = sumT2 = 0;
    }
    inline void doIntegral(double thr, double volt)
    {
        if (n == 0)
        {
            min_thr = max_thr = thr;
        }
        else
        {
            if (thr < min_thr)
                min_thr = thr;
            else if (thr > max_thr)
                max_thr = thr;
        }
        sumT += thr;
        sumV += volt;
        sumTV += thr * volt;
        sumT2 += sq(thr);
        ++n;
    }
    inline double calcKp()
    {
        if (n < 20)
            return 0;
        double res = n * sumT2 - sq(sumT);
        if (fabs(res) < 0.001)
            return 0;
        res = (n * sumTV - sumT * sumV) / res;
        if (res > -0.00001)
            return 0;
        return res;
    }
};
// 电池信息
static int8_t batId = -1;
static float batStVolt = 0;
static VoltKpLS thrVoltKpLS;
static VoltKpLS curVoltKpLS;
static double thrVoltKp = 0;
static double curVoltKp = 0;
/*参数*/

// 姿态控制器开关
static bool Attitude_Control_Enabled = false;
// 飞行状态（已真正离地）
static bool inFlight = false;

/*内部接口*/
const AttCtrlCfg *getAttCtrlCfg()
{
    return (const AttCtrlCfg *) g_att_cfg_buf;
}

float getVoltKp()
{
    return thrVoltKp;
}
int8_t getCtrlBatId()
{
    return batId;
}

/*内部接口*/

/*起飞地点*/
static bool HomeLatLonAvailable;
static bool HomeAvailable;
// 地面站(Msg243)手动设置返航点后置位：挂起地面对
// HomeLatLon/HomePoint 的自动重采样，直到下次解锁
static bool HomeManualSet = false;
static vector2<double> HomeLatLon;
static vector2<double> HomePoint;
static double HomeLocalZ = 0;
static double HomeYaw = 0;
struct SENSOR_HOME_Z
{
    float trust;
    double posz;
};
static SENSOR_HOME_Z homeSensorsZ[Position_Sensors_Count] = {0};
bool getHomeLocalZ(double *home, double *home_yaw, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (home)
            *home = HomeLocalZ;
        if (home_yaw)
            *home_yaw = HomeYaw;
        UnlockCtrl();
        return true;
    }
    return false;
}

bool getHomeOptSensorZ(double *z, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        float optTrust = -1;
        for (uint8_t i = 0; i < Position_Sensors_Count; ++i)
        {
            if (homeSensorsZ[i].trust >= 0)
            {
                Position_Sensor_Data sensor;
                if (GetPositionSensorData(i, &sensor) && sensor.available &&
                    (sensor.sensor_type == Position_Sensor_Type_RelativePositioning || sensor.sensor_type == Position_Sensor_Type_GlobalPositioning) &&
                    (sensor.sensor_DataType == Position_Sensor_DataType_s_z || sensor.sensor_DataType == Position_Sensor_DataType_s_xyz || sensor.sensor_DataType == Position_Sensor_DataType_sv_z ||
                     sensor.sensor_DataType == Position_Sensor_DataType_sv_xyz || sensor.sensor_DataType == Position_Sensor_DataType_s_z_nAC ||
                     sensor.sensor_DataType == Position_Sensor_DataType_s_xyz_nAC || sensor.sensor_DataType == Position_Sensor_DataType_sv_z_nAC ||
                     sensor.sensor_DataType == Position_Sensor_DataType_sv_xyz_nAC))
                {
                    float trust = sensor.z_LTtrustD < homeSensorsZ[i].trust ? sensor.z_LTtrustD : homeSensorsZ[i].trust;
                    if (optTrust < 0 || trust < optTrust)
                    {
                        optTrust = trust;
                        *z = sensor.position.z - homeSensorsZ[i].posz;
                    }
                }
            }
        }

        UnlockCtrl();
        if (optTrust >= 0)
            return true;
        else
            return false;
    }
    return false;
}

bool getHomePoint(vector2<double> *home, double *home_yaw, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        bool available = HomeAvailable;
        if (available)
        {
            if (home)
                *home = HomePoint;
        }
        if (home_yaw)
            *home_yaw = HomeYaw;
        UnlockCtrl();
        if (available)
            return true;
        else
            return false;
    }
    return false;
}
bool getHomeLatLon(vector2<double> *home, double *home_yaw, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        bool available = HomeLatLonAvailable;
        if (available)
        {
            if (home)
                *home = HomeLatLon;
        }
        if (home_yaw)
            *home_yaw = HomeYaw;
        UnlockCtrl();
        if (available)
            return true;
        else
            return false;
    }
    return false;
}
bool setHomeLatLon(const vector2<double> *home, const double *home_yaw, double TIMEOUT)
{
    // 加锁前校验经纬度范围（NaN 比较结果为 false，可一并拒绝）
    if (home && !(fabs(home->x) < 90 && fabs(home->y) < 180))
        return false;

    // 在 Ctrl 锁外推导本地 X/Y：测量系统接口内部
    // 自带锁，避免与 LockCtrl 嵌套持锁
    bool xy_ok = false;
    double lx = 0, ly = 0;
    if (home)
        xy_ok = get_localXY_fromOptimalGlobal(home->x, home->y, &lx, &ly);

    if (LockCtrl(TIMEOUT))
    {
        if (home)
        {
            HomeLatLon = *home;
            HomeLatLonAvailable = true;
            // 挂起地面自动重采样，防止本设置在下一个
            // 控制周期被 update_output_throttle() 覆盖
            // （飞行中无自动重采样，此标志不影响飞行中设置；
            //   降落下锁后保持手动返航点，下次解锁恢复自动）
            HomeManualSet = true;
            if (xy_ok)
            {
                // 本地坐标与新的经纬度保持对齐，保证 Msg242 的
                // 本地字段与经纬度字段一致
                HomePoint.x = lx;
                HomePoint.y = ly;
                HomeAvailable = true;
            }
        }
        if (home_yaw)
            HomeYaw = *home_yaw;
        UnlockCtrl();
        return true;
    }
    return false;
}
/*起飞地点*/

/*状态观测器*/
// 姿态ESO
//	Static_DTCMBuf ESO_AngularRate ESO[3];
//	Static_DTCMBuf ESO_AngularRateHeli ESOHeli[3];
Static_DTCMBuf ESO_AngularRate_Base *CtrlM_ESO[3] = {0};
// 高度ESO
static double throttle_u = 0;
static double outputThrottle = 0;
static ESO_h ESO_height;
static double hover_throttle = 0;
static double WindDisturbance_x = 0;
static double WindDisturbance_y = 0;
static bool pre_inFlightRP = false;
static bool pre_inFlightY = false;
static Filter_Butter4_LP AccZ_filter;
static Filter_Butter2_LP forceVect_filter[3];
static Filter_Butter2_LP leanCosin_filter;
static TD4_Lite WindDisturbance_filter[2];
// 侧翻检测
static bool DebugSafeEna = false;
static uint32_t crash_counter = 0;
static double AC_angle_error = 0;
static double AC_rate_error = 0;
static bool crashed = false;
static inline void update_output_throttle(double throttle, double h)
{
    Quaternion quat;
    get_AirframeY_quat(&quat, 0.1);
    double lean_cosin = quat.get_lean_angle_cosin();

    // 计算推力向量(z轴)
    vector3<double> forceVect = quat.rotate_axis_z();
    vector3<double> forceVect_filted;
    forceVect_filted.x = forceVect_filter[0].run(forceVect.x);
    forceVect_filted.y = forceVect_filter[1].run(forceVect.y);
    forceVect_filted.z = forceVect_filter[2].run(forceVect.z);
    double leanCosin_filted = leanCosin_filter.run(lean_cosin);

    // 加速度滤波
    vector3<double> AccENU;
    get_AccelerationENU_Ctrl(&AccENU);
    double AccZ = AccZ_filter.run(AccENU.z);

    // 观测悬停油门
    outputThrottle = throttle;
    double r_throttle = throttle;
    if (r_throttle < cfg.minThrottlePct)
        r_throttle = cfg.minThrottlePct;
    // r_throttle *= lean_cosin;
    throttle_u = r_throttle;
    hover_throttle = ESO_height.get_hover_throttle();

    // 更新电池内阻估计
    float volt = 0;
    int8_t cBatId = -1;
    getCurrentBatteryTotalVoltRaw(&volt, &cBatId);
    if (inFlight == false && throttle > 2 && cBatId >= 0 && cBatId == batId)
    {
        thrVoltKpLS.doIntegral(throttle, volt);
    }
    else
    {
        thrVoltKpLS.reset();
        curVoltKpLS.reset();
    }

    // 更新飞行状态
    static uint16_t onGround_counter = 0;
    if (inFlight == false)
    {
        onGround_counter = 0;

        if (throttle > 2.5)
        {
            if (Attitude_Control_Enabled)
            { // 判断预起飞
                // 飞机倾斜则判断为预起飞马上启动扰动控制器
                if (sq(CtrlM_ESO[0]->get_EsAngularRate()) + sq(CtrlM_ESO[1]->get_EsAngularRate()) > sq(degree2rad(10.0)))
                    pre_inFlightRP = true;
                if (fabs(CtrlM_ESO[2]->get_EsAngularRate()) > degree2rad(10.0))
                    pre_inFlightY = true;
            }
            if (AccZ > 20)
            { // 判断起飞
                thrVoltKp = thrVoltKpLS.calcKp();
                pre_inFlightRP = pre_inFlightY = true;
                inFlight = true;
            }
        }
    }
    else
    {
        if (((hover_throttle < 2)) && leanCosin_filted > 0 && fabs(AccZ) < 35)
        {
            if (++onGround_counter >= 1.0 * CtrlRateHz)
                inFlight = pre_inFlightRP = pre_inFlightY = false;
        }
        else
            onGround_counter = 0;
    }

    // 侧翻保护
    if (inFlight && DebugSafeEna == false && AC_angle_error > degree2rad(30.0) && AC_rate_error > 0.3 && throttle > 5)
    {
        // 判断降落伞是否存在（在线即视为存在，未就绪也允许触发：侧翻坠机时开伞利大于弊）
        bool has_parachute = Parachute_isPresent(0);

        // 倾斜角（rad）：由天向余弦换算，用于可读的侧翻判定（60° = cosin 0.5）
        double leanAngle = acos(constrain(leanCosin_filted, -1.0, 1.0));

        bool flip_condition;
        if (has_parachute)
        { // 有伞：倾斜角超过60° 且姿态误差大 → 侧翻失控，尽快开伞（越早开伞救援效果越好）
            flip_condition = (leanAngle > degree2rad(60.0)) && (AC_angle_error > degree2rad(30.0));
        }
        else
        { // 无伞：需确认已彻底坠落才停桨
            // 倾斜角超过60° 且机体角速度已衰减（< 30°/s，原始陀螺仪滤波值）
            // 角速度仍很大说明飞机还在空中翻滚，此时停桨会导致高空坠落，必须等待机体静止（已触地）
            vector3<double> angular_rate;
            get_AngularRateNC_filted(&angular_rate);
            double rate_norm = safe_sqrt(angular_rate.get_square());
            bool has_settled = (rate_norm < degree2rad(30.0));

            flip_condition = (leanAngle > degree2rad(45.0)) && (AC_angle_error > degree2rad(30.0)) && has_settled;
        }

        // 侧翻持续时间阈值：
        // 有伞：1秒（开伞可救援机体，宽松判定优先保障开伞时机）
        // 无伞：3秒（侧翻静止持续3秒，确认彻底坠落后再停桨）
        uint32_t crash_time_thr = has_parachute ? (CtrlRateHz * 1) : (CtrlRateHz * 3);

        if (flip_condition)
        {
            if (++crash_counter >= crash_time_thr)
            {
                crash_counter = crash_time_thr;
                crashed = true;
            }
        }
        else if (crashed == false)
        {
            // 条件不满足且尚未判定坠机：计数缓慢衰减（避免临界抖动反复清零延迟判定）
            if (crash_counter > 0)
                --crash_counter;
        }
    }
    else
    {
        crashed = false;
        crash_counter = 0;
    }

    // 观测水平分力
    if (inFlight)
    {
        vector3<double> active_force_xy_vec = forceVect_filted;
        double temp_leanCosin_filted = leanCosin_filted;
        if (temp_leanCosin_filted < 0.3f)
            temp_leanCosin_filted = 0.3f;
        active_force_xy_vec = active_force_xy_vec * ((AccENU.z + GravityAcc) / temp_leanCosin_filted);
        vector3<double> WindDisturbance_xy;
        WindDisturbance_xy.x = AccENU.x - active_force_xy_vec.x;
        WindDisturbance_xy.y = AccENU.y - active_force_xy_vec.y;

        // 剔除气动阻力项（F_drag = -k*|v|*v）：原始观测=风力+机体气动阻力，
        // 高速飞行时阻力项主导，会导致刹车停稳后估计值残留反向错误前馈（回退现象）。
        // 无空速传感器，用地速近似相对空速（大风天残差 k*|v|*|vw| 为一阶小量）。
        // windDragK 为 SI 系数（阻力加速度=k*|v|m/s*v，m/s²），速度/加速度内部单位为 cm/s，
        // 故换算：a[cm/s²]=k*|v|*v*0.01（|v|、v 取 cm/s 数值）
        vector2<double> windDrag;
        {
            vector3<double> vel;
            if (get_VelocityENU_Ctrl(&vel))
            {
                double spd = safe_sqrt(sq(vel.x) + sq(vel.y));
                double dragK = cfg.windDragK[0];
                if (dragK > 0 && spd > 10 && isvalid(spd))
                {
                    double dragAcc = dragK * spd * 0.01;
                    windDrag.x = dragAcc * vel.x;
                    windDrag.y = dragAcc * vel.y;
                    WindDisturbance_xy.x += windDrag.x;
                    WindDisturbance_xy.y += windDrag.y;
                }
            }
        }

        double WindDisturbance_Filter_P = cfg.windBeta[0];
        if (WindDisturbance_Filter_P > 0.02)
        {
            WindDisturbance_x = WindDisturbance_filter[0].track4(WindDisturbance_xy.x, h, WindDisturbance_Filter_P, WindDisturbance_Filter_P, WindDisturbance_Filter_P, WindDisturbance_Filter_P);
            WindDisturbance_y = WindDisturbance_filter[1].track4(WindDisturbance_xy.y, h, WindDisturbance_Filter_P, WindDisturbance_Filter_P, WindDisturbance_Filter_P, WindDisturbance_Filter_P);
            WindDisturbance_x -= windDrag.x;
            WindDisturbance_y -= windDrag.y;
            //			double lp_factor = 2 * Pi * (1.0/CtrlRateHz) * 1.0;
            //			WindDisturbance_x += lp_factor * ( WindDisturbance_xy.x - WindDisturbance_x );
            //			WindDisturbance_y += lp_factor * ( WindDisturbance_xy.y - WindDisturbance_y );
        }
        else
        {
            WindDisturbance_filter[0].reset();
            WindDisturbance_filter[1].reset();
            WindDisturbance_x = WindDisturbance_y = 0;
        }
    }
    else
    {
        WindDisturbance_filter[0].reset();
        WindDisturbance_filter[1].reset();
        WindDisturbance_x = WindDisturbance_y = 0;
    }

    // 更新Home点位置
    if (Attitude_Control_Enabled == false)
    {
        vector3<double> position;
        get_Position_Ctrl(&position);
        HomeLocalZ = position.z;

        // 手动设置返航点后挂起自动重采样，保持GCS设置的经纬度/本地点
        if (HomeManualSet == false)
        {
            PosSensorHealthInf2 posInf;
            if (get_Health_XY(&posInf))
            {
                HomeAvailable = true;
                HomePoint.x = posInf.PositionENU.x;
                HomePoint.y = posInf.PositionENU.y;
            }
            else
                HomeAvailable = false;

            if (get_OptimalGlobal_XY(&posInf))
            {
                HomeLatLonAvailable = true;
                map_projection_reproject(&posInf.mp, posInf.PositionENU.x + posInf.HOffset.x, posInf.PositionENU.y + posInf.HOffset.y, &HomeLatLon.x, &HomeLatLon.y);
            }
        }

        HomeYaw = quat.getYaw();

        // 更新传感器高度
        static uint8_t currentScanSensor = 0;
        Position_Sensor_Data sensor;
        if (GetPositionSensorData(currentScanSensor, &sensor) && sensor.available &&
            (sensor.sensor_type == Position_Sensor_Type_RelativePositioning || sensor.sensor_type == Position_Sensor_Type_GlobalPositioning) &&
            (sensor.sensor_DataType == Position_Sensor_DataType_s_z || sensor.sensor_DataType == Position_Sensor_DataType_s_xyz || sensor.sensor_DataType == Position_Sensor_DataType_sv_z ||
             sensor.sensor_DataType == Position_Sensor_DataType_sv_xyz || sensor.sensor_DataType == Position_Sensor_DataType_s_z_nAC || sensor.sensor_DataType == Position_Sensor_DataType_s_xyz_nAC ||
             sensor.sensor_DataType == Position_Sensor_DataType_sv_z_nAC || sensor.sensor_DataType == Position_Sensor_DataType_sv_xyz_nAC))
        {
            homeSensorsZ[currentScanSensor].trust = sensor.z_LTtrustD;
            homeSensorsZ[currentScanSensor].posz = sensor.position.z;
        }
        else
            homeSensorsZ[currentScanSensor].trust = -1;
        ++currentScanSensor;
        if (currentScanSensor >= Position_Sensors_Count)
            currentScanSensor = 0;
    }
    else if (get_Position_MSStatus() != MS_Ready)
        HomeAvailable = false;
}

static double Roll_u = 0;
static double Pitch_u = 0;
static double Yaw_u = 0;
void update_ESO_1()
{
    // 更新角速度观测器
    vector3<double> angular_rate;
    get_AngularRate_Ctrl(&angular_rate);
    if (Attitude_Control_Enabled)
    {
        CtrlM_ESO[0]->run(angular_rate.x, inFlight);
        CtrlM_ESO[1]->run(angular_rate.y, inFlight);
        CtrlM_ESO[2]->run(angular_rate.z, inFlight);
    }

    vector3<double> acc;
    get_AccelerationENU_Ctrl(&acc);
    ESO_height.run(acc.z);
}
void update_ESO_2()
{
    if (Attitude_Control_Enabled)
    {
        CtrlM_ESO[0]->update_u(Roll_u);
        CtrlM_ESO[1]->update_u(Pitch_u);
        CtrlM_ESO[2]->update_u(Yaw_u);
    }
    Quaternion quat;
    get_AirframeY_quat(&quat, 0.1);
    double lean_cosin = quat.get_lean_angle_cosin();
    ESO_height.update_u(throttle_u, lean_cosin);
}
/*状态观测器*/

/*观测器接口*/
bool get_hover_throttle(double *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        *result = hover_throttle;
        UnlockCtrl();
        return true;
    }
    return false;
}
bool get_throttle_force(double *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        *result = ESO_height.get_force();
        UnlockCtrl();
        return true;
    }
    return false;
}
bool get_es_AccZ(double *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        *result = ESO_height.get_EsAcc();
        UnlockCtrl();
        return true;
    }
    return false;
}
bool get_throttle_b(double *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        *result = ESO_height.get_b();
        UnlockCtrl();
        return true;
    }
    return false;
}
bool get_ESO_height_T(double *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        *result = ESO_height.get_T();
        UnlockCtrl();
        return true;
    }
    return false;
}
bool get_is_inFlight(bool *result, double TIMEOUT)
{
    *result = inFlight;
    return true;
}
bool get_WindDisturbance(vector3<double> *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        *result = vector3<double>(WindDisturbance_x, WindDisturbance_y, 0);
        UnlockCtrl();
        return true;
    }
    return false;
}

bool get_EsAngularRate(vector3<double> *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        *result = vector3<double>(CtrlM_ESO[0]->get_EsAngularRate(), CtrlM_ESO[1]->get_EsAngularRate(), CtrlM_ESO[2]->get_EsAngularRate());
        UnlockCtrl();
        return true;
    }
    return false;
}
bool get_EsAngularAcc(vector3<double> *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        *result = vector3<double>(CtrlM_ESO[0]->get_EsAngularAcceleration(), CtrlM_ESO[1]->get_EsAngularAcceleration(), CtrlM_ESO[2]->get_EsAngularAcceleration());
        UnlockCtrl();
        return true;
    }
    return false;
}
bool get_CrashedState()
{
    return crashed;
}
/*观测器接口*/

/*控制接口*/
// 初始化计数
static int32_t StartCounter = 0;
#define Ctrl_initing (StartCounter < 1e6)
#define set_Ctrl_inited (StartCounter = 1e9)
// 保护方式
static uint8_t SafeBt = 0;

// 控制器打开关闭时间
static TIME attController_EnableTime(false);
static TIME attController_DisableTime(false);

// 期望TD4滤波器
static TD3_2DSL Target_tracker_RP;
static TD4_SL Target_trackerY;

// 角加速度滤波器
static double d_filted[3] = {0};

// 姿态控制模式
static Attitude_ControlMode RollPitch_ControlMode = Attitude_ControlMode_Angle;
static Attitude_ControlMode Yaw_ControlMode = Attitude_ControlMode_Angle;

// 输出滤波器
static double outRoll_filted = 0;
static double outPitch_filted = 0;
static double outYaw_filted = 0;

// 扰动包络滤波器
static double disturbance_filted[3] = {0};

// 期望姿态
static double throttle = 0;
static double target_Roll;
static double target_Pitch;
static double target_Yaw;
static vector3<double> target_AngularRate;
// 最大偏航角速度覆盖
static double maxYSp_override = -1;
bool set_maxYawSpOverride(double maxYSp, double TIMEOUT)
{
    if (maxYSp <= 0)
        return false;
    if (LockCtrl(TIMEOUT))
    {
        if (!Attitude_Control_Enabled)
        { // 控制器未打开
            UnlockCtrl();
            return false;
        }

        if (maxYSp_override < 0)
            maxYSp_override = cfg.maxYSp[0];
        cfg.maxYSp[0] = rad2degree(maxYSp);

        UnlockCtrl();
        return true;
    }
    return false;
}
bool reset_maxYawSpOverride(double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (!Attitude_Control_Enabled)
        { // 控制器未打开
            UnlockCtrl();
            return false;
        }

        if (maxYSp_override > 0)
            cfg.maxYSp[0] = maxYSp_override;
        else
        {
            UnlockCtrl();
            return false;
        }

        UnlockCtrl();
        return true;
    }
    return false;
}

static uint32_t motorTestMt = 0;
static TIME motorTestEndTime(false);
static int16_t motorTestFixThr = -1;
bool doMotorTest(uint32_t mt, double time, int16_t fixThr, double TIMEOUT)
{
    if (time > 8 || time < 0.001)
        return false;
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == true)
        { // 控制器已打开
            UnlockCtrl();
            return false;
        }

        motorTestMt = mt;
        motorTestFixThr = fixThr;
        if (motorTestMt == 0)
            motorTestEndTime.set_invalid();
        else
        {
            motorTestEndTime = TIME::now();
            motorTestEndTime += time;
        }

        UnlockCtrl();
        return true;
    }
    return false;
}

bool is_Attitude_Control_Enabled(bool *enabled, double TIMEOUT)
{
    *enabled = Attitude_Control_Enabled;
    return true;
}
bool Attitude_Control_Enable(double TIMEOUT)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    Quaternion quat;
    if (get_Airframe_quat(&quat, TIMEOUT) == false)
        return false;
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == true)
        { // 控制器已打开
            UnlockCtrl();
            return false;
        }

        // 读参数
        if (ReadParamGroup("AttCtrl", getAttCfgParamBase(), 0, TIMEOUT) != PR_OK)
        {
            UnlockCtrl();
            return false;
        }
        uint8_t safe_bt[8];
        if (ReadParam("MFunc_SafeBt", 0, 0, (uint64_t *) safe_bt, 0) != PR_OK)
            SafeBt = 0;
        else
            SafeBt = safe_bt[0];

        UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
        if (mt_count.MTCount == 0 || UAVCtrlMs[cfg.UAVType[0]] == 0)
        {
            UnlockCtrl();
            return false;
        }
        set_MainMotorCount(mt_count.MTCount, mt_count.STCount, cfg.st_Freq[0]);
        CtrlM = *UAVCtrlMs[cfg.UAVType[0]];
        CtrlM_available = true;
        StartCounter = 0;

        // 解锁时恢复Home自动重采样（手动返航点仅保留到本次起飞前）
        HomeManualSet = false;

        Attitude_Control_Enabled = true;
        attController_EnableTime = TIME::now();

        /*初始化*/
        // 读取电池电压
        BatteryInfo batInfo;
        batId = -1;
        getCurrentBatteryInfo(&batInfo, &batId);
        if (batId >= 0)
        { // 获取到电池
            batStVolt = batInfo.stVolt;
        }
        else
            batStVolt = 0;

        // 初始化高度ESO
        ESO_height.init(cfg.T[0], cfg.beta_h[0], cfg.beta_hAcc[0], CtrlRateHz * CtrlRateDiv);

        // 初始化期望TD4滤波器
        Target_tracker_RP.P1 = cfg.TD4_P1[0];
        Target_tracker_RP.P2 = cfg.TD4_P2[0];
        Target_tracker_RP.P3 = cfg.TD4_P3[0];
        Target_tracker_RP.r2 = degree2rad(cfg.maxRPSp[0]);
        Target_tracker_RP.r3 = degree2rad(cfg.maxRPAcc[0]);
        Target_tracker_RP.r4 = degree2rad(10000000.0);

        d_filted[0] = 0;
        d_filted[1] = 0;
        d_filted[2] = 0;

        Target_trackerY.P1 = cfg.TD4_P1[4];
        Target_trackerY.P2 = cfg.TD4_P2[4];
        Target_trackerY.P3 = cfg.TD4_P3[4];
        Target_trackerY.P4 = cfg.TD4_P4[4];
        Target_trackerY.r2n = Target_trackerY.r2p = degree2rad(cfg.maxYSp[0]);
        Target_trackerY.r3n = Target_trackerY.r3p = degree2rad(cfg.maxYAcc[0]);

        for (uint8_t i = 0; i < 3; ++i)
        {
            if (CtrlM_ESO[i])
                delete CtrlM_ESO[i];
        }
        switch (CtrlM.RP_eso_type)
        {
            case 0:
            { // 二阶
                CtrlM_ESO[0] = new ESO_AngularRate();
                CtrlM_ESO[1] = new ESO_AngularRate();
                break;
            }
            case 1:
            { // 二阶
                CtrlM_ESO[0] = new ESO_AngularRateHeli();
                CtrlM_ESO[1] = new ESO_AngularRateHeli();
                break;
            }
        }
        switch (CtrlM.Y_eso_type)
        {
            case 0:
            { // 二阶
                CtrlM_ESO[2] = new ESO_AngularRate();
                break;
            }
            case 1:
            { // 二阶
                CtrlM_ESO[2] = new ESO_AngularRateHeli();
                break;
            }
        }
        CtrlM.Init();
        /*初始化*/
        target_Yaw = quat.getYaw();
        target_Roll = target_Pitch = 0;
        RollPitch_ControlMode = Attitude_ControlMode_Angle;
        Yaw_ControlMode = Attitude_ControlMode_Angle;
        maxYSp_override = -1;

        // 更新控制时间
        bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
        if (!isMSafe)
            last_ZCtrlTime = last_XYCtrlTime = TIME::now();

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_StartFlight(double TIMEOUT)
{
    if (get_Attitude_MSStatus() != MS_Ready)
        return false;

    Quaternion quat;
    if (get_Airframe_quat(&quat, TIMEOUT) == false)
        return false;
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        { // 控制器未打开
            UnlockCtrl();
            return false;
        }
        //			if( inFlight == true )
        //			{	//已处于飞行状态
        //				UnlockCtrl();
        //				return false;
        //			}

        // 立即给定一定的悬停油门
        ESO_height.init(cfg.T[0], cfg.beta_h[0], cfg.beta_hAcc[0], CtrlRateHz * CtrlRateDiv, 30);

        // 立即判定起飞
        set_Ctrl_inited;
        pre_inFlightRP = pre_inFlightY = true;
        inFlight = true;

        // 更新控制时间
        bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
        if (!isMSafe)
            last_ZCtrlTime = last_XYCtrlTime = TIME::now();

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_Disable(double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        if (maxYSp_override > 0)
        {
            cfg.maxYSp[0] = maxYSp_override;
            maxYSp_override = -1;
        }
        Altitude_Control_Disable();
        Position_Control_Disable();
        Attitude_Control_Enabled = false;
        attController_DisableTime = TIME::now();

        CtrlM.DeInit();

        UnlockCtrl();
        return true;
    }
    return false;
}

bool get_Target_Throttle(double *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        *result = throttle;
        UnlockCtrl();
        return true;
    }
    return false;
}
bool get_OutputThrottle(double *result, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        *result = outputThrottle;
        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_Throttle(double thr, double TIMEOUT)
{
    if (!isvalid(thr))
        return false;
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        bool alt_enabled;
        is_Altitude_Control_Enabled(&alt_enabled);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (alt_enabled && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }

        bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
        if (!isMSafe && alt_enabled == false && ForceMSafeCtrl)
        { // 屏蔽用户控制
            last_ZCtrlTime = TIME::now();
            UnlockCtrl();
            return false;
        }

        throttle = thr;

        // 更新控制时间
        if (!isMSafe && alt_enabled == false)
            last_ZCtrlTime = TIME::now();

        UnlockCtrl();
        return true;
    }
    return false;
}

bool Attitude_Control_get_Target_RollPitch(double *Roll, double *Pitch, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }

        *Roll = target_Roll;
        *Pitch = target_Pitch;
        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_Target_RollPitch(double Roll, double Pitch, double TIMEOUT)
{
    if (!isvalid(Roll) || !isvalid(Pitch))
        return false;
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        bool pos_enabled;
        is_Position_Control_Enabled(&pos_enabled);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (pos_enabled && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }

        bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
        if (!isMSafe && pos_enabled == false && ForceMSafeCtrl)
        { // 屏蔽用户控制
            last_XYCtrlTime = TIME::now();
            UnlockCtrl();
            return false;
        }

        double angle = safe_sqrt(Roll * Roll + Pitch * Pitch);
        if (angle > degree2rad(cfg.maxLean[0]))
        {
            double scale = degree2rad(cfg.maxLean[0]) / angle;
            Roll *= scale;
            Pitch *= scale;
        }
        target_Roll = Roll;
        target_Pitch = Pitch;
        if (RollPitch_ControlMode != Attitude_ControlMode_Angle)
        {
            Target_tracker_RP.x1.zero();
            Target_tracker_RP.x2.zero();
            Target_tracker_RP.x3.zero();
            RollPitch_ControlMode = Attitude_ControlMode_Angle;
        }

        // 更新控制时间
        if (!isMSafe && pos_enabled == false)
            last_XYCtrlTime = TIME::now();

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_Target_RollPitchRate(double RollRate, double PitchRate, double TIMEOUT)
{
    if (!isvalid(RollRate) || !isvalid(PitchRate))
        return false;
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        bool pos_enabled;
        is_Position_Control_Enabled(&pos_enabled);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (pos_enabled && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }

        bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
        if (!isMSafe && pos_enabled == false && ForceMSafeCtrl)
        { // 屏蔽用户控制
            last_XYCtrlTime = TIME::now();
            UnlockCtrl();
            return false;
        }

        target_AngularRate.x = RollRate;
        target_AngularRate.y = PitchRate;
        if (RollPitch_ControlMode != Attitude_ControlMode_AngularRate)
        {
            Target_tracker_RP.x1.zero();
            Target_tracker_RP.x2.zero();
            Target_tracker_RP.x3.zero();
            RollPitch_ControlMode = Attitude_ControlMode_AngularRate;
        }

        if (Yaw_ControlMode != Attitude_ControlMode_AngularRate)
        { // 航向必须也为角速率模式
            target_AngularRate.z = 0;
            Yaw_ControlMode = Attitude_ControlMode_AngularRate;
        }

        // 更新控制时间
        if (!isMSafe && pos_enabled == false)
            last_XYCtrlTime = TIME::now();

        UnlockCtrl();
        return true;
    }
    return false;
}

bool Attitude_Control_get_TargetYaw(double *TargetYaw, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        *TargetYaw = target_Yaw;

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_get_TargetTrackYaw(double *TargetYaw, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        *TargetYaw = Target_trackerY.x1;

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_get_YawTrackErr(double *YawErr, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        // 非负误差绝对值：x1与target_Yaw运行期同域(差值域<2π)，fabs直读即为真实角度差。
        // 注意：调用方以 yawTrackErr < 0.01 判完成，此处必须保证非负（勿改为有符号差值）
        *YawErr = fabs(Target_trackerY.x1 - target_Yaw);

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_get_YawTrackVel(double *YawVel, double TIMEOUT)
{
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }
        *YawVel = Target_trackerY.x2;

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_Target_Yaw(double Yaw, double TIMEOUT)
{
    if (!isvalid(Yaw))
        return false;

    // 屏蔽用户控制
    bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
    if (!isMSafe && ForceMSafeCtrl)
        return false;

    Quaternion quat;
    get_Airframe_quat(&quat);
    double yaw_declination;
    get_YawDeclination(&yaw_declination);
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }

        Position_ControlMode pos_mode;
        get_Position_ControlMode(&pos_mode);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (Is_YawAutoMode(pos_mode) && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }
        if (RollPitch_ControlMode != Attitude_ControlMode_Angle)
        { // 水平姿态必须为角度模式才能控制航向角
            UnlockCtrl();
            return false;
        }

        if (Yaw_ControlMode != Attitude_ControlMode_Angle)
        {
            Target_trackerY.x1 = quat.getYaw();
            Yaw_ControlMode = Attitude_ControlMode_Angle;
        }

        double yaw_err = fmod(Yaw - Target_trackerY.x1 - yaw_declination, 2 * Pi);
        if (yaw_err > Pi)
            yaw_err -= 2 * Pi;
        else if (yaw_err < -Pi)
            yaw_err += 2 * Pi;
        target_Yaw = Target_trackerY.x1 + yaw_err;

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_Target_YawRelative(double Yaw, double TIMEOUT)
{
    if (!isvalid(Yaw))
        return false;

    // 屏蔽用户控制
    bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
    if (!isMSafe && ForceMSafeCtrl)
        return false;

    Quaternion quat;
    get_Airframe_quat(&quat);
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }

        Position_ControlMode pos_mode;
        get_Position_ControlMode(&pos_mode);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (Is_YawAutoMode(pos_mode) && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }
        if (RollPitch_ControlMode != Attitude_ControlMode_Angle)
        { // 水平姿态必须为角度模式才能控制航向角
            UnlockCtrl();
            return false;
        }

        double currentYaw = quat.getYaw();
        if (Yaw_ControlMode != Attitude_ControlMode_Angle)
        {
            Target_trackerY.x1 = currentYaw;
            Yaw_ControlMode = Attitude_ControlMode_Angle;
        }
        target_Yaw = Target_trackerY.x1 + Yaw;

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_Target_Yaw_Offboard(double Yaw, double YawRate, double TIMEOUT)
{
    if (!isvalid(Yaw) || !isvalid(YawRate))
        return false;

    {
        double maxYawRate = degree2rad(cfg.maxYSp[0]);
        if (YawRate > maxYawRate)
            YawRate = maxYawRate;
        else if (YawRate < -maxYawRate)
            YawRate = -maxYawRate;
    }

    // 屏蔽用户控制
    bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
    if (!isMSafe && ForceMSafeCtrl)
        return false;

    Quaternion quat, quatY;
    get_Airframe_quat(&quat);
    get_AirframeY_quat(&quatY);
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }

        Position_ControlMode pos_mode;
        get_Position_ControlMode(&pos_mode);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (Is_YawAutoMode(pos_mode) && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }
        if (RollPitch_ControlMode != Attitude_ControlMode_Angle)
        { // 水平姿态必须为角度模式才能控制航向角
            UnlockCtrl();
            return false;
        }

        if (Yaw_ControlMode != Attitude_ControlMode_Angle && Yaw_ControlMode != Attitude_ControlMode_OffBoard)
            Target_trackerY.x1 = quat.getYaw();

        // 计算航向角误差
        double yaw_declination;
        get_YawDeclination(&yaw_declination);
        double yaw_err = fmod(Yaw - Target_trackerY.x1 - yaw_declination, 2 * Pi);

        if (yaw_err > Pi)
            yaw_err -= 2 * Pi;
        else if (yaw_err < -Pi)
            yaw_err += 2 * Pi;
        target_Yaw = Target_trackerY.x1 + yaw_err;
        target_AngularRate.z = YawRate;
        Yaw_ControlMode = Attitude_ControlMode_OffBoard;

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_Target_YawRelative_Offboard(double Yaw, double YawRate, double TIMEOUT)
{
    if (!isvalid(Yaw) || !isvalid(YawRate))
        return false;

    {
        double maxYawRate = degree2rad(cfg.maxYSp[0]);
        if (YawRate > maxYawRate)
            YawRate = maxYawRate;
        else if (YawRate < -maxYawRate)
            YawRate = -maxYawRate;
    }

    // 屏蔽用户控制
    bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
    if (!isMSafe && ForceMSafeCtrl)
        return false;

    Quaternion quat, quatY;
    get_Airframe_quat(&quat);
    get_AirframeY_quat(&quatY);
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }

        Position_ControlMode pos_mode;
        get_Position_ControlMode(&pos_mode);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (Is_YawAutoMode(pos_mode) && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }
        if (RollPitch_ControlMode != Attitude_ControlMode_Angle)
        { // 水平姿态必须为角度模式才能控制航向角
            UnlockCtrl();
            return false;
        }

        double currentYaw = quat.getYaw();
        if (Yaw_ControlMode != Attitude_ControlMode_Angle && Yaw_ControlMode != Attitude_ControlMode_OffBoard)
        {
            Target_trackerY.x1 = currentYaw;
            Yaw_ControlMode = Attitude_ControlMode_Angle;
        }
        target_Yaw = Target_trackerY.x1 + Yaw;
        target_AngularRate.z = YawRate;
        Yaw_ControlMode = Attitude_ControlMode_OffBoard;

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_Target_YawRate(double YawRate, double TIMEOUT)
{
    if (!isvalid(YawRate))
        return false;

    {
        double maxYawRate = degree2rad(cfg.maxYSp[0]);
        if (YawRate > maxYawRate)
            YawRate = maxYawRate;
        else if (YawRate < -maxYawRate)
            YawRate = -maxYawRate;
    }

    // 屏蔽用户控制
    bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
    if (!isMSafe && ForceMSafeCtrl)
        return false;

    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }

        Position_ControlMode pos_mode;
        get_Position_ControlMode(&pos_mode);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (Is_YawAutoMode(pos_mode) && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }

        target_AngularRate.z = YawRate;
        Yaw_ControlMode = Attitude_ControlMode_AngularRate;

        UnlockCtrl();
        return true;
    }
    return false;
}
bool Attitude_Control_set_YawLock(double TIMEOUT)
{
    // 屏蔽用户控制
    bool isMSafe = (xTaskGetCurrentTaskHandle() == MSafeTaskHandle);
    if (!isMSafe && ForceMSafeCtrl)
        return false;
    if (LockCtrl(TIMEOUT))
    {
        if (Attitude_Control_Enabled == false)
        {
            UnlockCtrl();
            return false;
        }

        Position_ControlMode pos_mode;
        get_Position_ControlMode(&pos_mode);
        bool isAtCtrl = (xTaskGetCurrentTaskHandle() == ControlTaskHandle);
        if (Is_YawAutoMode(pos_mode) && !isAtCtrl)
        { // 位置控制器起作用时不允许用户控制
            UnlockCtrl();
            return false;
        }
        if (RollPitch_ControlMode != Attitude_ControlMode_Angle)
        { // 水平姿态必须为角度模式才能控制航向角
            UnlockCtrl();
            return false;
        }

        if (Yaw_ControlMode == Attitude_ControlMode_AngularRate || Yaw_ControlMode == Attitude_ControlMode_OffBoard)
            Yaw_ControlMode = Attitude_ControlMode_Locking;

        UnlockCtrl();
        return true;
    }
    return false;
}
/*控制接口*/

/*机型控制方式*/

// PWM映射
enum STMapType
{
    STMapType_standard = 0, // 非对称映射 大范围侧占50%
    STMapType_standardS,    // 非对称映射 小范围侧占50%
    STMapType_symmetry,     // 对称映射
};
static inline void ST_PWMMap(double pwm_out[], uint8_t start, uint8_t STmotors, STMapType mapType)
{
    for (uint8_t i = start; i < start + STmotors; ++i)
    {
        float st_min = getStMin(i);
        float st_mid = getStMid(i);
        float st_max = getStMax(i);

        float scale;
        if (mapType == STMapType_standard)
        { // 非对称模式
            if (fabsf(st_max - st_mid) > fabsf(st_mid - st_min))
                scale = st_max - st_mid;
            else
                scale = st_mid - st_min;
        }
        else
        { // 对称模式
            if (fabsf(st_max - st_mid) > fabsf(st_mid - st_min))
                scale = st_mid - st_min;
            else
                scale = st_max - st_mid;
            // 限制输出范围
            if (mapType == STMapType_symmetry)
                pwm_out[i] = constrain(pwm_out[i], 100.0);
        }
        pwm_out[i] = pwm_out[i] / 100.0 * scale + st_mid;

        if (st_max > st_min)
        {
            if (pwm_out[i] > st_max)
                pwm_out[i] = st_max;
            else if (pwm_out[i] < st_min)
                pwm_out[i] = st_min;
        }
        else
        {
            if (pwm_out[i] > st_min)
                pwm_out[i] = st_min;
            else if (pwm_out[i] < st_max)
                pwm_out[i] = st_max;
        }
        pwm_out[i] = (pwm_out[i] - 1000) / 1000.0 * 100;
    }
}

// 电机非线性输出 线性修正
static inline void throttle_nonlinear_compensation(uint8_t mt_count, double out[])
{
    double output_minimum_throttle = cfg.STThrottle[0];
    double output_range = 100.0f - output_minimum_throttle;
    double inv_output_range = 1.0 / output_range;

    // a：非线性因子(0-1)
    // m：最大油门比例(0.6-1)

    // 设油门-力曲线方程为：
    // F = kx^2 + (1-a)x ( 0<=x<=m F最大值为1 )
    // x = m时：km^2 + (1-a)m = 1
    // 得k = ( 1 - (1-a)m ) / m^2
    // a_1 = a - 1
    // Hk  = 1 / 2k
    // K4  = 4* k
    // 解方程组：kx^2 + (1-a)x = out
    // 得到的x即为线性化后的输出
    double _lift_max = cfg.FullThrRatio[0];
    double a_1 = cfg.NonlinearFactor[0] - 1;
    double k = (1 + a_1 * _lift_max) / (_lift_max * _lift_max);
    double Hk = 1.0f / (2 * k);
    double K4 = 4 * k;

    for (uint8_t i = 0; i < mt_count; ++i)
    {
        if (out[i] > output_minimum_throttle - 0.1f)
        {
            out[i] -= output_minimum_throttle;
            out[i] *= inv_output_range;
            if (out[i] < 0)
                out[i] = 0;
            out[i] = Hk * (a_1 + safe_sqrt(a_1 * a_1 + K4 * out[i]));
            out[i] *= output_range;
            out[i] += output_minimum_throttle;
        }
        else
            out[i] = 0;
    }
}
static inline void throttle_nonlinear_compensation_getRatio(uint8_t mt_count, double out[])
{
    double output_minimum_throttle = cfg.STThrottle[0];
    double output_range = 100.0f - output_minimum_throttle;
    double inv_output_range = 1.0 / output_range;

    // a：非线性因子(0-1)
    // m：最大油门比例(0.6-1)

    // 设油门-力曲线方程为：
    // F = kx^2 + (1-a)x ( 0<=x<=m F最大值为1 )
    // x = m时：km^2 + (1-a)m = 1
    // 得k = ( 1 - (1-a)m ) / m^2
    // a_1 = a - 1
    // Hk  = 1 / 2k
    // K4  = 4* k
    // 解方程组：kx^2 + (1-a)x = out
    // 得到的x即为线性化后的输出
    double _lift_max = cfg.FullThrRatio[0];
    double a_1 = cfg.NonlinearFactor[0] - 1;
    double k = (1 + a_1 * _lift_max) / (_lift_max * _lift_max);
    double Hk = 1.0f / (2 * k);
    double K4 = 4 * k;

    for (uint8_t i = 0; i < mt_count; ++i)
    {
        if (out[i] > output_minimum_throttle - 0.1f)
        {
            out[i] -= output_minimum_throttle;
            out[i] *= inv_output_range;
            if (out[i] < 0)
                out[i] = 0;
            double inv = safe_sqrt(a_1 * a_1 + K4 * out[i]);
            if (inv > 0)
            {
                out[i] = 2.0 / inv;
                if (out[i] > 100)
                    out[i] = 100;
            }
            else
                out[i] = 100;
        }
        else
            out[i] = 0;
    }
}

/*多旋翼*/

static inline double MultiRotor_percent_to_throttle(double percent, double output_minimum_throttle, double output_range)
{
    return constrain(percent / 100.0 * output_range + output_minimum_throttle, output_minimum_throttle, 100.0);
}
static inline double MultiRotor_throttle_to_percent(double throttle, double output_minimum_throttle, double output_range)
{
    return (throttle - output_minimum_throttle) / output_range * 100;
}

static void MultiRotor_Init()
{
    // 初始化姿态ESO
    ((ESO_AngularRate *) CtrlM_ESO[0])
        ->init(cfg.order[0], cfg.T[0], cfg.b[0], cfg.beta[0], cfg.beta[0], cfg.orderD[0], cfg.betaD[0], cfg.rolBS[0], cfg.rolBSBW[0], cfg.rolBS2[0], cfg.rolBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxRP[0]);
    ((ESO_AngularRate *) CtrlM_ESO[1])
        ->init(cfg.order[0], cfg.T[0], cfg.b[2], cfg.beta[0], cfg.beta[0], cfg.orderD[0], cfg.betaD[0], cfg.pitBS[0], cfg.pitBSBW[0], cfg.pitBS2[0], cfg.pitBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxRP[0]);
    ((ESO_AngularRate *) CtrlM_ESO[2])
        ->init(cfg.order2[0], 1.0 / (CtrlRateHz * CtrlRateDiv), cfg.b[4], cfg.beta2[0], cfg.beta2[0], cfg.orderD2[0], cfg.betaD2[0], cfg.yawBS[0], cfg.yawBSBW[0], cfg.yawBS2[0], cfg.yawBSBW2[0],
               CtrlRateHz * CtrlRateDiv, cfg.aMaxYaw[0]);

    // 最低油门
    cfg.minThrottlePct = 0;
    // 怠速油门
    cfg.idleThrottlePct = 0;
}

static void MultiRotor_DeInit() {}

static void MultiRotor_PreArmControl(Receiver rc)
{
    bool needMotorTest = false;
    if (motorTestEndTime.is_valid() == false)
    {
    }
    else if (motorTestEndTime.get_pass_time() > 0 || motorTestMt == 0)
    {
        motorTestEndTime.set_invalid();
    }
    else
        needMotorTest = true;

    if (!needMotorTest)
        MainMotor_PullDownAll();
    else
    { // 电机测试
        UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
        double pwm_out[MAX_PWM_COUNT] = {0};
        for (uint8_t i = 0; i < mt_count.MTCount; ++i)
        {
            if (motorTestMt & (1 << i))
            {
                if (motorTestFixThr < 0)
                    pwm_out[i] = cfg.STThrottle[0];
                else
                    pwm_out[i] = motorTestFixThr * 0.1 - 100;
            }
        }
        MainMotor_PWM_Out(pwm_out);
    }
}

static void MultiRotor_InitControl(bool DbgSafe)
{
    if (DbgSafe)
    {
        MainMotor_PullDownAll();
        StartCounter = 0;
        return;
    }

    // 启动初始化
    if (StartCounter < 1e6)
    { // 还未进行启动初始化
        if (StartCounter >= 0)
        { // 启动动作
            switch (cfg.Mode[0] & 0xff)
            { // 顺序起转电机
                case 1:
                {
                    UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
                    if (StartCounter < 0.3 * mt_count.MTCount * CtrlRateHz)
                    {
                        double pwm_out[MAX_PWM_COUNT] = {0};
                        for (uint8_t i = 0; i < mt_count.MTCount; ++i)
                        {
                            if (StartCounter > 0.3 * i * CtrlRateHz)
                                pwm_out[i] = cfg.STThrottle[0];
                            else
                                break;
                        }
                        MainMotor_PWM_Out(pwm_out);
                        ++StartCounter;
                    }
                    else
                        StartCounter = -1;
                    break;
                }

                default:
                {
                    double pwm_out[MAX_PWM_COUNT] = {0};
                    UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
                    for (uint8_t i = 0; i < mt_count.MTCount; ++i)
                        pwm_out[i] = cfg.STThrottle[0];
                    MainMotor_PWM_Out(pwm_out);
                    StartCounter = -1;
                    break;
                }
            }
        }
        else
        { // 启动延时
            if ((-StartCounter) > cfg.STDelay[0] * CtrlRateHz)
                StartCounter = 1e9;
            else
                --StartCounter;
        }
        return;
    }
}

static void MultiRotor_MotorControl(uint8_t mt_count, double output_throttle, double thrRatio, double outRoll, double outPitch, double outYaw, double rp_out[], double yaw_out[], double *rotor_output,
                                    double *u_thrPct, double *u_rol, double *u_pit, double *u_yaw)
{
    // 计算增益修正系数
    double bat_b_scale = 1.0;
    BatteryInfo batInfo;
    int8_t current_batId = -1;
    getCurrentBatteryInfo(&batInfo, &current_batId);
    if (batId >= 0 && current_batId == batId)
    { // 获取到电池
        if ((cfg.Mode[0] & ATTCTRL_MODE_BATCOMP_DIS_BIT) == 0 && batStVolt > 7 && batInfo.totalVoltRawFilted > 7)
            bat_b_scale = constrain(batInfo.totalVoltRawFilted / batStVolt, 0.35f, 3.0f);
    }

    // 海拔推力补偿
    double baro_b_scale = 1.0;
    extern float INTERNAL_TEMPERATURE;
    extern float INTERNAL_PRESSURE;
    double tempK = INTERNAL_TEMPERATURE + C_TO_KELVIN;
    // 加入气压范围保护，覆盖 +25km 以内（约 2500Pa）的真实气压，避免 SPL06 未正确读取时 0/异常值导致补偿突变
    if (tempK > 100 && INTERNAL_PRESSURE > 2500.0f && INTERNAL_PRESSURE < 200000.0f)
    {
        double eas2tas_squared = SSL_AIR_DENSITY / (INTERNAL_PRESSURE / (ISA_GAS_CONSTANT * tempK));
        if (eas2tas_squared > 0.7 && eas2tas_squared < 10.0)
            baro_b_scale = 1.0 / eas2tas_squared;
    }
    // 钳制 baro_b_scale 范围，避免极端值（几乎是防止 b 接近 0 导致后续除零）
    baro_b_scale = constrain(baro_b_scale, 0.2, 3.0);

    // 根据电池电压调整控制对象增益
    double b_scale = bat_b_scale * baro_b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[0])->b = cfg.b[0] * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[1])->b = cfg.b[2] * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[2])->b = cfg.b[4] * b_scale;

    double output_minimum_throttle = cfg.STThrottle[0];

    double output_range = 100.0 - output_minimum_throttle;
    double output_midpoint = output_range / 2;

    // 油门百分比转换为实际油门量
    output_throttle *= thrRatio;
    output_throttle = constrain(output_throttle, 0.0, 100.0);
    output_throttle = MultiRotor_percent_to_throttle(output_throttle, output_minimum_throttle, output_range);

    /*pitch roll 输出限幅*/
    // 如果需要的pitch roll输出超出当前油门能提供的输出范围
    // 调整油门获得尽量满足pitch roll输出
    double output_max = fabs(rp_out[0]);
    for (uint8_t i = 1; i < mt_count; ++i)
    {
        double abs_out = fabs(rp_out[i]);
        if (abs_out > output_max)
            output_max = abs_out;
    }

    double max_allow_output = 100.0f - output_throttle;
    double min_allow_output = output_minimum_throttle - output_throttle;
    double allow_ouput_range;
    if (max_allow_output < -min_allow_output)
    { // 降低油门确保姿态输出
        allow_ouput_range = max_allow_output;
        if (output_max > allow_ouput_range)
        { // 需要降低油门
            if (output_max > output_midpoint)
            { // 输出超过最大输出范围
                // 将油门调整为50%确保可以进行最大输出
                output_throttle = output_midpoint + output_minimum_throttle;
                allow_ouput_range = output_midpoint;
            }
            else
            { // 降低油门到所需值
                output_throttle = 100.0f - output_max;
                allow_ouput_range = output_max;
            }
        }
    }
    else
    { // 抬高油门保证姿态输出
        allow_ouput_range = -min_allow_output;
        if (output_max > allow_ouput_range)
        { // 需要抬高油门

            // 求最高允许的油门值（不能大于悬停油门）
            double max_allowed_output_range;
            if (cfg.Mode[0] & ATTCTRL_MODE_FPV_BIT)
            { // 穿越机模式
                Receiver rc;
                if (getReceiver(&rc) && rc.available && rc.data[0] > 40) // 穿越机油门杆在高位时允许大幅度抬升油门
                    max_allowed_output_range = output_midpoint;
                else
                {
                    double hover_throttle_force = MultiRotor_percent_to_throttle(hover_throttle * thrRatio, output_minimum_throttle, output_range) - output_minimum_throttle;
                    max_allowed_output_range = hover_throttle_force * 0.85;
                    if (output_midpoint < max_allowed_output_range)
                        max_allowed_output_range = output_midpoint;
                }
                if (max_allowed_output_range < output_throttle - output_minimum_throttle)
                    max_allowed_output_range = output_throttle - output_minimum_throttle;
            }
            else
            { // 普通模式
                double hover_throttle_force = MultiRotor_percent_to_throttle(hover_throttle * thrRatio, output_minimum_throttle, output_range) - output_minimum_throttle;
                max_allowed_output_range = hover_throttle_force * 0.85;
                if (output_midpoint < max_allowed_output_range)
                    max_allowed_output_range = output_midpoint;
                if (max_allowed_output_range < output_throttle - output_minimum_throttle)
                    max_allowed_output_range = output_throttle - output_minimum_throttle;
            }
            double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;

            if (output_max > max_allowed_output_range)
            { // 输出范围大于最大允许范围
                // 抬高油门至最大允许范围
                output_throttle = max_allowed_throttle;
                allow_ouput_range = max_allowed_output_range;
            }
            else
            { // 抬高油门到所需值
                output_throttle = output_minimum_throttle + output_max;
                allow_ouput_range = output_max;
            }
        }
    }

    // 输出限幅修正
    if (output_max > allow_ouput_range)
    { // 需要修正输出
        double scale = allow_ouput_range / output_max;
        for (uint8_t i = 0; i < mt_count; ++i)
            rotor_output[i] = rp_out[i] * scale;
        *u_rol = outRoll * scale;
        *u_pit = outPitch * scale;
    }
    else
    {
        for (uint8_t i = 0; i < mt_count; ++i)
            rotor_output[i] = rp_out[i];
        *u_rol = outRoll;
        *u_pit = outPitch;
    }
    /*pitch roll 输出限幅*/

    /*yaw output 输出限幅*/
    double rotorYawChk[16];
    for (uint8_t i = 0; i < mt_count; ++i)
    {
        rotorYawChk[i] = output_throttle + rotor_output[i];
    }
    throttle_nonlinear_compensation_getRatio(mt_count, rotorYawChk);
    double yawRatioSum = 0;
    for (uint8_t i = 0; i < mt_count; ++i)
    {
        yawRatioSum += rotorYawChk[i];
    }
    if (mt_count > 0)
        yawRatioSum *= 1.0 / mt_count;
    if (yawRatioSum > 0)
    {
        double scale = 1.0 / yawRatioSum;
        for (uint8_t i = 0; i < mt_count; ++i)
        {
            yaw_out[i] *= scale;
        }
    }

    // 计算偏航输出到上下界距离
    double yaw_out_up = 100, yaw_out_dn = 100;
    for (uint8_t i = 0; i < mt_count; ++i)
    {
        double current_rotor_output = output_throttle + rotor_output[i];
        max_allow_output = 100.0f - current_rotor_output;
        min_allow_output = current_rotor_output - output_minimum_throttle;

        if (yaw_out[i] > 0)
        { // 更新距离上界距离
            double out_up = max_allow_output - yaw_out[i];
            if (out_up < yaw_out_up)
                yaw_out_up = out_up;
        }
        else
        { // 更新距离下界距离
            double out_dn = min_allow_output - -yaw_out[i];
            if (out_dn < yaw_out_dn)
                yaw_out_dn = out_dn;
        }
    }

    if (yaw_out_dn < 0)
    { // 偏航输出超出下界
        // 抬升油门保住偏航输出

        double max_allowed_output_range;
        if (cfg.Mode[0] & ATTCTRL_MODE_FPV_BIT)
        { // 穿越机模式
            Receiver rc;
            if (getReceiver(&rc) && rc.available && rc.data[0] > 40) // 穿越机油门杆在高位时允许大幅度抬升油门
                max_allowed_output_range = output_midpoint;
            else
            {
                double hover_throttle_force = MultiRotor_percent_to_throttle(hover_throttle * thrRatio, output_minimum_throttle, output_range) - output_minimum_throttle;
                max_allowed_output_range = hover_throttle_force * 0.85;
                if (output_midpoint < max_allowed_output_range)
                    max_allowed_output_range = output_midpoint;
            }
            if (max_allowed_output_range < output_throttle - output_minimum_throttle)
                max_allowed_output_range = output_throttle - output_minimum_throttle;
        }
        else
        { // 普通模式
            double hover_throttle_force = MultiRotor_percent_to_throttle(hover_throttle * thrRatio, output_minimum_throttle, output_range) - output_minimum_throttle;
            max_allowed_output_range = hover_throttle_force * 0.85;
            if (output_midpoint < max_allowed_output_range)
                max_allowed_output_range = output_midpoint;
            if (max_allowed_output_range < output_throttle - output_minimum_throttle)
                max_allowed_output_range = output_throttle - output_minimum_throttle;
        }
        double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;

        // 期望抬升的油门量
        double req_up = -yaw_out_dn;
        // 抬升油门量不能使输出超出上界
        if (req_up > yaw_out_up)
            req_up = yaw_out_up;
        if (req_up < 0)
            req_up = 0;

        // 抬升油门
        output_throttle += req_up;
        if (output_throttle > max_allowed_throttle)
            output_throttle = max_allowed_throttle;
    }
    else if (cfg.YawPri[0] > 0.1 && yaw_out_up < 0)
    { // 降低油门确保姿态输出

        // 求最多允许降低的油门值
        double max_dn = 0;
        max_dn = cfg.YawPri[0];
        // 期望降低的油门量
        double req_dn = -yaw_out_up;
        // 降低油门量不能使输出超出下界
        if (req_dn > max_dn)
            req_dn = max_dn;
        if (req_dn > yaw_out_dn)
            req_dn = yaw_out_dn;
        if (req_dn < 0)
            req_dn = 0;

        // 降低油门
        output_throttle -= req_dn;
    }

    /*yaw输出限幅计算*/
    double yaw_scale = 1.0;
    for (uint8_t i = 0; i < mt_count; ++i)
    {
        double current_rotor_output = output_throttle + rotor_output[i];
        max_allow_output = 100.0f - current_rotor_output;
        min_allow_output = output_minimum_throttle - current_rotor_output;
        if (yaw_out[i] > max_allow_output + 0.01f)
        {
            double new_yaw_scale = max_allow_output / yaw_out[i];
            if (new_yaw_scale < yaw_scale)
                yaw_scale = new_yaw_scale;
        }
        else if (yaw_out[i] < min_allow_output - 0.01f)
        {
            double new_yaw_scale = min_allow_output / yaw_out[i];
            if (new_yaw_scale < yaw_scale)
                yaw_scale = new_yaw_scale;
        }
    }
    /*yaw输出限幅计算*/

    // lower yaw output to ensure attitude control and alt control
    if (yaw_scale < 0)
        yaw_scale = 0;
    outYaw *= yaw_scale;
    *u_yaw = outYaw;
    /*yaw output 输出限幅*/

    // 更新油门油门观测
    double throttle_percent = (output_throttle - output_minimum_throttle) / output_range * 100;
    *u_thrPct = throttle_percent;
    // 补偿非线性输出
    for (uint8_t i = 0; i < mt_count; ++i)
        rotor_output[i] += output_throttle + yaw_scale * yaw_out[i];
    throttle_nonlinear_compensation(mt_count, rotor_output);
}

static void MultiRotor4X_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[4];
    rp_out[0] = -outPitch + outRoll;
    rp_out[1] = +outPitch + outRoll;
    rp_out[2] = +outPitch - outRoll;
    rp_out[3] = -outPitch - outRoll;
    double yaw_out[4];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct, r_rol, r_pit, r_yaw;
    MultiRotor_MotorControl(4, output_throttle, 1, outRoll, outPitch, outYaw, rp_out, yaw_out, rotor_output, &r_thrPct, &r_rol, &r_pit, &r_yaw);
    Roll_u = r_rol;
    Pitch_u = r_pit;
    Yaw_u = r_yaw;
    update_output_throttle(r_thrPct, 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor4X_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor4X_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor4C_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[4];
    rp_out[0] = -outPitch;
    rp_out[1] = +outRoll;
    rp_out[2] = +outPitch;
    rp_out[3] = -outRoll;
    double yaw_out[4];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct, r_rol, r_pit, r_yaw;
    MultiRotor_MotorControl(4, output_throttle, 1, outRoll, outPitch, outYaw, rp_out, yaw_out, rotor_output, &r_thrPct, &r_rol, &r_pit, &r_yaw);
    Roll_u = r_rol;
    Pitch_u = r_pit;
    Yaw_u = r_yaw;
    update_output_throttle(r_thrPct, 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor4C_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor4C_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor6X_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[6];
    double RollS = outRoll * 1.1547005383792515290182975610039f;
    double half_outRoll = 0.5f * RollS;
    rp_out[0] = -outPitch + half_outRoll;
    rp_out[1] = RollS;
    rp_out[2] = +outPitch + half_outRoll;
    rp_out[3] = +outPitch - half_outRoll;
    rp_out[4] = -RollS;
    rp_out[5] = -outPitch - half_outRoll;
    double yaw_out[6];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct, r_rol, r_pit, r_yaw;
    MultiRotor_MotorControl(6, output_throttle, 1, outRoll, outPitch, outYaw, rp_out, yaw_out, rotor_output, &r_thrPct, &r_rol, &r_pit, &r_yaw);
    Roll_u = r_rol;
    Pitch_u = r_pit;
    Yaw_u = r_yaw;
    update_output_throttle(r_thrPct, 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor6X_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor6X_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor6C_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[6];
    double PitchS = outPitch * 1.1547005383792515290182975610039f;
    double half_outPitch = 0.5f * PitchS;
    rp_out[0] = -PitchS;
    rp_out[1] = -half_outPitch + outRoll;
    rp_out[2] = +half_outPitch + outRoll;
    rp_out[3] = +PitchS;
    rp_out[4] = +half_outPitch - outRoll;
    rp_out[5] = -half_outPitch - outRoll;
    double yaw_out[6];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct, r_rol, r_pit, r_yaw;
    MultiRotor_MotorControl(6, output_throttle, 1, outRoll, outPitch, outYaw, rp_out, yaw_out, rotor_output, &r_thrPct, &r_rol, &r_pit, &r_yaw);
    Roll_u = r_rol;
    Pitch_u = r_pit;
    Yaw_u = r_yaw;
    update_output_throttle(r_thrPct, 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor6C_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor6C_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor8C_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[8];
    double half_outPitch = 0.5f * outPitch;
    double half_outRoll = 0.5f * outRoll;
    rp_out[0] = -outPitch + 0;
    rp_out[1] = -half_outPitch + half_outRoll;
    rp_out[2] = 0 + outRoll;
    rp_out[3] = +half_outPitch + half_outRoll;
    rp_out[4] = +outPitch + 0;
    rp_out[5] = +half_outPitch - half_outRoll;
    rp_out[6] = 0 - outRoll;
    rp_out[7] = -half_outPitch - half_outRoll;
    double yaw_out[8];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +outYaw;
    yaw_out[6] = -outYaw;
    yaw_out[7] = +outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct, r_rol, r_pit, r_yaw;
    MultiRotor_MotorControl(8, output_throttle, 1, outRoll, outPitch, outYaw, rp_out, yaw_out, rotor_output, &r_thrPct, &r_rol, &r_pit, &r_yaw);
    Roll_u = r_rol;
    Pitch_u = r_pit;
    Yaw_u = r_yaw;
    update_output_throttle(r_thrPct, 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor8C_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor8C_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor62X_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double RollS = outRoll * 1.1547005383792515290182975610039f;
    double half_outRoll = 0.5f * RollS;
    double rp_out[12];
    rp_out[0] = -outPitch + half_outRoll;
    rp_out[1] = RollS;
    rp_out[2] = +outPitch + half_outRoll;
    rp_out[3] = +outPitch - half_outRoll;
    rp_out[4] = -RollS;
    rp_out[5] = -outPitch - half_outRoll;
    rp_out[6] = -outPitch + half_outRoll;
    rp_out[7] = RollS;
    rp_out[8] = +outPitch + half_outRoll;
    rp_out[9] = +outPitch - half_outRoll;
    rp_out[10] = -RollS;
    rp_out[11] = -outPitch - half_outRoll;
    double yaw_out[12];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +outYaw;
    yaw_out[6] = +outYaw;
    yaw_out[7] = -outYaw;
    yaw_out[8] = +outYaw;
    yaw_out[9] = -outYaw;
    yaw_out[10] = +outYaw;
    yaw_out[11] = -outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct1, r_rol1, r_pit1, r_yaw1;
    double r_thrPct2, r_rol2, r_pit2, r_yaw2;
    double upRatio = constrain(cfg.upThrRatio[0], 0.0f, 2.0f);
    double dnRatio = 2 - upRatio;
    MultiRotor_MotorControl(6, output_throttle, upRatio, outRoll, outPitch, outYaw, &rp_out[0], &yaw_out[0], &rotor_output[0], &r_thrPct1, &r_rol1, &r_pit1, &r_yaw1);
    MultiRotor_MotorControl(6, output_throttle, dnRatio, outRoll, outPitch, outYaw, &rp_out[6], &yaw_out[6], &rotor_output[6], &r_thrPct2, &r_rol2, &r_pit2, &r_yaw2);
    Roll_u = 0.5 * (r_rol1 + r_rol2);
    Pitch_u = 0.5 * (r_pit1 + r_pit2);
    Yaw_u = 0.5 * (r_yaw1 + r_yaw2);
    update_output_throttle(0.5 * (r_thrPct1 + r_thrPct2), 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor62X_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor62X_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};
static void MultiRotor62C_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[12];
    double PitchS = outPitch * 1.1547005383792515290182975610039f;
    double half_outPitch = 0.5f * PitchS;
    rp_out[0] = -PitchS;
    rp_out[1] = -half_outPitch + outRoll;
    rp_out[2] = +half_outPitch + outRoll;
    rp_out[3] = +PitchS;
    rp_out[4] = +half_outPitch - outRoll;
    rp_out[5] = -half_outPitch - outRoll;
    rp_out[6] = -PitchS;
    rp_out[7] = -half_outPitch + outRoll;
    rp_out[8] = +half_outPitch + outRoll;
    rp_out[9] = +PitchS;
    rp_out[10] = +half_outPitch - outRoll;
    rp_out[11] = -half_outPitch - outRoll;
    double yaw_out[12];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +outYaw;
    yaw_out[6] = +outYaw;
    yaw_out[7] = -outYaw;
    yaw_out[8] = +outYaw;
    yaw_out[9] = -outYaw;
    yaw_out[10] = +outYaw;
    yaw_out[11] = -outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct1, r_rol1, r_pit1, r_yaw1;
    double r_thrPct2, r_rol2, r_pit2, r_yaw2;
    double upRatio = constrain(cfg.upThrRatio[0], 0.0f, 2.0f);
    double dnRatio = 2 - upRatio;
    MultiRotor_MotorControl(6, output_throttle, upRatio, outRoll, outPitch, outYaw, &rp_out[0], &yaw_out[0], &rotor_output[0], &r_thrPct1, &r_rol1, &r_pit1, &r_yaw1);
    MultiRotor_MotorControl(6, output_throttle, dnRatio, outRoll, outPitch, outYaw, &rp_out[6], &yaw_out[6], &rotor_output[6], &r_thrPct2, &r_rol2, &r_pit2, &r_yaw2);
    Roll_u = 0.5 * (r_rol1 + r_rol2);
    Pitch_u = 0.5 * (r_pit1 + r_pit2);
    Yaw_u = 0.5 * (r_yaw1 + r_yaw2);
    update_output_throttle(0.5 * (r_thrPct1 + r_thrPct2), 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor62C_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor62C_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor62H_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double RollS = outRoll * 1.1547005383792515290182975610039f;
    double half_outRoll = 0.5f * RollS;
    double rp_out[12];
    rp_out[0] = -outPitch + half_outRoll;
    rp_out[1] = RollS;
    rp_out[2] = +outPitch + half_outRoll;
    rp_out[3] = +outPitch - half_outRoll;
    rp_out[4] = -RollS;
    rp_out[5] = -outPitch - half_outRoll;
    rp_out[6] = -outPitch + half_outRoll;
    rp_out[7] = RollS;
    rp_out[8] = +outPitch + half_outRoll;
    rp_out[9] = +outPitch - half_outRoll;
    rp_out[10] = -RollS;
    rp_out[11] = -outPitch - half_outRoll;
    double yaw_out[12];
    yaw_out[0] = -0.5 * outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -0.5 * outYaw;
    yaw_out[3] = +0.5 * outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +0.5 * outYaw;
    yaw_out[6] = +0.5 * outYaw;
    yaw_out[7] = -outYaw;
    yaw_out[8] = +0.5 * outYaw;
    yaw_out[9] = -0.5 * outYaw;
    yaw_out[10] = +outYaw;
    yaw_out[11] = -0.5 * outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct1, r_rol1, r_pit1, r_yaw1;
    double r_thrPct2, r_rol2, r_pit2, r_yaw2;
    double upRatio = constrain(cfg.upThrRatio[0], 0.0f, 2.0f);
    double dnRatio = 2 - upRatio;
    MultiRotor_MotorControl(6, output_throttle, upRatio, outRoll, outPitch, outYaw, &rp_out[0], &yaw_out[0], &rotor_output[0], &r_thrPct1, &r_rol1, &r_pit1, &r_yaw1);
    MultiRotor_MotorControl(6, output_throttle, dnRatio, outRoll, outPitch, outYaw, &rp_out[6], &yaw_out[6], &rotor_output[6], &r_thrPct2, &r_rol2, &r_pit2, &r_yaw2);
    Roll_u = 0.5 * (r_rol1 + r_rol2);
    Pitch_u = 0.5 * (r_pit1 + r_pit2);
    Yaw_u = 0.5 * (r_yaw1 + r_yaw2);
    update_output_throttle(0.5 * (r_thrPct1 + r_thrPct2), 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor62H_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor62H_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor62H2_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double PitchS = outPitch * 1.1547005383792515290182975610039f;
    double half_Pitch = 0.5f * PitchS;
    double rp_out[12];
    rp_out[0] = -half_Pitch + outRoll;
    rp_out[1] = +half_Pitch + outRoll;
    rp_out[2] = +PitchS;
    rp_out[3] = +half_Pitch - outRoll;
    rp_out[4] = -half_Pitch - outRoll;
    rp_out[5] = -PitchS;
    rp_out[6] = -half_Pitch + outRoll;
    rp_out[7] = +half_Pitch + outRoll;
    rp_out[8] = +PitchS;
    rp_out[9] = +half_Pitch - outRoll;
    rp_out[10] = -half_Pitch - outRoll;
    rp_out[11] = -PitchS;
    double yaw_out[12];
    yaw_out[0] = -0.5 * outYaw;
    yaw_out[1] = +0.5 * outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +0.5 * outYaw;
    yaw_out[4] = -0.5 * outYaw;
    yaw_out[5] = +outYaw;

    yaw_out[6] = +0.5 * outYaw;
    yaw_out[7] = -0.5 * outYaw;
    yaw_out[8] = +outYaw;
    yaw_out[9] = -0.5 * outYaw;
    yaw_out[10] = +0.5 * outYaw;
    yaw_out[11] = -outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct1, r_rol1, r_pit1, r_yaw1;
    double r_thrPct2, r_rol2, r_pit2, r_yaw2;
    double upRatio = constrain(cfg.upThrRatio[0], 0.0f, 2.0f);
    double dnRatio = 2 - upRatio;
    MultiRotor_MotorControl(6, output_throttle, upRatio, outRoll, outPitch, outYaw, &rp_out[0], &yaw_out[0], &rotor_output[0], &r_thrPct1, &r_rol1, &r_pit1, &r_yaw1);
    MultiRotor_MotorControl(6, output_throttle, dnRatio, outRoll, outPitch, outYaw, &rp_out[6], &yaw_out[6], &rotor_output[6], &r_thrPct2, &r_rol2, &r_pit2, &r_yaw2);
    Roll_u = 0.5 * (r_rol1 + r_rol2);
    Pitch_u = 0.5 * (r_pit1 + r_pit2);
    Yaw_u = 0.5 * (r_yaw1 + r_yaw2);
    update_output_throttle(0.5 * (r_thrPct1 + r_thrPct2), 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor62H2_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor62H2_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor8X_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[8];
    rp_out[0] = -outPitch + outRoll;
    rp_out[1] = -outPitch + outRoll;
    rp_out[2] = +outPitch + outRoll;
    rp_out[3] = +outPitch + outRoll;
    rp_out[4] = +outPitch - outRoll;
    rp_out[5] = +outPitch - outRoll;
    rp_out[6] = -outPitch - outRoll;
    rp_out[7] = -outPitch - outRoll;
    double yaw_out[8];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +outYaw;
    yaw_out[6] = -outYaw;
    yaw_out[7] = +outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct, r_rol, r_pit, r_yaw;
    MultiRotor_MotorControl(8, output_throttle, 1, outRoll, outPitch, outYaw, rp_out, yaw_out, rotor_output, &r_thrPct, &r_rol, &r_pit, &r_yaw);
    Roll_u = r_rol;
    Pitch_u = r_pit;
    Yaw_u = r_yaw;
    update_output_throttle(r_thrPct, 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor8X_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor8X_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor6H_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[6];
    double RollS = outRoll * 1.1547005383792515290182975610039f;
    double half_outRoll = 0.5f * RollS;
    rp_out[0] = -outPitch + half_outRoll;
    rp_out[1] = RollS;
    rp_out[2] = +outPitch + half_outRoll;
    rp_out[3] = +outPitch - half_outRoll;
    rp_out[4] = -RollS;
    rp_out[5] = -outPitch - half_outRoll;
    double yaw_out[6];
    yaw_out[0] = -0.5 * outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -0.5 * outYaw;
    yaw_out[3] = +0.5 * outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +0.5 * outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct, r_rol, r_pit, r_yaw;
    MultiRotor_MotorControl(6, output_throttle, 1, outRoll, outPitch, outYaw, rp_out, yaw_out, rotor_output, &r_thrPct, &r_rol, &r_pit, &r_yaw);
    Roll_u = r_rol;
    Pitch_u = r_pit;
    Yaw_u = r_yaw;
    update_output_throttle(r_thrPct, 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor6H_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor6H_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor42X_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[8];
    rp_out[0] = -outPitch + outRoll;
    rp_out[1] = +outPitch + outRoll;
    rp_out[2] = +outPitch - outRoll;
    rp_out[3] = -outPitch - outRoll;
    rp_out[4] = -outPitch + outRoll;
    rp_out[5] = +outPitch + outRoll;
    rp_out[6] = +outPitch - outRoll;
    rp_out[7] = -outPitch - outRoll;
    double yaw_out[8];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = +outYaw;
    yaw_out[5] = -outYaw;
    yaw_out[6] = +outYaw;
    yaw_out[7] = -outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct1, r_rol1, r_pit1, r_yaw1;
    double r_thrPct2, r_rol2, r_pit2, r_yaw2;
    double upRatio = constrain(cfg.upThrRatio[0], 0.0f, 2.0f);
    double dnRatio = 2 - upRatio;
    MultiRotor_MotorControl(4, output_throttle, upRatio, outRoll, outPitch, outYaw, &rp_out[0], &yaw_out[0], &rotor_output[0], &r_thrPct1, &r_rol1, &r_pit1, &r_yaw1);
    MultiRotor_MotorControl(4, output_throttle, dnRatio, outRoll, outPitch, outYaw, &rp_out[4], &yaw_out[4], &rotor_output[4], &r_thrPct2, &r_rol2, &r_pit2, &r_yaw2);
    Roll_u = 0.5 * (r_rol1 + r_rol2);
    Pitch_u = 0.5 * (r_pit1 + r_pit2);
    Yaw_u = 0.5 * (r_yaw1 + r_yaw2);
    update_output_throttle(0.5 * (r_thrPct1 + r_thrPct2), 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor42X_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor42X_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor82X_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[16];
    rp_out[0] = -outPitch + outRoll;
    rp_out[1] = -outPitch + outRoll;
    rp_out[2] = +outPitch + outRoll;
    rp_out[3] = +outPitch + outRoll;
    rp_out[4] = +outPitch - outRoll;
    rp_out[5] = +outPitch - outRoll;
    rp_out[6] = -outPitch - outRoll;
    rp_out[7] = -outPitch - outRoll;
    rp_out[8] = -outPitch + outRoll;
    rp_out[9] = -outPitch + outRoll;
    rp_out[10] = +outPitch + outRoll;
    rp_out[11] = +outPitch + outRoll;
    rp_out[12] = +outPitch - outRoll;
    rp_out[13] = +outPitch - outRoll;
    rp_out[14] = -outPitch - outRoll;
    rp_out[15] = -outPitch - outRoll;
    double yaw_out[16];
    yaw_out[0] = -outYaw;
    yaw_out[1] = -outYaw;
    yaw_out[2] = +outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = -outYaw;
    yaw_out[6] = +outYaw;
    yaw_out[7] = +outYaw;
    yaw_out[8] = +outYaw;
    yaw_out[9] = +outYaw;
    yaw_out[10] = -outYaw;
    yaw_out[11] = -outYaw;
    yaw_out[12] = +outYaw;
    yaw_out[13] = +outYaw;
    yaw_out[14] = -outYaw;
    yaw_out[15] = -outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct1, r_rol1, r_pit1, r_yaw1;
    double r_thrPct2, r_rol2, r_pit2, r_yaw2;
    double upRatio = constrain(cfg.upThrRatio[0], 0.0f, 2.0f);
    double dnRatio = 2 - upRatio;
    MultiRotor_MotorControl(8, output_throttle, upRatio, outRoll, outPitch, outYaw, &rp_out[0], &yaw_out[0], &rotor_output[0], &r_thrPct1, &r_rol1, &r_pit1, &r_yaw1);
    MultiRotor_MotorControl(8, output_throttle, dnRatio, outRoll, outPitch, outYaw, &rp_out[8], &yaw_out[8], &rotor_output[8], &r_thrPct2, &r_rol2, &r_pit2, &r_yaw2);
    Roll_u = 0.5 * (r_rol1 + r_rol2);
    Pitch_u = 0.5 * (r_pit1 + r_pit2);
    Yaw_u = 0.5 * (r_yaw1 + r_yaw2);
    update_output_throttle(0.5 * (r_thrPct1 + r_thrPct2), 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor82X_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor82X_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};

static void MultiRotor82X2_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double rp_out[16];
    rp_out[0] = -outPitch + outRoll;
    rp_out[1] = -outPitch + outRoll;
    rp_out[2] = +outPitch + outRoll;
    rp_out[3] = +outPitch + outRoll;
    rp_out[4] = +outPitch - outRoll;
    rp_out[5] = +outPitch - outRoll;
    rp_out[6] = -outPitch - outRoll;
    rp_out[7] = -outPitch - outRoll;
    rp_out[8] = -outPitch + outRoll;
    rp_out[9] = -outPitch + outRoll;
    rp_out[10] = +outPitch + outRoll;
    rp_out[11] = +outPitch + outRoll;
    rp_out[12] = +outPitch - outRoll;
    rp_out[13] = +outPitch - outRoll;
    rp_out[14] = -outPitch - outRoll;
    rp_out[15] = -outPitch - outRoll;
    double yaw_out[16];
    yaw_out[0] = -outYaw;
    yaw_out[1] = +outYaw;
    yaw_out[2] = -outYaw;
    yaw_out[3] = +outYaw;
    yaw_out[4] = -outYaw;
    yaw_out[5] = +outYaw;
    yaw_out[6] = -outYaw;
    yaw_out[7] = +outYaw;
    yaw_out[8] = +outYaw;
    yaw_out[9] = -outYaw;
    yaw_out[10] = +outYaw;
    yaw_out[11] = -outYaw;
    yaw_out[12] = +outYaw;
    yaw_out[13] = -outYaw;
    yaw_out[14] = +outYaw;
    yaw_out[15] = -outYaw;

    double rotor_output[MAX_PWM_COUNT];
    double r_thrPct1, r_rol1, r_pit1, r_yaw1;
    double r_thrPct2, r_rol2, r_pit2, r_yaw2;
    double upRatio = constrain(cfg.upThrRatio[0], 0.0f, 2.0f);
    double dnRatio = 2 - upRatio;
    MultiRotor_MotorControl(8, output_throttle, upRatio, outRoll, outPitch, outYaw, &rp_out[0], &yaw_out[0], &rotor_output[0], &r_thrPct1, &r_rol1, &r_pit1, &r_yaw1);
    MultiRotor_MotorControl(8, output_throttle, dnRatio, outRoll, outPitch, outYaw, &rp_out[8], &yaw_out[8], &rotor_output[8], &r_thrPct2, &r_rol2, &r_pit2, &r_yaw2);
    Roll_u = 0.5 * (r_rol1 + r_rol2);
    Pitch_u = 0.5 * (r_pit1 + r_pit2);
    Yaw_u = 0.5 * (r_yaw1 + r_yaw2);
    update_output_throttle(0.5 * (r_thrPct1 + r_thrPct2), 1.0 / CtrlRateHz);
    MainMotor_PWM_Out(rotor_output);
}
static const UAVCtrlM MultiRotor82X2_CtrlM = {
    MultiRotor_Init,
    MultiRotor_DeInit,
    MultiRotor_PreArmControl,
    MultiRotor_InitControl,
    MultiRotor82X2_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数 偏航控加速度
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};
/*多旋翼*/

/*三轴*/
static void TriRotor_Init()
{
    // 初始化姿态ESO
    ((ESO_AngularRate *) CtrlM_ESO[0])
        ->init(cfg.order[0], cfg.T[0], cfg.b[0], cfg.beta[0], cfg.beta[0], cfg.orderD[0], cfg.betaD[0], cfg.rolBS[0], cfg.rolBSBW[0], cfg.rolBS2[0], cfg.rolBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxRP[0]);
    ((ESO_AngularRate *) CtrlM_ESO[1])
        ->init(cfg.order[0], cfg.T[0], cfg.b[2], cfg.beta[0], cfg.beta[0], cfg.orderD[0], cfg.betaD[0], cfg.pitBS[0], cfg.pitBSBW[0], cfg.pitBS2[0], cfg.pitBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxRP[0]);
    ((ESO_AngularRate *) CtrlM_ESO[2])
        ->init(cfg.order[0], cfg.T2[0], cfg.b[4], cfg.beta2[0], cfg.beta2[0], cfg.orderD2[0], cfg.betaD2[0], cfg.yawBS[0], cfg.yawBSBW[0], cfg.yawBS2[0], cfg.yawBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxYaw[0]);

    // 最低油门
    cfg.minThrottlePct = 0;
    // 怠速油门
    cfg.idleThrottlePct = 0;
}
static void TriRotor_DeInit() {}

static void TriRotor_PreArmControl(Receiver rc)
{
    bool needMotorTest = false;
    if (motorTestEndTime.is_valid() == false)
    {
    }
    else if (motorTestEndTime.get_pass_time() > 0 || motorTestMt == 0)
    {
        motorTestEndTime.set_invalid();
    }
    else
        needMotorTest = true;

    if (!needMotorTest)
    {
        double pwmout[6];
        pwmout[0] = pwmout[1] = pwmout[2] = pwmout[3] = 0;
        if (rc.available)
            pwmout[4] = pwmout[5] = 2.0 * (50.0 - rc.data[1]);
        else
            pwmout[4] = pwmout[5] = 0;
        ST_PWMMap(pwmout, 4, 2, STMapType_symmetry);
        MainMotor_PWM_Out(pwmout);
        MainMotor_PullDownAll();
    }
    else
    { // 电机测试
        UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
        double pwm_out[MAX_PWM_COUNT] = {0};
        for (uint8_t i = 0; i < mt_count.MTCount; ++i)
        {
            if (motorTestMt & (1 << i))
                pwm_out[i] = cfg.STThrottle[0];
        }
        if (rc.available)
            pwm_out[4] = pwm_out[5] = 2.0 * (50.0 - rc.data[1]);
        else
            pwm_out[4] = pwm_out[5] = 0;
        ST_PWMMap(pwm_out, 4, 2, STMapType_symmetry);
        MainMotor_PWM_Out(pwm_out);
    }
}

static void TriRotor_InitControl(bool DbgSafe)
{
    if (DbgSafe)
    {
        MainMotor_PullDownAll();
        StartCounter = 0;
    }

    // 启动初始化
    if (StartCounter < 1e6)
    { // 还未进行启动初始化
        if (StartCounter >= 0)
        { // 启动动作
            switch (cfg.Mode[0] & 0xff)
            { // 顺序起转电机
                case 1:
                {
                    if (StartCounter < 0.3 * 3 * CtrlRateHz)
                    {
                        double pwm_out[MAX_PWM_COUNT] = {0};
                        for (uint8_t i = 0; i < 3; ++i)
                        {
                            if (StartCounter > 0.3 * i * CtrlRateHz)
                                pwm_out[i] = cfg.STThrottle[0];
                            else
                                break;
                        }
                        pwm_out[4] = pwm_out[5] = 0;
                        ST_PWMMap(pwm_out, 4, 2, STMapType_symmetry);
                        MainMotor_PWM_Out(pwm_out);
                        ++StartCounter;
                    }
                    else
                        StartCounter = -1;
                    break;
                }

                default:
                {
                    double pwm_out[MAX_PWM_COUNT] = {0};
                    for (uint8_t i = 0; i < 3; ++i)
                        pwm_out[i] = cfg.STThrottle[0];
                    pwm_out[4] = pwm_out[5] = 0;
                    ST_PWMMap(pwm_out, 4, 2, STMapType_symmetry);
                    MainMotor_PWM_Out(pwm_out);
                    StartCounter = -1;
                    break;
                }
            }
        }
        else
        { // 启动延时
            if ((-StartCounter) > cfg.STDelay[0] * CtrlRateHz)
                StartCounter = 1e9;
            else
                --StartCounter;
        }
        return;
    }
}

static void TriRotorX_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    // 计算增益修正系数
    double bat_b_scale = 1.0;
    BatteryInfo batInfo;
    int8_t current_batId = -1;
    getCurrentBatteryInfo(&batInfo, &current_batId);
    if (batId >= 0 && current_batId == batId)
    { // 获取到电池
        if ((cfg.Mode[0] & ATTCTRL_MODE_BATCOMP_DIS_BIT) == 0 && batStVolt > 7 && batInfo.totalVoltRawFilted > 7)
            bat_b_scale = constrain(batInfo.totalVoltRawFilted / batStVolt, 0.35f, 3.0f);
    }

    // 海拔推力补偿
    double baro_b_scale = 1.0;
    extern float INTERNAL_TEMPERATURE;
    double tempK = INTERNAL_TEMPERATURE + C_TO_KELVIN;
    extern float INTERNAL_PRESSURE;
    // 加入气压范围保护
    if (tempK > 100 && INTERNAL_PRESSURE > 2500.0f && INTERNAL_PRESSURE < 200000.0f)
    {
        double eas2tas_squared = SSL_AIR_DENSITY / (INTERNAL_PRESSURE / (ISA_GAS_CONSTANT * tempK));
        if (eas2tas_squared > 0.7 && eas2tas_squared < 10.0)
            baro_b_scale = 1.0 / eas2tas_squared;
    }
    // 钳制 baro_b_scale 范围
    baro_b_scale = constrain(baro_b_scale, 0.2, 3.0);

    // 根据电池电压调整控制对象增益
    double b_scale = bat_b_scale * baro_b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[0])->b = cfg.b[0] * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[1])->b = cfg.b[2] * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[2])->b = cfg.b[4] * b_scale;

    double rp_out[3];
    rp_out[0] = -0.5 * outPitch + outRoll;
    rp_out[1] = +outPitch;
    rp_out[2] = -0.5 * outPitch - outRoll;

    double rotor_output[12];
    double output_minimum_throttle = cfg.STThrottle[0];

    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double output_range = 100.0 - output_minimum_throttle;
    double output_midpoint = output_range / 2;

    // 油门百分比转换为实际油门量
    output_throttle = MultiRotor_percent_to_throttle(output_throttle, output_minimum_throttle, output_range);

    /*pitch roll 输出限幅*/
    // 如果需要的pitch roll输出超出当前油门能提供的输出范围
    // 调整油门获得尽量满足pitch roll输出
    double output_max = fabs(rp_out[0]);
    for (uint8_t i = 1; i < 3; ++i)
    {
        double abs_out = fabs(rp_out[i]);
        if (abs_out > output_max)
            output_max = abs_out;
    }

    double max_allow_output = 100.0f - output_throttle;
    double min_allow_output = output_minimum_throttle - output_throttle;
    double allow_ouput_range;
    if (max_allow_output < -min_allow_output)
    { // 降低油门确保姿态输出
        allow_ouput_range = max_allow_output;
        if (output_max > allow_ouput_range)
        { // 需要降低油门
            if (output_max > output_midpoint)
            { // 输出超过最大输出范围
                // 将油门调整为50%确保可以进行最大输出
                output_throttle = output_midpoint + output_minimum_throttle;
                allow_ouput_range = output_midpoint;
            }
            else
            { // 降低油门到所需值
                output_throttle = 100.0f - output_max;
                allow_ouput_range = output_max;
            }
        }
    }
    else
    { // 抬高油门保证姿态输出
        allow_ouput_range = -min_allow_output;
        if (output_max > allow_ouput_range)
        { // 需要抬高油门

            // 求最高允许的油门值（不能大于悬停油门）
            double hover_throttle_force = MultiRotor_percent_to_throttle(hover_throttle, output_minimum_throttle, output_range) - output_minimum_throttle;
            double max_allowed_output_range = hover_throttle_force * 0.85;
            if (output_midpoint < max_allowed_output_range)
                max_allowed_output_range = output_midpoint;
            if (max_allowed_output_range < output_throttle - output_minimum_throttle)
                max_allowed_output_range = output_throttle - output_minimum_throttle;
            double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;

            if (output_max > max_allowed_output_range)
            { // 输出范围大于最大允许范围
                // 抬高油门至最大允许范围
                output_throttle = max_allowed_throttle;
                allow_ouput_range = max_allowed_output_range;
            }
            else
            { // 抬高油门到所需值
                output_throttle = output_minimum_throttle + output_max;
                allow_ouput_range = output_max;
            }
        }
    }

    // 输出限幅修正
    if (output_max > allow_ouput_range)
    { // 需要修正输出
        double scale = allow_ouput_range / output_max;
        for (uint8_t i = 0; i < 3; ++i)
            rotor_output[i] = rp_out[i] * scale;
        Roll_u = outRoll * scale;
        Pitch_u = outPitch * scale;
    }
    else
    {
        for (uint8_t i = 0; i < 3; ++i)
            rotor_output[i] = rp_out[i];
        Roll_u = outRoll;
        Pitch_u = outPitch;
    }
    /*pitch roll 输出限幅*/

    rotor_output[3] = 0;
    /*yaw输出限幅*/
    double absYaw = fabs(outYaw);
    if (absYaw > 50)
        outYaw *= 50.0 / absYaw;
    rotor_output[4] = rotor_output[5] = 2.0 * outYaw;
    Yaw_u = outYaw;
    /*yaw输出限幅*/

    // 更新油门油门观测
    double throttle_percent = (output_throttle - output_minimum_throttle) / output_range * 100;
    update_output_throttle(throttle_percent, 1.0 / CtrlRateHz);
    // 补偿非线性输出
    for (uint8_t i = 0; i < 3; ++i)
        rotor_output[i] += output_throttle;
    throttle_nonlinear_compensation(3, rotor_output);
    ST_PWMMap(rotor_output, 4, 2, STMapType_symmetry);
    MainMotor_PWM_Out(rotor_output);
}

static const UAVCtrlM TriRotorX_CtrlM = {
    TriRotor_Init,
    TriRotor_DeInit,
    TriRotor_PreArmControl,
    TriRotor_InitControl,
    TriRotorX_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    0,     // 横滚俯仰控推力导数
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};
/*三轴*/

/*直升机*/
#define HeliIdleThrottle -15
#define Heli_RPb_scale 0.04
static inline double Heli131_Map(double output_throttle, double outRoll, double outPitch, double outYaw, double out[], double *outRoll_u, double *outPitch_u, double *outYaw_u)
{
    if (Attitude_Control_Enabled == false)
    {
        // 计算最低油门
        bool first = true;
        cfg.minThrottlePct = 0;
        for (uint8_t i = 2; i < 5; ++i)
        {
            float st_min = getStMin(i);
            float st_mid = getStMid(i);
            float st_max = getStMax(i);

            float scale;
            if (fabsf(st_max - st_mid) > fabsf(st_mid - st_min))
                scale = st_max - st_mid;
            else
                scale = st_mid - st_min;

            if (!is_zero(scale))
            {
                float new_minThrottlePct = (st_min - st_mid) / scale;
                if (first)
                {
                    first = false;
                    cfg.minThrottlePct = new_minThrottlePct;
                }
                else if (new_minThrottlePct > cfg.minThrottlePct)
                    cfg.minThrottlePct = new_minThrottlePct;
            }
        }
        cfg.minThrottlePct *= 100;
        // 怠速油门
        cfg.idleThrottlePct = HeliIdleThrottle;
        if (cfg.idleThrottlePct < cfg.minThrottlePct)
            cfg.idleThrottlePct = cfg.minThrottlePct;
    }

    float output_minimum_throttle = cfg.minThrottlePct;
    double output_midpoint = (100.0f + output_minimum_throttle) / 2;

    // if( output_throttle < 0.1f )
    if (0)
    {
        out[0] = out[1] = out[2] = output_throttle;
    }
    else
    { // 横滚俯仰动力分配
        out[0] = +outRoll - 0.5 * outPitch;
        out[1] = +outPitch;
        out[2] = -outRoll - 0.5 * outPitch;

        // 如果需要的pitch roll输出超出当前油门能提供的输出范围
        // 调整油门获得尽量满足pitch roll输出
        double output_max = fabs(out[0]);
        for (uint8_t i = 0; i < 3; ++i)
        {
            double abs_out = fabs(out[i]);
            if (abs_out > output_max)
                output_max = abs_out;
        }

        double max_allow_output = 100.0f - output_throttle;
        double min_allow_output = output_minimum_throttle - output_throttle;
        double allow_ouput_range;
        if (max_allow_output < -min_allow_output)
        { // 降低油门确保姿态输出
            allow_ouput_range = max_allow_output;
            if (output_max > allow_ouput_range)
            { // 需要降低油门
                if (output_max > 100.0f - output_midpoint)
                { // 输出超过最大输出范围
                    // 将油门调整为50%确保可以进行最大输出
                    output_throttle = output_midpoint;
                    allow_ouput_range = 100.0f - output_midpoint;
                }
                else
                { // 降低油门到所需值
                    output_throttle = 100.0f - output_max;
                    allow_ouput_range = output_max;
                }
            }
        }
        else
        { // 抬高油门保证姿态输出
            allow_ouput_range = -min_allow_output;
            if (output_max > allow_ouput_range)
            { // 需要抬高油门

                // 求最高允许的油门值（不能大于悬停油门）
                double hover_throttle_force = hover_throttle;
                double max_allowed_throttle = hover_throttle_force * 0.85;
                if (output_midpoint < max_allowed_throttle)
                    max_allowed_throttle = output_midpoint;
                if (max_allowed_throttle < output_throttle)
                    max_allowed_throttle = output_throttle;
                double max_allowed_output_range = max_allowed_throttle - output_minimum_throttle;

                if (output_max > max_allowed_output_range)
                { // 输出范围大于最大允许范围
                    // 抬高油门至最大允许范围
                    output_throttle = max_allowed_throttle;
                    allow_ouput_range = max_allowed_output_range;
                }
                else
                { // 抬高油门到所需值
                    output_throttle = output_max + output_minimum_throttle;
                    allow_ouput_range = output_max;
                }
            }
        }

        // 输出限幅修正
        if (output_max > allow_ouput_range)
        { // 需要修正输出
            double scale = allow_ouput_range / output_max;
            for (uint8_t i = 0; i < 3; ++i)
                out[i] *= scale;
            if (outRoll_u)
                *outRoll_u = outRoll * scale;
            if (outPitch_u)
                *outPitch_u = outPitch * scale;
        }
        else
        {
            if (outRoll_u)
                *outRoll_u = outRoll;
            if (outPitch_u)
                *outPitch_u = outPitch;
        }

        // 加上油门
        for (uint8_t i = 0; i < 3; ++i)
            out[i] += output_throttle;
    }

    /*yaw output 输出限幅*/
    // 抬升油门保住偏航输出

    /*yaw输出限幅计算*/
    float st_min = getStMin(5);
    float st_mid = getStMid(5);
    float st_max = getStMax(5);
    double maxYawOut, minYawOut;
    if (fabsf(st_max - st_mid) > fabsf(st_mid - st_min))
    {
        minYawOut = -50 * fabsf(st_mid - st_min) / fabsf(st_max - st_mid);
        maxYawOut = 50;
    }
    else
    {
        minYawOut = -50;
        maxYawOut = 50 * fabsf(st_max - st_mid) / fabsf(st_mid - st_min);
    }
    if (outYaw < minYawOut)
        outYaw = minYawOut;
    if (outYaw > maxYawOut)
        outYaw = maxYawOut;

    if (outYaw_u)
        *outYaw_u = outYaw;
    out[3] = outYaw * 2;
    /*yaw output 输出限幅*/

    // 更新油门油门观测
    return output_throttle;
}

static void Heli131_Init()
{
    // 初始化姿态ESO
    ((ESO_AngularRateHeli *) CtrlM_ESO[0])->init(cfg.order[0], cfg.T[0], cfg.b[0] * Heli_RPb_scale, cfg.beta[0], cfg.orderD[0], cfg.betaD[0], CtrlRateHz * CtrlRateDiv);
    ((ESO_AngularRateHeli *) CtrlM_ESO[1])->init(cfg.order[0], cfg.T[0], cfg.b[2] * Heli_RPb_scale, cfg.beta[0], cfg.orderD[0], cfg.betaD[0], CtrlRateHz * CtrlRateDiv);
    ((ESO_AngularRate *) CtrlM_ESO[2])
        ->init(cfg.order2[0], cfg.T2[0], cfg.b[4], cfg.beta2[0], cfg.beta2[0], cfg.orderD2[0], cfg.betaD2[0], cfg.yawBS[0], cfg.yawBSBW[0], cfg.yawBS2[0], cfg.yawBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxYaw[0]);
}
static void Heli131_DeInit() {}

static void Heli131_PreArmControl(Receiver rc)
{
    double pwmout[6];
    // 电机关闭
    pwmout[0] = pwmout[1] = 0;
    if (rc.available && attController_DisableTime.get_pass_time() > 15)
    {
        float thr;
        if (rc.data[0] > 50)
            thr = (rc.data[0] - 50) * 2;
        else if (rc.data[0] > 30)
            thr = 0;
        else if (rc.data[0] > 10)
            thr = cfg.idleThrottlePct;
        else
            thr = cfg.minThrottlePct;
        Heli131_Map(thr, rc.data[3] - 50, rc.data[2] - 50, 50 - rc.data[1], &pwmout[2], 0, 0, 0);
    }
    else
        Heli131_Map(cfg.idleThrottlePct, 0, 0, 0, &pwmout[2], 0, 0, 0);
    ST_PWMMap(pwmout, 2, 4, STMapType_standard);
    MainMotor_PWM_Out(pwmout);
    MainMotor_PullDownAll();
}

static void Heli131_InitControl(bool DbgSafe)
{
    if (DbgSafe)
    {
        MainMotor_PullDownAll();
        StartCounter = 0;
    }

    ++StartCounter;
    if (StartCounter > cfg.STDelay[0] * CtrlRateHz)
        set_Ctrl_inited;
}

static void Heli131_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    //			if( DbgSafe )
    //			{
    //				double pwmout[6];
    //				//电机关闭
    //				pwmout[0] = pwmout[1] = 0;
    //				Receiver rc;
    //				getReceiver(&rc);
    //				double thr;
    //				if( rc.available )
    //				{
    //					if( rc.data[0] > 25 )
    //						thr = 0;
    //					else
    //						thr = HeliIdleThrottle;
    //					Heli131_Map( thr, rc.data[3]-50 , rc.data[2]-50 , 50-rc.data[1], &pwmout[2], 0,0,0 );
    //				}
    //				else
    //				{
    //					thr = 0;
    //					Heli131_Map( thr, 0 , 0 , 0, &pwmout[2], 0,0,0 );
    //				}
    //				ST_PWMMap( pwmout, 2, 4 );
    //				MainMotor_PWM_Out( pwmout );
    //				update_output_throttle( thr , 1.0/CtrlRateHz );
    //				return;
    //			}

    // 计算增益修正系数
    double bat_b_scale = 1.0;
    BatteryInfo batInfo;
    int8_t current_batId = -1;
    getCurrentBatteryInfo(&batInfo, &current_batId);
    if (batId >= 0 && current_batId == batId)
    { // 获取到电池
        if ((cfg.Mode[0] & ATTCTRL_MODE_BATCOMP_DIS_BIT) == 0 && batStVolt > 7 && batInfo.totalVoltRawFilted > 7)
            bat_b_scale = constrain(batInfo.totalVoltRawFilted / batStVolt, 0.35f, 3.0f);
    }

    // 海拔推力补偿
    double baro_b_scale = 1.0;
    extern float INTERNAL_TEMPERATURE;
    double tempK = INTERNAL_TEMPERATURE + C_TO_KELVIN;
    extern float INTERNAL_PRESSURE;
    // 加入气压范围保护
    if (tempK > 100 && INTERNAL_PRESSURE > 2500.0f && INTERNAL_PRESSURE < 200000.0f)
    {
        double eas2tas_squared = SSL_AIR_DENSITY / (INTERNAL_PRESSURE / (ISA_GAS_CONSTANT * tempK));
        if (eas2tas_squared > 0.7 && eas2tas_squared < 10.0)
            baro_b_scale = 1.0 / eas2tas_squared;
    }
    // 钳制 baro_b_scale 范围
    baro_b_scale = constrain(baro_b_scale, 0.2, 3.0);

    // 根据电池电压调整控制对象增益
    double b_scale = bat_b_scale * baro_b_scale;
    ((ESO_AngularRateHeli *) CtrlM_ESO[0])->b = cfg.b[0] * Heli_RPb_scale * b_scale;
    ((ESO_AngularRateHeli *) CtrlM_ESO[1])->b = cfg.b[2] * Heli_RPb_scale * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[2])->b = cfg.b[4] * b_scale;

    double pwmout[6];
    // 求电机输出
    float fly_pwm = (getStMid(0) - 1000) * 0.1f;
    float yaw_thr_min = (getStMin(5) - 1000) * 0.1f;
    float yaw_thr_mid = (getStMid(5) - 1000) * 0.1f;
    float yaw_thr_max = (getStMax(5) - 1000) * 0.1f;
    if (Ctrl_initing)
    {
        output_throttle = HeliIdleThrottle;
        pwmout[0] = ((float) StartCounter / (cfg.STDelay[0] * CtrlRateHz)) * (fly_pwm - cfg.STThrottle[0]) + cfg.STThrottle[0];
        outYaw = 0;
        pwmout[1] = ((float) StartCounter / (cfg.STDelay[0] * CtrlRateHz)) * (yaw_thr_mid - yaw_thr_min) + yaw_thr_min;
    }
    else
    {
        pwmout[0] = fly_pwm;
    }
    if (DbgSafe)
    {
        outYaw = 0;
    }

    // 计算舵机输出
    output_throttle = Heli131_Map(output_throttle, outRoll, outPitch, outYaw, &pwmout[2], &Roll_u, &Pitch_u, &Yaw_u);
    ST_PWMMap(pwmout, 2, 4, STMapType_standard);

    // 电机输出
    if (!Ctrl_initing)
    {
        pwmout[1] = Yaw_u + yaw_thr_mid;
        if (pwmout[1] < yaw_thr_min)
            pwmout[1] = yaw_thr_min;
        if (pwmout[1] > yaw_thr_max)
            pwmout[1] = yaw_thr_max;
    }
    if (DbgSafe)
    {
        pwmout[0] = pwmout[1] = 0;
    }
    MainMotor_PWM_Out(pwmout);
    update_output_throttle(output_throttle, 1.0 / CtrlRateHz);
}

static const UAVCtrlM Heli131_CtrlM = {
    Heli131_Init,
    Heli131_DeInit,
    Heli131_PreArmControl,
    Heli131_InitControl,
    Heli131_MotorControl,
    1,
    0, // 横滚俯仰二阶eso
    6,
    0,    // 横滚俯仰控推力导数
    true, // 初始化时进行控制
    true, // 起飞前不添加扰动
};
/*直升机*/

/*共轴双桨 M2S2*/
static void CoaxialM2S2_Init()
{
    // 初始化姿态ESO
    //			ESOHeli[0].init( cfg.T[0], cfg.b[0]*Heli_RPb_scale, cfg.beta[0], CtrlRateHz*CtrlRateDiv );
    //			ESOHeli[1].init( cfg.T[0], cfg.b[2]*Heli_RPb_scale, cfg.beta[0], CtrlRateHz*CtrlRateDiv );
    ((ESO_AngularRate *) CtrlM_ESO[0])
        ->init(cfg.order[0], cfg.T2[0], cfg.b[0], cfg.beta[0], cfg.beta[0], cfg.orderD[0], cfg.betaD[0], cfg.rolBS[0], cfg.rolBSBW[0], cfg.rolBS2[0], cfg.rolBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxRP[0]);
    ((ESO_AngularRate *) CtrlM_ESO[1])
        ->init(cfg.order[0], cfg.T2[0], cfg.b[2], cfg.beta[0], cfg.beta[0], cfg.orderD[0], cfg.betaD[0], cfg.pitBS[0], cfg.pitBSBW[0], cfg.pitBS2[0], cfg.pitBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxRP[0]);
    ((ESO_AngularRate *) CtrlM_ESO[2])
        ->init(cfg.order2[0], 1.0 / (CtrlRateHz * CtrlRateDiv), cfg.b[4], cfg.beta2[0], cfg.beta2[0], cfg.orderD2[0], cfg.betaD2[0], cfg.yawBS[0], cfg.yawBSBW[0], cfg.yawBS2[0], cfg.yawBSBW2[0],
               CtrlRateHz * CtrlRateDiv, cfg.aMaxYaw[0]);

    // 最小油门
    cfg.minThrottlePct = 0;
    // 怠速油门
    cfg.idleThrottlePct = 0;
}
static void CoaxialM2S2_DeInit() {}

static inline void CoaxialM2S2_Map(double outRoll, double outPitch, double pwmout[4]) {}

static void CoaxialM2S2_PreArmControl(Receiver rc)
{
    double pwmout[4];
    pwmout[0] = pwmout[1] = 0;
    if (rc.available)
    {
        pwmout[2] = (rc.data[3] - 50) * 2;
        pwmout[3] = (rc.data[2] - 50) * 2;
    }
    else
    {
        pwmout[2] = 0;
        pwmout[3] = 0;
    }
    ST_PWMMap(pwmout, 2, 2, STMapType_symmetry);
    MainMotor_PWM_Out(pwmout);
    MainMotor_PullDownAll();
}

static void CoaxialM2S2_InitControl(bool DbgSafe)
{
    if (DbgSafe)
    {
        MainMotor_PullDownAll();
        StartCounter = 0;
    }

    // 启动初始化
    if (StartCounter < 1e6)
    { // 还未进行启动初始化
        if (StartCounter >= 0)
        { // 启动动作
            switch (cfg.Mode[0] & 0xff)
            { // 顺序起转电机
                case 1:
                {
                    if (StartCounter < 0.3 * 2 * CtrlRateHz)
                    {
                        double pwm_out[MAX_PWM_COUNT] = {0};
                        pwm_out[2] = pwm_out[3] = 0;
                        for (uint8_t i = 0; i < 2; ++i)
                        {
                            if (StartCounter > 0.3 * i * CtrlRateHz)
                                pwm_out[i] = cfg.STThrottle[0];
                            else
                                break;
                        }
                        ST_PWMMap(pwm_out, 2, 2, STMapType_symmetry);
                        MainMotor_PWM_Out(pwm_out);
                        ++StartCounter;
                    }
                    else
                        StartCounter = -1;
                    break;
                }

                default:
                {
                    double pwm_out[MAX_PWM_COUNT] = {0};
                    pwm_out[2] = pwm_out[3] = 0;
                    for (uint8_t i = 0; i < 2; ++i)
                        pwm_out[i] = cfg.STThrottle[0];
                    ST_PWMMap(pwm_out, 2, 2, STMapType_symmetry);
                    MainMotor_PWM_Out(pwm_out);
                    StartCounter = -1;
                    break;
                }
            }
        }
        else
        { // 启动延时
            if ((-StartCounter) > cfg.STDelay[0] * CtrlRateHz)
                StartCounter = 1e9;
            else
                --StartCounter;
        }
        return;
    }
}

static void CoaxialM2S2_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    double h = 1.0 / CtrlRateHz;
    if (DbgSafe)
    {
        double pwmout[4];
        pwmout[0] = pwmout[1] = 0;
        pwmout[2] = 0;
        pwmout[3] = 0;
        ST_PWMMap(pwmout, 2, 2, STMapType_symmetry);
        MainMotor_PWM_Out(pwmout);
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    // 计算增益修正系数
    double bat_b_scale = 1.0;
    BatteryInfo batInfo;
    int8_t current_batId = -1;
    getCurrentBatteryInfo(&batInfo, &current_batId);
    if (batId >= 0 && current_batId == batId)
    { // 获取到电池
        if ((cfg.Mode[0] & ATTCTRL_MODE_BATCOMP_DIS_BIT) == 0 && batStVolt > 7 && batInfo.totalVoltRawFilted > 7)
            bat_b_scale = constrain(batInfo.totalVoltRawFilted / batStVolt, 0.35f, 3.0f);
    }

    //			float batVolt = 0;
    //			if( batStVolt>7 && getCurrentBatteryTotalVoltRawFilted(&batVolt) && batVolt>7 )
    //				bat_b_scale = constrain( batVolt / batStVolt, 0.35f, 3.0f );

    // 海拔推力补偿
    double baro_b_scale = 1.0;
    extern float INTERNAL_TEMPERATURE;
    double tempK = INTERNAL_TEMPERATURE + C_TO_KELVIN;
    extern float INTERNAL_PRESSURE;
    // 加入气压范围保护
    if (tempK > 100 && INTERNAL_PRESSURE > 2500.0f && INTERNAL_PRESSURE < 200000.0f)
    {
        double eas2tas_squared = SSL_AIR_DENSITY / (INTERNAL_PRESSURE / (ISA_GAS_CONSTANT * tempK));
        if (eas2tas_squared > 0.7 && eas2tas_squared < 10.0)
            baro_b_scale = 1.0 / eas2tas_squared;
    }
    // 钳制 baro_b_scale 范围
    baro_b_scale = constrain(baro_b_scale, 0.2, 3.0);

    // 油门修正系数
    double throttle_b_scale = 1.0;
    throttle_b_scale = constrain(safe_sqrt(output_throttle / 50), 0.5, 2.0);

    // 根据电池电压调整控制对象增益
    double b_scale = bat_b_scale * baro_b_scale * throttle_b_scale;
    double b_scaleYaw = bat_b_scale * baro_b_scale;
    //			ESOHeli[0].b = cfg.b[0] * Heli_RPb_scale*b_scale;
    //			ESOHeli[1].b = cfg.b[2] * Heli_RPb_scale*b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[0])->b = cfg.b[0] * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[1])->b = cfg.b[2] * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[2])->b = cfg.b[4] * b_scaleYaw;

    double rotor_output[12];
    double output_minimum_throttle = cfg.STThrottle[0];

    if (output_throttle < 0 || DbgSafe)
    {
        MainMotor_PullDownAll();
        update_output_throttle(0, 1.0 / CtrlRateHz);
        return;
    }

    double output_range = 100.0 - output_minimum_throttle;
    double output_midpoint = output_range / 2;

    // 油门百分比转换为实际油门量
    output_throttle = MultiRotor_percent_to_throttle(output_throttle, output_minimum_throttle, output_range);

    /*pitch roll 输出限幅*/
    double absRoll = fabs(outRoll);
    if (absRoll > 50)
        outRoll *= 50.0 / absRoll;
    rotor_output[2] = outRoll * 2;
    Roll_u = outRoll;

    double absPitch = fabs(outPitch);
    if (absPitch > 50)
        outPitch *= 50.0 / absPitch;
    rotor_output[3] = outPitch * 2;
    Pitch_u = outPitch;
    /*pitch roll 输出限幅*/

    /*油门推力补偿*/
    static double output_throttle_comp_filted = 0;
    double output_throttle_comp =
        cfg.thrComp[0] * safe_sqrt(sq(((ESO_AngularRate *) CtrlM_ESO[0])->get_EsMainPower()) + sq(((ESO_AngularRate *) CtrlM_ESO[1])->get_EsMainPower())) / ((ESO_AngularRate *) CtrlM_ESO[0])->get_b();
    output_throttle_comp_filted += cfg.beta_thrComp[0] * h * (output_throttle_comp - output_throttle_comp_filted);
    output_throttle_comp = output_throttle_comp_filted;
    if (output_throttle_comp > hover_throttle * 1.5)
        output_throttle_comp = hover_throttle * 1.5;
    output_throttle_comp += output_throttle;
    if (output_throttle_comp > 100)
        output_throttle_comp = 100;
    /*油门推力补偿*/

    /*yaw输出限幅*/
    double allow_output_up = 100 - output_throttle_comp;
    double allow_output_dn = output_throttle_comp - output_minimum_throttle;
    double allow_output = (allow_output_up < allow_output_dn) ? allow_output_up : allow_output_dn;
    if (fabs(outYaw) > allow_output)
    {
        double scale = allow_output / fabs(outYaw);
        outYaw *= scale;
    }
    Yaw_u = outYaw;
    /*yaw输出限幅*/

    // 更新油门油门观测
    double throttle_percent = (output_throttle - output_minimum_throttle) / output_range * 100;
    update_output_throttle(throttle_percent, 1.0 / CtrlRateHz);
    // 补偿非线性输出
    rotor_output[0] = output_throttle_comp - outYaw;
    rotor_output[1] = output_throttle_comp + outYaw;
    throttle_nonlinear_compensation(2, rotor_output);
    ST_PWMMap(rotor_output, 2, 2, STMapType_symmetry);
    MainMotor_PWM_Out(rotor_output);
}

static const UAVCtrlM CoaxialM2S2_CtrlM = {
    CoaxialM2S2_Init,
    CoaxialM2S2_DeInit,
    CoaxialM2S2_PreArmControl,
    CoaxialM2S2_InitControl,
    CoaxialM2S2_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    1,     // 横滚俯仰控推力导数
    false, // 初始化时不进行控制
    true,  // 起飞前不添加扰动
};
/*共轴双桨 M2S2*/

/*共轴双桨 M2S3*/
#define CoaxialM2S3IdleThrottle -15
static inline double CoaxialM2S3_Map(double output_throttle, double outRoll, double outPitch, double out[], double *outRoll_u, double *outPitch_u)
{
    if (Attitude_Control_Enabled == false)
    {
        // 计算最低油门
        bool first = true;
        cfg.minThrottlePct = 0;
        for (uint8_t i = 2; i < 5; ++i)
        {
            float st_min = getStMin(i);
            float st_mid = getStMid(i);
            float st_max = getStMax(i);

            float scale;
            if (fabsf(st_max - st_mid) > fabsf(st_mid - st_min))
                scale = st_max - st_mid;
            else
                scale = st_mid - st_min;

            if (!is_zero(scale))
            {
                float new_minThrottlePct = (st_min - st_mid) / scale;
                if (first)
                {
                    first = false;
                    cfg.minThrottlePct = new_minThrottlePct;
                }
                else if (new_minThrottlePct > cfg.minThrottlePct)
                    cfg.minThrottlePct = new_minThrottlePct;
            }
        }
        cfg.minThrottlePct *= 100;
        // 怠速油门
        cfg.idleThrottlePct = CoaxialM2S3IdleThrottle;
        if (cfg.idleThrottlePct < cfg.minThrottlePct)
            cfg.idleThrottlePct = cfg.minThrottlePct;
    }

    if (output_throttle < cfg.minThrottlePct)
        output_throttle = cfg.minThrottlePct;

    float output_minimum_throttle = cfg.minThrottlePct;
    double output_midpoint = (100.0f + output_minimum_throttle) / 2;

    // if( output_throttle < 0.1f )
    if (0)
    {
        out[0] = out[1] = out[2] = output_throttle;
    }
    else
    { // 横滚俯仰动力分配
        out[0] = +outRoll - 0.5 * outPitch;
        out[1] = +outPitch;
        out[2] = -outRoll - 0.5 * outPitch;

        // 如果需要的pitch roll输出超出当前油门能提供的输出范围
        // 调整油门获得尽量满足pitch roll输出
        double output_max = fabs(out[0]);
        for (uint8_t i = 0; i < 3; ++i)
        {
            double abs_out = fabs(out[i]);
            if (abs_out > output_max)
                output_max = abs_out;
        }

        double max_allow_output = 100.0f - output_throttle;
        double min_allow_output = output_minimum_throttle - output_throttle;
        double allow_ouput_range;
        if (max_allow_output < -min_allow_output)
        { // 降低油门确保姿态输出
            allow_ouput_range = max_allow_output;
            if (output_max > allow_ouput_range)
            { // 需要降低油门
                if (output_max > 100.0f - output_midpoint)
                { // 输出超过最大输出范围
                    // 将油门调整为50%确保可以进行最大输出
                    output_throttle = output_midpoint;
                    allow_ouput_range = 100.0f - output_midpoint;
                }
                else
                { // 降低油门到所需值
                    output_throttle = 100.0f - output_max;
                    allow_ouput_range = output_max;
                }
            }
        }
        else
        { // 抬高油门保证姿态输出
            allow_ouput_range = -min_allow_output;
            if (output_max > allow_ouput_range)
            { // 需要抬高油门

                // 求最高允许的油门值（不能大于悬停油门）
                double hover_throttle_force = hover_throttle;
                double max_allowed_throttle = hover_throttle_force * 0.85;
                if (output_midpoint < max_allowed_throttle)
                    max_allowed_throttle = output_midpoint;
                if (max_allowed_throttle < output_throttle)
                    max_allowed_throttle = output_throttle;
                double max_allowed_output_range = max_allowed_throttle - output_minimum_throttle;

                if (output_max > max_allowed_output_range)
                { // 输出范围大于最大允许范围
                    // 抬高油门至最大允许范围
                    output_throttle = max_allowed_throttle;
                    allow_ouput_range = max_allowed_output_range;
                }
                else
                { // 抬高油门到所需值
                    output_throttle = output_max + output_minimum_throttle;
                    allow_ouput_range = output_max;
                }
            }
        }

        // 输出限幅修正
        if (output_max > allow_ouput_range)
        { // 需要修正输出
            double scale = allow_ouput_range / output_max;
            for (uint8_t i = 0; i < 3; ++i)
                out[i] *= scale;
            if (outRoll_u)
                *outRoll_u = outRoll * scale;
            if (outPitch_u)
                *outPitch_u = outPitch * scale;
        }
        else
        {
            if (outRoll_u)
                *outRoll_u = outRoll;
            if (outPitch_u)
                *outPitch_u = outPitch;
        }

        // 加上油门
        for (uint8_t i = 0; i < 3; ++i)
            out[i] += output_throttle;
    }

    // 更新油门油门观测
    return output_throttle;
}

static void CoaxialM2S3_Init()
{
    // 初始化姿态ESO
    ((ESO_AngularRate *) CtrlM_ESO[0])
        ->init(cfg.order[0], cfg.T[0], cfg.b[0], cfg.beta[0], cfg.beta[0], cfg.orderD[0], cfg.betaD[0], cfg.rolBS[0], cfg.rolBSBW[0], cfg.rolBS2[0], cfg.rolBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxRP[0]);
    ((ESO_AngularRate *) CtrlM_ESO[1])
        ->init(cfg.order[0], cfg.T[0], cfg.b[2], cfg.beta[0], cfg.beta[0], cfg.orderD[0], cfg.betaD[0], cfg.pitBS[0], cfg.pitBSBW[0], cfg.pitBS2[0], cfg.pitBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxRP[0]);
    ((ESO_AngularRate *) CtrlM_ESO[2])
        ->init(cfg.order2[0], cfg.T2[0], cfg.b[4], cfg.beta2[0], cfg.beta2[0], cfg.orderD2[0], cfg.betaD2[0], cfg.yawBS[0], cfg.yawBSBW[0], cfg.yawBS2[0], cfg.yawBSBW2[0], CtrlRateHz * CtrlRateDiv,
               cfg.aMaxYaw[0]);
}
static void CoaxialM2S3_DeInit() {}

static void CoaxialM2S3_PreArmControl(Receiver rc)
{
    double pwmout[6];
    // 电机关闭
    pwmout[0] = pwmout[1] = 0;
    if (rc.available && attController_DisableTime.get_pass_time() > 15)
    {
        float thr;
        if (rc.data[0] > 50)
            thr = (rc.data[0] - 50) * 2;
        else if (rc.data[0] > 30)
            thr = 0;
        else if (rc.data[0] > 10)
            thr = cfg.idleThrottlePct;
        else
            thr = cfg.minThrottlePct;
        CoaxialM2S3_Map(thr, rc.data[3] - 50, rc.data[2] - 50, &pwmout[2], 0, 0);
    }
    else
        CoaxialM2S3_Map(cfg.idleThrottlePct, 0, 0, &pwmout[2], 0, 0);
    ST_PWMMap(pwmout, 2, 3, STMapType_standard);
    MainMotor_PWM_Out(pwmout);
    MainMotor_PullDownAll();
}

static void CoaxialM2S3_InitControl(bool DbgSafe)
{
    if (DbgSafe)
    {
        MainMotor_PullDownAll();
        StartCounter = 0;
    }

    ++StartCounter;
    if (StartCounter > (cfg.STDelay[0] + 1) * CtrlRateHz)
        set_Ctrl_inited;
}

static void CoaxialM2S3_MotorControl(double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe)
{
    //			if( DbgSafe )
    //			{
    //				double pwmout[6];
    //				//电机关闭
    //				pwmout[0] = pwmout[1] = 0;
    //				Receiver rc;
    //				getReceiver(&rc);
    //				double thr;
    //				if( rc.available )
    //				{
    //					if( rc.data[0] > 25 )
    //						thr = 0;
    //					else
    //						thr = CoaxialM2S3IdleThrottle;
    //					CoaxialM2S3_Map( thr, rc.data[3]-50 , rc.data[2]-50 , 50-rc.data[1], &pwmout[2], 0,0,0 );
    //				}
    //				else
    //				{
    //					thr = 0;
    //					CoaxialM2S3_Map( thr, 0 , 0 , 0, &pwmout[2], 0,0,0 );
    //				}
    //				ST_PWMMap( pwmout, 2, 4 );
    //				MainMotor_PWM_Out( pwmout );
    //				update_output_throttle( thr , 1.0/CtrlRateHz );
    //				return;
    //			}
    double h = 1.0 / CtrlRateHz;

    // 计算增益修正系数
    double bat_b_scale = 1.0;
    BatteryInfo batInfo;
    int8_t current_batId = -1;
    getCurrentBatteryInfo(&batInfo, &current_batId);
    if (batId >= 0 && current_batId == batId)
    { // 获取到电池
        if ((cfg.Mode[0] & ATTCTRL_MODE_BATCOMP_DIS_BIT) == 0 && batStVolt > 7 && batInfo.totalVoltRawFilted > 7)
            bat_b_scale = constrain(batInfo.totalVoltRawFilted / batStVolt, 0.35f, 3.0f);
    }

    // 海拔推力补偿
    double baro_b_scale = 1.0;
    extern float INTERNAL_TEMPERATURE;
    double tempK = INTERNAL_TEMPERATURE + C_TO_KELVIN;
    extern float INTERNAL_PRESSURE;
    // 加入气压范围保护，避免 INTERNAL_PRESSURE=0/异常时导致 eas2tas_squared 爆炸→baro_b_scale=0→b=0 装机
    if (tempK > 100 && INTERNAL_PRESSURE > 2500.0f && INTERNAL_PRESSURE < 200000.0f)
    {
        double eas2tas_squared = SSL_AIR_DENSITY / (INTERNAL_PRESSURE / (ISA_GAS_CONSTANT * tempK));
        if (eas2tas_squared > 0.7 && eas2tas_squared < 10.0)
            baro_b_scale = 1.0 / eas2tas_squared;
    }
    // 钳制 baro_b_scale 范围，避免极端值
    baro_b_scale = constrain(baro_b_scale, 0.2, 3.0);

    // 根据电池电压调整控制对象增益
    double b_scale = bat_b_scale * baro_b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[0])->b = cfg.b[0] * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[1])->b = cfg.b[2] * b_scale;
    ((ESO_AngularRate *) CtrlM_ESO[2])->b = cfg.b[4] * b_scale;

    double output_minimum_throttle = cfg.STThrottle[0];
    double output_range = 100.0 - output_minimum_throttle;
    double output_midpoint = output_range / 2;

    double pwmout[6];
    // 求电机输出
    float fly_pwm = (getStMid(0) - 1000) * 0.1f;
    if (Ctrl_initing)
    {
        output_throttle = cfg.idleThrottlePct;
        if (StartCounter < 0.5 * CtrlRateHz)
        {
            pwmout[0] = cfg.STThrottle[0];
            pwmout[1] = 0;
        }
        else if (StartCounter < 1 * CtrlRateHz)
        {
            pwmout[0] = cfg.STThrottle[0];
            pwmout[1] = cfg.STThrottle[0];
        }
        else
            pwmout[0] = pwmout[1] = ((float) (StartCounter - 1 * CtrlRateHz) / (cfg.STDelay[0] * CtrlRateHz)) * (fly_pwm - cfg.STThrottle[0]) + cfg.STThrottle[0];
    }
    else
        pwmout[0] = pwmout[1] = fly_pwm;

    if (DbgSafe)
    {
        pwmout[0] = pwmout[1] = 0;
    }

    //			/*油门推力补偿*/
    //				static double output_throttle_comp_filted = 0;
    //				double output_throttle_comp = cfg.thrComp[0] * safe_sqrt(
    //					sq(((ESO_AngularRate*)CtrlM_ESO[0])->get_EsMainPower()) +
    //					sq(((ESO_AngularRate*)CtrlM_ESO[1])->get_EsMainPower())
    //				) / ((ESO_AngularRate*)CtrlM_ESO[0])->get_b();
    //				output_throttle_comp_filted += cfg.beta_thrComp[0]*h * (output_throttle_comp - output_throttle_comp_filted);
    //				output_throttle_comp = output_throttle_comp_filted;
    //				if( output_throttle_comp > hover_throttle*1.5 )
    //					output_throttle_comp = hover_throttle * 1.5;
    //				output_throttle_comp += output_throttle;
    //				if( output_throttle_comp > 100 )
    //					output_throttle_comp = 100;
    //			/*油门推力补偿*/

    /*yaw输出限幅*/
    double allow_output_up = 100 - pwmout[0];
    double allow_output_dn = pwmout[0] - output_minimum_throttle;
    if (allow_output_dn < 0)
        allow_output_dn = 0;
    double allow_output = (allow_output_up < allow_output_dn) ? allow_output_up : allow_output_dn;
    if (fabs(outYaw) > allow_output)
    {
        double scale = allow_output / fabs(outYaw);
        outYaw *= scale;
    }
    Yaw_u = outYaw;
    /*yaw输出限幅*/

    // 计算舵机输出
    output_throttle = CoaxialM2S3_Map(output_throttle, outRoll, outPitch, &pwmout[2], &Roll_u, &Pitch_u);

    pwmout[0] = pwmout[0] - outYaw;
    pwmout[1] = pwmout[1] + outYaw;

    ST_PWMMap(pwmout, 2, 3, STMapType_standard);
    MainMotor_PWM_Out(pwmout);
    update_output_throttle(output_throttle, 1.0 / CtrlRateHz);
}

static const UAVCtrlM CoaxialM2S3_CtrlM = {
    CoaxialM2S3_Init,
    CoaxialM2S3_DeInit,
    CoaxialM2S3_PreArmControl,
    CoaxialM2S3_InitControl,
    CoaxialM2S3_MotorControl,
    0,
    0, // 横滚俯仰二阶eso
    0,
    0, // 横滚俯仰控推力导数
    //			1, 0,	//横滚俯仰二阶eso
    //			6, 0,	//横滚俯仰控推力导数
    true, // 初始化时进行控制
    true, // 起飞前不添加扰动
};
/*共轴双桨 M2S3*/

const UAVCtrlM *UAVCtrlMs[] = {
    /*000-*/ 0,
    /*001-*/ 0,
    /*002-*/ 0,
    /*003-*/ 0,
    /*004-*/ 0,
    /*005-*/ 0,
    /*006-*/ 0,
    /*007-*/ 0,
    /*008-*/ 0,
    /*009-*/ 0,
    /*010-*/ &MultiRotor4X_CtrlM,
    /*011-*/ &MultiRotor6X_CtrlM,
    /*012-*/ &MultiRotor8X_CtrlM,
    /*013-*/ 0,
    /*014-*/ 0,
    /*015-*/ &MultiRotor4C_CtrlM,
    /*016-*/ &MultiRotor6C_CtrlM,
    /*017-*/ &MultiRotor8C_CtrlM,
    /*018-*/ 0,
    /*019-*/ 0,
    /*020-*/ &MultiRotor42X_CtrlM,
    /*021-*/ &MultiRotor62X_CtrlM,
    /*022-*/ &MultiRotor82X_CtrlM,
    /*023-*/ 0,
    /*024-*/ 0,
    /*025-*/ 0,
    /*026-*/ 0,
    /*027-*/ 0,
    /*028-*/ 0,
    /*029-*/ 0,
    /*030-*/ 0,
    /*031-*/ &MultiRotor62C_CtrlM,
    /*032-*/ 0,
    /*033-*/ 0,
    /*034-*/ 0,
    /*035-*/ 0,
    /*036-*/ 0,
    /*037-*/ 0,
    /*038-*/ 0,
    /*039-*/ 0,
    /*040-*/ 0,
    /*041-*/ &MultiRotor6H_CtrlM,
    /*042-*/ 0,
    /*043-*/ 0,
    /*044-*/ 0,
    /*045-*/ 0,
    /*046-*/ 0,
    /*047-*/ 0,
    /*048-*/ 0,
    /*049-*/ 0,
    /*050-*/ 0,
    /*051-*/ &MultiRotor62H_CtrlM,
    /*052-*/ &MultiRotor82X2_CtrlM,
    /*053-*/ 0,
    /*054-*/ 0,
    /*055-*/ 0,
    /*056-*/ 0,
    /*057-*/ 0,
    /*058-*/ 0,
    /*059-*/ 0,
    /*060-*/ 0,
    /*061-*/ 0,
    /*062-*/ 0,
    /*063-*/ 0,
    /*064-*/ 0,
    /*065-*/ 0,
    /*066-*/ 0,
    /*067-*/ 0,
    /*068-*/ 0,
    /*069-*/ 0,
    /*070-*/ 0,
    /*071-*/ &MultiRotor62H2_CtrlM,
    /*072-*/ 0,
    /*073-*/ 0,
    /*074-*/ 0,
    /*075-*/ 0,
    /*076-*/ 0,
    /*077-*/ 0,
    /*078-*/ 0,
    /*079-*/ 0,
    /*080-*/ &TriRotorX_CtrlM,
    /*081-*/ 0,
    /*082-*/ 0,
    /*083-*/ 0,
    /*084-*/ 0,
    /*085-*/ 0,
    /*086-*/ 0,
    /*087-*/ 0,
    /*088-*/ 0,
    /*089-*/ 0,
    /*090-*/ 0,
    /*091-*/ 0,
    /*092-*/ 0,
    /*093-*/ 0,
    /*094-*/ 0,
    /*095-*/ 0,
    /*096-*/ 0,
    /*097-*/ 0,
    /*098-*/ 0,
    /*099-*/ 0,
    /*100-*/ &Heli131_CtrlM,
    /*101-*/ 0,
    /*102-*/ 0,
    /*103-*/ 0,
    /*104-*/ 0,
    /*105-*/ 0,
    /*106-*/ 0,
    /*107-*/ 0,
    /*108-*/ 0,
    /*109-*/ 0,
    /*110-*/ 0,
    /*111-*/ 0,
    /*112-*/ 0,
    /*113-*/ 0,
    /*114-*/ 0,
    /*115-*/ 0,
    /*116-*/ 0,
    /*117-*/ 0,
    /*118-*/ 0,
    /*119-*/ 0,
    /*120-*/ 0,
    /*121-*/ 0,
    /*122-*/ 0,
    /*123-*/ 0,
    /*124-*/ 0,
    /*125-*/ 0,
    /*126-*/ 0,
    /*127-*/ 0,
    /*128-*/ 0,
    /*129-*/ 0,
    /*130-*/ 0,
    /*131-*/ 0,
    /*132-*/ 0,
    /*133-*/ 0,
    /*134-*/ 0,
    /*135-*/ 0,
    /*136-*/ 0,
    /*137-*/ 0,
    /*138-*/ 0,
    /*139-*/ 0,
    /*140-*/ 0,
    /*141-*/ 0,
    /*142-*/ 0,
    /*143-*/ 0,
    /*144-*/ 0,
    /*145-*/ 0,
    /*146-*/ 0,
    /*147-*/ 0,
    /*148-*/ 0,
    /*149-*/ 0,
    /*150-*/ 0,
    /*151-*/ 0,
    /*152-*/ 0,
    /*153-*/ 0,
    /*154-*/ 0,
    /*155-*/ 0,
    /*156-*/ 0,
    /*157-*/ 0,
    /*158-*/ 0,
    /*159-*/ 0,
    /*160-*/ &CoaxialM2S2_CtrlM,
    /*161-*/ &CoaxialM2S3_CtrlM,
    /*162-*/ 0,
    /*163-*/ 0,
    /*164-*/ 0,
    /*165-*/ 0,
    /*166-*/ 0,
    /*167-*/ 0,
    /*168-*/ 0,
    /*169-*/ 0,
    /*170-*/ 0,
    /*171-*/ 0,
    /*172-*/ 0,
    /*173-*/ 0,
    /*174-*/ 0,
    /*175-*/ 0,
    /*176-*/ 0,
    /*177-*/ 0,
    /*178-*/ 0,
    /*179-*/ 0,
    /*180-*/ 0,
    /*181-*/ 0,
    /*182-*/ 0,
    /*183-*/ 0,
    /*184-*/ 0,
    /*185-*/ 0,
    /*186-*/ 0,
    /*187-*/ 0,
    /*188-*/ 0,
    /*189-*/ 0,
    /*190-*/ 0,
    /*191-*/ 0,
    /*192-*/ 0,
    /*193-*/ 0,
    /*194-*/ 0,
    /*195-*/ 0,
    /*196-*/ 0,
    /*197-*/ 0,
    /*198-*/ 0,
    /*199-*/ 0,
    /*200-*/ 0,
};

/*机型控制方式*/

void ctrl_Attitude()
{
    double h = 1.0 / CtrlRateHz;

    Receiver rc;
    getReceiver(&rc);

    bool DbgSafe = false;
    if (SafeBt)
    { // Debug保护
        if (SafeBt == 10)
        {
            if ((rc.available && rc.data[0] < cfg.STThrottle[0] - 0.1) && (rc.data[1] < 10))
                DbgSafe = true;
        }
        else if (SafeBt >= 2 && SafeBt <= 4)
        {
            if (rc.available && rc.available_channels >= 4 + SafeBt && rc.data[4 + SafeBt - 1] > 90)
                DbgSafe = true;
        }
    }
    DebugSafeEna = DbgSafe;

    if (Attitude_Control_Enabled == false)
    { // 控制器未打开

        // 定时同步参数
        static uint16_t ParamUpdateCounter = 65500;
        if (getInitializationCompleted())
        {
            if (ParamUpdateCounter >= 3 * CtrlRateHz)
            {
                ParamUpdateCounter = 0;
                ReadParamGroup("AttCtrl", getAttCfgParamBase(), 0, 0);

                UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
                if (mt_count.MTCount == 0 || UAVCtrlMs[cfg.UAVType[0]] == 0)
                {
                    CtrlM_available = false;
                    set_MainMotorCount(0, 0);
                }
                else
                {
                    set_MainMotorCount(mt_count.MTCount, mt_count.STCount, cfg.st_Freq[0]);
                    CtrlM = *UAVCtrlMs[cfg.UAVType[0]];
                    CtrlM_available = true;
                }
            }
            else
                ++ParamUpdateCounter;
        }

        Roll_u = Pitch_u = Yaw_u = 0;
        update_output_throttle(0, h);
        if (CtrlM_available)
            CtrlM.PreArmControl(rc);
        return;
    }

    // 启动初始化
    if (Ctrl_initing)
    { // 还未进行启动初始化
        CtrlM.InitControl(DbgSafe);
        if (CtrlM.ctrl_on_init == false)
            return;
    }

    Quaternion AirframeQuat;
    get_Airframe_quat(&AirframeQuat, 0.1);

    // 获取控制参数
    double Ps = cfg.P1[0];
    double PsY = cfg.P1[4];
    vector3<double> P2(cfg.P2[0], cfg.P2[2], cfg.P2[4]);
    vector3<double> P3(cfg.P3[0], cfg.P3[2], cfg.P3[4]);

    // 目标Roll Pitch四元数
    Quaternion target_quat_PR;
    // 目标角速度
    vector3<double> target_angular_velocity;

    // 获取当前四元数的Pitch Roll分量四元数
    double Yaw;
    Quaternion_Ef current_quat_PR = AirframeQuat.get_RP_quat(&Yaw);

    // 计算旋转矩阵
    current_quat_PR.conjugate();
    double Rotation_Matrix[3][3]; // 反向旋转
    current_quat_PR.get_rotation_matrix(Rotation_Matrix);
    current_quat_PR.conjugate();
    double Rotation_Matrix_P[3][3]; // 正向旋转
    current_quat_PR.get_rotation_matrix(Rotation_Matrix_P);

    // 运行扩张状态观测器得到估计角速度、角加速度

    vector3<double> AngularRateCtrl;
    get_AngularRate_Ctrl(&AngularRateCtrl, 0.1);
    vector3<double> angular_rate_ESO;
    vector3<double> angular_acceleration_ESO;
    // 使用ESO估计角速度、角加速度
    angular_rate_ESO.set_vector(CtrlM_ESO[0]->get_EsAngularRate(), CtrlM_ESO[1]->get_EsAngularRate(), CtrlM_ESO[2]->get_EsAngularRate());
    angular_acceleration_ESO.set_vector(CtrlM_ESO[0]->get_EsAngularAcceleration(), CtrlM_ESO[1]->get_EsAngularAcceleration(), CtrlM_ESO[2]->get_EsAngularAcceleration());

    // 计算ENU坐标系下的角速度、角加速度
    vector3<double> angular_rate_ENU;
    angular_rate_ENU.x = Rotation_Matrix_P[0][0] * angular_rate_ESO.x + Rotation_Matrix_P[0][1] * angular_rate_ESO.y + Rotation_Matrix_P[0][2] * angular_rate_ESO.z;
    angular_rate_ENU.y = Rotation_Matrix_P[1][0] * angular_rate_ESO.x + Rotation_Matrix_P[1][1] * angular_rate_ESO.y + Rotation_Matrix_P[1][2] * angular_rate_ESO.z;
    angular_rate_ENU.z = Rotation_Matrix_P[2][0] * angular_rate_ESO.x + Rotation_Matrix_P[2][1] * angular_rate_ESO.y + Rotation_Matrix_P[2][2] * angular_rate_ESO.z;
    vector3<double> angular_acceleration_ENU;
    angular_acceleration_ENU.x = Rotation_Matrix_P[0][0] * angular_acceleration_ESO.x + Rotation_Matrix_P[0][1] * angular_acceleration_ESO.y + Rotation_Matrix_P[0][2] * angular_acceleration_ESO.z;
    angular_acceleration_ENU.y = Rotation_Matrix_P[1][0] * angular_acceleration_ESO.x + Rotation_Matrix_P[1][1] * angular_acceleration_ESO.y + Rotation_Matrix_P[1][2] * angular_acceleration_ESO.z;
    angular_acceleration_ENU.z = Rotation_Matrix_P[2][0] * angular_acceleration_ESO.x + Rotation_Matrix_P[2][1] * angular_acceleration_ESO.y + Rotation_Matrix_P[2][2] * angular_acceleration_ESO.z;

    // 角加速度滤波
    vector3<double> TD3_ENU = {Target_tracker_RP.x3.x, Target_tracker_RP.x3.y, Target_trackerY.x3};

    vector3<double> angular_acceleration_ENU_filted;
    vector3<double> angular_acceleration_filted;

    if (cfg.dFilter[0] > 0.3 && cfg.dFilter[0] < 1 / (2 * Pi * h))
    {
        double accFiltP = cfg.dFilter[0];
        d_filted[0] += accFiltP * 2 * Pi * h * ((TD3_ENU.x - angular_acceleration_ENU.x) - d_filted[0]);
        d_filted[1] += accFiltP * 2 * Pi * h * ((TD3_ENU.y - angular_acceleration_ENU.y) - d_filted[1]);
        d_filted[2] += accFiltP * 2 * Pi * h * ((TD3_ENU.z - angular_acceleration_ENU.z) - d_filted[2]);

        angular_acceleration_ENU_filted.x = TD3_ENU.x - d_filted[0];
        angular_acceleration_ENU_filted.y = TD3_ENU.y - d_filted[1];
        angular_acceleration_ENU_filted.z = TD3_ENU.z - d_filted[2];

        angular_acceleration_filted.x =
            Rotation_Matrix[0][0] * angular_acceleration_ENU_filted.x + Rotation_Matrix[0][1] * angular_acceleration_ENU_filted.y + Rotation_Matrix[0][2] * angular_acceleration_ENU_filted.z;
        angular_acceleration_filted.y =
            Rotation_Matrix[1][0] * angular_acceleration_ENU_filted.x + Rotation_Matrix[1][1] * angular_acceleration_ENU_filted.y + Rotation_Matrix[1][2] * angular_acceleration_ENU_filted.z;
        angular_acceleration_filted.z =
            Rotation_Matrix[2][0] * angular_acceleration_ENU_filted.x + Rotation_Matrix[2][1] * angular_acceleration_ENU_filted.y + Rotation_Matrix[2][2] * angular_acceleration_ENU_filted.z;
    }
    else
    {
        d_filted[0] = TD3_ENU.x - angular_acceleration_ENU.x;
        d_filted[1] = TD3_ENU.y - angular_acceleration_ENU.y;
        d_filted[2] = TD3_ENU.z - angular_acceleration_ENU.z;
        angular_acceleration_ENU_filted = angular_acceleration_ENU;
        angular_acceleration_filted = angular_acceleration_ESO;
    }
    // 角加速度滤波

    // 由Roll Pitch控制模式
    // 计算Roll Pitch目标角速度（ENU系）
    vector3<double> target_angular_rate_RP;
    switch (RollPitch_ControlMode)
    {
        default:
        case Attitude_ControlMode_Angle:
        {
            // TD4滤目标角度
            Target_tracker_RP.track3(vector2<double>(target_Roll, target_Pitch), 1.0 / CtrlRateHz);

            // 使用目标角度构造目标四元数
            // calculate target quat Q1
            //       front
            //        x
            //        ^
            //        |
            //  y < --O
            double angle = safe_sqrt(sq(Target_tracker_RP.x1.x) + sq(Target_tracker_RP.x1.y));
            if (angle > 0.01)
            {
                double scale = 1.0 / angle;
                double n_x = Target_tracker_RP.x1.x * scale;
                double n_y = Target_tracker_RP.x1.y * scale;

                double half_sinA, half_cosA;
                fast_sin_cos(0.5 * angle, &half_sinA, &half_cosA);
                target_quat_PR = Quaternion(half_cosA, n_x * half_sinA, n_y * half_sinA, 0);
            }
            else
            {
                double half_sinR, half_cosR;
                fast_sin_cos(0.5 * Target_tracker_RP.x1.x, &half_sinR, &half_cosR);
                double half_sinP, half_cosP;
                fast_sin_cos(0.5 * Target_tracker_RP.x1.y, &half_sinP, &half_cosP);
                target_quat_PR = Quaternion(half_cosR * half_cosP, half_cosP * half_sinR, half_cosR * half_sinP, -half_sinR * half_sinP);
            }

            // 计算误差四元数Q
            // Q*Q1=Qt  Q1为当前机体四元数，Qt为目标四元数
            // Q=Qt*inv(Q1)
            Quaternion current_quat_conj = current_quat_PR;
            current_quat_conj.conjugate();
            vector3<double> PR_rotation = (target_quat_PR * current_quat_conj).get_Rotation_vec();
            double PR_rotation_len = safe_sqrt(PR_rotation.get_square());
            if (PR_rotation_len > Pi)
            {
                double len = fmod(PR_rotation_len, 2 * Pi);
                if (len > Pi)
                    len -= 2 * Pi;
                else if (len < -Pi)
                    len += 2 * Pi;
                PR_rotation *= len / PR_rotation_len;
            }
            else if (PR_rotation_len < -Pi)
            {
                double len = fmod(PR_rotation_len, 2 * Pi);
                if (len > Pi)
                    len -= 2 * Pi;
                else if (len < -Pi)
                    len += 2 * Pi;
                PR_rotation *= len / PR_rotation_len;
            }

            vector3<double> feed_foward_ratePR = {Target_tracker_RP.x2.x, Target_tracker_RP.x2.y, 0};
            target_angular_rate_RP = (PR_rotation * Ps);
            target_angular_rate_RP.constrain(cfg.maxRPSp[0]);
            target_angular_rate_RP += feed_foward_ratePR;

            AC_angle_error = safe_sqrt(PR_rotation.get_square());
            break;
        }
        case Attitude_ControlMode_AngularRate:
        {
            double sp = sq(target_AngularRate.x) + sq(target_AngularRate.y);
            if (sp > sq(cfg.maxRPSp[0]))
            {
                sp = safe_sqrt(sp);
                double scale = cfg.maxRPSp[0] / sp;
                target_AngularRate.x *= scale;
                target_AngularRate.y *= scale;
            }
            Target_tracker_RP.track2(vector2<double>(target_AngularRate.x, target_AngularRate.y), 1.0 / CtrlRateHz);
            break;
        }
    }

    double target_angular_rate_Y;
    switch (Yaw_ControlMode)
    {
        case Attitude_ControlMode_Angle:
        {
            if (inFlight)
            {
                // TD4滤目标角度
                Target_trackerY.r2n = Target_trackerY.r2p = degree2rad(cfg.maxYSp[0] * 0.7);
                if (cfg.YawTDOrder[0] == 4)
                    Target_trackerY.track4(target_Yaw, h);
                else
                    Target_trackerY.track4_order3(target_Yaw, h);

                // 角度误差化为-180 - +180
                double angle_error = Target_trackerY.x1 - Yaw;
                angle_error = fmod(angle_error, 2 * Pi);
                if (angle_error < -Pi)
                    angle_error += 2 * Pi;
                else if (angle_error > Pi)
                    angle_error -= 2 * Pi;

                // 求目标角速度
                target_angular_rate_Y = angle_error * Ps + Target_trackerY.x2;
                target_angular_rate_Y = constrain(target_angular_rate_Y, 2.5);
            }
            else
            {
                Target_trackerY.reset();
                Target_trackerY.x1 = target_Yaw = Yaw;
                target_angular_rate_Y = 0;
            }
            break;
        }
        case Attitude_ControlMode_AngularRate:
        {
            if (inFlight)
            {
                Target_trackerY.r2n = Target_trackerY.r2p = degree2rad(cfg.maxYSp[0]);
                if (cfg.YawTDOrder[0] == 4)
                    Target_trackerY.track3(target_AngularRate.z, h);
                else
                    Target_trackerY.track3_order3(target_AngularRate.z, h);
                target_angular_rate_Y = Target_trackerY.x2;
                Target_trackerY.x1 = target_Yaw = Yaw;
            }
            else
            {
                Target_trackerY.reset();
                Target_trackerY.x1 = target_Yaw = Yaw;
                target_angular_rate_Y = 0;
            }
            break;
        }
        case Attitude_ControlMode_OffBoard:
        {
            if (inFlight)
            {
                // TD4滤目标角度
                Target_trackerY.r2n = Target_trackerY.r2p = degree2rad(cfg.maxYSp[0] * 0.7);
                if (cfg.YawTDOrder[0] == 4)
                    Target_trackerY.track4(target_Yaw, target_AngularRate.z, 0, 0, 0, h);
                else
                    Target_trackerY.track4_order3(target_Yaw, target_AngularRate.z, 0, 0, h);

                // 角度误差化为-180 - +180
                double angle_error = Target_trackerY.x1 - Yaw;
                angle_error = fmod(angle_error, 2 * Pi);
                if (angle_error < -Pi)
                    angle_error += 2 * Pi;
                else if (angle_error > Pi)
                    angle_error -= 2 * Pi;

                // 求目标角速度
                target_angular_rate_Y = angle_error * Ps + Target_trackerY.x2;
                target_angular_rate_Y = constrain(target_angular_rate_Y, 2.5);
            }
            else
            {
                Target_trackerY.reset();
                Target_trackerY.x1 = target_Yaw = Yaw;
                target_angular_rate_Y = 0;
            }
            break;
        }
        case Attitude_ControlMode_Locking:
        {
            if (inFlight)
            {
                if (cfg.YawTDOrder[0] == 4)
                    Target_trackerY.track3(0, h);
                else
                    Target_trackerY.track3_order3(0, h);
                target_angular_rate_Y = Target_trackerY.x2;
                Target_trackerY.x1 = target_Yaw = Yaw;
                if (in_symmetry_range(target_angular_rate_Y, 0.001) && in_symmetry_range(angular_rate_ENU.z, 0.05))
                {
                    Target_trackerY.x1 = target_Yaw = Yaw;
                    Yaw_ControlMode = Attitude_ControlMode_Angle;
                }
            }
            else
            {
                Target_trackerY.reset();
                Target_trackerY.x1 = target_Yaw = Yaw;
                target_angular_rate_Y = 0;
            }
            break;
        }
    }

    // 机体系期望角速度
    vector3<double> target_angular_rate_body;
    // 机体系期望角速度一阶导数
    vector3<double> Tv1;
    // 机体系期望角速度二阶导数
    vector3<double> Tv2;
    if (Target_tracker_RP.get_tracking_mode() == 3)
    { // 角度模式

        // 计算前馈量
        double YawAngleP = (Target_trackerY.get_tracking_mode() == 4) ? (Ps) : 0;
        vector3<double> Tv1_ENU = {Ps * (Target_tracker_RP.x2.x - angular_rate_ENU.x) + Target_tracker_RP.x3.x, Ps * (Target_tracker_RP.x2.y - angular_rate_ENU.y) + Target_tracker_RP.x3.y,
                                   YawAngleP * (Target_trackerY.x2 - angular_rate_ENU.z) + Target_trackerY.x3};
        vector3<double> Tv2_ENU = {Ps * (Target_tracker_RP.x3.x - angular_acceleration_ENU_filted.x) + Target_tracker_RP.T4.x,
                                   Ps * (Target_tracker_RP.x3.y - angular_acceleration_ENU_filted.y) + Target_tracker_RP.T4.y,
                                   YawAngleP * (Target_trackerY.x3 - angular_acceleration_ENU_filted.z) + Target_trackerY.x4};

        Tv1.x = Rotation_Matrix[0][0] * Tv1_ENU.x + Rotation_Matrix[0][1] * Tv1_ENU.y + Rotation_Matrix[0][2] * Tv1_ENU.z;
        Tv1.y = Rotation_Matrix[1][0] * Tv1_ENU.x + Rotation_Matrix[1][1] * Tv1_ENU.y + Rotation_Matrix[1][2] * Tv1_ENU.z;
        Tv1.z = Rotation_Matrix[2][0] * Tv1_ENU.x + Rotation_Matrix[2][1] * Tv1_ENU.y + Rotation_Matrix[2][2] * Tv1_ENU.z;

        Tv2.x = Rotation_Matrix[0][0] * Tv2_ENU.x + Rotation_Matrix[0][1] * Tv2_ENU.y + Rotation_Matrix[0][2] * Tv2_ENU.z;
        Tv2.y = Rotation_Matrix[1][0] * Tv2_ENU.x + Rotation_Matrix[1][1] * Tv2_ENU.y + Rotation_Matrix[1][2] * Tv2_ENU.z;
        Tv2.z = Rotation_Matrix[2][0] * Tv2_ENU.x + Rotation_Matrix[2][1] * Tv2_ENU.y + Rotation_Matrix[2][2] * Tv2_ENU.z;

        // 计算前馈量

        // 把目标速度从Bodyheading旋转到机体
        vector3<double> target_angular_rate_ENU;
        target_angular_rate_ENU.x = target_angular_rate_RP.x;
        target_angular_rate_ENU.y = target_angular_rate_RP.y;
        target_angular_rate_ENU.z = target_angular_rate_RP.z + target_angular_rate_Y;

        target_angular_rate_body.x = Rotation_Matrix[0][0] * target_angular_rate_ENU.x + Rotation_Matrix[0][1] * target_angular_rate_ENU.y + Rotation_Matrix[0][2] * target_angular_rate_ENU.z;
        target_angular_rate_body.y = Rotation_Matrix[1][0] * target_angular_rate_ENU.x + Rotation_Matrix[1][1] * target_angular_rate_ENU.y + Rotation_Matrix[1][2] * target_angular_rate_ENU.z;
        target_angular_rate_body.z = Rotation_Matrix[2][0] * target_angular_rate_ENU.x + Rotation_Matrix[2][1] * target_angular_rate_ENU.y + Rotation_Matrix[2][2] * target_angular_rate_ENU.z;
        // 把目标速度从Bodyheading旋转到机体
    }
    else
    { // 角速率模式
        target_angular_rate_body.x = Target_tracker_RP.x2.x;
        target_angular_rate_body.y = Target_tracker_RP.x2.y;
        target_angular_rate_body.z = Target_trackerY.x2;

        Tv1.x = Target_tracker_RP.x3.x;
        Tv1.y = Target_tracker_RP.x3.y;
        Tv1.z = Target_trackerY.x3;

        Tv2.x = Target_tracker_RP.T4.x;
        Tv2.y = Target_tracker_RP.T4.y;
        Tv2.z = Target_trackerY.x4;
    }

    // 计算目标角加速度
    vector3<double> angular_rate_error = target_angular_rate_body - angular_rate_ESO;
    AC_rate_error = safe_sqrt(angular_rate_error.get_square());
    vector3<double> target_angular_acceleration = target_angular_rate_body - angular_rate_ESO;
    target_angular_acceleration.x *= P2.x;
    target_angular_acceleration.y *= P2.y;
    target_angular_acceleration.z *= P2.z;
    target_angular_acceleration = target_angular_acceleration + Tv1;

    vector3<double> Ta1 = {P2.x * (Tv1.x - angular_acceleration_filted.x) + Tv2.x, P2.y * (Tv1.y - angular_acceleration_filted.y) + Tv2.y, P2.z * (Tv1.z - angular_acceleration_filted.z) + Tv2.z};
    // 计算目标角加速度

    // 计算角加速度误差
    vector3<double> angular_acceleration_error = target_angular_acceleration - angular_acceleration_filted;

    // 获取扰动
    vector3<double> disturbance(CtrlM_ESO[0]->get_EsDisturbance(), CtrlM_ESO[1]->get_EsDisturbance(), CtrlM_ESO[2]->get_EsDisturbance());
    vector3<double> disturbanceFilted(CtrlM_ESO[0]->get_EsDisturbanceFilted(), CtrlM_ESO[1]->get_EsDisturbanceFilted(), CtrlM_ESO[2]->get_EsDisturbanceFilted());

// 扰动包络滤波
#define DISTURBANCE_MMFILTER_F 1.5
    for (uint8_t i = 0; i < 3; ++i)
    {
        // 斜率限幅跟踪扰动(替代原一阶低通+包络自适应滤波系数): 每周期最大变化量为aMax*h
        // Roll/Pitch共用aMaxRP, Yaw用aMaxYaw
        // aMax<=10视为无效参数不进行限幅(直接跟踪, 与原ESO内部a_max<5不限制的防呆规则一致)
        // 原ESO内部z_inertia斜率限幅(电调限流模拟)已移除, 改在此处限制,
        // aMax过小时不会造成ESO模型失配(无高频震颤), 仅降低扰动跟踪斜率
        double aMax = (i < 2) ? cfg.aMaxRP[0] : cfg.aMaxYaw[0];
        if (aMax > 10)
        {
            double dAMax = aMax * h;
            if (disturbanceFilted[i] > disturbance_filted[i] + dAMax)
                disturbance_filted[i] += dAMax;
            else if (disturbanceFilted[i] < disturbance_filted[i] - dAMax)
                disturbance_filted[i] -= dAMax;
            else
                disturbance_filted[i] = disturbanceFilted[i];
        }
        else
            disturbance_filted[i] = disturbanceFilted[i];
    }

    vector3<double> disturbanceD(CtrlM_ESO[0]->get_EsDisturbanceD(), CtrlM_ESO[1]->get_EsDisturbanceD(), CtrlM_ESO[2]->get_EsDisturbanceD());

    double outRoll;
    double outPitch;
    double outYaw;
    if (pre_inFlightRP || (CtrlM.RP_NoDisturbanceOG == false))
    {
        switch (CtrlM.RP_ctrl_type)
        {
            default:
            case 0:
            { // 控制推力导数
                vector2<double> disturbanceCtrl;
                if (cfg.Mode[0] & ATTCTRL_MODE_RP_ACTIVE_DISTURBANCE_BIT)
                {
                    disturbanceCtrl[0] = disturbanceFilted[0];
                    disturbanceCtrl[1] = disturbanceFilted[1];
                }
                else
                {
                    disturbanceCtrl[0] = disturbance_filted[0];
                    disturbanceCtrl[1] = disturbance_filted[1];
                }

                outRoll = (angular_acceleration_filted.x - disturbanceCtrl[0] + CtrlM_ESO[0]->get_T() * (angular_acceleration_error.x * P3.x + Ta1.x /*- disturbanceD[0]*/)) / CtrlM_ESO[0]->get_b();
                outPitch = (angular_acceleration_filted.y - disturbanceCtrl[1] + CtrlM_ESO[1]->get_T() * (angular_acceleration_error.y * P3.y + Ta1.y /*- disturbanceD[1]*/)) / CtrlM_ESO[1]->get_b();
                break;
            }
            case 1:
            { // 期望角加速度P减扰动
                outRoll = ((target_angular_acceleration.x - disturbanceFilted.x)) / CtrlM_ESO[0]->get_b();
                outPitch = ((target_angular_acceleration.y - disturbanceFilted.y)) / CtrlM_ESO[1]->get_b();
                break;
            }
            case 2:
            { // 期望角加速度PD减扰动
                outRoll = ((target_angular_acceleration.x + P3.x * CtrlM_ESO[0]->get_T() * angular_acceleration_error.x - disturbance.x)) / CtrlM_ESO[0]->get_b();
                outPitch = ((target_angular_acceleration.y + P3.x * CtrlM_ESO[1]->get_T() * angular_acceleration_error.y - disturbance.y)) / CtrlM_ESO[1]->get_b();
                break;
            }

            case 5:
            { // 期望角速度P (直升机姿态控制1)
                outRoll = (target_angular_rate_body.x - disturbanceFilted.x) / CtrlM_ESO[0]->get_b();
                outPitch = (target_angular_rate_body.y - disturbanceFilted.y) / CtrlM_ESO[1]->get_b();
                break;
            }
            case 6:
            { // 控制角速度导数 (直升机姿态控制2)
                vector2<double> disturbanceCtrl;
                if (cfg.Mode[0] & ATTCTRL_MODE_RP_ACTIVE_DISTURBANCE_BIT)
                {
                    disturbanceCtrl[0] = disturbanceFilted[0];
                    disturbanceCtrl[1] = disturbanceFilted[1];
                }
                else
                {
                    disturbanceCtrl[0] = disturbance_filted[0];
                    disturbanceCtrl[1] = disturbance_filted[1];
                }

                outRoll = (angular_rate_ESO.x - disturbanceCtrl[0] + CtrlM_ESO[0]->get_T() * (angular_rate_error.x * P2.x + Tv1.x /*- disturbanceD[0]*/)) / CtrlM_ESO[0]->get_b();
                outPitch = (angular_rate_ESO.y - disturbanceCtrl[1] + CtrlM_ESO[1]->get_T() * (angular_rate_error.y * P2.y + Tv1.y /*- disturbanceD[1]*/)) / CtrlM_ESO[1]->get_b();
                break;
            }
        }
    }
    else
    { // 起飞前
        switch (CtrlM.RP_ctrl_type)
        {
            default:
            case 0:
            { // 控制推力导数
                outRoll = ((target_angular_rate_body.x * P2.x)) / CtrlM_ESO[0]->get_b();
                outPitch = ((target_angular_rate_body.y * P2.y)) / CtrlM_ESO[1]->get_b();
                break;
            }
            case 1:
            { // 期望角加速度P减扰动
                outRoll = ((target_angular_acceleration.x)) / CtrlM_ESO[0]->get_b();
                outPitch = ((target_angular_acceleration.y)) / CtrlM_ESO[1]->get_b();
                break;
            }
            case 2:
            { // 期望角加速度PD减扰动
                outRoll = ((target_angular_acceleration.x + P3.x * CtrlM_ESO[0]->get_T() * angular_acceleration_error.x)) / CtrlM_ESO[0]->get_b();
                outPitch = ((target_angular_acceleration.y + P3.x * CtrlM_ESO[1]->get_T() * angular_acceleration_error.y)) / CtrlM_ESO[1]->get_b();
                break;
            }
            case 5:
            case 6:
            { // 期望角速度P (直升机姿态控制1)
                // 控制角速度导数 (直升机姿态控制2)
                outRoll = (target_angular_rate_body.x) / CtrlM_ESO[0]->get_b();
                outPitch = (target_angular_rate_body.y) / CtrlM_ESO[1]->get_b();
                break;
            }
        }
    }

    if (pre_inFlightY || (CtrlM.RP_NoDisturbanceOG == false))
    {
        switch (CtrlM.Y_ctrl_type)
        {
            default:
            case 0:
            { // 控制推力导数

                double disturbanceCtrl;
                if (cfg.Mode[0] & ATTCTRL_MODE_Y_ACTIVE_DISTURBANCE_BIT)
                    disturbanceCtrl = disturbanceFilted[2];
                else
                    disturbanceCtrl = disturbance_filted[2];

                outYaw = (angular_acceleration_filted.z - disturbanceCtrl + CtrlM_ESO[2]->get_T() * (angular_acceleration_error.z * P3.z + Ta1.z /*- disturbanceD[2]*/)) / CtrlM_ESO[2]->get_b();

                // outYaw = ( CtrlM_ESO[2]->get_EsMainPower() + CtrlM_ESO[2]->get_T() * ( angular_acceleration_error.z * P3.z + Ta1.z ) ) / CtrlM_ESO[2]->get_b();
                break;
            }
            case 1:
            { // 期望角加速度P减扰动
                outYaw = (target_angular_acceleration.z - disturbanceFilted.z) / CtrlM_ESO[2]->get_b();
                break;
            }
            case 2:
            { // 期望角加速度PD减扰动
                outYaw = (target_angular_acceleration.z + P3.z * CtrlM_ESO[2]->get_T() * angular_acceleration_error.z - disturbanceFilted.z) / CtrlM_ESO[2]->get_b();
                break;
            }
            case 5:
            { // 期望角速度P
                outYaw = (target_angular_rate_body.z - disturbanceFilted.z) / CtrlM_ESO[2]->get_b();
                break;
            }
        }
    }
    else
    { // 起飞前
        switch (CtrlM.Y_ctrl_type)
        {
            default:
            case 0:
            { // 控制推力导数
                // outYaw = ( CtrlM_ESO[2]->get_EsMainPower() + CtrlM_ESO[2]->get_T() * (
                // angular_acceleration_error.z * P3.z + Ta1.z ) ) / CtrlM_ESO[2]->get_b();
                outYaw = ((target_angular_rate_body.z * P2.z)) / CtrlM_ESO[2]->get_b();
                break;
            }
            case 1:
            { // 期望角加速度P减扰动
                outYaw = (target_angular_rate_body.z) / CtrlM_ESO[2]->get_b();
                break;
            }
            case 2:
            { // 期望角加速度PD减扰动
                outYaw = (target_angular_acceleration.z + P3.z * CtrlM_ESO[2]->get_T() * angular_acceleration_error.z - disturbanceFilted.z) / CtrlM_ESO[2]->get_b();
                break;
            }
            case 5:
            { // 期望角速度P
                outYaw = (target_angular_rate_body.z - disturbanceFilted.z) / CtrlM_ESO[2]->get_b();
                break;
            }
        }
    }

    //	outRoll_filted += 80 * h * ( outRoll - outRoll_filted );
    //	outPitch_filted += 80 * h * ( outPitch - outPitch_filted );
    //	outYaw_filted += 80 * h * ( outYaw - outYaw_filted );

    // 起飞前静态重心补偿：仅在 pre_inFlight 与 inFlight 均为 false（未进入扰动控制）时生效
    // 用于抵消固定的重心偏移（如左重/前重）。参数单位为油门百分比，直接叠加到姿态输出；
    // 该补偿同样计入 ESO 的输入 u，飞机起飞前基本不动，ESO 会把该补偿估计为扰动，
    // 从而在刚起飞、扰动控制器接管时也能延续补偿效果。
    if (inFlight == false && pre_inFlightRP == false)
    {
        outRoll += cfg.stComp[0];
        outPitch += cfg.stComp[2];
    }
    if (inFlight == false && pre_inFlightY == false)
    {
        outYaw += cfg.stComp[4];
    }

    if (Attitude_Control_Enabled)
    {
        double logbuf[12];
        logbuf[0] = target_angular_rate_body.x;
        logbuf[1] = AngularRateCtrl.x;
        logbuf[2] = angular_rate_ESO.x;
        logbuf[3] = target_angular_acceleration.x;
        logbuf[4] = angular_acceleration_ESO.x;
        logbuf[5] = CtrlM_ESO[0]->get_EsMainPower();
        logbuf[6] = outRoll;
        logbuf[7] = disturbance.x;
        logbuf[8] = CtrlM_ESO[0]->get_u();
        SDLog_Msg_DebugVect("rol", logbuf, 9);

        logbuf[0] = target_angular_rate_body.y;
        logbuf[1] = AngularRateCtrl.y;
        logbuf[2] = angular_rate_ESO.y;
        logbuf[3] = target_angular_acceleration.y;
        logbuf[4] = angular_acceleration_ESO.y;
        logbuf[5] = CtrlM_ESO[1]->get_EsMainPower();
        logbuf[6] = outPitch;
        logbuf[7] = disturbance.y;
        logbuf[8] = CtrlM_ESO[1]->get_u();
        SDLog_Msg_DebugVect("pit", logbuf, 9);

        logbuf[0] = target_angular_rate_body.z;
        logbuf[1] = AngularRateCtrl.z;
        logbuf[2] = angular_rate_ESO.z;
        logbuf[3] = target_angular_acceleration.z;
        logbuf[4] = angular_acceleration_ESO.z;
        logbuf[5] = CtrlM_ESO[2]->get_EsMainPower();
        logbuf[6] = outYaw;
        logbuf[7] = disturbance.z;
        logbuf[8] = CtrlM_ESO[2]->get_u();
        SDLog_Msg_DebugVect("yaw", logbuf, 9);

        // 陷波选频观测：20Hz采样记录
        // freqXX[0..19]: ft_data幅度谱bin1~20(10~200Hz,平方域)，由ft_freqs现算 —— 需与ctrlRate(400Hz)对齐
        // fbsXX[0..5]: 当前滤波器1/2频点频宽 + 滤波器3频点频宽(0=关闭)
        // 注意：Heli机型的横滚俯仰ESO为ESO_AngularRateHeli(无此接口)，仅二阶ESO记录
        static uint8_t bsLogCounter = 0;
        if (++bsLogCounter >= 20)
        {
            bsLogCounter = 0;
            double ampLog[FT_N / 2];
            if (CtrlM.RP_eso_type == 0)
            {
                const complex<double> *freqs = ((ESO_AngularRate *) CtrlM_ESO[0])->get_ftFreqs();
                for (uint16_t i = 0; i < FT_N / 2; ++i)
                    ampLog[i] = 4 * freqs[i].get_square();
                SDLog_Msg_DebugVect("freqRol", ampLog + 1, 20);
                freqs = ((ESO_AngularRate *) CtrlM_ESO[1])->get_ftFreqs();
                for (uint16_t i = 0; i < FT_N / 2; ++i)
                    ampLog[i] = 4 * freqs[i].get_square();
                SDLog_Msg_DebugVect("freqPit", ampLog + 1, 20);
            }
            if (CtrlM.Y_eso_type == 0)
            {
                const complex<double> *freqs = ((ESO_AngularRate *) CtrlM_ESO[2])->get_ftFreqs();
                for (uint16_t i = 0; i < FT_N / 2; ++i)
                    ampLog[i] = 4 * freqs[i].get_square();
                SDLog_Msg_DebugVect("freqYaw", ampLog + 1, 20);
            }

            double fbs[6];
            if (CtrlM.RP_eso_type == 0)
            {
                ESO_AngularRate *eso0 = (ESO_AngularRate *) CtrlM_ESO[0];
                fbs[0] = eso0->get_bsFreq1();
                fbs[1] = eso0->get_bsRange1();
                fbs[2] = eso0->get_bsFreq2();
                fbs[3] = eso0->get_bsRange2();
                fbs[4] = eso0->get_bsFreq3();
                fbs[5] = eso0->get_bsRange3();
                SDLog_Msg_DebugVect("fbsRol", fbs, 6);

                ESO_AngularRate *eso1 = (ESO_AngularRate *) CtrlM_ESO[1];
                fbs[0] = eso1->get_bsFreq1();
                fbs[1] = eso1->get_bsRange1();
                fbs[2] = eso1->get_bsFreq2();
                fbs[3] = eso1->get_bsRange2();
                fbs[4] = eso1->get_bsFreq3();
                fbs[5] = eso1->get_bsRange3();
                SDLog_Msg_DebugVect("fbsPit", fbs, 6);
            }
            if (CtrlM.Y_eso_type == 0)
            {
                ESO_AngularRate *eso2 = (ESO_AngularRate *) CtrlM_ESO[2];
                fbs[0] = eso2->get_bsFreq1();
                fbs[1] = eso2->get_bsRange1();
                fbs[2] = eso2->get_bsFreq2();
                fbs[3] = eso2->get_bsRange2();
                fbs[4] = eso2->get_bsFreq3();
                fbs[5] = eso2->get_bsRange3();
                SDLog_Msg_DebugVect("fbsYaw", fbs, 6);
            }
        }

        //		logbuf[0] = CtrlM_ESO[0]->getVibeFreq1();
        //		logbuf[1] = CtrlM_ESO[1]->getVibeFreq1();
        //		logbuf[2] = CtrlM_ESO[2]->getVibeFreq1();
        //		logbuf[3] = CtrlM_ESO[0]->getVibeFreq2();
        //		logbuf[4] = CtrlM_ESO[1]->getVibeFreq2();
        //		logbuf[5] = CtrlM_ESO[2]->getVibeFreq2();
        //		SDLog_Msg_DebugVect( "freq", logbuf, 6 );

        //		logbuf[0] = target_angular_rate_body.z;
        //		logbuf[1] = AngularRateCtrl.z;
        //		logbuf[2] = angular_rate_ESO.z;
        //		logbuf[3] = target_angular_acceleration.z;
        //		logbuf[4] = angular_acceleration_ESO.z;
        //		logbuf[5] = ESO[2].get_EsMainPower();
        //		logbuf[6] = outYaw;
        //		logbuf[7] = disturbance.z;
        //		logbuf[8] = ESO[2].u;
        //		SDLog_Msg_DebugVect( "yaw", logbuf, 9 );
    }
    CtrlM.MotorControl(throttle, outRoll, outPitch, outYaw, DbgSafe);
}

void init_Ctrl_Attitude()
{
    AccZ_filter.set_cutoff_frequency(CtrlRateHz, 1.5);
    forceVect_filter[0].set_cutoff_frequency(CtrlRateHz, 5);
    forceVect_filter[1].set_cutoff_frequency_from(forceVect_filter[0]);
    forceVect_filter[2].set_cutoff_frequency_from(forceVect_filter[0]);
    leanCosin_filter.set_cutoff_frequency_from(forceVect_filter[0]);

    for (uint8_t i = 0; i < Position_Sensors_Count; ++i)
        homeSensorsZ[i].trust = -1;

    // 根据 PWM 通道总数确定舵机数量（与 AuxFuncs 保持一致）
    g_att_st_count = get_PWMChannelsCount();
    // 分配缓冲区（零初始化）: [AttCtrlCfg固定部分(含开头8字节内参)] + [st_count组: min[2],mid[2],max[2]]
    g_att_cfg_buf = new uint8_t[sizeof(AttCtrlCfg) + (uint32_t) g_att_st_count * 6 * sizeof(float)]();

    // 设置注册参数默认值
    cfg.UAVType[0] = UAVType_Rotor4_X;
    cfg.STThrottle[0] = 10;
    cfg.NonlinearFactor[0] = 0.2;
    cfg.FullThrRatio[0] = 0.95;
    cfg.Mode[0] = ATTCTRL_MODE_TURNCOMP_ENA_BIT | ATTCTRL_MODE_RP_ACTIVE_DISTURBANCE_BIT | (1 << 0);
    cfg.STDelay[0] = 3.5;
    cfg.T[0] = 0.07;
    cfg.T2[0] = 0.07;
    cfg.b[0] = 5.5;
    cfg.b[2] = 5.5;
    cfg.b[4] = 1.0;
    cfg.TD4_P1[0] = 15;
    cfg.TD4_P1[2] = 15;
    cfg.TD4_P1[4] = 2;
    cfg.TD4_P2[0] = 15;
    cfg.TD4_P2[2] = 15;
    cfg.TD4_P2[4] = 5;
    cfg.TD4_P3[0] = 25;
    cfg.TD4_P3[2] = 25;
    cfg.TD4_P3[4] = 25;
    cfg.TD4_P4[0] = 25;
    cfg.TD4_P4[2] = 25;
    cfg.TD4_P4[4] = 25;
    cfg.P1[0] = 4;
    cfg.P1[2] = 4;
    cfg.P1[4] = 2;
    cfg.P2[0] = 10;
    cfg.P2[2] = 10;
    cfg.P2[4] = 5;
    cfg.P3[0] = 36;
    cfg.P3[2] = 36;
    cfg.P3[4] = 25;
    cfg.order[0] = 2;
    cfg.order2[0] = 2;
    cfg.orderD[0] = 2;
    cfg.orderD2[0] = 2;
    cfg.beta[0] = 12;
    cfg.beta2[0] = 12;
    cfg.betaD[0] = 7;
    cfg.betaD2[0] = 8;
    cfg.dFilter[0] = 8;
    cfg.beta_h[0] = 2;
    cfg.beta_hAcc[0] = 4;
    cfg.thrComp[0] = 0;
    cfg.beta_thrComp[0] = 15;
    cfg.maxLean[0] = 35;
    cfg.maxRPSp[0] = 350;
    cfg.maxRPAcc[0] = 7000;
    cfg.maxYSp[0] = 80;
    cfg.maxYAcc[0] = 1000;
    cfg.YawPri[0] = 0;
    cfg.windBeta[0] = 2;
    cfg.windDragK[0] = 0; // 0=关闭阻力补偿（仅保留瞬态防护），调试确认后再启用
    cfg.YawTDOrder[0] = 4;
    cfg.upThrRatio[0] = 1.0;
    cfg.aMaxRP[0] = 200;
    cfg.aMaxYaw[0] = 200;
    cfg.rolBS[0] = 0;
    cfg.rolBSBW[0] = 0;
    cfg.rolBS2[0] = 0;
    cfg.rolBSBW2[0] = 0;
    cfg.pitBS[0] = 0;
    cfg.pitBSBW[0] = 0;
    cfg.pitBS2[0] = 0;
    cfg.pitBSBW2[0] = 0;
    cfg.yawBS[0] = 0;
    cfg.yawBSBW[0] = 0;
    cfg.yawBS2[0] = 0;
    cfg.yawBSBW2[0] = 0;
    cfg.st_Freq[0] = 333;
    // 静态重心补偿默认 0（用于抵消固定重心偏移，如左重/前重）
    cfg.stComp[0] = 0; // Roll
    cfg.stComp[2] = 0; // Pitch
    cfg.stComp[4] = 0; // Yaw
    // 舵机参数默认值（动态部分，根据机型 STCount 初始化）
    for (uint8_t i = 0; i < g_att_st_count; ++i)
    {
        _stPtr(i)[0] = 1000;
        _stPtr(i)[2] = 1500;
        _stPtr(i)[4] = 2000;
    }
    // cfg.minThrottlePct 和 cfg.idleThrottlePct 已被零初始化（new uint8_t[...]()）

    // 动态构建参数类型和名称数组
    // 固定部分: (sizeof(AttCtrlCfg) - 8) / 8，减去开头内参槽（minThrottlePct+idleThrottlePct=8字节）
    uint32_t fixed_count = (sizeof(AttCtrlCfg) - offsetof(AttCtrlCfg, UAVType)) / 8;
    uint32_t total_count = fixed_count + (uint32_t) g_att_st_count * 3;
    MAV_PARAM_TYPE *param_types = new MAV_PARAM_TYPE[total_count];
    SName *param_names = new SName[total_count];

    uint32_t p = 0;
    param_types[p] = MAV_PARAM_TYPE_UINT32;
    param_names[p++] = "AC_UAVType";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_STThr";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_NonlinF";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_FullThrR";
    param_types[p] = MAV_PARAM_TYPE_UINT32;
    param_names[p++] = "AC_Mode";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_STDelay";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_T";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_T2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_b";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_b";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_b";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_TD4P1";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_TD4P1";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_TD4P1";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_TD4P2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_TD4P2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_TD4P2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_TD4P3";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_TD4P3";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_TD4P3";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_TD4P4";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_TD4P4";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_TD4P4";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_P1";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_P1";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_P1";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_P2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_P2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_P2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_P3";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_P3";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_P3";
    param_types[p] = MAV_PARAM_TYPE_UINT32;
    param_names[p++] = "AC_Order";
    param_types[p] = MAV_PARAM_TYPE_UINT32;
    param_names[p++] = "AC_Order2";
    param_types[p] = MAV_PARAM_TYPE_UINT32;
    param_names[p++] = "AC_OrderD";
    param_types[p] = MAV_PARAM_TYPE_UINT32;
    param_names[p++] = "AC_OrderD2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Beta";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Beta2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_BetaD";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_BetaD2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_dFilter";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_hBeta";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_hAccBeta";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_thrComp";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_BetaThrComp";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_maxLean";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_maxRPSp";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_maxRPAcc";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_maxYSp";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_maxYAcc";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_YawPri";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_windBeta";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_windDragK";
    param_types[p] = MAV_PARAM_TYPE_INT32;
    param_names[p++] = "AC_YawTDOrder";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_upThrRatio";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_AMaxRP";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_AMaxYaw";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_BS";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_BSBW";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_BS2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_BSBW2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_BS";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_BSBW";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_BS2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_BSBW2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_BS";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_BSBW";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_BS2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_BSBW2";
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_STFreq";
    // 起飞前静态重心补偿（油门百分比，用于抵消固定的重心偏移，如左重/前重）
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Roll_StComp"; // 静态重心补偿Roll
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Pitch_StComp"; // 静态重心补偿Pitch
    param_types[p] = MAV_PARAM_TYPE_REAL32;
    param_names[p++] = "AC_Yaw_StComp"; // 静态重心补偿Yaw
    // 动态舵机参数（根据当前机型 STCount 注册）
    for (uint8_t i = 0; i < g_att_st_count; ++i)
    {
        char buf[12];
        sprintf(buf, "AC_ST%dMin", i + 1);
        param_types[p] = MAV_PARAM_TYPE_REAL32;
        param_names[p++] = buf;
        sprintf(buf, "AC_ST%dMid", i + 1);
        param_types[p] = MAV_PARAM_TYPE_REAL32;
        param_names[p++] = buf;
        sprintf(buf, "AC_ST%dMax", i + 1);
        param_types[p] = MAV_PARAM_TYPE_REAL32;
        param_names[p++] = buf;
    }
    // 注册参数组（基址跳过开头8字节内参，从 UAVType 开始）
    InitialParametersInit(getAttCfgParamBase(), param_types, param_names, total_count);
    ParamGroupRegister("AttCtrl", 6, total_count, param_types, param_names, getAttCfgParamBase());
    delete[] param_types;
    delete[] param_names;
}
