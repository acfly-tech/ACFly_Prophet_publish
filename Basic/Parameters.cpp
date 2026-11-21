#include "Parameters.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "StorageSystem.hpp"
#include "TimeBase.hpp"
#include "drv_CRC.hpp"
#include "mavlink.h"
#include "semphr.h"
#include <map>

using namespace std;

// K9/STM 平台 RTC 时间获取 → FAT 时间戳 (替代 K9 的 RTC_TimeStruct + Get_RTC_Time)
// 注意: FAT 时间戳秒字段仅 5 bit(0-29), 按 fatgen103 规范应存 秒/2 (2秒粒度)。
// 若直接存原始秒, sec>=32 时会溢出污染分钟位(分+1)。update_date 仅作记录,
// 不参与参数加载校验, 修改编码不影响已存参数。
static inline uint32_t Param_GetFileCreateTime()
{
    RTC_TimeStruct RTCTime = Get_RTC_Time();
    return ((RTCTime.Year - 1980) << 25) | (RTCTime.Month << 21) | (RTCTime.Date << 16) | (RTCTime.Hours << 11) | (RTCTime.Minutes << 5) | (RTCTime.Seconds / 2);
}

// 参数组 CRC 算法配置（可在编译期覆盖）
#ifndef PARAMETERS_CRC_CONFIG
#define PARAMETERS_CRC_CONFIG cg_CRC_CONFIG_CRC32
#endif

/*
        ===== 参数组存储格式（v2，带 CRC） =====

        version(0:1) 15:14 为参数类型
                0 - 不加入参数列表的普通参数组（ParamData2 存储）
                1 - 变长参数组
                2 - 加入参数列表的普通参数组（ParamData1 存储）
        params_count/param_length(2:3)
        更新日期(4:7)

        普通参数组（ParamData1, version>>14==2）:
                [ParamGroupInf][ParamData1 * n][uint32_t crc]
                crc 覆盖全部 n 个 ParamData1（32*n 字节）
        普通参数组（ParamData2, version>>14==0）:
                [ParamGroupInf][ParamData2 * n][uint32_t crc]
                crc 覆盖全部 n 个 ParamData2（即全部 data）
        变长参数组（version>>14==1）:
                [VolatileParamGroupInf][ParamUnit * n]
                ParamUnit = [uint8_t data[param_length]][uint32_t crc]
                crc 覆盖该单元前 param_length 字节
                注意 param_length 字段语义保持“用户参数字节数”，实际单元长度=param_length+4
                不需要兼容无CRC旧格式，REGISTER 用 unit_len 整除校验文件长度即可；
                单元自身的 CRC 即为真实完整性保证

        ===== 旧版本兼容（懒升级） =====
        注册时识别到旧格式（文件长度匹配旧版公式）：
                读出旧内容到 RAM、填入 ParamData1（type/name/data 从旧数据迁移）
                标记 PGF_NEED_REWRITE=1，但不立即写盘
        CRC 计算与写盘推迟到 SaveParamGroup / UpdateParam：
                单一 CRC（覆盖全量 data 区），在持 ReadSemphr 期间算完
                持 WriteSemphr 期间写盘（磁盘为原子事务）

        ===== 并发设计 =====
        每组两个锁（ParamGroup 父类字段；普通组用两者，volatile 组仅用 ReadSemphr 作单锁）：
          ReadSemphr —— volatile 组的单锁：保护 RAM data、RAM cache 元数据、flash I/O 全部。
                       普通组的 RAM 读写互斥锁（短临界区 µs 级）。
          WriteSemphr —— 普通组的 flash I/O 串行锁（持锁算 CRC + 写盘）。

        VolatileParamGroup 各路径锁占用（单锁，全程持 ReadSemphr）：
          Read cache 命中：持 ReadSemphr（µs） → RAM memcpy 直接返回
          Read cache miss：持 ReadSemphr 全程 → flush + ReadFile + 更新 RAM cache + 拷贝
          Write cache_mode(st=false 命中)：持 ReadSemphr → 写 RAM cache + 设 ram_updated
          Write(st=true 或越界)：持 ReadSemphr 全程 → 同步 RAM + spans 写盘
          Save：持 ReadSemphr 全程 → 快照脏数据 + 清 ram_updated + CRC + 写盘
          Truncate：持 ReadSemphr 全程 → 清 cache + flash 截断 + 更新 params_count

        不变量：
          - 持 ReadSemphr：可读写 RAM data、params_count、flags、ram_*、flash 文件（无任何并发）
          - 单锁绝对无 race：ram_updated / ram_param_* / params_count / flags 修改与读取天然互斥
          - cache miss 阶段约 ms 级持锁，会阻塞同组的其他 cache hit；嵌入式实际命中率高，可接受
*/

// ParamGroup.flags 位定义
#define PGF_IS_NEW 0x01       // 参数组首次未写入存储器，需要先写头部
#define PGF_NEED_REWRITE 0x02 // 需要全量重写文件（旧版本升级 / CRC 错误 / 默认初始化）

// 内存普通参数组数据
struct ParamGroup
{
    // 参数组名称
    SName name;
    // 状态标志位（PGF_*）
    uint8_t flags;
    // 读访问互斥信号量（保护 RAM data 读 / 局部写）
    SemaphoreHandle_t ReadSemphr;
    // 写访问互斥信号量（串行化磁盘写盘，防止并发 Update 交错）
    SemaphoreHandle_t WriteSemphr;
    // 版本（带类型位）
    uint16_t version;
    // 参数个数
    uint16_t params_count;
    // 参数组数据指针：
    //  ParamData1 类型: 指向 ParamData1[params_count]，布局与存储一致
    //  ParamData2 类型: 指向 ParamData2[params_count]
    void *data;
    // 仅 ParamData1 有效：每参数最小最大边界；NaN 表示该边界不校验
    float *bounds_min;
    float *bounds_max;
};
// 内存可变参数组数据
struct VolatileParamGroup : ParamGroup
{
    uint16_t param_length;    // 用户参数单元长度（不含 CRC）
    uint16_t ram_param_start; // 内存中的参数起始序号（当前缓存范围）
    uint16_t ram_param_count; // 内存中当前加载的有效参数个数（0 表示缓存无效）
    uint16_t ram_capacity;    // RAM 缓冲区物理容量（注册时 ram_params_count，仅用于限制单次加载量）
    bool ram_updated;
};

// 一般参数组头
struct ParamGroupInf
{
    uint16_t version;
    uint16_t params_count;
    uint32_t update_date;
} __AC_PACKED;
// 不定长参数组头（8 字节）
struct VolatileParamGroupInf
{
    uint16_t version;
    uint16_t param_length;
    uint32_t update_date;
} __AC_PACKED;

// 存储格式 + RAM 格式（两者一致）
// 带参数名参数格式（32 字节）
struct ParamData1
{
    uint16_t type;
    uint8_t resv1[6]; // 全 0（磁盘与 RAM 均为 0，无 dirty 复用）
    SName name;
    uint64_t data;
} __AC_PACKED;
// 无参数名参数格式（8 字节）
struct ParamData2
{
    uint64_t data;
} __AC_PACKED;

struct Param
{
    ParamGroup *group;
    // ParamData1 类型: 指向 ParamData1
    // ParamData2 类型: 指向 ParamData2
    void *data;
    uint32_t index;
};

// 参数组表
static map<SName, ParamGroup *> ParamGroups;
// 参数组写互斥锁
static SemaphoreHandle_t ParamGroups_Semphr;
// 参数表
static map<SName, Param> GroupParameters;
// 参数表写互斥锁
static SemaphoreHandle_t GroupParameters_Semphr;
// 参数表长度
static uint32_t ParametersCount = 0;

/* ========== CRC 工具 ========== */

// 计算单段数据的 CRC32（一次性算完，无需多段锁）
// 注意：drv_CRC 的 CRC_Calc 形参 length 为 uint16_t，本工具内做范围校验。
// 实际使用场景（ParamData1=32B/参数, ParamData2=n*8B, Volatile 单元≤256B）均远小于 64KB。
// buffer_size 传入 length：长度即缓冲区可用空间（精确用尽），
// 但若 data 起始地址未 cache行对齐 或 aligned_size > buffer_size，drv_CRC 会自动走内部拷贝缓冲区。
static bool Param_CalcCRC(const uint8_t *data, uint32_t length, uint32_t *res)
{
    if (length > 0xFFFF)
    {
        *res = 0;
        return false;
    }
    return CRC_Calc(PARAMETERS_CRC_CONFIG, data, (uint16_t) length, res, (uint16_t) length);
}

/* ========== 大小工具 ========== */

static inline uint32_t ParamGroup1_StoredLengthNoCRC(uint32_t params_count)
{
    return sizeof(ParamGroupInf) + sizeof(ParamData1) * params_count;
}
static inline uint32_t ParamGroup1_StoredLengthWithCRC(uint32_t params_count)
{
    return sizeof(ParamGroupInf) + sizeof(ParamData1) * params_count + sizeof(uint32_t);
}
static inline uint32_t ParamGroup2_StoredLengthNoCRC(uint32_t params_count)
{
    return sizeof(ParamGroupInf) + sizeof(ParamData2) * params_count;
}
static inline uint32_t ParamGroup2_StoredLengthWithCRC(uint32_t params_count)
{
    return sizeof(ParamGroupInf) + sizeof(ParamData2) * params_count + sizeof(uint32_t);
}
static inline uint32_t Vol_UnitLenWithCRC(uint16_t param_length)
{
    return (uint32_t) param_length + sizeof(uint32_t);
}
static inline uint32_t Vol_StoredLengthWithCRC(uint16_t param_length, uint32_t n)
{
    return sizeof(VolatileParamGroupInf) + Vol_UnitLenWithCRC(param_length) * n;
}

/* ========== Volatile 组 flash I/O 共享辅助（带逐单元 CRC） ========== */
//
// 新 CRC 格式下 volatile 组在 flash 中每单元为 [data(param_length) | crc(4)]，
// RAM 中只存 param_length 字节（省内存）。因此每次写盘需逐单元计算 CRC 并拼接，
// 每次读取需逐单元校验 CRC 后 strip。这两个操作在 Read/Write/Save/Truncate 中重复出现，
// 统一为以下两个辅助函数：
//   - Vol_FlushUnits: RAM → flash（逐单元计算 CRC + spans 原子写）
//   - Vol_ReadUnits:  flash → RAM（逐单元读 + CRC 校验 + strip）
// 调用者均已持有 ReadSemphr（volatile 组单锁），函数内不再加锁。
//

// 将 RAM 缓冲中起始的 count 个单元（每单元 param_length 字节，紧跟 ram_base）
// 逐单元计算 CRC，拼接为 unit_len 格式，用 spans 原子写入 flash 中 [file_start, file_start+count) 位置。
// wr_inf 为本次写入的 header（总是同时更新到 offset 0）。
// is_new=true 时用 truncate 模式（重建文件）。
// 返回 SS_OK / SS_ERR / SS_TimeOut。
static inline SS_RESULT Vol_FlushUnits(const char *name_ch, const VolatileParamGroupInf *wr_inf, uint16_t param_length, uint32_t unit_len, const uint8_t *ram_base, uint32_t count, uint32_t file_start,
                                       bool is_new)
{
    if (count == 0)
        return SS_OK;
    uint8_t *wbuf = new uint8_t[count * unit_len];
    for (uint32_t i = 0; i < count; ++i)
    {
        memcpy(wbuf + i * unit_len, ram_base + i * param_length, param_length);
        uint32_t crc = 0;
        Param_CalcCRC(wbuf + i * unit_len, param_length, &crc);
        memcpy(wbuf + i * unit_len + param_length, &crc, sizeof(uint32_t));
    }
    StorageSpan spans[2];
    spans[0].content = wr_inf;
    spans[0].offset = 0;
    spans[0].length = sizeof(VolatileParamGroupInf);
    spans[1].content = wbuf;
    spans[1].offset = sizeof(VolatileParamGroupInf) + file_start * unit_len;
    spans[1].length = count * unit_len;
    SS_RESULT sr = InternalStorage_WriteFileSpans("Config", name_ch, spans, 2, is_new);
    delete[] wbuf;
    return sr;
}

// 从 flash 中 [file_start, file_start+count) 位置读取 count 个单元（每单元 unit_len 字节），
// 逐单元校验 CRC（3 次重试），strip CRC 后将 param_length 字节/单元拷贝到 ram_base（连续，无间隙）。
// 返回 true=成功，false=读取失败或 CRC 校验失败。
static inline bool Vol_ReadUnits(const char *name_ch, uint16_t param_length, uint32_t unit_len, uint8_t *ram_base, uint32_t file_start, uint32_t count)
{
    if (count == 0)
        return true;
    uint8_t *rdbuf = new uint8_t[count * unit_len];
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt)
    {
        uint32_t got = count * unit_len;
        if (InternalStorage_ReadFile("Config", name_ch, rdbuf, &got, sizeof(VolatileParamGroupInf) + file_start * unit_len, count * unit_len) == SS_OK && got == count * unit_len)
        {
            bool crc_ok = true;
            for (uint32_t i = 0; i < count; ++i)
            {
                uint8_t *pu = rdbuf + i * unit_len;
                uint32_t want = 0, calc = 0;
                memcpy(&want, pu + param_length, sizeof(uint32_t));
                Param_CalcCRC(pu, param_length, &calc);
                if (want != calc)
                {
                    crc_ok = false;
                    break;
                }
            }
            if (crc_ok)
                ok = true;
        }
        if (!ok)
            os_delay(0.005);
    }
    if (ok)
    {
        for (uint32_t i = 0; i < count; ++i)
            memcpy(ram_base + i * param_length, rdbuf + i * unit_len, param_length);
    }
    delete[] rdbuf;
    return ok;
}
/* ========== 旧格式数据读取（不写盘，仅填入 RAM） ========== */

// 旧参数类型 → 新参数类型的值转换（版本迁移时 type 变更场景）
// 策略：先按 old_type 把 data 解码为 double（REAL32 走 memcpy，NaN→0；整数按低位截断解释符号）
//      再把 double 按 new_type 范围饱和裁剪后编码回 uint64_t
// 存储约定与 Param_ClampCheckFloat 一致：数值在低 N 位，高位无关
static uint64_t Param_ConvertType(uint16_t old_type, uint64_t old_data, uint16_t new_type)
{
    if (old_type == new_type)
        return old_data;

    // 1. old_type → double（NaN 安全，±Inf 保留待下阶段饱和）
    double val = 0.0;
    switch ((MAV_PARAM_TYPE) old_type)
    {
        case MAV_PARAM_TYPE_REAL32:
        {
            float f;
            memcpy(&f, &old_data, sizeof(float));
            val = (f != f) ? 0.0 : (double) f;
            break;
        }
        case MAV_PARAM_TYPE_INT32:
            val = (double) (int32_t) (uint32_t) old_data;
            break;
        case MAV_PARAM_TYPE_UINT32:
            val = (double) (uint32_t) old_data;
            break;
        case MAV_PARAM_TYPE_INT16:
            val = (double) (int16_t) (uint16_t) old_data;
            break;
        case MAV_PARAM_TYPE_UINT16:
            val = (double) (uint16_t) old_data;
            break;
        case MAV_PARAM_TYPE_INT8:
            val = (double) (int8_t) (uint8_t) old_data;
            break;
        case MAV_PARAM_TYPE_UINT8:
            val = (double) (uint8_t) old_data;
            break;
        default:
            return old_data; // 未知旧类型，原样返回
    }

    // 2. double → new_type（饱和裁剪，覆盖 ±Inf；此阶段 val 不可能是 NaN 故无需再判）
    switch ((MAV_PARAM_TYPE) new_type)
    {
        case MAV_PARAM_TYPE_REAL32:
        {
            float f = (float) val;
            uint64_t out = 0;
            memcpy(&out, &f, sizeof(float));
            return out;
        }
        case MAV_PARAM_TYPE_INT8:
            if (val >= 127.0)
                val = 127.0;
            else if (val <= -128.0)
                val = -128.0;
            return (uint64_t) (uint8_t) (int8_t) val;
        case MAV_PARAM_TYPE_UINT8:
            if (val >= 255.0)
                val = 255.0;
            else if (val <= 0.0)
                val = 0.0;
            return (uint64_t) (uint8_t) val;
        case MAV_PARAM_TYPE_INT16:
            if (val >= 32767.0)
                val = 32767.0;
            else if (val <= -32768.0)
                val = -32768.0;
            return (uint64_t) (uint16_t) (int16_t) val;
        case MAV_PARAM_TYPE_UINT16:
            if (val >= 65535.0)
                val = 65535.0;
            else if (val <= 0.0)
                val = 0.0;
            return (uint64_t) (uint16_t) val;
        case MAV_PARAM_TYPE_INT32:
            if (val >= 2147483647.0)
                val = 2147483647.0;
            else if (val <= -2147483648.0)
                val = -2147483648.0;
            return (uint64_t) (uint32_t) (int32_t) val;
        case MAV_PARAM_TYPE_UINT32:
            if (val >= 4294967295.0)
                val = 4294967295.0;
            else if (val <= 0.0)
                val = 0.0;
            return (uint64_t) (uint32_t) val;
        default:
            return old_data; // 未知新类型，原样返回
    }
}

// 从旧格式/新格式文件缓冲读取 ParamData1 → 填入 RAM 中的 ParamData1[new_params_count]
// file_params_count 为文件中实际存储的参数个数（可能与 new_params_count 不同：增删参数场景）
// 第一段循环按 file_params_count 从文件构建名称→{type,data} 映射表
// 第二段循环按 new_params_count 填入 RAM：同名 → 复用旧值（同类型直接用，异类型按语义值转换）；否则用 initial_params / 0
// min/max 由调用者在 ParamGroup.bounds_min/max 中保存（本函数不处理）
static void ParamGroup1_LoadToRam(const uint8_t *filebuf, uint32_t file_params_count, uint32_t new_params_count, const MAV_PARAM_TYPE param_types[], const SName param_names[],
                                  const uint64_t initial_params[], uint8_t *newParamGroupData)
{
    struct OldEntry
    {
        SName name;
        uint16_t type;
        uint64_t data;
    };
    map<SName, OldEntry> oldmap;
    for (uint32_t i = 0; i < file_params_count; ++i)
    {
        const ParamData1 *pd = (const ParamData1 *) (filebuf + sizeof(ParamGroupInf) + i * sizeof(ParamData1));
        OldEntry e;
        e.name = pd->name;
        e.type = pd->type;
        e.data = pd->data;
        oldmap.insert(pair<SName, OldEntry>(pd->name, e));
    }
    for (uint32_t i = 0; i < new_params_count; ++i)
    {
        ParamData1 *dst = (ParamData1 *) (newParamGroupData + i * sizeof(ParamData1));
        dst->type = (uint16_t) param_types[i];
        memset(dst->resv1, 0, sizeof(dst->resv1)); // dirty=0, 其他保留位也清 0
        dst->name = param_names[i];
        auto it = oldmap.find(param_names[i]);
        if (it != oldmap.end())
        {
            if (it->second.type == (uint16_t) param_types[i])
                dst->data = it->second.data; // 同类型，直接复用
            else
                dst->data = Param_ConvertType(it->second.type, it->second.data, (uint16_t) param_types[i]); // 类型变更，按语义值转换
        }
        else if (initial_params != 0)
            dst->data = initial_params[i];
        else
            dst->data = 0;
    }
}

// 范围校验：min/max 非 NaN 时校验指定参数类型。超限返回 false。
// REAL32 → float 比较
// INT32/UINT16/INT16/UINT8/INT8 → 整型比较（按符号解释）
static bool Param_ClampCheckFloat(uint16_t type, uint64_t data_u, float min_v, float max_v)
{
    if (min_v != min_v && max_v != max_v)
        return true; // 两个边界都是 NaN，不校验

    switch ((MAV_PARAM_TYPE) type)
    {
        case MAV_PARAM_TYPE_REAL32:
        {
            float f;
            memcpy(&f, &data_u, sizeof(float));
            if (min_v == min_v && f < min_v)
                return false;
            if (max_v == max_v && f > max_v)
                return false;
            return true;
        }
        case MAV_PARAM_TYPE_INT32:
        {
            int32_t v = (int32_t) (uint32_t) data_u;
            if (min_v == min_v && (float) v < min_v)
                return false;
            if (max_v == max_v && (float) v > max_v)
                return false;
            return true;
        }
        case MAV_PARAM_TYPE_UINT32:
        {
            uint32_t v = (uint32_t) data_u;
            if (min_v == min_v && (float) v < min_v)
                return false;
            if (max_v == max_v && (float) v > max_v)
                return false;
            return true;
        }
        case MAV_PARAM_TYPE_INT16:
        {
            int16_t v = (int16_t) (uint16_t) data_u;
            if (min_v == min_v && (float) v < min_v)
                return false;
            if (max_v == max_v && (float) v > max_v)
                return false;
            return true;
        }
        case MAV_PARAM_TYPE_UINT16:
        {
            uint16_t v = (uint16_t) data_u;
            if (min_v == min_v && (float) v < min_v)
                return false;
            if (max_v == max_v && (float) v > max_v)
                return false;
            return true;
        }
        case MAV_PARAM_TYPE_INT8:
        {
            int8_t v = (int8_t) (uint8_t) data_u;
            if (min_v == min_v && (float) v < min_v)
                return false;
            if (max_v == max_v && (float) v > max_v)
                return false;
            return true;
        }
        case MAV_PARAM_TYPE_UINT8:
        {
            uint8_t v = (uint8_t) data_u;
            if (min_v == min_v && (float) v < min_v)
                return false;
            if (max_v == max_v && (float) v > max_v)
                return false;
            return true;
        }
        default:
            return true; // 未知类型不校验
    }
}

/*
        可变长参数组注册（所有名称小于等于16字节）
        注意：仅可在参数表初始化完成前调用
        name：参数组名称
        param_version：参数版本
        param_length：单个参数长度（不含 CRC）
        ram_params_count：在ram中可存放的参数单元个数

        返回值：
        PR_ERR：非法参数或操作失败
        PR_Existed: 参数表中已有同名参数
        PR_NewParam：注册的参数是新参数（存储器里没有）
        PR_OK：注册的参数在存储器已找到并更新了param
*/
PR_RESULT VolatileParamGroupRegister(SName name, uint16_t version, uint16_t param_length, uint32_t ram_params_count)
{
    if (param_length == 0)
        return PR_ERR;
    if (ram_params_count == 0 || ram_params_count > MAX_VOLATILE_PARAMS)
        return PR_ERR;

    version &= 0x3f;
    version |= (1 << 14);

    uint32_t unit_len = Vol_UnitLenWithCRC(param_length);

    if (ram_params_count > (0xFFFFFFFFUL - sizeof(VolatileParamGroupInf)) / unit_len)
        return PR_ERR;

    char name_ch[17];
    name.get_CharStr(name_ch);

    uint32_t stored_len = 0;
    SS_RESULT sres = InternalStorage_GetFileSize("Config", name_ch, &stored_len);

    // RAM 只存 param_length 字节/单元（磁盘存 unit_len=param_length+CRC），省 ~6% 内存
    uint32_t ram_len = sizeof(VolatileParamGroupInf) + (uint32_t) param_length * ram_params_count;
    uint8_t *newParamGroupData = new uint8_t[ram_len];

    PR_RESULT rt_res = PR_ERR;
    uint32_t params_count_in_storage = 0;

    if (sres == SS_OK && stored_len >= sizeof(VolatileParamGroupInf))
    {
        VolatileParamGroupInf inf_in = {0, 0, 0};
        uint32_t head_rd = sizeof(VolatileParamGroupInf);
        if (InternalStorage_ReadFile("Config", name_ch, &inf_in, &head_rd, 0, sizeof(VolatileParamGroupInf)) == SS_OK)
        {
            bool version_match = ((inf_in.version & 0x3fff) == (version & 0x3fff)) && inf_in.param_length == param_length;
            // 文件数据区能按 unit_len 整除即合法（每单元自身的 CRC 已校验完整性）
            if (version_match && ((stored_len - sizeof(VolatileParamGroupInf)) % unit_len) == 0)
            {
                uint32_t saved_n = (stored_len - sizeof(VolatileParamGroupInf)) / unit_len;
                uint32_t rd_unit = saved_n <= ram_params_count ? saved_n : ram_params_count;
                // 文件每单元含 CRC，RAM 不存 CRC；用辅助函数读 + CRC 校验 + strip 到 RAM
                uint8_t *ram_dst = newParamGroupData + sizeof(VolatileParamGroupInf);
                if (Vol_ReadUnits(name_ch, param_length, unit_len, ram_dst, 0, rd_unit))
                {
                    // 复制 group 头部
                    memcpy(newParamGroupData, &inf_in, sizeof(VolatileParamGroupInf));
                    params_count_in_storage = saved_n;
                    rt_res = PR_OK;
                }
            }
        }
    }

    if (rt_res != PR_OK)
    {
        uint32_t file_create_time = Param_GetFileCreateTime();
        VolatileParamGroupInf *inf = (VolatileParamGroupInf *) newParamGroupData;
        inf->version = version;
        inf->param_length = param_length;
        inf->update_date = file_create_time;
        rt_res = PR_NewParam;
    }

    VolatileParamGroup *newParamGroup = new VolatileParamGroup();
    newParamGroup->name = name;
    newParamGroup->version = version;
    newParamGroup->param_length = param_length;
    // 注册成功时直接建立 RAM cache：rd_unit 个单元已读到 group->data，避免首次访问重复读 flash
    if (rt_res == PR_OK)
    {
        uint32_t saved_n = (stored_len - sizeof(VolatileParamGroupInf)) / unit_len;
        uint32_t rd_unit = saved_n <= ram_params_count ? saved_n : ram_params_count;
        newParamGroup->ram_param_start = 0;
        newParamGroup->ram_param_count = (uint16_t) rd_unit;
    }
    else
    {
        // 新参数组：cache 无效（start=0xFFFF 不会与 valid start 相等）
        newParamGroup->ram_param_start = 0xFFFF;
        newParamGroup->ram_param_count = 0;
    }
    newParamGroup->ram_capacity = (uint16_t) ram_params_count;
    newParamGroup->params_count = (uint16_t) params_count_in_storage;
    newParamGroup->flags = (rt_res == PR_NewParam) ? PGF_IS_NEW : 0;
    newParamGroup->ReadSemphr = xSemaphoreCreateMutex();
    newParamGroup->WriteSemphr = nullptr; // volatile 组单锁方案，不用此锁
    newParamGroup->data = newParamGroupData;
    newParamGroup->ram_updated = false;
    // 补充判定：rt_res!=PR_OK 且有 params_count_in_storage 但 RAM 未读全→让 RAM cache 不命中
    // rt_res==PR_OK 时由 ReadVolatileParamGroup 首次访问时 lazy 加载单元

    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        if (ParamGroups.find(name) != ParamGroups.end())
        {
            vSemaphoreDelete(newParamGroup->ReadSemphr);
            delete[] newParamGroupData;
            delete newParamGroup;
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_Existed;
        }
        ParamGroups.insert(pair<SName, VolatileParamGroup *>(name, newParamGroup));
        xSemaphoreGive(ParamGroups_Semphr);
    }

    return PR_OK;
}

/*
        参数组注册（带名称、类型、初始值，加可选 min/max）
        name：参数组名称
        param_version：参数版本（低 14 位有效）
        params_count：参数个数
        param_types：参数类型数组（params_count 个）
        param_names：参数名称数组（params_count 个，NULL 表示不加入参数列表）
        initial_params：初始值数组（可 NULL）
        min_values：float 最小值数组（可 NULL，NaN 表示不校验）
        max_values：float 最大值数组（可 NULL，NaN 表示不校验）

        仅 param_names != NULL 时进入“带名称”路径（addParameters=true）。
*/
PR_RESULT ParamGroupRegister(SName name, uint16_t version, uint32_t params_count, const MAV_PARAM_TYPE param_types[], const SName param_names[], const uint64_t initial_params[],
                             const float min_values[], const float max_values[])
{
    bool addParameters = (param_names != 0);

    uint16_t reg_version = version & 0x3f;
    if (addParameters)
        reg_version |= (2 << 14);
    else
        reg_version |= (0 << 14);

    char name_ch[17];
    name.get_CharStr(name_ch);

    uint32_t ram_len = addParameters ? sizeof(ParamData1) * params_count : sizeof(ParamData2) * params_count;
    uint8_t *newParamGroupData = new uint8_t[ram_len];

    // 准备 min/max 数组，最终存入 ParamGroup.bounds_min/max
    // bmin 仅当 min_values!=0 时分配，bmax 仅当 max_values!=0 时分配（省内存）
    // 缺省侧保持 nullptr，UpdateParam 时传 NaN 给 ClampCheckFloat（已处理 NaN 跳过）
    float *bmin = nullptr, *bmax = nullptr;
    if (addParameters)
    {
        if (min_values != 0)
        {
            bmin = new float[params_count];
            for (uint32_t i = 0; i < params_count; ++i)
                bmin[i] = min_values[i];
        }
        if (max_values != 0)
        {
            bmax = new float[params_count];
            for (uint32_t i = 0; i < params_count; ++i)
                bmax[i] = max_values[i];
        }
    }

    // flags 初始化
    uint8_t new_flags = 0;
    // 仅“文件读出成功但无 CRC”为 true（注册末尾用于触发一次 SaveParamGroup 升级 CRC）。
    // 与 PGF_NEED_REWRITE 区分：新参数/CRC 失败/参数增删都不会设置本标志，避免启动时无谓写盘。
    bool no_crc_upgrade_needed = false;

    uint32_t stored_len = 0;
    bool storage_found = (InternalStorage_GetFileSize("Config", name_ch, &stored_len) == SS_OK);

    bool loaded = false;
    uint8_t *filebuf = nullptr;

    if (storage_found && stored_len >= sizeof(ParamGroupInf))
    {
        // 先读头部判断版本 + 文件实际参数个数（file_pc 可能与注册 params_count 不同：增删参数）
        ParamGroupInf inf_hdr;
        uint32_t hdr_got = sizeof(ParamGroupInf);
        if (InternalStorage_ReadFile("Config", name_ch, &inf_hdr, &hdr_got, 0, sizeof(ParamGroupInf)) == SS_OK && hdr_got == sizeof(ParamGroupInf) && inf_hdr.version == reg_version)
        {
            uint32_t data_unit = addParameters ? sizeof(ParamData1) : sizeof(ParamData2);
            uint32_t file_pc = inf_hdr.params_count;

            // 用文件实际参数个数推算文件长度，与 stored_len 比对以判定格式
            uint32_t len_with_crc = sizeof(ParamGroupInf) + data_unit * file_pc + sizeof(uint32_t);
            uint32_t len_no_crc = sizeof(ParamGroupInf) + data_unit * file_pc;
            bool has_crc = (stored_len == len_with_crc);
            bool no_crc = (stored_len == len_no_crc);

            // ParamData1（带名称）：允许 file_pc ≠ params_count，按名称迁移
            // ParamData2（无名称）：必须 file_pc == params_count 才能直接 memcpy
            bool count_ok = addParameters ? true : (file_pc == params_count);

            if ((has_crc || no_crc) && count_ok)
            {
                filebuf = new uint8_t[stored_len];
                uint32_t got = stored_len;
                if (InternalStorage_ReadFile("Config", name_ch, filebuf, &got, 0, stored_len) == SS_OK && got == stored_len)
                {
                    // 旧格式无 CRC，直接视为通过；新格式做 3 次重读校验
                    bool crc_pass = true;
                    if (has_crc)
                    {
                        crc_pass = false;
                        // 单一 CRC 覆盖全部 data（file_pc 个参数），尾部 4 字节
                        for (int attempt = 0; attempt < 3 && !crc_pass; ++attempt)
                        {
                            uint32_t want = 0, calc = 0;
                            memcpy(&want, filebuf + sizeof(ParamGroupInf) + data_unit * file_pc, sizeof(uint32_t));
                            Param_CalcCRC(filebuf + sizeof(ParamGroupInf), data_unit * file_pc, &calc);
                            crc_pass = (want == calc);
                            if (!crc_pass)
                            {
                                uint32_t got2 = stored_len;
                                if (InternalStorage_ReadFile("Config", name_ch, filebuf, &got2, 0, stored_len) != SS_OK || got2 != stored_len)
                                    break;
                            }
                        }
                    }

                    if (crc_pass)
                    {
                        if (addParameters)
                            ParamGroup1_LoadToRam(filebuf, file_pc, params_count, param_types, param_names, initial_params, newParamGroupData);
                        else
                            memcpy(newParamGroupData, filebuf + sizeof(ParamGroupInf), sizeof(ParamData2) * params_count);
                        loaded = true;
                        // 需要全量重写：旧格式升级 / 参数个数变更（文件尺寸与当前不匹配）
                        if (no_crc)
                        {
                            new_flags |= PGF_NEED_REWRITE;
                            no_crc_upgrade_needed = true; // 仅记录“无 CRC 旧格式成功加载”
                        }
                        if (file_pc != params_count)
                            new_flags |= PGF_NEED_REWRITE;
                    }
                    else
                    {
                        // CRC 失败（仅新格式）：回默认值，标记 NEED_REWRITE 以便下次 Save 修复
                        new_flags |= PGF_NEED_REWRITE;
                    }
                }
            }
        }
    }

    if (filebuf)
    {
        delete[] filebuf;
        filebuf = nullptr;
    }

    if (!loaded)
    {
        // 视为新参数 / CRC 失败 / 旧格式失败 → 回默认
        new_flags |= (PGF_IS_NEW | PGF_NEED_REWRITE);

        if (addParameters)
        {
            for (uint32_t i = 0; i < params_count; ++i)
            {
                ParamData1 *dst = (ParamData1 *) (newParamGroupData + i * sizeof(ParamData1));
                dst->type = (uint16_t) param_types[i];
                memset(dst->resv1, 0, sizeof(dst->resv1)); // dirty=0
                dst->name = param_names[i];
                dst->data = (initial_params != 0) ? initial_params[i] : 0;
            }
        }
        else
        {
            for (uint32_t i = 0; i < params_count; ++i)
            {
                ParamData2 *dst = (ParamData2 *) (newParamGroupData + i * sizeof(ParamData2));
                dst->data = (initial_params != 0) ? initial_params[i] : 0;
            }
        }
    }

    // bmin/bmax 转交给 ParamGroup（不释放）

    ParamGroup *group = new ParamGroup();
    group->name = name;
    group->version = reg_version;
    group->params_count = (uint16_t) params_count;
    group->flags = new_flags;
    group->ReadSemphr = xSemaphoreCreateMutex();
    group->WriteSemphr = xSemaphoreCreateMutex();
    group->data = newParamGroupData;
    group->bounds_min = bmin; // 可为 nullptr（ParamData2 路径）
    group->bounds_max = bmax;
    bmin = bmax = nullptr; // 所有权转移

    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        if (ParamGroups.find(name) != ParamGroups.end())
        {
            vSemaphoreDelete(group->ReadSemphr);
            vSemaphoreDelete(group->WriteSemphr);
            delete[] newParamGroupData;
            if (group->bounds_min)
                delete[] group->bounds_min;
            if (group->bounds_max)
                delete[] group->bounds_max;
            delete group;
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_Existed;
        }
        ParamGroups.insert(pair<SName, ParamGroup *>(name, group));
        xSemaphoreGive(ParamGroups_Semphr);
    }

    if (addParameters)
    {
        if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
        {
            for (uint32_t i = 0; i < params_count; ++i)
            {
                Param param;
                param.group = group;
                param.data = (newParamGroupData + i * sizeof(ParamData1));
                param.index = ParametersCount;
                GroupParameters.insert(pair<SName, Param>(param_names[i], param));
                ++ParametersCount;
            }
            xSemaphoreGive(GroupParameters_Semphr);
        }
    }

    // 注册期发现“文件是旧格式（无 CRC）且成功加载到 RAM” → 立即写盘升级到带 CRC 格式。
    // 新参数/CRC 失败/参数增删（PGF_IS_NEW/PGF_NEED_REWRITE 但非 no_crc）不触发此处，遵延迟写盘语义。
    // 注册是单线程 init 阶段进行，此处调用 SaveParamGroup 与并发读写无冲突。
    if (no_crc_upgrade_needed)
    {
        if (name != "WGA_Code")
            SaveParamGroup(name, 5.0);
    }

    return PR_OK;
}

/*
        获取参数表参数个数
*/
PR_RESULT GetParametersCount(uint32_t *count)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;
    *count = ParametersCount;
    return PR_OK;
}

parametersIterator::~parametersIterator()
{
    if (this->it)
        delete (map<SName, Param>::iterator *) this->it;
}

bool parametersIterator::reset()
{
    if (GroupParameters_Semphr == nullptr)
        return 0;
    if (this->it == 0)
        this->it = new map<SName, Param>::iterator;
    if (this->it)
    {
        if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
        {
            *(map<SName, Param>::iterator *) (this->it) = GroupParameters.begin();
            xSemaphoreGive(GroupParameters_Semphr);
        }
        return true;
    }
    return false;
}
void parametersIterator::setInAvailable()
{
    if (this->it)
    {
        delete (map<SName, Param>::iterator *) this->it;
        this->it = 0;
    }
}
parametersIterator &parametersIterator::operator++()
{
    if (this->it)
    {
        if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
        {
            if (*(map<SName, Param>::iterator *) (this->it) != GroupParameters.end())
                ++(*(map<SName, Param>::iterator *) (this->it));
            xSemaphoreGive(GroupParameters_Semphr);
        }
    }
    return *this;
}
parametersIterator &parametersIterator::operator--()
{
    if (this->it)
    {
        if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
        {
            if (*(map<SName, Param>::iterator *) (this->it) != GroupParameters.begin())
                --(*(map<SName, Param>::iterator *) (this->it));
            xSemaphoreGive(GroupParameters_Semphr);
        }
    }
    return *this;
}
bool parametersIterator::operator==(parametersIterator &ob)
{
    if (this->it && ob.it && (*(map<SName, Param>::iterator *) (this->it)) == (*(map<SName, Param>::iterator *) (ob.it)))
        return true;
    else
        return false;
}
bool parametersIterator::operator!=(parametersIterator &ob)
{
    if (this->it == 0 || ob.it == 0 || (*(map<SName, Param>::iterator *) (this->it)) != (*(map<SName, Param>::iterator *) (ob.it)))
        return true;
    else
        return false;
}
bool parametersIterator::isBegin()
{
    if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
    {
        bool result = *(map<SName, Param>::iterator *) (this->it) == GroupParameters.begin();
        xSemaphoreGive(GroupParameters_Semphr);
        return result;
    }
    return false;
}
bool parametersIterator::isEnd()
{
    if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
    {
        bool result = *(map<SName, Param>::iterator *) (this->it) == GroupParameters.end();
        xSemaphoreGive(GroupParameters_Semphr);
        return result;
    }
    return true;
}

PR_RESULT parametersIterator::read(SName *name, uint32_t *index, MAV_PARAM_TYPE *type, uint64_t *value, bool *is_new, double TIMEOUT)
{
    if (this->it == 0 || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    SName param_name;
    Param param;
    if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, Param>::iterator cIt = *(map<SName, Param>::iterator *) (this->it);
        if (cIt == GroupParameters.end())
        {
            xSemaphoreGive(GroupParameters_Semphr);
            return PR_ERR;
        }
        param_name = cIt->first;
        param = cIt->second;
        xSemaphoreGive(GroupParameters_Semphr);
    }
    else
        return PR_TimeOut;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE)
    {
        bool is_param1 = (param.group->version & (1 << 15)) != 0;
        if (is_param1)
        {
            ParamData1 *p = (ParamData1 *) param.data;
            if (name != 0)
                *name = param_name;
            if (type != 0)
                *type = (MAV_PARAM_TYPE) p->type;
            if (value != 0)
                memcpy((uint8_t *) value, (uint8_t *) &p->data, sizeof(uint64_t));
        }
        else
        {
            ParamData2 *p = (ParamData2 *) param.data;
            if (name != 0)
                *name = param_name;
            if (type != 0)
                *type = (MAV_PARAM_TYPE) 0;
            if (value != 0)
                memcpy((uint8_t *) value, (uint8_t *) &p->data, sizeof(uint64_t));
        }
        if (index != 0)
            *index = param.index;
        if (is_new != 0)
            *is_new = ((param.group->flags & PGF_IS_NEW) != 0);
        xSemaphoreGive(param.group->ReadSemphr);
        return PR_OK;
    }
    return PR_TimeOut;
}

PR_RESULT ReadParam(SName name, uint32_t *index, MAV_PARAM_TYPE *type, uint64_t *value, bool *is_new, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    Param param;
    if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, Param>::iterator it = GroupParameters.find(name);
        if (it == GroupParameters.end())
        {
            xSemaphoreGive(GroupParameters_Semphr);
            return PR_ERR;
        }
        param = it->second;
        xSemaphoreGive(GroupParameters_Semphr);
    }
    else
        return PR_TimeOut;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE)
    {
        bool is_param1 = (param.group->version & (1 << 15)) != 0;
        if (is_param1)
        {
            ParamData1 *p = (ParamData1 *) param.data;
            if (index != 0)
                *index = param.index;
            if (type != 0)
                *type = (MAV_PARAM_TYPE) p->type;
            if (value != 0)
                memcpy((uint8_t *) value, (uint8_t *) &p->data, sizeof(uint64_t));
        }
        else
        {
            ParamData2 *p = (ParamData2 *) param.data;
            if (index != 0)
                *index = param.index;
            if (type != 0)
                *type = (MAV_PARAM_TYPE) 0;
            if (value != 0)
                memcpy((uint8_t *) value, (uint8_t *) &p->data, sizeof(uint64_t));
        }
        if (is_new != 0)
            *is_new = ((param.group->flags & PGF_IS_NEW) != 0);
        xSemaphoreGive(param.group->ReadSemphr);
        return PR_OK;
    }
    return PR_TimeOut;
}

PR_RESULT ReadParam(uint32_t index, SName *name, MAV_PARAM_TYPE *type, uint64_t *value, bool *is_new, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;
    if (index >= ParametersCount)
        return PR_ERR;

    Param param;
    SName param_name;
    if (xSemaphoreTake(GroupParameters_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, Param>::iterator it = GroupParameters.begin();
        for (; it != GroupParameters.end(); ++it)
        {
            if (it->second.index == index)
                break;
        }
        if (it == GroupParameters.end())
        {
            xSemaphoreGive(GroupParameters_Semphr);
            return PR_ERR;
        }
        param_name = it->first;
        param = it->second;
        xSemaphoreGive(GroupParameters_Semphr);
    }
    else
        return PR_TimeOut;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE)
    {
        bool is_param1 = (param.group->version & (1 << 15)) != 0;
        if (is_param1)
        {
            ParamData1 *p = (ParamData1 *) param.data;
            if (name != 0)
                *name = param_name;
            if (type != 0)
                *type = (MAV_PARAM_TYPE) p->type;
            if (value != 0)
                memcpy((uint8_t *) value, (uint8_t *) &p->data, sizeof(uint64_t));
        }
        else
        {
            ParamData2 *p = (ParamData2 *) param.data;
            if (name != 0)
                *name = param_name;
            if (type != 0)
                *type = (MAV_PARAM_TYPE) 0;
            if (value != 0)
                memcpy((uint8_t *) value, (uint8_t *) &p->data, sizeof(uint64_t));
        }
        if (is_new != 0)
            *is_new = ((param.group->flags & PGF_IS_NEW) != 0);
        xSemaphoreGive(param.group->ReadSemphr);
        return PR_OK;
    }
    return PR_TimeOut;
}

PR_RESULT ReadParamGroup(SName name, uint64_t data[], bool *is_new, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    ParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;

    if (group->version & (1 << 14))
        return PR_ERR;
    bool is_param1 = (group->version & (1 << 15)) != 0;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE)
    {
        if (is_new != 0)
            *is_new = ((group->flags & PGF_IS_NEW) != 0);
        for (uint16_t i = 0; i < group->params_count; ++i)
        {
            if (is_param1)
            {
                ParamData1 *p = (ParamData1 *) ((uint8_t *) group->data + i * sizeof(ParamData1));
                memcpy((uint8_t *) &data[i], (uint8_t *) &p->data, sizeof(uint64_t));
            }
            else
            {
                ParamData2 *p = (ParamData2 *) ((uint8_t *) group->data + i * sizeof(ParamData2));
                memcpy((uint8_t *) &data[i], (uint8_t *) &p->data, sizeof(uint64_t));
            }
        }
        xSemaphoreGive(group->ReadSemphr);
        return PR_OK;
    }
    return PR_TimeOut;
}

PR_RESULT ReadParamGroup(SName name, uint64_t data[], bool *is_new, uint16_t start, uint16_t read_count, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    ParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;

    if (group->version & (1 << 14))
        return PR_ERR;

    bool is_param1 = (group->version & (1 << 15)) != 0;
    uint16_t read_to = start + read_count;
    if (start >= group->params_count || read_to > group->params_count)
        return PR_ERR;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE)
    {
        if (is_new != 0)
            *is_new = ((group->flags & PGF_IS_NEW) != 0);
        for (uint16_t i = start; i < read_to; ++i)
        {
            if (is_param1)
            {
                ParamData1 *p = (ParamData1 *) ((uint8_t *) group->data + i * sizeof(ParamData1));
                memcpy((uint8_t *) &data[i - start], (uint8_t *) &p->data, sizeof(uint64_t));
            }
            else
            {
                ParamData2 *p = (ParamData2 *) ((uint8_t *) group->data + i * sizeof(ParamData2));
                memcpy((uint8_t *) &data[i - start], (uint8_t *) &p->data, sizeof(uint64_t));
            }
        }
        xSemaphoreGive(group->ReadSemphr);
        return PR_OK;
    }
    return PR_TimeOut;
}

PR_RESULT GetVolatileParamGroupParamCount(SName name, uint16_t *count, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    VolatileParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = (VolatileParamGroup *) it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE)
    {
        *count = group->params_count;
        xSemaphoreGive(group->ReadSemphr);
        return PR_OK;
    }
    else
        return PR_TimeOut;
}

PR_RESULT ReadVolatileParamGroup(SName name, void *data, uint16_t start, uint16_t read_count, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;
    if (read_count == 0)
        return PR_ERR;

    VolatileParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = (VolatileParamGroup *) it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;

    VolatileParamGroupInf *group_inf = (VolatileParamGroupInf *) group->data;
    if ((group_inf->version >> 14) != 1)
        return PR_ERR;
    uint16_t param_length = group_inf->param_length;
    uint32_t unit_len = Vol_UnitLenWithCRC(param_length);

    char name_ch[17];
    name.get_CharStr(name_ch);

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    // ===== 单锁方案：全程持 ReadSemphr，绝对无 race =====
    if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) != pdTRUE)
        return PR_TimeOut;

    // goto 统一退出（flash I/O 由 Vol_*Units 内部管理，无外部临时缓冲需释放）
    PR_RESULT ret = PR_OK;
    // 提前声明所有后续使用的变量，避免 goto 跨越初始化报错
    uint32_t cache_start = 0, cache_count = 0;
    uint32_t cover_offset = 0, cover_count = 0, extra_count = 0;
    uint32_t read_to = (uint32_t) start + (uint32_t) read_count;
    if (read_to > group->params_count)
    {
        ret = PR_ERR;
        goto read_done;
    }

    // ===== 1. RAM 缓存命中检查（µs 级，命中即返回） =====
    if (group->ram_param_count > 0 && read_count <= group->ram_param_count && group->ram_param_start <= start && (uint32_t) group->ram_param_start + group->ram_param_count >= read_to)
    {
        for (uint16_t i = 0; i < read_count; ++i)
        {
            uint8_t *pu = (uint8_t *) group->data + sizeof(VolatileParamGroupInf) + (uint32_t) param_length * (start + i - group->ram_param_start);
            memcpy((uint8_t *) data + i * param_length, pu, param_length);
        }
        goto read_done;
    }

    // ===== 2. cache miss：脏数据回写 flush（仍在 ReadSemphr 内） =====
    // 防御：ram_param_count==0 时跳过（否则 new[0] UB）；同时清掉无效的 ram_updated 标志
    if (group->ram_updated && group->ram_param_count > 0 && group->params_count > group->ram_param_start)
    {
        uint32_t flush_count = group->params_count - group->ram_param_start;
        if (group->ram_param_count < flush_count)
            flush_count = group->ram_param_count;
        uint32_t flush_start = group->ram_param_start;
        bool flush_is_new = (group->flags & PGF_IS_NEW) != 0;

        VolatileParamGroupInf wr_inf;
        wr_inf.version = group_inf->version;
        wr_inf.param_length = group_inf->param_length;
        wr_inf.update_date = Param_GetFileCreateTime();
        uint8_t *ram_base = (uint8_t *) group->data + sizeof(VolatileParamGroupInf);
        SS_RESULT sr = Vol_FlushUnits(name_ch, &wr_inf, param_length, unit_len, ram_base, flush_count, flush_start, flush_is_new);
        if (sr != SS_OK)
        {
            ret = PR_ERR;
            goto read_done;
        }
        // flush 成功：清 IS_NEW + ram_updated（持锁内，安全）
        if (flush_is_new)
            group->flags &= ~PGF_IS_NEW;
        group->ram_updated = false;
    }

    // ===== 3. 计算最优缓存窗口（cache_start, cache_count） =====
    // 三个分支均保证：cache_count ≤ ram_capacity，cache_start ≤ start
    if (read_count >= group->ram_capacity)
    {
        cache_start = start;
        cache_count = group->ram_capacity;
    }
    else if (start >= group->ram_param_start)
    {
        cache_start = start;
        cache_count = group->ram_capacity;
    }
    else
    {
        uint32_t slack = group->ram_capacity - read_count;
        uint32_t ideal_start = (slack / 2 >= start) ? 0 : (start - slack / 2);
        if (ideal_start + group->ram_capacity > group->params_count)
        {
            if (group->params_count >= group->ram_capacity)
                ideal_start = group->params_count - group->ram_capacity;
            else
                ideal_start = 0;
        }
        cache_start = ideal_start;
        cache_count = group->ram_capacity;
    }
    // 统一越界裁剪：保证 cache_count ≤ params_count - cache_start
    if (cache_count > group->params_count - cache_start)
        cache_count = group->params_count - cache_start;

    // 用户请求 [start, start+read_count) 相对缓存窗口的覆盖关系
    cover_offset = start - cache_start;
    cover_count = cache_count - cover_offset;
    if (cover_count > read_count)
        cover_count = read_count;
    extra_count = read_count - cover_count;

    // ===== 4. 读 flash 缓存窗口部分（含 CRC 校验，3 次重试）→ 直接写入 RAM cache =====
    if (cache_count > 0)
    {
        uint8_t *cache_dst = (uint8_t *) group->data + sizeof(VolatileParamGroupInf);
        if (!Vol_ReadUnits(name_ch, param_length, unit_len, cache_dst, cache_start, cache_count))
        {
            ret = PR_ERR;
            goto read_done;
        }
        // 立即更新 cache 元数据（数据已写入 RAM，保证后续 goto 退出时元数据与数据一致）
        group->ram_param_start = (uint16_t) cache_start;
        group->ram_param_count = (uint16_t) cache_count;
        group->ram_updated = false;
    }

    // ===== 5. 读超出缓存窗口的额外部分（分段读取，直接拷贝给用户） =====
    // extra_count 可能远大于 ram_capacity（如 read_count > 2*ram_capacity），
    // 采用分段策略：每段最多 ram_capacity，逐段读 + CRC 校验 + strip CRC 直拷到 user data
    // 这样不需要一次性分配 extra_count*unit_len 巨量缓冲（嵌入式防 OOM）
    if (extra_count > 0)
    {
        uint32_t extra_pos = 0;                        // 已处理的用户侧 extra 单元数
        uint32_t file_pos = cache_start + cache_count; // flash 中下一待读单元序号
        while (extra_pos < extra_count)
        {
            uint32_t seg_cnt = extra_count - extra_pos;
            if (seg_cnt > group->ram_capacity)
                seg_cnt = group->ram_capacity;

            uint8_t *user_dst = (uint8_t *) data + (cover_count + extra_pos) * param_length;
            if (!Vol_ReadUnits(name_ch, param_length, unit_len, user_dst, file_pos, seg_cnt))
            {
                ret = PR_ERR;
                goto read_done;
            }

            extra_pos += seg_cnt;
            file_pos += seg_cnt;
        }
    }

    // ===== 6. 拷贝缓存窗口覆盖部分给用户（cache 元数据已在段4更新） =====
    if (cover_count > 0)
    {
        uint8_t *cache_base = (uint8_t *) group->data + sizeof(VolatileParamGroupInf);
        for (uint32_t i = 0; i < cover_count; ++i)
            memcpy((uint8_t *) data + i * param_length, cache_base + (cover_offset + i) * param_length, param_length);
    }

read_done:
    xSemaphoreGive(group->ReadSemphr);
    return ret;
}

PR_RESULT SaveVolatileParamGroup(SName name, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    VolatileParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = (VolatileParamGroup *) it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;

    VolatileParamGroupInf *group_inf = (VolatileParamGroupInf *) group->data;
    if ((group_inf->version >> 14) != 1)
        return PR_ERR;
    uint16_t param_length = group_inf->param_length;
    uint32_t unit_len = Vol_UnitLenWithCRC(param_length);

    char name_ch[17];
    name.get_CharStr(name_ch);

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    // ===== 单锁方案：全程持 ReadSemphr =====
    if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) != pdTRUE)
        return PR_TimeOut;

    PR_RESULT ret = PR_OK;

    // 防御：ram_param_count==0 时跳过（否则 new[0] UB）；同时清掉无效的 ram_updated 标志
    if (group->ram_updated && group->ram_param_count > 0 && group->params_count > group->ram_param_start)
    {
        uint32_t save_count = group->params_count - group->ram_param_start;
        if (group->ram_param_count < save_count)
            save_count = group->ram_param_count;
        uint32_t save_start = group->ram_param_start;
        bool is_new = (group->flags & PGF_IS_NEW) != 0;

        VolatileParamGroupInf wr_inf;
        wr_inf.version = group_inf->version;
        wr_inf.param_length = group_inf->param_length;
        wr_inf.update_date = Param_GetFileCreateTime();
        uint8_t *ram_base = (uint8_t *) group->data + sizeof(VolatileParamGroupInf);
        SS_RESULT sr = Vol_FlushUnits(name_ch, &wr_inf, param_length, unit_len, ram_base, save_count, save_start, is_new);
        if (sr == SS_OK)
        {
            // 写盘成功：清 IS_NEW + ram_updated（持锁内，安全）
            if (is_new)
                group->flags &= ~PGF_IS_NEW;
            group->ram_updated = false;
        }
        else
            ret = PR_ERR;
    }
    xSemaphoreGive(group->ReadSemphr);
    return ret;
}

PR_RESULT TruncateVolatileParamGroup(SName name, uint16_t size, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    VolatileParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = (VolatileParamGroup *) it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;

    VolatileParamGroupInf *group_inf = (VolatileParamGroupInf *) group->data;
    if ((group_inf->version >> 14) != 1)
        return PR_ERR;
    uint16_t param_length = group_inf->param_length;
    uint32_t unit_len = Vol_UnitLenWithCRC(param_length);

    char name_ch[17];
    name.get_CharStr(name_ch);

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    // ===== 单锁方案：全程持 ReadSemphr =====
    if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) != pdTRUE)
        return PR_TimeOut;

    if (group->params_count < size)
    {
        xSemaphoreGive(group->ReadSemphr);
        return PR_ERR;
    }

    // RAM 缓存立即失效（持锁内，与读线程互斥）
    group->ram_param_count = 0;
    group->ram_updated = false;

    // flash 截断 + 写新 header（持锁内执行 flash I/O）
    VolatileParamGroupInf wr_inf;
    wr_inf.version = group_inf->version;
    wr_inf.param_length = group_inf->param_length;
    wr_inf.update_date = Param_GetFileCreateTime();
    PR_RESULT ret = PR_OK;
    if (InternalStorage_TruncateFile("Config", name_ch, sizeof(VolatileParamGroupInf) + (uint32_t) size * unit_len) != SS_OK)
        ret = PR_ERR;
    else if (InternalStorage_WriteFile("Config", name_ch, &wr_inf, 0, sizeof(VolatileParamGroupInf)) != SS_OK)
        ret = PR_ERR;
    else
        group->params_count = size; // flash I/O 成功后再更新 params_count

    xSemaphoreGive(group->ReadSemphr);
    return ret;
}

PR_RESULT WriteVolatileParamGroup(SName name, const void *data, uint16_t start, uint16_t write_count, bool st, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    uint32_t write_to = (uint32_t) start + (uint32_t) write_count;
    if (write_to > MAX_VOLATILE_PARAMS || write_count == 0)
        return PR_ERR;

    VolatileParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = (VolatileParamGroup *) it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;

    VolatileParamGroupInf *group_inf = (VolatileParamGroupInf *) group->data;
    if ((group_inf->version >> 14) != 1)
        return PR_ERR;
    uint16_t param_length = group_inf->param_length;
    uint32_t unit_len = Vol_UnitLenWithCRC(param_length);

    char name_ch[17];
    name.get_CharStr(name_ch);

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    // ===== 单锁方案：全程持 ReadSemphr =====
    if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) != pdTRUE)
        return PR_TimeOut;

    if (start > group->params_count)
    {
        xSemaphoreGive(group->ReadSemphr);
        return PR_ERR;
    }

    // cache_mode：写范围在当前 RAM cache 内 → 直接改 RAM cache + 标 dirty
    bool cache_mode = (st == false) && (group->ram_param_count >= write_count) && (group->ram_param_start <= start) && ((uint32_t) group->ram_param_start + group->ram_param_count >= write_to);
    if (cache_mode)
    {
        if (start + write_count > group->params_count)
            group->params_count = start + write_count;
        group->ram_updated = true;
        for (uint16_t i = 0; i < write_count; ++i)
        {
            uint8_t *pu = (uint8_t *) group->data + sizeof(VolatileParamGroupInf) + (uint32_t) param_length * (start + i - group->ram_param_start);
            memcpy(pu, (const uint8_t *) data + i * param_length, param_length);
        }
        xSemaphoreGive(group->ReadSemphr);
        return PR_OK;
    }

    // ===== st=false 且 write_count ≤ ram_capacity → 全部走 RAM cache（不写 flash） =====
    // 与旧代码逻辑一致：cache miss 时主动重定位缓存窗口到 start，并尽量回填后续单元，
    // 让下一次顺序写入也命中 cache_mode。仅当 st=true 才同步落盘。
    if (st == false && write_count <= group->ram_capacity)
    {
        uint32_t cache_end = (uint32_t) group->ram_param_start + group->ram_param_count;

        // 快速路径：start ≥ 当前缓存起点，且扩展后总长 ≤ ram_capacity
        // 直接写 RAM cache，无需 flush
        // start 超出 cache_end 时（跳跃写入），从 flash 回填间隙单元
        if (group->ram_param_count > 0 && (uint32_t) start >= group->ram_param_start && (uint32_t) start + write_count - group->ram_param_start <= group->ram_capacity)
        {
            // 间隙 [cache_end, start)：从 flash 读取已存在但不在当前 cache 的参数填充 cache
            // 顺序追加时 start == cache_end，间隙为 0，无 flash 读
            if ((uint32_t) start > cache_end)
            {
                uint32_t gap_total = (uint32_t) start - cache_end;
                uint32_t cap_room = group->ram_capacity - group->ram_param_count;
                uint32_t gap_fill = gap_total; // 实际能从 flash 读到的单元数
                if (gap_fill > cap_room)
                    gap_fill = cap_room;
                if (group->params_count > cache_end)
                {
                    uint32_t file_avail = group->params_count - cache_end;
                    if (gap_fill > file_avail)
                        gap_fill = file_avail;
                }
                else
                    gap_fill = 0;

                // 先把整个 gap 区域（含 flash 不存在的空洞）清零，避免脏数据被标为有效
                uint32_t gap_actual = gap_total < cap_room ? gap_total : cap_room;
                if (gap_actual > 0)
                {
                    uint8_t *zero_dst = (uint8_t *) group->data + sizeof(VolatileParamGroupInf) + (uint32_t) param_length * group->ram_param_count;
                    memset(zero_dst, 0, (uint32_t) param_length * gap_actual);
                    group->ram_param_count += (uint16_t) gap_actual;
                }

                // flash 实际存在的部分覆盖清零值
                if (gap_fill > 0)
                {
                    uint8_t *gap_dst = (uint8_t *) group->data + sizeof(VolatileParamGroupInf) + (uint32_t) param_length * (group->ram_param_count - gap_actual);
                    if (!Vol_ReadUnits(name_ch, param_length, unit_len, gap_dst, cache_end, gap_fill))
                    {
                        xSemaphoreGive(group->ReadSemphr);
                        return PR_ERR;
                    }
                }
            }

            // 写入新数据到 RAM cache
            for (uint16_t i = 0; i < write_count; ++i)
            {
                uint8_t *pu = (uint8_t *) group->data + sizeof(VolatileParamGroupInf) + (uint32_t) param_length * ((uint32_t) start + i - group->ram_param_start);
                memcpy(pu, (const uint8_t *) data + i * param_length, param_length);
            }
            group->ram_param_count = (uint16_t) ((uint32_t) start + write_count - group->ram_param_start);
            if ((uint32_t) start + write_count > group->params_count)
                group->params_count = start + write_count;
            group->ram_updated = true;
            xSemaphoreGive(group->ReadSemphr);
            return PR_OK;
        }

        // 重定位路径：写位置远离当前缓存窗口（或缓存无效）→
        // 与旧代码一致：flush 旧脏数据 → 将 cache 窗口移到 [start, start+ram_capacity)
        //             → 从 flash 回填 [start+write_count, start+ram_capacity) 已存在单元
        //             → 写入新数据到 cache 首部
        // 这样下一次顺序写入 start+write_count 也会 cache 命中，避免每单元 flash 写

        // 1) flush 旧脏数据（如果有）
        if (group->ram_updated && group->ram_param_count > 0 && group->params_count > group->ram_param_start)
        {
            uint32_t flush_count = group->params_count - group->ram_param_start;
            if (group->ram_param_count < flush_count)
                flush_count = group->ram_param_count;
            uint32_t flush_start = group->ram_param_start;
            bool flush_is_new = (group->flags & PGF_IS_NEW) != 0;

            VolatileParamGroupInf wr_inf;
            wr_inf.version = group_inf->version;
            wr_inf.param_length = group_inf->param_length;
            wr_inf.update_date = Param_GetFileCreateTime();
            uint8_t *ram_base = (uint8_t *) group->data + sizeof(VolatileParamGroupInf);
            SS_RESULT sr = Vol_FlushUnits(name_ch, &wr_inf, param_length, unit_len, ram_base, flush_count, flush_start, flush_is_new);
            if (sr != SS_OK)
            {
                xSemaphoreGive(group->ReadSemphr);
                return PR_ERR;
            }
            if (flush_is_new)
                group->flags &= ~PGF_IS_NEW;
            group->ram_updated = false;
        }

        // 2) 提前更新 params_count（反映本次写入的范围），用于计算回填量上界
        uint32_t params_count_old = group->params_count;
        if ((uint32_t) start + write_count > group->params_count)
            group->params_count = start + write_count;

        // 3) 回填量（旧代码：min(params_count_old - start, ram_capacity) - write_count，下界 0）
        //    即：cache 容量内、写范围之后、flash 中已存在的参数总量
        uint32_t backfill_count = 0;
        if ((uint32_t) start < params_count_old)
            backfill_count = params_count_old - start;
        if (group->ram_capacity < backfill_count)
            backfill_count = group->ram_capacity;
        if (backfill_count > write_count)
            backfill_count -= write_count;
        else
            backfill_count = 0;

        // 4) 从 flash (start + write_count) 处回填 backfill_count 个单元，strip CRC 到 cache[write_count .. ]
        if (backfill_count > 0)
        {
            uint8_t *bf_dst = (uint8_t *) group->data + sizeof(VolatileParamGroupInf) + (uint32_t) param_length * write_count;
            if (!Vol_ReadUnits(name_ch, param_length, unit_len, bf_dst, (uint32_t) start + write_count, backfill_count))
            {
                xSemaphoreGive(group->ReadSemphr);
                return PR_ERR;
            }
        }

        // 5) 写入新数据到 cache[0..write_count)
        for (uint16_t i = 0; i < write_count; ++i)
        {
            uint8_t *pu = (uint8_t *) group->data + sizeof(VolatileParamGroupInf) + (uint32_t) param_length * i;
            memcpy(pu, (const uint8_t *) data + i * param_length, param_length);
        }

        // 6) 更新缓存元数据：窗口锚定在 start，有效长度 = write_count + backfill_count
        group->ram_param_start = start;
        group->ram_param_count = (uint16_t) ((uint32_t) write_count + backfill_count);
        group->ram_updated = true;
        xSemaphoreGive(group->ReadSemphr);
        return PR_OK;
    }

    // st=true 或范围超出缓存容量 → 同步 RAM cache 重叠部分 + 构造 wbuf + 原子写盘
    bool is_new_snapshot = (group->flags & PGF_IS_NEW) != 0;

    // 同步 RAM cache 重叠区
    uint32_t ram_end = (uint32_t) group->ram_param_start + group->ram_param_count;
    uint32_t a = start > group->ram_param_start ? start : group->ram_param_start;
    uint32_t b = write_to < ram_end ? write_to : ram_end;
    for (uint32_t i = a; i < b; ++i)
    {
        uint8_t *pu = (uint8_t *) group->data + sizeof(VolatileParamGroupInf) + (uint32_t) param_length * (i - group->ram_param_start);
        memcpy(pu, (const uint8_t *) data + (i - start) * param_length, param_length);
    }
    if (start + write_count > group->params_count)
        group->params_count = start + write_count;
    if (a < b)
        group->ram_updated = true;

    // 栈上 header
    VolatileParamGroupInf wr_inf;
    wr_inf.version = group_inf->version;
    wr_inf.param_length = group_inf->param_length;
    wr_inf.update_date = Param_GetFileCreateTime();
    // 原子写盘（用户 data 视作连续 RAM 缓冲）
    SS_RESULT sr = Vol_FlushUnits(name_ch, &wr_inf, param_length, unit_len, (const uint8_t *) data, write_count, start, is_new_snapshot);

    PR_RESULT ret = PR_OK;
    if (sr != SS_OK)
    {
        ret = PR_ERR;
    }
    else
    {
        // 写盘成功：清 IS_NEW + ram_updated（持锁内，安全）
        // ram_updated 清除：重叠区已同步 RAM，且 flash 已写，cache 与 flash 一致
        if (is_new_snapshot)
            group->flags &= ~PGF_IS_NEW;
        group->ram_updated = false;
    }
    xSemaphoreGive(group->ReadSemphr);
    return ret;
}

/* ========== 写盘辅助：单一 CRC 原子写 ========== */

// 将 ParamGroup 的 header + data + 单一 CRC 一次性原子写入存储器
// 调用者必须已持有 WriteSemphr；本函数不需 ReadSemphr（仅读常量 header info 和 group->data 快照）
// crc 由调用者在 ReadSemphr 内算好传入（覆盖全量 data 区）
// truncate=true 表示 IS_NEW/NEED_REWRITE，重建文件；false 表示覆盖更新现有文件
// group->data 中 resv1 全为 0（磁盘格式），调用者无需做任何 dirty 处理
static SS_RESULT ParamGroup_WriteAtomic(const ParamGroup *group, const char *name_ch, uint32_t crc, bool truncate)
{
    bool is_param1 = (group->version & (1 << 15)) != 0;
    uint32_t n = group->params_count;
    uint32_t data_unit = is_param1 ? sizeof(ParamData1) : sizeof(ParamData2);

    // 栈上 header
    ParamGroupInf inf;
    inf.version = group->version;
    inf.params_count = group->params_count;
    inf.update_date = Param_GetFileCreateTime();

    // 三段：header / data / crc
    StorageSpan spans[3];
    spans[0].content = &inf;
    spans[0].offset = 0;
    spans[0].length = sizeof(ParamGroupInf);
    spans[1].content = group->data;
    spans[1].offset = sizeof(ParamGroupInf);
    spans[1].length = data_unit * n;
    spans[2].content = &crc;
    spans[2].offset = sizeof(ParamGroupInf) + data_unit * n;
    spans[2].length = sizeof(uint32_t);

    return InternalStorage_WriteFileSpans("Config", name_ch, spans, 3, truncate);
}

PR_RESULT UpdateParamGroup(SName name, const uint64_t data[], uint16_t start, uint16_t write_count, bool st, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    ParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;

    if (group->version & (1 << 14))
        return PR_ERR;
    uint16_t write_to = start + write_count;
    if (start >= group->params_count || write_to > group->params_count)
        return PR_ERR;

    bool is_param1 = (group->version & (1 << 15)) != 0;
    uint32_t data_unit = is_param1 ? sizeof(ParamData1) : sizeof(ParamData2);

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    char name_ch[17];
    name.get_CharStr(name_ch);

    if (!st)
    {
        // 不写盘：需同时持 WriteSemphr+ReadSemphr 更新 RAM
        // WriteSemphr 用于与 st=true 路径的 CRC 计算+写盘序列串行化，避免 RAM 改动插入到 CRC 算完后
        if (xSemaphoreTake(group->WriteSemphr, TIMEOUT_Ticks) != pdTRUE)
            return PR_TimeOut;
        if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE)
        {
            for (uint16_t i = start; i < write_to; ++i)
            {
                if (is_param1)
                {
                    ParamData1 *p = (ParamData1 *) ((uint8_t *) group->data + i * sizeof(ParamData1));
                    p->data = data[i - start];
                }
                else
                {
                    ParamData2 *p = (ParamData2 *) ((uint8_t *) group->data + i * sizeof(ParamData2));
                    p->data = data[i - start];
                }
            }
            xSemaphoreGive(group->ReadSemphr);
            xSemaphoreGive(group->WriteSemphr);
            return PR_OK;
        }
        else
        {
            xSemaphoreGive(group->WriteSemphr);
            return PR_TimeOut;
        }
    }

    // st=true 路径：持 WriteSemphr 排他写盘
    if (xSemaphoreTake(group->WriteSemphr, TIMEOUT_Ticks) != pdTRUE)
        return PR_TimeOut;

    uint32_t crc = 0;
    bool truncate = false;
    {
        if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) != pdTRUE)
        {
            xSemaphoreGive(group->WriteSemphr);
            return PR_TimeOut;
        }
        // 写 RAM
        for (uint16_t i = start; i < write_to; ++i)
        {
            if (is_param1)
            {
                ParamData1 *p = (ParamData1 *) ((uint8_t *) group->data + i * sizeof(ParamData1));
                p->data = data[i - start];
            }
            else
            {
                ParamData2 *p = (ParamData2 *) ((uint8_t *) group->data + i * sizeof(ParamData2));
                p->data = data[i - start];
            }
        }
        truncate = (group->flags & (PGF_IS_NEW | PGF_NEED_REWRITE)) != 0;
        group->flags &= ~PGF_NEED_REWRITE;
        xSemaphoreGive(group->ReadSemphr);
    }
    // CRC 计算 / 写盘都在 WriteSemphr 内，不需 ReadSemphr
    // WriteSemphr 同时被 UpdateParamGroup(st=false) 在写 RAM 时获取，保证此期间 data 不变
    Param_CalcCRC((const uint8_t *) group->data, data_unit * group->params_count, &crc);

    // 写盘（WriteSemphr 内，不需 ReadSemphr）
    SS_RESULT ss_res = ParamGroup_WriteAtomic(group, name_ch, crc, truncate);

    if (ss_res == SS_OK)
    {
        // 持锁清 IS_NEW
        if (xSemaphoreTake(group->ReadSemphr, portMAX_DELAY) == pdTRUE)
        {
            group->flags &= ~PGF_IS_NEW;
            xSemaphoreGive(group->ReadSemphr);
        }
        xSemaphoreGive(group->WriteSemphr);
        return PR_OK;
    }
    else
    {
        xSemaphoreGive(group->WriteSemphr);
        if (ss_res == SS_TimeOut)
            return PR_TimeOut;
        else
            return PR_ERR;
    }
}

PR_RESULT SaveParamGroup(SName name, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    ParamGroup *group = nullptr;
    if (xSemaphoreTake(ParamGroups_Semphr, portMAX_DELAY) == pdTRUE)
    {
        map<SName, ParamGroup *>::iterator it = ParamGroups.find(name);
        if (it == ParamGroups.end())
        {
            xSemaphoreGive(ParamGroups_Semphr);
            return PR_ERR;
        }
        group = it->second;
        xSemaphoreGive(ParamGroups_Semphr);
    }
    if (group == nullptr)
        return PR_ERR;
    if (group->version & (1 << 14))
        return PR_ERR;

    bool is_param1 = (group->version & (1 << 15)) != 0;
    uint32_t data_unit = is_param1 ? sizeof(ParamData1) : sizeof(ParamData2);

    char name_ch[17];
    name.get_CharStr(name_ch);

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    if (xSemaphoreTake(group->WriteSemphr, TIMEOUT_Ticks) != pdTRUE)
        return PR_TimeOut;

    uint32_t crc = 0;
    bool truncate = false;
    {
        if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) != pdTRUE)
        {
            xSemaphoreGive(group->WriteSemphr);
            return PR_TimeOut;
        }
        truncate = (group->flags & (PGF_IS_NEW | PGF_NEED_REWRITE)) != 0;
        group->flags &= ~PGF_NEED_REWRITE;
        xSemaphoreGive(group->ReadSemphr);
    }
    // CRC 计算/写盘在 WriteSemphr 内
    Param_CalcCRC((const uint8_t *) group->data, data_unit * group->params_count, &crc);

    SS_RESULT ss_res = ParamGroup_WriteAtomic(group, name_ch, crc, truncate);

    if (ss_res == SS_OK)
    {
        if (xSemaphoreTake(group->ReadSemphr, portMAX_DELAY) == pdTRUE)
        {
            group->flags &= ~PGF_IS_NEW;
            xSemaphoreGive(group->ReadSemphr);
        }
        xSemaphoreGive(group->WriteSemphr);
        return PR_OK;
    }
    else
    {
        xSemaphoreGive(group->WriteSemphr);
        if (ss_res == SS_TimeOut)
            return PR_TimeOut;
        else
            return PR_ERR;
    }
}

PR_RESULT UpdateParam(SName name, const uint64_t data, double TIMEOUT)
{
    if (ParamGroups_Semphr == nullptr || GroupParameters_Semphr == nullptr)
        return PR_ERR;

    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;

    Param param;
    if (xSemaphoreTake(GroupParameters_Semphr, TIMEOUT_Ticks) == pdTRUE)
    {
        map<SName, Param>::iterator it = GroupParameters.find(name);
        if (it == GroupParameters.end())
        {
            xSemaphoreGive(GroupParameters_Semphr);
            return PR_ERR;
        }
        param = it->second;
        xSemaphoreGive(GroupParameters_Semphr);
    }
    else
        return PR_TimeOut;

    ParamGroup *group = param.group;
    bool is_param1 = (group->version & (1 << 15)) != 0;

    // 范围校验：缺省侧（nullptr）传 NaN，ClampCheckFloat 会跳过该侧校验
    if (is_param1)
    {
        ParamData1 *p = (ParamData1 *) param.data;
        uint32_t in_idx = (uint32_t) (((uint8_t *) param.data - (uint8_t *) group->data) / sizeof(ParamData1));
        float min_v = (group->bounds_min != nullptr) ? group->bounds_min[in_idx] : std::nanf("");
        float max_v = (group->bounds_max != nullptr) ? group->bounds_max[in_idx] : std::nanf("");
        if (!Param_ClampCheckFloat(p->type, data, min_v, max_v))
            return PR_ERR;
    }

    char name_ch[17];
    group->name.get_CharStr(name_ch);

    uint32_t data_unit = is_param1 ? sizeof(ParamData1) : sizeof(ParamData2);

    // 持 WriteSemphr 排他写盘
    if (xSemaphoreTake(group->WriteSemphr, TIMEOUT_Ticks) != pdTRUE)
        return PR_TimeOut;

    uint32_t crc = 0;
    bool truncate = false;
    {
        if (xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) != pdTRUE)
        {
            xSemaphoreGive(group->WriteSemphr);
            return PR_TimeOut;
        }
        if (is_param1)
            ((ParamData1 *) param.data)->data = data;
        else
            ((ParamData2 *) param.data)->data = data;
        truncate = (group->flags & (PGF_IS_NEW | PGF_NEED_REWRITE)) != 0;
        group->flags &= ~PGF_NEED_REWRITE;
        xSemaphoreGive(group->ReadSemphr);
    }
    // CRC 计算与写盘在 WriteSemphr 内
    Param_CalcCRC((const uint8_t *) group->data, data_unit * group->params_count, &crc);

    SS_RESULT ss_res = ParamGroup_WriteAtomic(group, name_ch, crc, truncate);

    if (ss_res == SS_OK)
    {
        if (xSemaphoreTake(group->ReadSemphr, portMAX_DELAY) == pdTRUE)
        {
            group->flags &= ~PGF_IS_NEW;
            xSemaphoreGive(group->ReadSemphr);
        }
        xSemaphoreGive(group->WriteSemphr);
        return PR_OK;
    }
    else
    {
        xSemaphoreGive(group->WriteSemphr);
        if (ss_res == SS_TimeOut)
            return PR_TimeOut;
        else
            return PR_ERR;
    }
}

void init_Parameters()
{
    ParamGroups_Semphr = xSemaphoreCreateMutex();
    GroupParameters_Semphr = xSemaphoreCreateMutex();
}
