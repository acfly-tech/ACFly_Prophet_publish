#include "StorageSystem.hpp"

#include "Basic.hpp"
#include "FlashIO.h"
#include "FreeRTOS.h"
#include "drv_ADC.hpp"
#include "drv_Flash.hpp"
#include "flash_diskio.h"
#include "lfs.h"
#include "sdInt_diskio.h"
#include "sd_diskio.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

// 先行缓冲大小 必须为8的倍数
#define LOOKAHEAD_SIZE 32
// 磨损均衡计数
#define BLOCK_CYCLES 200

// 是否已初始化
static bool InternalStorage_Initialized = false;
// 内部存储是否第一次运行（开机时初始化）
static bool InternalStorage_FirstTime = false;

// 设备访问互斥锁
static SemaphoreHandle_t InternalStorage_Semphr;
// 文件
static char UserFile_filename[256];
Static_AXIDMABuf lfs_t FlashFs;
Static_AXIDMABuf lfs_file_t UserFile;
// LittleFS DMA-safe cache buffers (must reside in AXI_DMA_RAM for direct SD DMA access,
// avoiding the scratch-path memcpy that would overwrite adjacent heap allocations)
Static_AXIDMABuf uint8_t lfs_read_buf[512];
Static_AXIDMABuf uint8_t lfs_prog_buf[512];
Static_AXIDMABuf uint8_t lfs_lookahead_buf[LOOKAHEAD_SIZE];
Static_AXIDMABuf uint8_t lfs_file_buf[512];
static lfs_file_config UserFile_cfg;

// 最近一次 init 的 lfs_mount 原始错误码(0=成功，负数=littlefs错误码)，诊断格式化触发原因
static int InternalStorage_LastMountError = 0;
// CORRUPT 路径探测结果：superblock 区是否为全0xFF(true=干净未格式化介质)
static bool InternalStorage_SuperblockClean = false;

SS_RESULT lock_InternalStorage(double TIMEOUT)
{
    // 获取互斥锁
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTakeRecursive(InternalStorage_Semphr, TIMEOUT_Ticks))
        return SS_OK;
    return SS_TimeOut;
}

void unlock_InternalStorage()
{
    xSemaphoreGiveRecursive(InternalStorage_Semphr);
}

/*文件读写*/
/*
        截断存储文件
        group_name：文件组别名称
        name：文件名称
        length：长度
        TIMEOUT：线程同步超时时间
*/
SS_RESULT InternalStorage_TruncateFile(const char *group_name, const char *name, uint32_t length, double TIMEOUT)
{
    if (InternalStorage_Initialized == false)
        return SS_ERR;

    if (lock_InternalStorage(TIMEOUT) == SS_OK)
    {
        // 获取文件名
        UserFile_filename[0] = 0;
        strcat(UserFile_filename, group_name);
        // 创建组别
        lfs_info fInfo;
        if (lfs_stat(&FlashFs, UserFile_filename, &fInfo) < 0)
            lfs_mkdir(&FlashFs, UserFile_filename);
        strcat(UserFile_filename, "/");
        strcat(UserFile_filename, name);
        strcat(UserFile_filename, ".cfg");

        int res = lfs_file_opencfg(&FlashFs, &UserFile, UserFile_filename, LFS_O_CREAT | LFS_O_WRONLY, &UserFile_cfg);
        // 打开文件
        if (res == LFS_ERR_OK)
        { // 写入文件
            res = lfs_file_truncate(&FlashFs, &UserFile, length);
            lfs_file_close(&FlashFs, &UserFile);
        }

        // 释放互斥锁
        unlock_InternalStorage();
        if (res >= 0)
            return SS_OK;
        else
            return SS_ERR;
    }
    else
        return SS_TimeOut;
}

/*
        写存储文件
        group_name：文件组别名称
        name：文件名称
        content：内容
        length：内容长度
        TIMEOUT：线程同步超时时间
*/
SS_RESULT InternalStorage_WriteFile(const char *group_name, const char *name, const void *content, uint32_t offset, uint32_t length, bool truncate, double TIMEOUT)
{
    if (InternalStorage_Initialized == false)
        return SS_ERR;

    if (lock_InternalStorage(TIMEOUT) == SS_OK)
    {
        // 获取文件名
        UserFile_filename[0] = 0;
        strcat(UserFile_filename, group_name);
        // 创建组别
        lfs_info fInfo;
        if (lfs_stat(&FlashFs, UserFile_filename, &fInfo) < 0)
            lfs_mkdir(&FlashFs, UserFile_filename);
        strcat(UserFile_filename, "/");
        strcat(UserFile_filename, name);
        strcat(UserFile_filename, ".cfg");

        int flags = LFS_O_CREAT | LFS_O_WRONLY;
        if (truncate)
            flags |= LFS_O_TRUNC;
        int res = lfs_file_opencfg(&FlashFs, &UserFile, UserFile_filename, flags, &UserFile_cfg);
        // 打开文件
        if (res == LFS_ERR_OK)
        { // 写入文件
            lfs_file_seek(&FlashFs, &UserFile, offset, LFS_SEEK_SET);
            lfs_ssize_t byteswritten = lfs_file_write(&FlashFs, &UserFile, content, length);
            if (byteswritten < length)
                res = -1;
            // truncate=true 时截断到写入末尾（防止原文件尾部残留）
            if (res >= 0 && truncate)
                res = lfs_file_truncate(&FlashFs, &UserFile, offset + length);
            lfs_file_close(&FlashFs, &UserFile);
        }

        // 释放互斥锁
        unlock_InternalStorage();
        if (res >= 0)
            return SS_OK;
        else
            return SS_ERR;
    }
    else
        return SS_TimeOut;
}

/*
        原子多段写文件
        所有段在同一次 lfs open/write.../close 事务内写入

        group_name：文件组别名称
        name：文件名称
        spans：StorageSpan 数组（每段含 content/offset/length）
        span_count：段数
        truncate：true 时使用 LFS_O_TRUNC（重建文件，新文件最终大小由 truncate+末尾写位置决定）
                  false 时为已有文件追加/覆盖写
*/
SS_RESULT InternalStorage_WriteFileSpans(const char *group_name, const char *name, const StorageSpan *spans, uint32_t span_count, bool truncate, double TIMEOUT)
{
    if (InternalStorage_Initialized == false)
        return SS_ERR;
    if (span_count == 0 || spans == nullptr)
        return SS_ERR;

    if (lock_InternalStorage(TIMEOUT) == SS_OK)
    {
        UserFile_filename[0] = 0;
        strcat(UserFile_filename, group_name);
        lfs_info fInfo;
        if (lfs_stat(&FlashFs, UserFile_filename, &fInfo) < 0)
            lfs_mkdir(&FlashFs, UserFile_filename);
        strcat(UserFile_filename, "/");
        strcat(UserFile_filename, name);
        strcat(UserFile_filename, ".cfg");

        int flags = LFS_O_CREAT | LFS_O_WRONLY;
        if (truncate)
            flags |= LFS_O_TRUNC;

        int res = lfs_file_opencfg(&FlashFs, &UserFile, UserFile_filename, flags, &UserFile_cfg);
        if (res == LFS_ERR_OK)
        {
            for (uint32_t i = 0; i < span_count; ++i)
            {
                if (spans[i].length == 0)
                    continue;
                if (lfs_file_seek(&FlashFs, &UserFile, spans[i].offset, LFS_SEEK_SET) < 0)
                {
                    res = -1;
                    break;
                }
                lfs_ssize_t w = lfs_file_write(&FlashFs, &UserFile, spans[i].content, spans[i].length);
                if (w < (lfs_ssize_t) spans[i].length)
                {
                    res = -1;
                    break;
                }
            }
            // close 内才真正提交事务到 flash（littlefs 保证原子性：写过程中掉电不影响旧文件）
            lfs_file_close(&FlashFs, &UserFile);
        }

        unlock_InternalStorage();
        if (res >= 0)
            return SS_OK;
        else
            return SS_ERR;
    }
    else
        return SS_TimeOut;
}

/*
        保存存储文件
        group_name：文件组别名称
        name：文件名称
        content：内容
        length：内容长度
        TIMEOUT：线程同步超时时间
*/
SS_RESULT InternalStorage_SaveFile(const char *group_name, const char *name, const void *content, uint32_t length, double TIMEOUT)
{
    if (InternalStorage_Initialized == false)
        return SS_ERR;

    if (lock_InternalStorage(TIMEOUT) == SS_OK)
    {
        // 获取文件名
        UserFile_filename[0] = 0;
        strcat(UserFile_filename, group_name);
        // 创建组别
        lfs_info fInfo;
        if (lfs_stat(&FlashFs, UserFile_filename, &fInfo) < 0)
            lfs_mkdir(&FlashFs, UserFile_filename);
        strcat(UserFile_filename, "/");
        strcat(UserFile_filename, name);
        strcat(UserFile_filename, ".cfg");

        int res = lfs_file_opencfg(&FlashFs, &UserFile, UserFile_filename, LFS_O_CREAT | LFS_O_TRUNC | LFS_O_WRONLY, &UserFile_cfg);
        // 打开文件
        if (res == LFS_ERR_OK)
        { // 写入文件
            lfs_ssize_t byteswritten = lfs_file_write(&FlashFs, &UserFile, content, length);
            if (byteswritten < length)
                res = -1;
            lfs_file_close(&FlashFs, &UserFile);
        }

        // 释放互斥锁
        unlock_InternalStorage();
        if (res >= 0)
            return SS_OK;
        else
            return SS_ERR;
    }
    else
        return SS_TimeOut;
}

/*
        读取存储文件大小
        group_name：文件组别名称
        name：文件名称
        size：读取的文件大小
        TIMEOUT：线程同步超时时间
*/
SS_RESULT InternalStorage_GetFileSize(const char *group_name, const char *name, uint32_t *size, double TIMEOUT)
{
    if (InternalStorage_Initialized == false)
        return SS_ERR;

    if (lock_InternalStorage(TIMEOUT) == SS_OK)
    {
        // 获取文件名
        UserFile_filename[0] = 0;
        strcat(UserFile_filename, group_name);
        strcat(UserFile_filename, "/");
        strcat(UserFile_filename, name);
        strcat(UserFile_filename, ".cfg");

        lfs_info info;
        int res = lfs_stat(&FlashFs, UserFile_filename, &info);
        if (res == LFS_ERR_OK)
            *size = info.size;

        // 释放互斥锁
        unlock_InternalStorage();
        if (res == LFS_ERR_OK)
            return SS_OK;
        else
            return SS_ERR;
    }
    else
        return SS_TimeOut;
}
/*
        读取存储文件
        group_name：文件组别名称
        name：文件名称
        content：读取的内容
        length：读取的文件长度
        TIMEOUT：线程同步超时时间
*/
SS_RESULT InternalStorage_ReadFile(const char *group_name, const char *name, void *content, uint32_t *length, uint32_t read_offset, int32_t read_length, double TIMEOUT)
{
    if (InternalStorage_Initialized == false)
        return SS_ERR;

    if (lock_InternalStorage(TIMEOUT) == SS_OK)
    {
        // 获取文件名
        UserFile_filename[0] = 0;
        strcat(UserFile_filename, group_name);
        strcat(UserFile_filename, "/");
        strcat(UserFile_filename, name);
        strcat(UserFile_filename, ".cfg");

        int res = lfs_file_opencfg(&FlashFs, &UserFile, UserFile_filename, LFS_O_RDONLY, &UserFile_cfg);
        // 打开文件
        if (res == LFS_ERR_OK)
        { // 读取文件
            uint32_t file_size = lfs_file_size(&FlashFs, &UserFile);
            if (read_length < 0)
                read_length = file_size - read_offset;
            else if (read_length + read_offset > file_size)
                res = -1;

            if (res >= 0)
            {
                lfs_file_seek(&FlashFs, &UserFile, read_offset, LFS_SEEK_SET);
                lfs_ssize_t bytesread = lfs_file_read(&FlashFs, &UserFile, content, read_length);
                if (bytesread < read_length)
                    res = -1;
                *length = bytesread;
            }
            lfs_file_close(&FlashFs, &UserFile);
        }

        // 释放互斥锁
        unlock_InternalStorage();
        if (res >= 0)
            return SS_OK;
        else
            return SS_ERR;
    }
    else
        return SS_TimeOut;
}
/*文件读写*/

/*复位配置文件*/
SS_RESULT InternalStorage_ResetCfg_ExceptWGA(double TIMEOUT)
{
    if (InternalStorage_Initialized == false)
        return SS_ERR;

    if (lock_InternalStorage(TIMEOUT) == SS_OK)
    {
        UserFile_filename[0] = 0;
        strcat(UserFile_filename, "Config");

        lfs_dir_t dir;
        int res = lfs_dir_open(&FlashFs, &dir, UserFile_filename);
        uint32_t pathLen = strlen(UserFile_filename);
        if (res == 0)
        { // 打开文件夹
            for (;;)
            { // 遍历文件夹内容
                lfs_info fno;
                res = lfs_dir_read(&FlashFs, &dir, &fno);
                if (res < 0 || fno.name[0] == 0)
                    break; /* Break on error or end of dir */
                if (fno.type == LFS_TYPE_DIR)
                { // 文件夹
                }
                else
                { // 文件
                    if (strcmp(fno.name, "WGA_Code.cfg") == 0)
                        continue;

                    sprintf(&UserFile_filename[pathLen], "/%s", fno.name);
                    res = lfs_remove(&FlashFs, UserFile_filename);
                    if (res == 0)
                    {
                    }
                }
            }
            lfs_dir_close(&FlashFs, &dir);
        }

        // 释放互斥锁
        unlock_InternalStorage();
        if (res >= 0)
            return SS_OK;
        else
            return SS_ERR;
    }
    else
        return SS_TimeOut;
}
/*复位配置文件*/

// 迁移辅助：递归把 flash_lfs 中 path_buf[0..path_len-1] 目录的所有内容复制到 sd_lfs
// dst_file_cfg 必须指向 DMA 安全缓冲区（用于 SD 写入通道），顺序打开，不同时持有两个文件
static void migrate_copy_dir(lfs_t *src, lfs_t *dst, char *path_buf, size_t path_len, size_t path_buf_size, uint8_t *copy_buf, size_t copy_buf_size, uint32_t src_page_size,
                             lfs_file_config *dst_file_cfg)
{
    const char *open_path = (path_len == 0) ? "/" : path_buf;

    lfs_dir_t *dir = new lfs_dir_t;
    if (!dir)
        return;

    if (lfs_dir_open(src, dir, open_path) < 0)
    {
        delete dir;
        return;
    }

    for (;;)
    {
        lfs_info fno;
        if (lfs_dir_read(src, dir, &fno) <= 0 || fno.name[0] == 0)
            break;
        if (strcmp(fno.name, ".") == 0 || strcmp(fno.name, "..") == 0)
            continue;

        size_t name_len = strlen(fno.name);
        size_t child_len;

        // 拼接子路径（根目录不加前导斜杠，与 LittleFS 路径约定一致）
        if (path_len == 0)
        {
            if (name_len >= path_buf_size)
                continue;
            memcpy(path_buf, fno.name, name_len + 1);
            child_len = name_len;
        }
        else
        {
            if (path_len + 1 + name_len >= path_buf_size)
                continue;
            path_buf[path_len] = '/';
            memcpy(path_buf + path_len + 1, fno.name, name_len + 1);
            child_len = path_len + 1 + name_len;
        }

        if (fno.type == LFS_TYPE_DIR)
        {
            lfs_mkdir(dst, path_buf); // 忽略已存在错误
            migrate_copy_dir(src, dst, path_buf, child_len, path_buf_size, copy_buf, copy_buf_size, src_page_size, dst_file_cfg);
        }
        else
        {
            // flash 侧文件缓冲区不需要 DMA-safe，heap 即可
            uint8_t *sf_buf = new uint8_t[src_page_size];
            lfs_file_t *sf = new lfs_file_t;
            lfs_file_t *df = new lfs_file_t;

            if (sf_buf && sf && df)
            {
                lfs_file_config sf_cfg = {.buffer = sf_buf};
                if (lfs_file_opencfg(src, sf, path_buf, LFS_O_RDONLY, &sf_cfg) == LFS_ERR_OK)
                {
                    if (lfs_file_opencfg(dst, df, path_buf, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, dst_file_cfg) == LFS_ERR_OK)
                    {
                        lfs_ssize_t n;
                        while ((n = lfs_file_read(src, sf, copy_buf, copy_buf_size)) > 0)
                            lfs_file_write(dst, df, copy_buf, n);
                        lfs_file_close(dst, df);
                    }
                    lfs_file_close(src, sf);
                }
            }

            delete[] sf_buf;
            delete sf;
            delete df;
        }

        // 恢复路径缓冲区
        path_buf[path_len] = '\0';
    }

    lfs_dir_close(src, dir);
    delete dir;
}

static lfs_config cfg;

// 探测 superblock 区(littlefs block 0 起始扇区)是否为全 0xFF：
// 全 0xFF = 干净未格式化介质(格式化无数据损失)；否则 = 曾有数据但已损坏(格式化丢弃残留数据)
static bool is_superblock_clean(uint32_t sector_size)
{
    // 静态缓冲仅512字节，扇区超过512时防越界(实际lfs cache缓冲同样假设512，此处一致)
    if (sector_size == 0 || sector_size > sizeof(lfs_read_buf))
        return false;
    // 复用 DMA-safe 的 lfs 读取缓冲(此时 mount 已失败，该缓冲随后仅被 format/mount 重写)
    if (disk_read(DEV_SD_INT, lfs_read_buf, 0, 1) != RES_OK)
        return false;
    for (uint32_t i = 0; i < sector_size; ++i)
    {
        if (lfs_read_buf[i] != 0xFF)
            return false;
    }
    return true;
}

int InternalStorage_GetLastMountError()
{
    return InternalStorage_LastMountError;
}

bool InternalStorage_WasSuperblockClean()
{
    return InternalStorage_SuperblockClean;
}

void init_InternalStorage()
{
    while (adcGet_VDDA_Voltage() < 3.0)
    { // 电压过低不初始化flash防止误擦除
        os_delay(0.1);
    }

    // 永远以 SD_INT 为主存储，获取磁盘信息
    uint32_t sector_size = 0, block_size = 0, sector_count = 0;
    if (disk_ioctl(DEV_SD_INT, GET_SECTOR_SIZE, &sector_size) != RES_OK || sector_size == 0 || disk_ioctl(DEV_SD_INT, GET_BLOCK_SIZE, &block_size) != RES_OK || block_size == 0 ||
        disk_ioctl(DEV_SD_INT, GET_SECTOR_COUNT, &sector_count) != RES_OK || sector_count == 0)
    { // SD 未就绪，稍后由外部重试
        return;
    }
    uint32_t block_count = sector_count / block_size;
    block_size *= sector_size;

    cfg = {
        // block device operations
        .read = sdInt_read,
        .prog = sdInt_write,
        .erase = sdInt_erase,
        .sync = sdInt_sync,

        // block device configuration
        .read_size = sector_size,
        .prog_size = sector_size,
        .block_size = block_size,
        .block_count = block_count,
        .block_cycles = BLOCK_CYCLES,
        .cache_size = sector_size,
        .lookahead_size = LOOKAHEAD_SIZE,
        // static DMA-safe cache buffers: avoid heap alloc, enable direct SD DMA
        .read_buffer = lfs_read_buf,
        .prog_buffer = lfs_prog_buf,
        .lookahead_buffer = lfs_lookahead_buf,
    };

    UserFile_cfg.buffer = lfs_file_buf;

    /* 挂载重试：LFS_ERR_IO 视为瞬态读写错误(DMA/总线抖动等)，重试后仍失败则放弃。
     * 防止偶发 IO 错误误触发格式化导致整个内部参数文件系统被清空 */
    int res = LFS_ERR_IO;
    for (uint8_t attempt = 0; attempt < 3; ++attempt)
    {
        res = lfs_mount(&FlashFs, &cfg);
        if (res == LFS_ERR_OK || res != LFS_ERR_IO)
            break; // 成功，或非瞬态错误(如 CORRUPT)重试无意义
        os_delay(0.1);
    }
    InternalStorage_LastMountError = res;

    if (res != LFS_ERR_OK)
    {
        if (res != LFS_ERR_CORRUPT)
        {
            /* LFS_ERR_IO 重试耗尽，或其他错误(INVAL/NOMEM等)：
             * 不格式化，保持未初始化状态；参数系统回退默认参数，下次启动自动重试 */
            while (1)
                os_delay(1.0);
        }

        /* LFS_ERR_CORRUPT：文件系统未格式化或元数据损坏。
         * 探测 superblock 区区分"干净未格式化介质"(全0xFF，格式化无数据损失)与
         * "曾有数据但已损坏"(格式化将丢弃残留数据)，结果供诊断查询 */
        InternalStorage_SuperblockClean = is_superblock_clean(sector_size);

        // SD_INT 无文件系统，检查是否需要从 flash 迁移数据
        if (FLASH_isExist())
        {
            uint32_t flash_page_size = getFlashPageSize();
            uint8_t *flash_read_buf_m = new uint8_t[flash_page_size];
            uint8_t *flash_prog_buf_m = new uint8_t[flash_page_size];
            uint8_t *flash_look_buf_m = new uint8_t[LOOKAHEAD_SIZE];
            lfs_t *flash_lfs = new lfs_t;

            if (flash_read_buf_m && flash_prog_buf_m && flash_look_buf_m && flash_lfs)
            {
                lfs_config flash_cfg = {
                    .read = Flash_read,
                    .prog = Flash_write,
                    .erase = Flash_erase,
                    .sync = Flash_sync,
                    .read_size = flash_page_size,
                    .prog_size = flash_page_size,
                    .block_size = getFlashSectorSize(),
                    .block_count = getFlashSectorCount(),
                    .block_cycles = BLOCK_CYCLES,
                    .cache_size = flash_page_size,
                    .lookahead_size = LOOKAHEAD_SIZE,
                    .read_buffer = flash_read_buf_m,
                    .prog_buffer = flash_prog_buf_m,
                    .lookahead_buffer = flash_look_buf_m,
                };

                if (lfs_mount(flash_lfs, &flash_cfg) == LFS_ERR_OK)
                {
                    // flash 有文件系统，格式化 SD 后迁移文件
                    bool migrated = false;
                    lfs_format(&FlashFs, &cfg);
                    if (lfs_mount(&FlashFs, &cfg) == LFS_ERR_OK)
                    {
                        char *mig_path = new char[256];
                        uint8_t *copy_buf = new uint8_t[4096];
                        if (mig_path && copy_buf)
                        {
                            mig_path[0] = '\0';
                            migrate_copy_dir(flash_lfs, &FlashFs, mig_path, 0, 256, copy_buf, 4096, flash_page_size, &UserFile_cfg);
                        }
                        delete[] mig_path;
                        delete[] copy_buf;
                        migrated = true;
                        res = LFS_ERR_OK;
                    }

                    lfs_unmount(flash_lfs);
                    if (migrated)
                    {
                        // 迁移成功，格式化 flash 防止下次重复迁移
                        lfs_format(flash_lfs, &flash_cfg);
                    }
                }
            }

            delete[] flash_read_buf_m;
            delete[] flash_prog_buf_m;
            delete[] flash_look_buf_m;
            delete flash_lfs;
        }

        if (res != LFS_ERR_OK)
        {
            // 无可迁移数据：仅 CORRUPT 路径可达此处(IO错误已提前返回)，格式化 SD_INT 重建文件系统
            lfs_format(&FlashFs, &cfg);
            res = lfs_mount(&FlashFs, &cfg);
        }
    }

    // 新建Config目录
    char name[50] = {0};
    lfs_info fInfo;
    strcat(name, "Config");
    if (lfs_stat(&FlashFs, name, &fInfo) < 0)
        lfs_mkdir(&FlashFs, name);

    // 新建Log目录
    name[0] = 0;
    strcat(name, "Log");
    if (lfs_stat(&FlashFs, name, &fInfo) < 0)
        lfs_mkdir(&FlashFs, name);

    // 更新是否已初始化状态
    if (res == LFS_ERR_OK)
    {
        InternalStorage_Semphr = xSemaphoreCreateRecursiveMutex();
        InternalStorage_Initialized = true;
    }
}