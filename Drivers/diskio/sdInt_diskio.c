
/* Includes ------------------------------------------------------------------*/
#include "sdInt_diskio.h"
#include "Basic.h"
#include "lfs.h"
#include "sd_diskio.h"
#include "stm32h743xx.h"
#include <string.h>

#define Flash_TIMEOUT 1.0
#define pdrv DEV_SD_INT

// 缓存 block_size / sector_size，避免每次 IO 都调用 disk_ioctl
// disk_ioctl 在 SD 未就绪时不写 buff，会导致栈上未初始化值被使用（除零 -> HardFault）
static uint32_t s_block_size = 0;  // 擦除块大小（单位：扇区数），由 GET_BLOCK_SIZE 返回
static uint32_t s_sector_size = 0; // 扇区字节数，由 GET_SECTOR_SIZE 返回，SD 卡固定 512

// 内部辅助：首次调用时从 disk_ioctl 读取并缓存，失败返回 0
static int sdInt_ensure_geometry(void)
{
    if (s_sector_size != 0 && s_block_size != 0)
        return 1; // 已缓存，直接返回

    uint32_t tmp_block = 0;
    uint32_t tmp_sector = 0;
    if (disk_ioctl(pdrv, GET_BLOCK_SIZE, &tmp_block) != RES_OK || tmp_block == 0)
        return 0;
    if (disk_ioctl(pdrv, GET_SECTOR_SIZE, &tmp_sector) != RES_OK || tmp_sector == 0)
        return 0;

    s_block_size = tmp_block;
    s_sector_size = tmp_sector;
    return 1;
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int sdInt_erase(const struct lfs_config *c, lfs_block_t block)
{
    if (!sdInt_ensure_geometry())
        return LFS_ERR_IO;

    uint32_t sectorsInBlock = s_block_size;
    if (SD_erase(pdrv, block * sectorsInBlock, sectorsInBlock) == RES_OK)
        return 0;
    return LFS_ERR_IO;
}

// Read a region in a block. Negative error codes are propogated
// to the user.
int sdInt_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buff, lfs_size_t size)
{
    if (!sdInt_ensure_geometry())
        return LFS_ERR_IO;

    uint32_t sector = off / s_sector_size;
    uint32_t count = size / s_sector_size;

    uint32_t sectorsInBlock = s_block_size;
    if (disk_read(pdrv, buff, block * sectorsInBlock + sector, count) == RES_OK)
        return 0;

    return LFS_ERR_IO;
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int sdInt_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buff, lfs_size_t size)
{
    if (!sdInt_ensure_geometry())
        return LFS_ERR_IO;

    uint32_t sector = off / s_sector_size;
    uint32_t count = size / s_sector_size;

    uint32_t sectorsInBlock = s_block_size;
    if (disk_write(pdrv, buff, block * sectorsInBlock + sector, count) == RES_OK)
        return 0;

    return LFS_ERR_IO;
}

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
int sdInt_sync(const struct lfs_config *c)
{
    return 0;
}