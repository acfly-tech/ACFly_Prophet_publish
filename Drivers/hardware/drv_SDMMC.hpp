#pragma once
#include <stdint.h>

uint8_t Check_USB_SD_Busy();
void Update_USB_SD_Busy_Time();

extern "C" uint8_t Lock_SD(double TIMEOUT);
extern "C" void UnLock_SD(void);

/* 卷级互斥锁接口：用于 f_mount/f_mkfs 等不可重入操作前串行化 */
/* FatFs 启用了 FF_FS_REENTRANT 文件级保护，但 f_mount 不受保护 */
/* 使用独立信号量，不影响底层 disk_read/disk_write 的 Lock_SD */
uint8_t Lock_SDVolume(double TIMEOUT);
void UnLock_SDVolume(void);

void init_drv_SDMMC(void);
