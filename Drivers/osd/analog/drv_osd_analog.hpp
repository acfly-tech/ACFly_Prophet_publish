#pragma once

#include <stdint.h>

// =====================================================================
// AT7456E 寄存器定义
// =====================================================================
#define VM0 0x00
#define VM1 0x01
#define HOS 0x02
#define VOS 0x03
#define DMM 0x04
#define DMAH 0x05
#define DMAL 0x06
#define DMDI 0x07
#define CMM 0x08
#define CMAH 0x09
#define CMAL 0x0a
#define CMDI 0x0b
#define OSDM 0x0c
#define RB0 0x10
#define RB1 0x11
#define RB2 0x12
#define RB3 0x13
#define RB4 0x14
#define RB5 0x15
#define RB6 0x16
#define RB7 0x17
#define RB8 0x18
#define RB9 0x19
#define RB10 0x1a
#define RB11 0x1b
#define RB12 0x1c
#define RB13 0x1d
#define RB14 0x1e
#define RB15 0x1f
#define OSDBL 0x6c
#define STAT 0x20
#define DMDO 0x30
#define CMDO 0x40
#define NVM_RAM 0x50
#define RAM_NVM 0xa0
// VM0
#define NTSC (0 << 6)
#define PAL (1 << 6)
#define SYNC_AUTO (0 << 4)
#define SYNC_EXTERNAL (2 << 4)
#define SYNC_INTERNAL (3 << 4)
#define OSD_ENABLE (1 << 3)
#define OSD_DISABLE (0 << 3)
#define SOFT_RESET (1 << 1)
#define VOUT_ENABLE (0 << 0)
#define VOUT_DISABLE (1 << 0)
// VM1
#define BACKGND_0 (0 << 4)
#define BACKGND_7 (1 << 4)
#define BACKGND_14 (2 << 4)
#define BACKGND_21 (3 << 4)
#define BACKGND_28 (4 << 4)
#define BACKGND_35 (5 << 4)
#define BACKGND_42 (6 << 4)
#define BACKGND_49 (7 << 4)
#define BLINK_TIME40 (0 << 2)
#define BLINK_TIME80 (1 << 2)
#define BLINK_TIME120 (2 << 2)
#define BLINK_TIME160 (3 << 2)
#define BLINK_DUTY_1_1 0
#define BLINK_DUTY_1_2 1
#define BLINK_DUTY_1_3 2
#define BLINK_DUTY_3_1 3
// DMM
#define SPI_BIT16 (0 << 6)
#define SPI_BIT8 (1 << 6)
#define CLEAR_SRAM (1 << 2)
#define VETICAL_SYNC (1 << 1)
#define AUTO_INC (1 << 0)
// RBi
#define BLACK_LEVEL_0 (0 << 2)
#define BLACK_LEVEL_10 (1 << 2)
#define BLACK_LEVEL_20 (2 << 2)
#define BLACK_LEVEL_30 (3 << 2)
#define WHITE_LEVEL_120 (0 << 0)
#define WHITE_LEVEL_100 (1 << 0)
#define WHITE_LEVEL_90 (2 << 0)
#define WHITE_LEVEL_80 (3 << 0)
// STAT
#define PAL_DETECT (1 << 0)
#define NTSC_DETECT (1 << 1)
#define LOS_DETECT (1 << 2)
#define VSYNC (1 << 4)

// =====================================================================
// 类型定义
// =====================================================================
typedef enum
{
    OLD7456,
    NEW7456,
    BAD7456
} VER7456;
typedef enum
{
    MODE_PAL,
    MODE_NTSC,
    MODE_ERR
} VIDEO_MODE;

#define CHAR_LBC 4
#define CHAR_BLK 2
#define CHAR_INV 1

typedef struct
{
    unsigned short index;
    unsigned char attrib;
} CHAR_ATTRIB;

typedef struct
{
    unsigned char fileName[128];
    unsigned char fileDate[128];
    unsigned char dots[54 * 512];
} FONT_INFOR;

void init_drv_osd_analog();