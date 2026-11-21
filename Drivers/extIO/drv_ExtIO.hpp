#pragma once

#include "basic.h"
#include <stdint.h>

struct EXTIO_CONFIG
{
    // 校准功能
    uint32_t calib[2];
    // id
    uint32_t calib_id[2];
    // 校准参数
    uint32_t calib_param1[2];
    uint32_t calib_param2[2];
} __ALIGNED_PACKED;

void init_drv_ExtIO();