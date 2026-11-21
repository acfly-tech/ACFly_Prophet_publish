#pragma once

#include <stdint.h>

struct FLOWMETER_CONFIG
{
    float anglog1_offset[2];
    float anglog1_kp[2];

    float anglog2_offset[2];
    float anglog2_kp[2];

    float anglog3_offset[2];
    float anglog3_kp[2];

    float anglog4_offset[2];
    float anglog4_kp[2];

    float tmr_ch1_offset[2];
    float tmr_ch1_kp[2];

    float tmr_ch2_offset[2];
    float tmr_ch2_kp[2];

    float tmr_ch3_offset[2];
    float tmr_ch3_kp[2];

    float tmr_ch4_offset[2];
    float tmr_ch4_kp[2];

    float tmr_ch5_offset[2];
    float tmr_ch5_kp[2];

    float weight_offset[2];
    float weight_kp[2];

} __AC_PACKED;

//
void init_drv_extIO_Expansion_C();