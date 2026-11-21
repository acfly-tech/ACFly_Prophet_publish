#pragma once

#include "Commulink.hpp"
#include <stdint.h>

void init_drv_RTCMSd();

void set_Rtcm_WritePort(Port port);
void set_Rtcm_MavlinkChan(uint8_t chan);
