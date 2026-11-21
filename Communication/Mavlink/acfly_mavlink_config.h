#pragma once

/*
 * K9 MAVLink board configuration.
 *
 * Injected at the top of the generated acfly/mavlink.h by
 * GCSPRO/script/mavlink/regen_acfly.py --target-fc. This file is NOT
 * overwritten by the generator.
 */

#include "mavlink_types.h"

#ifndef MAVLINK_EXTERNAL_RX_STATUS
#define MAVLINK_EXTERNAL_RX_STATUS
#endif

#ifndef MAVLINK_EXTERNAL_RX_BUFFER
#define MAVLINK_EXTERNAL_RX_BUFFER
#endif

/* The RX status/buffer arrays are defined in Communication/Mavlink.cpp with
   MAVLINK_COMM_NUM_BUFFERS + CUSTOM_CHANS entries. Upstream mavlink_helpers.h
   no longer declares them internally when the EXTERNAL options are used. */
extern mavlink_status_t  m_mavlink_status[];
extern mavlink_message_t m_mavlink_buffer[];
