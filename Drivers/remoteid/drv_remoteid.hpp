#pragma once

#include "Basic.hpp"
#include "mavlink.h"
typedef enum
{
    RID_STATUS_IDLE = 0,
    RID_UNREGISTERED = 1,
    RID_STATUS_BROADCAST_FAIL,
    RID_STATUS_DISCONNECT_WITH_DRONE,
    RID_STATUS_BROADCAST_OK,
} RID_STATUS;

bool SendRidMsgToTask(mavlink_message_t msg, double TIMEOUT);
bool TaskReceiveRidMsg(mavlink_message_t *msg, double TIMEOUT);
void set_rid_status(RID_STATUS status);
RID_STATUS get_rid_status(void);
void init_drv_Remoteid(void);