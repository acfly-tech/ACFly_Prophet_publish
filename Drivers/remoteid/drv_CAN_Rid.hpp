#pragma once
#include "dronecan_msgs.h"

void set_BasicID_Pkt(remoteid_BasicID *pkt);

void set_OperatorID_Pkt(remoteid_OperatorID *pkt);

void set_SelfID_Pkt(remoteid_SelfID *pkt);

void set_System_Pkt(remoteid_System *pkt);

void init_drv_DroneCanRid();