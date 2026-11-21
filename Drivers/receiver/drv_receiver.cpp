#include "drv_receiver.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "ReceiverBackend.hpp"
#include "StorageSystem.hpp"
#include "task.h"

// analog
#include "drv_rc_ppm.hpp"

// uart
#include "drv_rc_crsf.hpp"
#include "drv_rc_sbus.hpp"
#include "drv_rc_sbus2.hpp"

void init_drv_receiver()
{
    // 注册接收机通用参数
    RC_COMMON_CONFIG init_rc_common_cfg;
    init_rc_common_cfg.configs[0] = RC_COMMON_CONFIG_SW_PRIO;
    MAV_PARAM_TYPE param_types[] = {
        MAV_PARAM_TYPE_UINT32, // configs
    };
    SName param_names[] = {
        "RC_Configs",
    };
    InitialParametersInit((uint64_t *) &init_rc_common_cfg, param_types, param_names, sizeof(RC_COMMON_CONFIG) / 8);
    ParamGroupRegister("RC_COMMON", 1, sizeof(RC_COMMON_CONFIG) / 8, param_types, param_names, (uint64_t *) &init_rc_common_cfg);

    // analog
    init_drv_rc_ppm();

    // sbus
    init_drv_rc_sbus();
    init_drv_rc_crsf();
    init_drv_rc_sbus2();
}