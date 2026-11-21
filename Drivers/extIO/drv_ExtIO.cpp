#include "drv_ExtIO.hpp"
#include "Commulink.hpp"
#include "HardWareConfig.hpp"
#include "Parameters.hpp"

#include "drv_extIO_AuxChannel_C.hpp"
#include "drv_extIO_Expansion_C.hpp"
#include "drv_extIO_HTL.hpp"
#include "drv_extIO_UAVCanESC_C.hpp"

#include "drv_extIO_FPVESC_U.hpp"
void init_drv_ExtIO()
{
    EXTIO_CONFIG init_cfg;
    memset(&init_cfg, 0, sizeof(init_cfg));

    MAV_PARAM_TYPE param_types[] = {
        MAV_PARAM_TYPE_UINT32,
        MAV_PARAM_TYPE_UINT32,
        MAV_PARAM_TYPE_UINT32,
        MAV_PARAM_TYPE_UINT32,
    };
    SName param_names[] = {
        "extIO_Calib",
        "extIO_C_Id",
        "extIO_C_Param1",
        "extIO_C_Param2",
    };
    InitialParametersInit((uint64_t *) &init_cfg, param_types, param_names, sizeof(init_cfg) / 8);
    ParamGroupRegister("EXTIOCFG", 1, sizeof(init_cfg) / 8, param_types, param_names, (uint64_t *) &init_cfg);

    if (HTL_isEnabled())
        init_drv_extIO_HTL();
    else
    {
        // CAN
        {
            // 电调
            init_drv_extIO_UAVCanESC_C();
            // 扩展IO
            init_drv_extIO_Expansion_C();
            init_drv_extIO_AuxChannel_C();
        }

        // UART
        {
            // 电调
            init_drv_extIO_FPVESC_U();
        }
    }
}
