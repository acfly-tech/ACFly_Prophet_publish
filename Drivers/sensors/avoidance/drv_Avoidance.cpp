#include "drv_Avoidance.hpp"

// can
#include "drv_avoidance_MR72_C.hpp"

// i2c
#include "drv_avoidance_TFMini_I.hpp"

// uart
#include "drv_avoidance_MR72_U.hpp"
#include "drv_avoidance_TFMini_U.hpp"

void init_drv_Avoidance()
{
    // can
    init_drv_avoidance_MR72_C();

    // i2c
    init_drv_avoidance_TFMini_I();

    // uart
    init_drv_avoidance_MR72_U();
    init_drv_avoidance_TFMini_U();
}
