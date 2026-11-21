#include "drv_Gimbal.hpp"

// uart
#include "drv_gimbal_SIYI_U.hpp"
#include "drv_gimbal_SimpleBGCv1_U.hpp"

void init_drv_Gimbal()
{
    // uart
    init_drv_gimbal_SIYI();
    // init_drv_gimbal_SimpleBGCv1_U();
}
