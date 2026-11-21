#include "drv_OpticalFlow.hpp"

// uart
#include "drv_opticalFlow_GL9306_U.hpp"
#include "drv_opticalFlow_JL32xx_U.hpp"
#include "drv_opticalFlow_LC302_U.hpp"
#include "drv_opticalFlow_LC306_U.hpp"
#include "drv_opticalFlow_T1_T2_U.hpp"

void init_drv_OpticalFlow()
{
    init_drv_opticalFlow_LC302_U();
    init_drv_opticalFlow_LC306_U();
    init_drv_opticalFlow_JL32xx_U();
    init_drv_opticalFlow_GL9306_U();
    init_drv_opticalFlow_T1_T2_U();
}
