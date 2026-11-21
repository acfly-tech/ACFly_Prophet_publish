#include "drv_OSD.hpp"
#include "Commulink.hpp"
#include "Parameters.hpp"

#include "drv_msp_DJI_FPV.hpp"
#include "drv_msp_DisplayPort.hpp"
#include "drv_osd_analog.hpp"

void init_drv_OSD()
{
    init_drv_osd_analog();
    init_drv_msp_DisplayPort();
    init_drv_msp_DJI_FPV();
}