#include "drv_Altimeter.hpp"

#include "basic.hpp"

// can
#include "drv_Alt1_NRA24_C.hpp"
#include "drv_Alt1_TR60_C.hpp"

// uart
#include "drv_Alt1_NRA24_U.hpp"
#include "drv_Alt1_TFMini_U.hpp"
#include "drv_Alt1_US100_U.hpp"

// i2c
#include "drv_Alt1_SPL06_I.hpp"
#include "drv_Alt1_TFMini_I.hpp"

// 其它
#include "drv_Alt1_Ultrasonic_TE.hpp"

void init_drv_Altimeter()
{
    // can
    init_drv_Alt1_TR60_C();
    init_drv_Alt1_NRA24_C();

    // uart
    init_drv_Alt1_TFMini_U();
    init_drv_Alt1_NRA24_U();
    init_drv_Alt1_US100_U();

    // i2c
    init_drv_Alt1_SPL06_I();
    init_drv_Alt1_TFMini_I();

    // 其它
    // 超声波定高(Trig/Echo): 依赖 MFunc 参数, 需在模式功能参数注册后初始化
    init_drv_Alt1_Ultrasonic_TE();
}