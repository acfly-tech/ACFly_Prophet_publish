#include "drv_extLed.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "gpsConfig.hpp"

#include "drv_led_ACFly_C.hpp"
#include "drv_led_TCA62724FMG_I.hpp"

void init_drv_ExtLed()
{
    // can
    init_drv_led_ACFly_C();

    // i2c
    init_drv_led_TCA62724FMG_I();
}