#include "drv_PrecLand.hpp"

// i2c
#include "drv_precLand_IRLock_I.hpp"

// uart
#include "drv_precLand_AprilTag_U.hpp"

void init_drv_PrecLand()
{
    // i2c
    init_drv_precLand_IRLock_I();

    // uart
    init_drv_precLand_AprilTag_U();
}
