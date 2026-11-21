#include "Basic.hpp"
#include "HardWareConfig.hpp"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "AC_Math.hpp"
#include "AuxFuncs.hpp"
#include "ControlSystem.hpp"
#include "Fences.hpp"
#include "FlashIO.h"
#include "MS_Main.hpp"
#include "MSafe.hpp"
#include "Missions.hpp"
#include "Modes.hpp"
#include "Parameters.hpp"
#include "ctrl_Main.hpp"
#include "drv_Main.hpp"
#include "drv_PWMOut.hpp"

#include "debug.hpp"

#if 0 // 如果没有这段，则需要在target选项中选择使用USE microLIB

	__asm(".global __use_no_semihosting\n\t") ;//注释本行, 方法1
	extern "C"
	{
//		struct __FILE {
//		int handle;
//		};
//		std::FILE __stdout;

		void _sys_exit(int x)
		{
			x = x;
		}

		//__use_no_semihosting was requested, but _ttywrch was referenced, 增加如下函数, 方法2
		void _ttywrch(int ch)
		{
			ch = ch;
		}
		
		char *_sys_command_string(char *cmd, int len)
		{
				return 0;
		}
	}
#endif

// 固件版本
extern const uint32_t PROG_VERSION = 118670;
const uint32_t PROG_SIGN = 0xACa026AC;
volatile const uint64_t PROGRAM_SIGN __attribute__((section(".ARM.__AT_0x08040000"))) __attribute__((used)) = ((uint64_t) PROG_SIGN << 32) | PROG_VERSION;
// 硬件版本
uint16_t HW_VERSION = 0;
uint16_t HW_Revision = 0;

volatile bool ESC_Calib_Completed = false;
extern "C" void startCPUStatsTask(void);
void DriverInit_task(void *pvParameters)
{
    // Start per-task CPU load statistics (debug helper).
    startCPUStatsTask();

    // 初始化设备驱动
    init_drv_Main();
    // 初始化解算系统
    init_MS_Main();
    init_Debug();
    init_Modes();
    init_MSafe();
    init_ControlSystem();

    /*注册初始化参数*/
    struct
    {
        uint32_t calib_ESC[2];     // 校准电调
        float calib_ESC_T[2];      // 电调校准时间
        uint32_t boot_count[2];    // 系统运行次数
        float Firmvare_Version[2]; // 固件版本
        uint32_t flight_count[2];  // 飞行架次
        float flight_time_s[2];    // 飞行总时间(秒)
        float flight_dist_m[2];    // 飞行总距离(米)
    } init_cfg;
    init_cfg.calib_ESC[0] = 0;
    init_cfg.calib_ESC_T[0] = 3.5;
    init_cfg.boot_count[0] = 0;
    init_cfg.Firmvare_Version[0] = PROG_VERSION * 0.001f;
    init_cfg.flight_count[0] = 0;
    init_cfg.flight_time_s[0] = 0;
    init_cfg.flight_dist_m[0] = 0;
    MAV_PARAM_TYPE param_types[] = {
        MAV_PARAM_TYPE_UINT32, // 校准电调
        MAV_PARAM_TYPE_REAL32, // 电调校准时间
        MAV_PARAM_TYPE_UINT32, // 系统运行次数
        MAV_PARAM_TYPE_REAL32, // 固件版本
        MAV_PARAM_TYPE_UINT32, // 飞行架次
        MAV_PARAM_TYPE_REAL32, // 飞行总时间(秒)
        MAV_PARAM_TYPE_REAL32  // 飞行总距离(米)
    };
    SName param_names[] = {
        "Init_CalibESC",   // 校准电调
        "Init_CalibESC_T", // 电调校准时间
        "Init_Boot_Count", // 系统运行次数
        "Init_Firmware_V", // 固件版本
        "Init_FlightCnt",  // 飞行架次
        "Init_FlightTime", // 飞行总时间(秒)
        "Init_FlightDist"  // 飞行总距离(米)
    };
    InitialParametersInit((uint64_t *) &init_cfg, param_types, param_names, sizeof(init_cfg) / 8);
    ParamGroupRegister("Init", 1, sizeof(init_cfg) / 8, param_types, param_names, (uint64_t *) &init_cfg);
    /*注册初始化参数*/

    /*注册20位唯一产品识别码参数*/
    struct
    {
        uint8_t id_1[8];
        uint8_t id_2[8];
        uint8_t id_3[8];
        uint8_t id_4[8];
        uint8_t id_5[8];
        uint8_t id_6[8];
        uint8_t id_7[8];
        uint8_t id_8[8];
        uint8_t id_9[8];
        uint8_t id_10[8];
        uint8_t id_11[8];
        uint8_t id_12[8];
        uint8_t id_13[8];
        uint8_t id_14[8];
        uint8_t id_15[8];
        uint8_t id_16[8];
        uint8_t id_17[8];
        uint8_t id_18[8];
        uint8_t id_19[8];
        uint8_t id_20[8];
        uint8_t id_regStaus[8];
    } uom_cfg;

    memset(&uom_cfg, 0, sizeof(uom_cfg));
    MAV_PARAM_TYPE uom_param_types[sizeof(uom_cfg) / 8];
    for (int i = 0; i < sizeof(uom_cfg) / 8; i++)
        uom_param_types[i] = MAV_PARAM_TYPE_UINT8;

    SName uom_param_names[] = {
        "ID_1",        "ID_2", "ID_3", "ID_4", "ID_5", "ID_6", "ID_7", "ID_8", "ID_9", "ID_10", "ID_11", "ID_12", "ID_13", "ID_14", "ID_15", "ID_16", "ID_17", "ID_18", "ID_19", "ID_20",
        "ID_regStatus" // 激活状态. [0: 无需激活(生产模式)], [1: 未激活(出货模式), [2: 已激活(出货模式)]
    };
    ParamGroupRegister("UOM", 1, sizeof(uom_cfg) / 8, uom_param_types, uom_param_names, (uint64_t *) &uom_cfg);
    /*注册20位唯一产品识别码参数*/

    // 完成初始化
    // 完成后不能再进行初始化操作
    while (getInitializationCompleted() == false)
    {
        setInitializationCompleted();
        os_delay(0.1);
    }

    uint32_t uav_type[2];
    if (ReadParam("AC_UAVType", 0, 0, (uint64_t *) uav_type, 0) == PR_OK)
    { // 获取主电机数量
        UAV_MTCount mt_count = UAV_MainMotorCount(uav_type[0]);
        set_MainMotorCount(mt_count.MTCount, mt_count.STCount);
    }

    /*读取初始化参数进行初始化操作*/
    // 读初始化参数
    ReadParamGroup("Init", (uint64_t *) &init_cfg, 0);

    // 校准电调
    if (init_cfg.calib_ESC[0] == 21586)
    {
        MainMotor_PullUpAll();
        os_delay(init_cfg.calib_ESC_T[0]);
        MainMotor_PullDownAll();

        init_cfg.calib_ESC[0] = 0;
    }

    // 系统重启次数加1
    init_cfg.boot_count[0]++;
    init_cfg.Firmvare_Version[0] = PROG_VERSION * 0.001f;
    // 重置初始化参数
    UpdateParamGroup("Init", (uint64_t *) &init_cfg, 0, sizeof(init_cfg) / 8);

    // 拉低电机输出
    MainMotor_PullDownAll();
    ESC_Calib_Completed = true;
    /*读取初始化参数进行初始化操作*/

    // 飞行任务初始化
    init_Missions();
    init_Fences();

    // 删除本任务
    vTaskDelete(0);
}

int main(void)
{
    // 初始化芯片时钟
    // 时间基准等基础功能
    init_Basic();

    // 创建初始化任务并进入任务调度器
    xTaskCreate(DriverInit_task, "Init", 4000, NULL, 3, NULL);
    vTaskStartScheduler();
    while (1)
        ;
}

extern "C" void HardFault_Handler()
{
    // 错误中断拉低所有输出
    PWM_PullDownAll();
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    static char StackOvTaskName[20];
    strcpy(StackOvTaskName, (char *) pcTaskName);
}
