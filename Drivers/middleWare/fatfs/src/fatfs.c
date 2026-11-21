/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "fatfs.h"
#include "TimeBase.h"

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */    


/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
	/* FAT 时间戳格式(微软 fatgen103 规范):
	 * bit15-11 小时, bit10-5 分钟, bit4-0 秒数/2(2秒粒度, 0-29)。
	 * 秒字段只有 5 bit, 存原始秒时 sec>=32 会溢出污染分钟位(分+1)。
	 * 因此这里必须存 Seconds/2。 */
	RTC_TimeStruct RTCTime = Get_RTC_Time();
	return ( (RTCTime.Year - 1980) << 25 ) 	|
				 ( RTCTime.Month << 21 ) 	|
				 ( RTCTime.Date << 16 ) 	|
				 ( RTCTime.Hours << 11 ) 	|
				 ( RTCTime.Minutes << 5 ) 	|
				 ( RTCTime.Seconds / 2 );
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
