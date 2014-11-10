/**
  ******************************************************************************
  * @file    stm32_eval.h
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   Header file for stm32_eval.c module.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32GPS_BOARD_H
#define __STM32GPS_BOARD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/


typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3
} Led_TypeDef;

typedef enum 
{
	COM1_GPS = 0,
	COM2_GSM = 1,
	COM3_DEBUG = 2
} COM_TypeDef;   


/** 
  * @brief  Uncomment the line corresponding to the STMicroelectronics evaluation
  *   board used in your application.
  *   
  *  Tip: To avoid modifying this file each time you need to switch between these
  *       boards, you can define the board in your toolchain compiler preprocessor.    
  */ 
#if !defined (USE_STM32_GPS_BOARD_VA) && !defined (USE_STM32_GPS_BOARD_VB)
 //#define USE_STM32_GPS_BOARD_VA
 //#define USE_STM32_GPS_BOARD_VB
#endif

#ifdef USE_STM32_GPS_BOARD_VA
 #include "stm32f10x.h"
 #include "STM32_GPS_BOARD_VA/stm32_gps_board.h"
#elif defined USE_STM32_GPS_BOARD_VB
 #include "stm32f10x.h"
 #include "STM32_GPS_BOARD_VB/stm32_gps_board.h" 
#else 
 #error "Please select first the STM32 GPS board to be used (in stm32_eval.h)"
#endif                      



#ifdef __cplusplus
}
#endif


#endif /* __STM32GPS_BOARD_H */

/**
  * @}
  */ 



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
