/**
  ******************************************************************************
  * @file    USART/Printf/stm32f10x_it.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32_SIM908_CFG_H
#define __STM32_SIM908_CFG_H

#include "stm32_eval.h"

#define SYS_TICK_PER_SEC 100u

#define AT_RESEND_TIMES  5

#define USART_GSM_BUFSIZE_SEND 2048
#define USART_GSM_BUFSIZE_RECEIVE 1024
#define USART_GSM_BUFSIZE    1024
#define USART_GPS_BUFSIZE    1024
#define GSM_USART_TIMEOUT_MS 20
#define GPS_USART_TIMEOUT_MS 20

#define STM32_SIM908_GPS_COM COM1_GPS
#define STM32_SIM908_GSM_COM COM2_GSM

#define USART_GSM_BAUD 9600
#define USART_DBG_BAUD 9600
#ifdef USE_STM32_GPS_BOARD_VA
 	#define USART_GPS_BAUD 9600
#elif defined USE_STM32_GPS_BOARD_VB
 	#define USART_GPS_BAUD 115200
#endif

#define NULL 0

void SysTick_Configuration(void);
void GPIO_Configuration(void);
void Led_Configuration(void);
void UsartDbg_Configuration(void);

void UsartDbg_Configuration(void);
void UsartGps_Configuration(void);
void UsartGsm_Configuration(void);
void NVIC_Configuration(void);
void IT_Configuration(void);
void RTC_Configuration(void);
void EXTI_Configuration(void);


#endif /* __STM32_SIM908_CFG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
