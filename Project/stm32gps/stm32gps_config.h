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

#include "stm32f10x.h"
#include "stm32gps_board.h"

#define SEND_SUCCESS_TIMES  4  // 成功发送次数

#define SYS_TICK_PER_SEC 100u
#define TIM2_PRESCALER_TIMER 2000
#define TIM2_PERIOD_TIMER 60000  // 30s

// intimer [1 - 65535]
#define SLEEP_SEC_INTIMER  600 // 10 min
#define NORMAL_SEC_INTIMER 1200 // 20min

#define AT_RESEND_TIMES  5

#define GSM_SERVER_IP   "121.40.200.84"
#define GSM_SERVER_PORT "6666"

#define EELINK_LOGIN_MSGLEN  17
#define EELINK_GPS_MSGLEN    42
#define EELINK_LANG  0x01  // English
#define EELINK_ZONE  0x20  // east 8


#define USART_GSM_BUFSIZE_SEND 2048
#define USART_GSM_BUFSIZE_RECEIVE 1024
#define USART_GSM_BUFSIZE    1024
#define USART_GPS_BUFSIZE    1024
#define GSM_USART_TIMEOUT_MS 20
#define GPS_USART_TIMEOUT_MS 20

#define IMEI_BUFSIZE   15

#define STM32_SIM908_GPS_COM COM1_GPS
#define STM32_SIM908_GSM_COM COM2_GSM

#define USART_GSM_BAUD 9600
#define USART_GPS_BAUD 9600
#define USART_DBG_BAUD 9600

#define NULL 0
#define RST_OK   0xAA
#define RST_FAIL 0x55

/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);

void delay_10ms(__IO uint32_t nTime);
void delay_ms(uint32_t Timer);
void stm32gps_sys_tick_cfg(void);
void RTC_Configuration(void);
void TIM2_Configuration(void);
void RTC_NVIC_Configuration(void);
void TIM2_NVIC_Configuration(void);
void TIM2_Start(void);
void TIM2_Stop(void);
void IWDG_Configuration(void);
void stm32gps_led_cfg(void);
void stm32gps_com_debug_cfg(void);
void stm32gps_com_gps_cfg(void);
void stm32gps_com_gsm_cfg(void);

#endif /* __STM32_SIM908_CFG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
