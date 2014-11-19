/**
  ******************************************************************************
  * @file    USART/Printf/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x_it.h"
#include "stm32f10x_it_api.h"
#include "stm32gps_config.h"
#include "stm32gps_board.h"
#include "usart.h"
#include "GSM_App.h"
#include "GPS_App.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
static volatile uint32_t mSysTick = 0;
uint32_t sysTickGet(void)
{
    return mSysTick;
}
uint32_t sysTickPerSec(void)
{
    return SYS_TICK_PER_SEC;
}

void sysTickDelay(uint32_t tickCnt)
{
    uint32_t tick;

    tick = sysTickGet();
    while (sysTickGet() - tick <= tickCnt);
}

void SysTick_Handler(void)
{
    mSysTick ++;

    usart_timeout(0, 0);
    usart_timeout(1, 0);
	TimingDelay_Decrement();
}

static UART_INT_HANDLER uart_int_handler[COMn] = {NULL, NULL, NULL};
static uint32_t uart_int_arg[COMn] = {0, 0, 0};

void USART_IRQHandler_register(uint32_t com, UART_INT_HANDLER handler, uint32_t arg)
{
    if (com >= COMn)
    {
        return;
    }
    uart_int_handler[com] = handler;
    uart_int_arg[com] = arg;
}

void USART1_IRQHandler(void)
{
    if (uart_int_handler[0] != NULL)
    {
        uart_int_handler[0](0, uart_int_arg[0]);
    }
    else {
        while(1);
    }
}

void USART2_IRQHandler(void)
{
    if (uart_int_handler[1] != NULL)
    {
        uart_int_handler[1](1, uart_int_arg[1]);
    }
    else {
        while(1);
    }
}

void USART3_IRQHandler(void)
{
    if (uart_int_handler[2] != NULL)
    {
        uart_int_handler[2](2, uart_int_arg[2]);
    }
    else {
        while(1);
    }
}

void RTC_IRQHandler(void)
{

	RTC_ClearITPendingBit(RTC_IT_SEC|RTC_IT_ALR);  
	RTC_WaitForLastTask(); 
	//STM_EVAL_LEDToggle(LED1);
	/* Set the RTC Alarm after 5s */
	//RTC_SetAlarm(RTC_GetCounter()+ 5);
	/* Wait until last write operation on RTC registers has finished */
	//RTC_WaitForLastTask();
}

/**
  * @brief  This function handles RTC Alarm interrupt request.
  * @param  None
  * @retval None
  */
void RTCAlarm_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
  {
    /* Clear EXTI line17 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line17);

    /* Check if the Wake-Up flag is set */
    if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
    {
      /* Clear Wake Up flag */
      PWR_ClearFlag(PWR_FLAG_WU);
    }

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();   
    /* Clear RTC Alarm interrupt pending bit */
    RTC_ClearITPendingBit(RTC_IT_ALR);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
  }
}

void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
#ifdef MACRO_FOR_TEST
		STM_EVAL_LEDToggle(LED1);
#endif
		DEBUG("going in timer2 standby mode\n");
#if 1
		/////////////////////////////////////////////////////////////////
		// Power OFF GPS and GSM before go into standby mode
		/////////////////////////////////////////////////////////////////
		GSM_TurnOnOff_delay();
#ifdef USE_STM32_GPS_BOARD_VB
		GPSPowerOff();
		GSM_PowerOff();
#endif
		/* Wait till RTC Second event occurs */
		RTC_ClearFlag(RTC_FLAG_SEC);
		while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

		// normal working state
		if(BKP_TRUE == BKP_ReadBackupRegister(BKP_DR1))
		{
			/* Set the RTC Alarm after xx s */
			DEBUG("in timer2 bkptrue %d\n", BKP_ReadBackupRegister(BKP_DR4));
			RTC_SetAlarm(RTC_GetCounter()+ (BKP_ReadBackupRegister(BKP_DR4)/SLEEP_TIM2_RATIO));
		}
		else
		{
			DEBUG("in timer2 bkpfalse\n");
			RTC_SetAlarm(RTC_GetCounter()+ (SLEEP_NORMAL_SEC/SLEEP_TIM2_RATIO));
		}
	
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();		
		/* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
		PWR_EnterSTANDBYMode();
#endif
	}		 	
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
