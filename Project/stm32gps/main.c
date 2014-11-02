/******************************************************************************

              Copyright (C), 2014, IntVita Corporation 

 ******************************************************************************
  File Name     : main.c
  Version       : Initial Draft
  Author        : Qixb
  Created       : 2014/11/1
  Last Modified :
  Description   : Main program body
  Function List :
              assert_failed
              main
  History       :
  1.Date        : 2014/11/1
    Author      : Qixb
    Modification: Created file

******************************************************************************/
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32_eval.h"
#include "stm32gps_config.h"
#include "global_utilities.h"
#include "usart.h"
#include "GSM_App.h"
#include "GPS_App.h"

/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/

/*----------------------------------------------*
 * routines' implementations                    *
 *----------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 USART_InitTypeDef USART_InitStructure;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

    
    GPIO_Configuration();
    Led_Configuration();
	
    UsartDbg_Configuration();
/*
    usart_init(COM1_GPS);
    UsartGps_Configuration();

    usart_init(COM2_GSM);
    UsartGsm_Configuration();
*/
	NVIC_Configuration();
	IT_Configuration();

	/* Configure EXTI Line to generate an interrupt on falling edge */
  EXTI_Configuration();

	/* Enable PWR and BKP clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
PWR_BackupAccessCmd(ENABLE); 

	RTC_Configuration();
	
	SysTick_Configuration();

	/* Wait till RTC Second event occurs */
    RTC_ClearFlag(RTC_FLAG_SEC);
    while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

    /* Alarm in 3 second */
    RTC_SetAlarm(RTC_GetCounter()+ 3);
	
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
	STM_EVAL_LEDOn(LED3);
#ifdef TEST_MACRO
	//test_led();
	//test_uart1_send();
	//test_uart2_send();
	test_uart3_send();
	//test_uart3_send();
#endif

    //GSM_Init();
	//GPS_Init();


	//GSM_test_once();
	//printf("\n END \n");
    //GSM_SendSMS(targetNumber, targetMsg, 1);

/* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
//PWR_EnterSTANDBYMode();

    while(1)
    {
        //GSM_test();
        //GPSInfoAnalyze();
		//GPSShow();
		//GPRS_ReceiveData("+CMTI");
		//Delay(1000);
		/* Set the RTC Alarm after 3s */
	    /* Insert 1.5 second delay */
		STM_EVAL_LEDOn(LED2);
	    Delay(1500);
		STM_EVAL_LEDOff(LED2);
Delay(1500);

    }
}


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE 
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(EVAL_COM3, (uint8_t) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(EVAL_COM3, USART_FLAG_TC) == RESET) {
    }

    return ch;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
