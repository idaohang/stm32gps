#include "stm32f10x.h"
#include "stm32f10x_it_api.h"
#include "stm32gps_board.h"
#include "stm32gps_config.h"
#include <stdio.h>

USART_InitTypeDef USART_InitStructure;

/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void delay_10ms(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  Configures the SysTick to generate an interrupt each 10 ms.
  * @param  None
  * @retval None
  */
void stm32gps_sys_tick_cfg(void)
{
	/* SysTick interrupt each 10 ms */
	if (SysTick_Config(SystemCoreClock / SYS_TICK_PER_SEC))
    {
      /* Capture error */
      while (1);
    }
}

/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
  /* Check if the StandBy flag is set */
  if(PWR_GetFlagStatus(PWR_FLAG_SB) != RESET)
  {/* System resumed from STANDBY mode */

    /* Clear StandBy flag */
    PWR_ClearFlag(PWR_FLAG_SB);

    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();
    /* No need to configure the RTC as the RTC configuration(clock source, enable,
       prescaler,...) is kept after wake-up from STANDBY */
  }
  else
  {/* StandBy flag is not set */

    /* RTC clock source configuration ----------------------------------------*/
    /* Reset Backup Domain */
    BKP_DeInit();
  
    /* Enable LSE OSC */
	RCC_LSICmd(ENABLE);
    /* Wait till LSE is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* RTC configuration -----------------------------------------------------*/
    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();

    /* Set the RTC time base to 1s */
    RTC_SetPrescaler(32767);  
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
	RTC_ITConfig(RTC_IT_ALR, ENABLE);
	RTC_WaitForLastTask();
  }
}

/**
  * @brief  Configures the NVIC.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{  
NVIC_InitTypeDef NVIC_InitStructure;  /*������ռ���ȼ�1λ����ռ���ȼ�3λ*/  
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  /*ѡ��RTC��IRQͨ��*/   
NVIC_InitStructure.NVIC_IRQChannel =RTC_IRQn;  /*�����ж���ռ���ȼ�Ϊ1*/   
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;  /*�����жϴ�ռ���ȼ�Ϊ1*/   
NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;  /*ʹ��RTC��IRQͨ��*/   
NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;  
NVIC_Init(&NVIC_InitStructure);

}


/**
  * @brief  Configures the WatchDog.
  * @param  None
  * @retval None
  */
void IWDG_Configuration(void)
{
	/* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
	     dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	/* Set counter reload value to obtain 10s IWDG TimeOut.
	 Counter Reload Value = 10s/6.4ms = 1562
	*/
	IWDG_SetReload(1562);
}

/**
  * @brief  Configures the SysTick to generate an interrupt each 250 ms.
  * @param  None
  * @retval None
  */
void stm32gps_led_cfg(void)
{
    STM_EVAL_LEDInit_Test(LED1);
    //STM_EVAL_LEDInit(LED2);
    //STM_EVAL_LEDInit(LED3);
    //STM_EVAL_LEDInit(LED4);

    STM_EVAL_LEDOn(LED1);
    //STM_EVAL_LEDOff(LED2);
    //STM_EVAL_LEDOff(LED3);
    //STM_EVAL_LEDOn(LED4);
}

void stm32gps_com_debug_cfg(void)
{
    /* USARTx configured as follow:
     - BaudRate = 9600 baud
     - Word Length = 8 Bits
     - One Stop Bit
     - No parity
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = USART_DBG_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM3_DEBUG, &USART_InitStructure);
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

void stm32gps_com_gps_cfg(void)
{
    USART_InitStructure.USART_BaudRate = USART_GPS_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM1_GPS, &USART_InitStructure);
}

void stm32gps_com_gsm_cfg(void)
{
    USART_InitStructure.USART_BaudRate = USART_GSM_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM2_GSM, &USART_InitStructure);
}
