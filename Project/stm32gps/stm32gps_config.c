
#include "stm32f10x.h"
#include "stm32f10x_it_api.h"
#include "stm32_eval.h"
#include "stm32gps_config.h"
#include "usart.h"
#include <stdio.h>


/**
  * @brief  Configures the SysTick to generate an interrupt each 1 ms.
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void)
{
	/* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
  /* Set SysTick Priority to 3 */
  NVIC_SetPriority(SysTick_IRQn, 0x0C);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

#ifdef USE_STM32_GPS_BOARD_VA
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
#elif defined USE_STM32_GPS_BOARD_VB
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
}


/**
  * @brief  Configures the Leds.
  * @param  None
  * @retval None
  */
void Led_Configuration(void)
{
#ifdef USE_STM32_GPS_BOARD_VA
    STM_EVAL_LEDInit_Test(LED1);
    STM_EVAL_LEDOn(LED1);
#elif defined USE_STM32_GPS_BOARD_VB
	STM_EVAL_LEDInit(LED1);
    STM_EVAL_LEDInit(LED2);
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);

    STM_EVAL_LEDOn(LED1);
    STM_EVAL_LEDOff(LED2);
    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOn(LED4);
#endif
}

void UsartDbg_Configuration(void)
{
    /* USARTx configured as follow:
     - BaudRate = 9600 baud
     - Word Length = 8 Bits
     - One Stop Bit
     - No parity
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
     */
	USART_InitTypeDef USART_InitStructure;
	
    USART_InitStructure.USART_BaudRate = USART_DBG_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM3_DEBUG, &USART_InitStructure);
}

void UsartGps_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
#ifdef USE_STM32_GPS_BOARD_VA
 	USART_InitStructure.USART_BaudRate = USART_GPS_BAUD;
#elif defined USE_STM32_GPS_BOARD_VB
 	USART_InitStructure.USART_BaudRate = 115200;
#endif
    USART_InitStructure.USART_BaudRate = USART_GPS_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM1_GPS, &USART_InitStructure);
}

void UsartGsm_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	
    USART_InitStructure.USART_BaudRate = USART_GSM_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM2_GSM, &USART_InitStructure);
}

/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure one bit for preemption priority -------------------------------- */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);


	/* Enable the USARTz Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

    /* Enable the USARTz Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USARTz Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void IT_Configuration(void)
{
	USART_IRQHandler_register(COM1_GPS,(UART_INT_HANDLER)usart_irq,0);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_IRQHandler_register(COM2_GSM,(UART_INT_HANDLER)usart_irq,0);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_IRQHandler_register(COM3_DEBUG,uart3_int_handler,0);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}


