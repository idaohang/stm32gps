#include "stm32f10x.h"
#include "stm32f10x_it_api.h"
#include "stm32gps_board.h"
#include "stm32gps_config.h"
#include <stdio.h>

USART_InitTypeDef USART_InitStructure;

void delay_ms(uint32_t Timer)
{
    volatile uint32_t i=0;
    uint32_t tickPerMs = SystemCoreClock/1000;

    while(Timer)
    {
        i=tickPerMs/6-1;
        while(i--);
        Timer--;
    }
}

void stm32_sim908_sys_tick_cfg(void)
{
    if (SysTick_Config(SystemCoreClock / SYS_TICK_PER_SEC))
    {
      /* Capture error */
      while (1);
    }
}
void stm32_sim908_led_cfg(void)
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

void stm32_sim908_com_debug_cfg(void)
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

void stm32_sim908_com_gps_cfg(void)
{
    USART_InitStructure.USART_BaudRate = USART_GPS_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM1_GPS, &USART_InitStructure);
}

void stm32_sim908_com_gsm_cfg(void)
{
    USART_InitStructure.USART_BaudRate = USART_GSM_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM2_GSM, &USART_InitStructure);
}
