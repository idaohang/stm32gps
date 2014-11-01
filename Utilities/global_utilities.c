
#include "stm32_eval.h"
#include "global_utilities.h"

extern __IO uint32_t TimingDelay;

void delay_ms(uint32_t Timer)
{
/*
    volatile uint32_t i=0;
    uint32_t tickPerMs = (1000);

    while(Timer)
    {
        i=tickPerMs/6-1;
        while(i--);
        Timer--;
    }  */
	TimingDelay = Timer;

  while(TimingDelay != 0);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

}

