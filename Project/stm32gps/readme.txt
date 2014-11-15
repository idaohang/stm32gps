
delay_ms函数的定时不准，delay_ms(1000)大概只有0.6s左右。该函数使用语句延时实现。
delay_10ms函数定时准，delay_10ms(100)是1s，该函数是用systick实现。

在 D:\GitHub\stm32gps\Project\stm32gps\system_stm32f10x.c 中修改了时钟频率为24MHz，如果时钟为72MHz，TIM2的最长定时时间为59s左右，可能不符合要求。


1. 在project的options中选择USE_STM32_GPS_BOARD_VB或USE_STM32_GPS_BOARD_VA
2. 关注main函数中的#if 0都去掉或修改为#if 1。
