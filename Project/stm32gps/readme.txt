
delay_ms�����Ķ�ʱ��׼��delay_ms(1000)���ֻ��0.6s���ҡ��ú���ʹ�������ʱʵ�֡�
delay_10ms������ʱ׼��delay_10ms(100)��1s���ú�������systickʵ�֡�

�� D:\GitHub\stm32gps\Project\stm32gps\system_stm32f10x.c ���޸���ʱ��Ƶ��Ϊ24MHz�����ʱ��Ϊ72MHz��TIM2�����ʱʱ��Ϊ59s���ң����ܲ�����Ҫ��
��stm32gps_config.c�е�RCC_Configuration����������APHB1ΪRCC_HCLK_Div4���������ʱ�Ӷ�ʱΪ3min��������Ҫ�趨��ֵ��

1. ��project��options��ѡ��USE_STM32_GPS_BOARD_VB��USE_STM32_GPS_BOARD_VA
2. ��עmain�����е�#if 0��ȥ�����޸�Ϊ#if 1��Ŀǰ���޸�ΪMACRO_FOR_TEST
3. DBG_ENABLE_MACRO����ʽ�汾��ȥ��
4. ��Ҫ����MACRO_FOR_TEST

��printf�޸�ΪDEBUG�궨��
TIM4�ĺ�����ȥ��