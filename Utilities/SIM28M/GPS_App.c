/****************************************Copyright (c)****************************************************
 **                                  		 广州星海科技
 **                                  http://simhi.taobao.com
 **																		 TEL:15018750559
 **																			 QQ:86817447
 **------------------------File Info----------------------------------------------------------------------
 ** File Name:             GPS_App.c
 ** Last modified Date:   	2012.5.15
 ** Last Version:          V1.0
 ** Description:           GPS管理解析
 **
 **--------------------------------------------------------------------------------------------------------
 ** Created By:            Chenht
 ** Created date:          2012-5-3
 ** Version:               v1.0
 ** Descriptions:          The original version 初始版本
 **
 **--------------------------------------------------------------------------------------------------------
 ** Modified by:
 ** Modified date:
 ** Version:
 ** Description:
 **
 *********************************************************************************************************/
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "stm32gps_config.h"
#include "stm32_eval.h"
#include "usart.h"
#include "GSM_App.h"
#include "string_operator.h"

char GPSBuffer[USART_GPS_BUFSIZE];

/*
 $GPGGA,054753.000,2308.161553,N,11322.462574,E,1,9,0.88,45.402,M,-6.310,M,,*72
 $GPGLL,2308.161553,N,11322.462574,E,054753.000,A,A*50
 $GPGSA,A,3,14,22,31,25,30,18,32,29,16,,,,1.66,0.88,1.41*00
 $GPGSV,3,1,12,14,56,032,35,22,56,183,50,31,52,312,39,25,44,046,21*7C
 $GPGSV,3,2,12,30,37,210,49,18,17,156,47,32,17,319,29,29,16,107,51*75
 $GPGSV,3,3,12,16,10,207,45,12,03,036,26,01,00,295,,40,,,45*43
 $GPRMC,054753.000,A,2308.161553,N,11322.462574,E,0.000,0,010512,,,A*7E
 $GPVTG,0,T,,M,0.000,N,0.000,K,A*13
 $GPZDA,054753.000,01,05,2012,,*53
 */
char GPS_Einfo[30] = "E:\0";
char GPS_Ninfo[30] = "N:\0";
char Altitude_info[5] = "\0";
char Speed_info[6] = "\0";
char Degrees_info[4] = "\0";
/*********************************************************************************************************
 ** Function name:       GPSInfoAnalyze
 ** Descriptions:        解析GPS数据
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      NONE
 *********************************************************************************************************/
#define DBG_GPSInfoAnalyze
void GPSInfoAnalyze(void)
{
    unsigned int uLen;
    unsigned char i;
    char *phead, *psearch;
    uLen = 1024;

    if (USART_ENPROCESS
        != usart_readbuffer(STM32_SIM908_GPS_COM, GPSBuffer, &uLen))
    {
        return;
    }

    if (uLen > 0)
    {
#ifdef DBG_GPSInfoAnalyze
        {
            unsigned int i;

            printf("GPSInfoAnalyze recv %d bytes\r\n", uLen);
            for (i = 0; i < uLen; i++)
            {
                printf("%c", GPSBuffer[i]);
            }

            printf("\r\n");
            printf("GPSInfoAnalyze done\r\n");
        }
#endif

        phead = strstr((const char *) GPSBuffer, (const char *) "$GPGGA");

        if (NULL != phead)
        {
            /* 解析经度 */
            psearch = NULL;
            psearch = strnchr(phead, ',', 2);
            if (NULL != psearch)
            {
                psearch++;
                for (i = 0; *psearch != ','; i++)
                {
                    GPS_Einfo[i + 2] = *psearch;
                    psearch++;
                }
                GPS_Einfo[i + 2] = '*';
            }
            /* 解析纬度 */
            psearch = NULL;
            psearch = strnchr(phead, ',', 4);
            if (NULL != psearch)
            {
                psearch++;
                for (i = 0; *psearch != ','; i++)
                {
                    GPS_Ninfo[i + 2] = *(psearch++);
                }
                GPS_Ninfo[i + 2] = '*';
            }

            /* 解析海拔高度 */
            psearch = NULL;
            psearch = strnchr(phead, ',', 9);
            if (NULL != psearch)
            {
                psearch++;
                for (i = 0; i < 4; i++) //只取前面4个值
                {
                    Altitude_info[i] = *(psearch++);
                }
                Altitude_info[i] = '\0';
            }
        }

        phead = strstr((const char *) GPSBuffer, (const char *) "$GPVTG");
        if (NULL != phead)
        {
            /* 解析方向角度 */
            psearch = NULL;
            psearch = strnchr(phead, ',', 1);
            if (NULL != psearch)
            {
                psearch++;
                for (i = 0; *psearch != ','; i++)
                {
                    Degrees_info[i] = *psearch;
                    psearch++;
                }
                Degrees_info[i] = '\0';
            }
            /* 解析速度 */
            psearch = NULL;
            psearch = strnchr(phead, ',', 7);
            if (NULL != psearch)
            {
                psearch++;
                for (i = 0; i < 5; i++)
                {
                    Speed_info[i] = *(psearch++);
                }
                Speed_info[i] = '\0';
            }
        }
		
        memset(GPSBuffer, 0, uLen);
    }
}

void GPSShow(void)
{
	unsigned int i;
	
	int uLen ;
		
	uLen = strlen(GPS_Einfo);
    printf("\n GPS_Einfo : ");
    for (i = 0; i < uLen; i++)
    {
        printf("%c", GPS_Einfo[i]);
    }

	uLen = strlen(GPS_Ninfo);
    printf("\n GPS_Ninfo : ");
    for (i = 0; i < uLen; i++)
    {
        printf("%c", GPS_Ninfo[i]);
    }

	uLen = strlen(Altitude_info);
    printf("\n Altitude_info : ");
    for (i = 0; i < uLen; i++)
    {
        printf("%c", Altitude_info[i]);
    }

	uLen = strlen(Speed_info);
    printf("\n Speed_info : ");
    for (i = 0; i < uLen; i++)
    {
        printf("%c", Speed_info[i]);
    }
}

