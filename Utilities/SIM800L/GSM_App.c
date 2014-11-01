/****************************************Copyright (c)****************************************************
 **                                  		 广州星海科技
 **                                  http://simhi.taobao.com
 **																		 TEL:15018750559
 **																			 QQ:86817447
 **------------------------File Info----------------------------------------------------------------------
 ** File Name:             GSM_App.c
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
#include "at_command.h"
#include "string_operator.h"
#include "global_utilities.h"
#include "GSM_App.h"


static char BackBuf[USART_GSM_BUFSIZE];
static char sendBuf[USART_GSM_BUFSIZE_SEND];
static char receiveBuf[USART_GSM_BUFSIZE_RECEIVE];

pST_GSMCMD pGSM_msg;
extern volatile ST_RTCTIME CurSysTime;
unsigned char GSMNetType;


/*********************************************************************************************************
 ** Function name:       GSM_RingPinInit()
 ** Descriptions:        Ring状态检测IO初始化
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:
 *********************************************************************************************************/
void GSM_RingPinInit(void)
{
#if 0
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource8);

    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
#endif
}

/*********************************************************************************************************
 ** Function name:       GSM_ChkRingSta()
 ** Descriptions:        检测Ring状态
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      Ring状态
 *********************************************************************************************************/
unsigned char GSM_ChkRingSta(void)
{
#if 0
    if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8))
    {
        return 1;
    }
#endif
    return 0;
}

/*********************************************************************************************************
 ** Function name:       GSM_PowerCtrlInit()
 ** Descriptions:        启动控制IO初始化
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:
 *********************************************************************************************************/
void GSM_PowerCtrlInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*********************************************************************************************************
 ** Function name:       GSM_PowerOnOff()
 ** Descriptions:        启动或关闭模块
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:
 *********************************************************************************************************/
void GSM_PowerOnOff(void)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_10);
    delay_ms(3000);
    GPIO_SetBits(GPIOC, GPIO_Pin_10);
    delay_ms(3000);
}

/*********************************************************************************************************
 ** Function name:       GSM_ClearBuffer()
 ** Descriptions:        清空接收缓存
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:
 *********************************************************************************************************/
void GSM_ClearBuffer(void)
{
    memset(BackBuf, 0, USART_GSM_BUFSIZE);
}
/*********************************************************************************************************
 ** Function name:       GSM_SendAT()
 ** Descriptions:        发送一条AT指令
 ** input parameters:    pCMD：指令指针
 **                      pCMDBack： 指令返回参考指针
 **                      CMDLen:  指令长度
 ** output parameters:   NONE
 ** Returned value:      发送指令返回状态
 *********************************************************************************************************/
#define DBG_GSM_SendAT
unsigned char GSM_SendAT(char *pCMD, char *pCMDBack, uint32_t CMDLen)
{
    unsigned char i = AT_RESEND_TIMES;
    unsigned int len;
    char *pBackBuf = BackBuf;
    unsigned char retFlag = 0;

    len = CMDLen;

#ifdef DBG_GSM_SendAT
    {
        uint32_t tmpIdx;

        if (len > 0)
        {
            printf("GSM_SendAT send\r\n");
            for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
            {
                //printf("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                printf("%c",pCMD[tmpIdx]);
            }
            printf("\r\n");
            printf("GSM_SendAT done\r\n");
        }
    }
#endif

    usart_sendbuffer(STM32_SIM908_GSM_COM, pCMD, (unsigned int *) &len);

    if (NULL != pCMDBack)
    {
        while (--i)
        {
            len = USART_GSM_BUFSIZE;
            delay_ms(100);

            //printf("GSM_SendAT before usart_readbuffer len %d\n", len);

            retFlag = usart_readbuffer(STM32_SIM908_GSM_COM, pBackBuf, &len);
            //printf("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_GSM_SendAT
            {
                uint32_t tmpIdx;

                if (len > 0 && retFlag == USART_ENPROCESS)
                {
                    printf("GSM_SendAT recv %d bytes\r\n", len);
                    for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                    {
                        //printf("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                        printf("%c",pBackBuf[tmpIdx]);
                    }
                    printf("\r\n");
                    printf("GSM_SendAT recv done\r\n");
                }
            }
#endif
            if (len > 0 && retFlag == USART_ENPROCESS)
            {
                if (NULL != strstr_len(pBackBuf, pCMDBack, len))
                {
                    break;
                }
            }
        }
		
        if (i == 0)
        {
            return USART_FAIL;
        }
    }
    return USART_SUCESS;
}

#define DBG_GSM_SendAT_buffer
unsigned char GSM_SendAT_rsp(char *pCMD, char *pCMDBack,
        uint32_t CMDLen, char **ppRecvBuf, uint32_t *pRecvLen)
{
    unsigned char i = 5;
    unsigned int len;
    char *pBackBuf = BackBuf;
    unsigned char retFlag = 0;

    len = CMDLen;

#ifdef DBG_GSM_SendAT_buffer
            {
                uint32_t tmpIdx;

                if (len > 0)
                {
                    printf("GSM_SendAT send\r\n");
                    for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                    {
                        //printf("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                        printf("%c",pCMD[tmpIdx]);
                    }
                    printf("\r\n");
                    printf("GSM_SendAT done\r\n");
                }
            }
#endif

    usart_sendbuffer(STM32_SIM908_GSM_COM, pCMD, (unsigned int *) &len);

    while (--i)
    {
        len = USART_GSM_BUFSIZE;
        delay_ms(100);

        //printf("GSM_SendAT before usart_readbuffer len %d\n", len);
        retFlag = usart_readbuffer(STM32_SIM908_GSM_COM, pBackBuf, &len);
        //printf("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_GSM_SendAT_buffer
        {
            uint32_t tmpIdx;

            if (len > 0 && retFlag == USART_ENPROCESS)
            {
                printf("GSM_SendAT recv\r\n");
                for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                {
                    //printf("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                    printf("%c",pBackBuf[tmpIdx]);
                }
                printf("\r\n");
                printf("GSM_SendAT done\r\n");
            }
        }
#endif

        if (len > 0 && retFlag == USART_ENPROCESS)
        {
            if (ppRecvBuf != NULL)
            {
                *ppRecvBuf = pBackBuf;
            }

            if (pRecvLen != NULL)
            {
                *pRecvLen = len;
            }

            if (pCMDBack != NULL)
            {
                if (NULL
                        != strstr((const char *) pBackBuf,
                                  (const char *) pCMDBack))
                {
                    break;
                }
            }
            else {
                break;
            }
        }


        if (i == 0)
        {
            return USART_FAIL;
        }
    }
    return USART_SUCESS;
}
/*********************************************************************************************************
 ** Function name:       GSM_QueryNetType
 ** Descriptions:        查询运营商名称
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      运营商名称
 *********************************************************************************************************/
unsigned char GSM_QueryNetType(void)
{
    if (USART_SUCESS
            == GSM_SendAT((char *) AT_COPS, (char *) "MOBILE", sizeof(AT_COPS) - 1))
    {
        return CHINAMOBILE;
    }
    else if (USART_SUCESS
            == GSM_SendAT((char *) AT_COPS, (char *) "UNICOM", sizeof(AT_COPS) - 1))
    {
        return CHINAUNICOM;
    }
    else
    {
        return USART_FAIL;
    }
}
/*********************************************************************************************************
 ** Function name:       GSM_QuerySignal
 ** Descriptions:        查询信号强度
 ** input parameters:    NONE
 ** output parameters:   pSig: 信号强度
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_QuerySignal(unsigned char *pSig)
{
    unsigned int cmdLen;
    static char *pfeed = NULL;

    cmdLen = strlen(AT_CSQ);
    GSM_ClearBuffer();
    if (USART_SUCESS == GSM_SendAT((char *) AT_CSQ, (char *) AT_OK, cmdLen))
    {
        pfeed = strnchr(BackBuf, ',', 1);
        if (pfeed == NULL)
            return USART_FAIL;
        pfeed -= 2;
        if (pfeed[0] < 0x30 || pfeed[0] > 0x39)
        {
            *pSig = pfeed[1] - 0x30;
        }
        else
        {
            *pSig = (pfeed[0] - 0x30) * 10 + (pfeed[1] - 0x30);
        }
        return USART_SUCESS;
    }
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_SendSMS
 ** Descriptions:        发送一条短信
 ** input parameters:    pNumb: 接收短信的号码指针
 **											pSMS: 短信内容
 **											type: 短信发送格式 1-TEXT 0-PUD
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_SendSMS(char *pNumb, char *pSMS, unsigned char type)
{
    static char *pcmdbuf = NULL;
    static char smsBuffer[128];
    static char numbBuffer[15];
    static char smsLen[2];
    unsigned int cmdLen = 0;
    unsigned char i, j;

    GSM_ClearBuffer();
    pcmdbuf = sendBuf;
    if (pcmdbuf == NULL)
    {
        return USART_FAIL;
    }
    sprintf(pcmdbuf, AT_CMGF, type);
    cmdLen = strlen(pcmdbuf);
    GSM_SendAT(pcmdbuf, (char *) AT_OK, cmdLen);	 //设置短信模式

    if (1 == type)	//TEXT模式
    {
        strcpy(smsBuffer, pSMS);								//拷贝需要发送的短信内容
        GSM_ClearBuffer();
        pcmdbuf = sendBuf;
        if (pcmdbuf == NULL)
        {
            return USART_FAIL;
        }
        sprintf(pcmdbuf, AT_CMGS, pNumb);
        cmdLen = strlen(pcmdbuf);
        GSM_SendAT(pcmdbuf, ">", cmdLen);					  //设置目标号码

        delay_ms(500);
        strcat(smsBuffer, "\x1A");					 				//添加结束符号0x1A
        cmdLen = strlen(smsBuffer);
        GSM_ClearBuffer();
        usart_sendbuffer(STM32_SIM908_GSM_COM, smsBuffer,
                (unsigned int *) &cmdLen);
        delay_ms(2000);
        cmdLen = USART_GSM_BUFSIZE;
        usart_sendbuffer(STM32_SIM908_GSM_COM, BackBuf,
                (unsigned int*) &cmdLen);
        if (NULL != strstr((const char *) BackBuf, "OK"))
        {
            return USART_SUCESS;
        }
    }
    else if (0 == type)
    {
        i = 0;
        j = 0;
        do
        {
            numbBuffer[i++] = pNumb[j + 1];
            numbBuffer[i++] = pNumb[j];
            j = j + 2;
            if (pNumb[j + 1] == 0x0d || pNumb[j + 1] == 0)
            {
                numbBuffer[i++] = 'F';
                numbBuffer[i++] = pNumb[j];
                break;
            }
        } while (j < 11);
        strcpy(smsBuffer, "0011000D9168");
        strcat(smsBuffer, (const char *) &numbBuffer);
        strcat(smsBuffer, "0008A0");
        cmdLen = strlen(pSMS) / 2;
        sprintf(smsLen, "%X%X", cmdLen / 16, cmdLen % 16);
        strcat(smsBuffer, smsLen);
        strcat(smsBuffer, pSMS);

        GSM_ClearBuffer();
        pcmdbuf = sendBuf;
        if (pcmdbuf == NULL)
        {
            return USART_FAIL;
        }
        cmdLen = strlen(smsBuffer) / 2 - 1;
        sprintf(pcmdbuf, AT_CMGSPDU, cmdLen);
        cmdLen = strlen(pcmdbuf);
        GSM_SendAT(pcmdbuf, ">", cmdLen);					  //设置目标号码

        delay_ms(500);

        strcat(smsBuffer, "\x1A");
        cmdLen = strlen(smsBuffer);
        GSM_ClearBuffer();
        usart_sendbuffer(STM32_SIM908_GSM_COM, smsBuffer,
                (unsigned int *) &cmdLen);
        delay_ms(2000);
        cmdLen = USART_GSM_BUFSIZE;
        usart_sendbuffer(STM32_SIM908_GSM_COM, BackBuf,
                (unsigned int*) &cmdLen);
        if (NULL != strstr((const char *) BackBuf, "OK"))
        {
            return USART_SUCESS;
        }
    }
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_Init
 ** Descriptions:        GSM初始化
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      NONE
 *********************************************************************************************************/
void GSM_simcard_Init(void)
{
    uint32_t i;
    uint32_t len;

    //查询卡状态
    i = 0;
    len = sizeof(AT_CPIN) - 1;
    while (USART_SUCESS != GSM_SendAT((char *) AT_CPIN, (char *) "READY", len))
    {
        delay_ms(100);
        i++;
#if 0
        if (i > 2)
        {
            break;
        }
#endif
    }
    i = 0;
    GSMNetType = USART_FAIL;
    //查询网络运营商
    while (USART_FAIL == GSMNetType)
    {
        GSMNetType = GSM_QueryNetType();
#if 0
        if (i++ > 5)
            break;
#endif
    }
	
}

void GPS_Init(void)
{
    uint32_t len;

    //打开GPS
    //可根据需要设置GPS输出数据选择 如AT+CGPSOUT=67
    len = sizeof(AT_CGPSPWR);
    while (USART_SUCESS != GSM_SendAT((char *) AT_CGPSPWR, (char *) "OK", len));
    len = sizeof(AT_CGPSRST);
    while (USART_SUCESS != GSM_SendAT((char *) AT_CGPSRST, (char *) "OK", len));
}

void GSM_Init(void)
{
    unsigned char i = 0;
    unsigned char len;

    GSM_PowerCtrlInit();
    GSM_RingPinInit();
//	GSM_PowerOnOff();											

    // AT握手
    len = sizeof(AT_Cmd) - 1;

    while (USART_SUCESS != GSM_SendAT((char *) AT_Cmd, (char *) AT_OK, len))
    {
        delay_ms(300);
        i++;
        if (i > 2)
        {
            GSM_PowerOnOff();
            i = 0;
        }
    }
    printf("AT DONE\r\n");

    //关闭回显
    len = sizeof(ATE0_Cmd) - 1;
    GSM_SendAT((char *) ATE0_Cmd, (char *) AT_OK, sizeof(ATE0_Cmd));

    GSM_simcard_Init();
    //GPS_Init();
}
/*********************************************************************************************************
 ** Function name:       GSM_CallNumber
 ** Descriptions:        呼叫一个电话号码
 ** input parameters:    pNumber: 呼叫的电话号码指针
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_CallNumber(char *pNumber)
{
    static char *pcmdbuf = NULL;
    unsigned int cmdLen = 0;

    pcmdbuf = sendBuf;
    if (NULL == pcmdbuf)
    {
        return USART_FAIL;
    }
    sprintf(pcmdbuf, ATD_Cmd, pNumber);
    cmdLen = strlen(pcmdbuf);
    if (USART_SUCESS == GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, cmdLen))
    {
        return USART_SUCESS;
    }

    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_ReadNumberFromSIM
 ** Descriptions:        SIM卡中读取一个电话号码
 ** input parameters:    No: SIM卡中电话号码位置
 ** output parameters:   pNumber: 读取到的电话号码
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_ReadNumberFromSIM(unsigned char No, ST_PHONEBOOKS *pNumber)
{
    static char *pcmdbuf = NULL;
    static char *pfeed_per = NULL, *pfeed_next = NULL;
    unsigned int cmdLen = 0;

    if (No == 0 || No > 250)
    {
        return USART_FAIL;
    }
    pcmdbuf = sendBuf;
    if (NULL == pcmdbuf)
    {
        return USART_FAIL;
    }
    sprintf(pcmdbuf, AT_CPBR, No);
    cmdLen = strlen(pcmdbuf);
    GSM_ClearBuffer();
    if (USART_SUCESS == GSM_SendAT((char *) pcmdbuf, (char *) "+CPBR", cmdLen))
    {
        pfeed_per = strnchr(BackBuf, '"', 1);
        pfeed_next = strnchr(BackBuf, '"', 2);
        if ((NULL == pfeed_per) || (NULL == pfeed_next))
        {

            return USART_FAIL;
        }
        memcpy(&(pNumber->PhoneNumber), pfeed_per + 1,
                abs(pfeed_next - pfeed_per - 1));

        pfeed_per = strnchr(BackBuf, '"', 3);
        pfeed_next = strnchr(BackBuf, '"', 4);
        if ((NULL == pfeed_per) || (NULL == pfeed_next))
        {

            return USART_FAIL;
        }
        memcpy(&(pNumber->Name), pfeed_per + 1,
                abs(pfeed_next - pfeed_per - 1));

        return USART_SUCESS;
    }

    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_CallSIMNumber
 ** Descriptions:        呼叫SIM卡中的一个电话号码
 ** input parameters:    numberNo: SIM卡中电话号码位置
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_CallSIMNumber(unsigned char numberNo)
{
    static char *pcmdbuf = NULL;
    unsigned int cmdLen;

    if (numberNo > 250 || numberNo == 0)
    {
        return USART_FAIL;
    }
    pcmdbuf = sendBuf;
    if (NULL == pcmdbuf)
    {
        return USART_FAIL;
    }
    sprintf(pcmdbuf, ATD_n, numberNo);
    cmdLen = strlen(pcmdbuf);
    if (USART_SUCESS == GSM_SendAT(pcmdbuf, (char *) AT_OK, cmdLen))
    {

        return USART_SUCESS;
    }
    else
    {

        return USART_FAIL;
    }
}
/*********************************************************************************************************
 ** Function name:       GSM_QueryCallStatus
 ** Descriptions:        查询呼叫状态
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回呼叫状态
 *********************************************************************************************************/
unsigned char GSM_QueryCallStatus(void)
{
    unsigned int cmdLen;
    static char *pfeed = NULL;
    char status = AT_ACTIVE;

    cmdLen = strlen(AT_CLCC);
    GSM_ClearBuffer();
    if (USART_SUCESS == GSM_SendAT((char *) AT_CLCC, (char *) AT_OK, cmdLen))
    {
        pfeed = strnchr(BackBuf, ',', 2);
        if (pfeed == NULL)
            return USART_FAIL;
        if (NULL != pfeed)
        {
            status = *(pfeed + 1);
            return status;
        }
    }
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_HangCall
 ** Descriptions:        挂断通话
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_HangCall(void)
{
    unsigned int cmdLen;

    cmdLen = strlen(ATH_Cmd);
    if (USART_SUCESS == GSM_SendAT((char *) ATH_Cmd, (char *) AT_OK, cmdLen))
    {
        return USART_SUCESS;
    }
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_AnswerCall
 ** Descriptions:        接听通话
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_AnswerCall(void)
{
    unsigned int cmdLen;

    cmdLen = strlen(ATA_Cmd);
    if (USART_SUCESS == GSM_SendAT((char *) ATA_Cmd, (char *) AT_OK, cmdLen))
    {
        return USART_SUCESS;
    }
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_GetRTCTime
 ** Descriptions:        查询RTC时钟时间
 ** input parameters:    NONE
 ** output parameters:   rtctime: 返回时间信息
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_GetRTCTime(pST_RTCTIME rtctime)
{
    unsigned int cmdLen;
    static char *pfeed = NULL;

    cmdLen = strlen(AT_CCLK);
    GSM_ClearBuffer();
    if (USART_SUCESS == GSM_SendAT((char *) AT_CCLK, (char *) AT_OK, cmdLen))
    {
        pfeed = strnchr(BackBuf, ',', 1);
        if (pfeed == NULL)
            return USART_FAIL;
        pfeed -= 8;
        rtctime->Year = (pfeed[0] - 0x30) * 10 + (pfeed[1] - 0x30);
        rtctime->Month = (pfeed[3] - 0x30) * 10 + (pfeed[4] - 0x30);
        rtctime->Day = (pfeed[6] - 0x30) * 10 + (pfeed[7] - 0x30);
        rtctime->Hour = (pfeed[9] - 0x30) * 10 + (pfeed[10] - 0x30);
        rtctime->Min = (pfeed[12] - 0x30) * 10 + (pfeed[13] - 0x30);
        rtctime->Sec = (pfeed[15] - 0x30) * 10 + (pfeed[16] - 0x30);
        return USART_SUCESS;
    }
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_SetRTCTime
 ** Descriptions:        设置RTC时钟时间
 ** input parameters:    rtctime: 需要设置的时间信息
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_SetRTCTime(ST_RTCTIME rtctime)
{
    static char *pcmdbuf = NULL;
    unsigned int cmdLen = 0;

    GSM_ClearBuffer();
    pcmdbuf = sendBuf;
    if (pcmdbuf == NULL)
    {
        return USART_FAIL;
    }
    //格式化时间
    sprintf(pcmdbuf, AT_CCLKSet, (rtctime.Year) / 10, (rtctime.Year) % 10,
            (rtctime.Month) / 10, (rtctime.Month) % 10, (rtctime.Day) / 10,
            (rtctime.Day) % 10, (rtctime.Hour) / 10, (rtctime.Hour) % 10,
            (rtctime.Min) / 10, (rtctime.Min) % 10, (rtctime.Sec) / 10,
            (rtctime.Sec) % 10, (rtctime.Timezone) / 10,
            (rtctime.Timezone) % 10);
    cmdLen = strlen(pcmdbuf);
    if (USART_SUCESS == GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, cmdLen))
    {

        return USART_SUCESS;
    }

    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GPRS_Init
 ** Descriptions:      	GPRS初始化
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      NONE
 *********************************************************************************************************/
void GPRS_Init(void)
{
    unsigned int cmdLen;

    cmdLen = strlen(AT_CIPHEAD);
    GSM_SendAT((char *) AT_CIPHEAD, (char *) AT_OK, cmdLen);
    cmdLen = strlen(AT_CIPQSEND);
    GSM_SendAT((char *) AT_CIPQSEND, (char *) AT_OK, cmdLen);
    cmdLen = strlen(AT_CIPSPRT);
    GSM_SendAT((char *) AT_CIPSPRT, (char *) AT_OK, cmdLen);
    cmdLen = strlen(AT_CSTT);
    GSM_SendAT((char *) AT_CSTT, (char *) AT_OK, cmdLen);
    cmdLen = strlen(AT_CIICR);
    GSM_SendAT((char *) AT_CIICR, (char *) AT_OK, cmdLen);
    cmdLen = strlen(AT_CIPMODE);
    GSM_SendAT((char *) AT_CIPMODE, (char *) AT_OK, cmdLen);
}
/*********************************************************************************************************
 ** Function name:       GPRS_LinkServer
 ** Descriptions:      	建立一个链接
 ** input parameters:    pnetconfig: 网络配置信息
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
/*
 char RemoteIP[4];
 unsigned int RemotePort;
 char TransferMode[4];
 char APN[10];
 */
unsigned char GPRS_LinkServer(pST_NETWORKCONFIG pnetconfig)
{
    ST_NETWORKCONFIG netcfg = *pnetconfig;
    static char *pcmdbuf = NULL;
    unsigned int cmdLen = 0;

    pcmdbuf = sendBuf;
    if (pcmdbuf == NULL)
    {
        return USART_FAIL;
    }
    sprintf(pcmdbuf, AT_CIPSTART, netcfg.TransferMode, netcfg.RemoteIP,
            netcfg.RemotePort);
    cmdLen = strlen(pcmdbuf);
    if (USART_SUCESS == GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, cmdLen))
    {
        return USART_SUCESS;
    }

    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GPRS_SendData
 ** Descriptions:      	GPRS非透传模式下发送数据
 ** input parameters:    pString: 需要发送的数据指针
 **											len:	需要发送的数据长度
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
#define DBG_GPRS_SendData
unsigned char GPRS_SendData(char *pString, unsigned int len)
{
	char *pBackBuf = BackBuf;
    unsigned int cmdLen;

    cmdLen = strlen(AT_CIPSEND);
    if (USART_SUCESS == GSM_SendAT((char *) AT_CIPSEND, (char *) '>', cmdLen))
    {
        cmdLen = sizeof(pString);
        usart_sendbuffer(STM32_SIM908_GSM_COM, pString, &len);
        cmdLen = 1;
        usart_sendbuffer(STM32_SIM908_GSM_COM, "\x1A", &cmdLen);
        //return USART_SUCESS;
    }

    {
        uint32_t i = 5;
        uint32_t len;
        unsigned char retFlag;

        while (--i)
        {
            len = USART_GSM_BUFSIZE;
            delay_ms(200);

            //printf("GSM_SendAT before usart_readbuffer len %d\n", len);

            retFlag =
                    usart_readbuffer(STM32_SIM908_GSM_COM, BackBuf, &len);
            //printf("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_GPRS_SendData
            {
                uint32_t tmpIdx;
				uint32_t tmpStart;

                if (len > 0 && retFlag == USART_ENPROCESS)
                {
                    printf("GPRS_SendData recv\r\n");
                    for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                    {
                        //printf("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                        printf("%c",BackBuf[tmpIdx]);
						
                    }
                    printf("\r\n");
                    printf("GPRS_SendData done\r\n");
                }
            }
			printf("%d retFlag = %d len = %d\n\n", i, retFlag, len);

			if (len > 0 && retFlag == USART_ENPROCESS)
            {
                if (NULL != strstr_len(pBackBuf, pString, len))
                {
                    break;
                }
            }
#endif
        }
        if (i == 0)
        {
            return USART_FAIL;
        }
    }
    return USART_SUCESS;
}
/*********************************************************************************************************
 ** Function name:       GPRS_ReceiveData
 ** Descriptions:      	GPRS接收数据,SIM908直接串口接收,不需要发送指令读取
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
#define DBG_GPRS_ReceiveData
unsigned char GPRS_ReceiveData(char *pString)
{
	uint32_t i = 5;
    uint32_t len;
    unsigned char retFlag;
	char *pReceiveBuf = receiveBuf;
	
    while (--i)
    {
        len = USART_GSM_BUFSIZE_RECEIVE;
        delay_ms(200);

        //printf("GSM_SendAT before usart_readbuffer len %d\n", len);

        retFlag =
                usart_readbuffer(STM32_SIM908_GSM_COM, receiveBuf, &len);
        //printf("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_GPRS_ReceiveData
        {
            uint32_t tmpIdx;
			uint32_t tmpStart;

            if (len > 0 && retFlag == USART_ENPROCESS)
            {
                printf("GPRS_ReceiveData recv\r\n");
                for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                {
                    //printf("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                    printf("%c",receiveBuf[tmpIdx]);
					
                }
                printf("\r\n");
                printf("GPRS_ReceiveData done\r\n");
            }
        }
		printf("%d retFlag = %d len = %d\n\n", i, retFlag, len);

		if (len > 0 && retFlag == USART_ENPROCESS)
        {
            if (NULL != strstr_len(pReceiveBuf, pString, len))
            {
				printf("Receive New Message.\n");
                break;
            }
        }
        
#endif
    }

    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GPRS_CheckLinkStatus
 ** Descriptions:      	查询GPRS链接状态
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回GPRS链接状态结果
 *********************************************************************************************************/
unsigned char GPRS_CheckLinkStatus(void)
{
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GPRS_CLoseLink
 ** Descriptions:      	关闭GPRS链接
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GPRS_CloseLink(void)
{
    unsigned int cmdLen;

    cmdLen = strlen(AT_CIPCLOSE);
    if (USART_SUCESS
            == GSM_SendAT((char *) AT_CIPCLOSE, (char *) AT_OK, cmdLen))
    {
        return USART_SUCESS;
    }
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GSM_QueryBattery
 ** Descriptions:      	查询电池状态
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_QueryBattery(pST_BATTERYSTATUS pSig)
{
    unsigned int cmdLen;
    static char *pfeed = NULL;

    cmdLen = strlen(AT_CBC);
    GSM_ClearBuffer();
    if (USART_SUCESS == GSM_SendAT((char *) AT_CBC, (char *) AT_OK, cmdLen))
    {
        pfeed = strnchr(BackBuf, ',', 1);	//提取充电状态
        if (pfeed == NULL)
            return USART_FAIL;
        pfeed -= 1;
        pSig->BatStatus = pfeed[0];

        pfeed = strnchr(BackBuf, ',', 2);	//提取电池电量信息
        if (pfeed == NULL)
            return USART_FAIL;
        pfeed -= 3;
        if (pfeed[0] >= 0x30 && pfeed[0] <= 0x39)
        {
            pSig->BatPower = (pfeed[0] - 0x30) * 100 + (pfeed[1] - 0x30) * 10
                    + (pfeed[2] - 0x30);
        }
        else
        {
            pSig->BatPower = (pfeed[1] - 0x30) * 10 + (pfeed[2] - 0x30);
        }
        return USART_SUCESS;
    }
    return USART_FAIL;
}

unsigned char GSM_creg(void)
{
    static char *pcmdbuf = NULL;
    unsigned int cmdLen = 0;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    char *pStr;

    pcmdbuf = sendBuf;
    sprintf(pcmdbuf, AT_CREG_SET, 2);
    cmdLen = strlen(pcmdbuf);
    while (USART_SUCESS != GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, cmdLen));

    while (1)
    {
        while (USART_SUCESS != GSM_SendAT_rsp((char *)AT_CREG, (char *) "CREG",
                sizeof(AT_CREG), &pRecvBuf, &recvLen));

        pStr = strnchr_len(pRecvBuf, ':', 1, recvLen);
        if (pStr != NULL)
        {
            do {
                pStr ++;
            } while (*pStr == ' ');

            if (*pStr == '2')
            {
                break;
            }
        }
    }

    // analyze AT_CREG rsp


    return USART_SUCESS;
}

unsigned char GSM_cgatt(void)
{
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    char *pStr;

    while (1)
    {
        while (USART_SUCESS != GSM_SendAT_rsp((char *) AT_CGATT, (char *) "CGATT",
                sizeof(AT_CGATT), &pRecvBuf, &recvLen));

        // analyze CGATT rsp
        pStr = strnchr_len(pRecvBuf, ':', 1, recvLen);
        if (pStr != NULL)
        {
            do {
                pStr ++;
            } while (*pStr == ' ');

            if (*pStr == '1')
            {
                break;
            }
        }
    }

    return USART_SUCESS;
}

unsigned char GSM_cstt(void)
{
    static char *pcmdbuf = NULL;
    unsigned int cmdLen = 0;

    pcmdbuf = sendBuf;
    sprintf(pcmdbuf, AT_CSTT_SET, "\"CMNET\"");
    cmdLen = strlen(pcmdbuf);
    while (USART_SUCESS != GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, cmdLen));

    return USART_SUCESS;
}

unsigned char GSM_ciicr(void)
{
    while (USART_SUCESS != GSM_SendAT((char *) AT_CIICR, (char *) AT_OK, sizeof(AT_CIICR)));

    return USART_SUCESS;
}

unsigned char GSM_cifsr(void)
{
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    while (USART_SUCESS != GSM_SendAT_rsp((char *) AT_CIFSR, NULL,
            sizeof(AT_CIFSR), &pRecvBuf, &recvLen));

    // analyze CGATT rsp


    return USART_SUCESS;
}

unsigned char GSM_cgpsstatus(void)
{
    //char *pRecvBuf = NULL;

    while (USART_SUCESS != GSM_SendAT((char *) AT_CGPSSTATUS,
            (char *) NULL/*AT_OK*/, sizeof(AT_CGPSSTATUS)));

    // analyze CGATT rsp


    return USART_SUCESS;
}

char str[30] = "123456789012345678901234567890";
void GSM_str_test(void)
{
    char *retPtr;

    retPtr = strstr_len(str, "2", 1);
    printf("GSM_str_test %d, %d, %d\n", retPtr-str, retPtr, str);
    retPtr = strstr_len(str, "2", 30);
    printf("GSM_str_test %d\n", retPtr-str);
    retPtr = strstr_len(str, "23456", 30);
    printf("GSM_str_test %d\n", retPtr-str);
    retPtr = strstr_len(str, str, 30);
    printf("GSM_str_test %d\n", retPtr-str);


    retPtr = strnchr_len(str, '1', 1, 30);
    printf("GSM_str_test %d\n", retPtr-str);
    retPtr = strnchr_len(str, '1', 2, 30);
    printf("GSM_str_test %d\n", retPtr-str);
    retPtr = strnchr_len(str, '1', 3, 30);
    printf("GSM_str_test %d\n", retPtr-str);
    retPtr = strnchr_len(str, '1', 4, 30);
    printf("GSM_str_test %d\n", retPtr-str);
}

s8 Test_TelNumber[12]="15306568880\0";
void GSM_test_once(void)
{
	unsigned int cmdLen;
	char str[24];
    ST_NETWORKCONFIG stNetCfg;
//[67 67 01  00 0C 00 01 01 23  45 67 89 01 23 45 00 20]
	str[0] = 0x67;
	str[1] = 0x67;
	str[2] = 0x01;
	str[3] = 0x00;
	str[4] = 0x0C;
	str[5] = 0x00;
	str[6] = 0x01;
	str[7] = 0x01;
	str[8] = 0x23;
	str[9] = 0x45;
	str[10] = 0x67;
	str[11] = 0x89;
	str[12] = 0x01;
	str[13] = 0x23;
	str[14] = 0x45;
	str[15] = 0x00;
	str[16] = 0x20;

	  sprintf(stNetCfg.TransferMode, "%s", "TCP");
    sprintf(stNetCfg.RemoteIP, "%s", "121.40.200.84");
    sprintf(stNetCfg.RemotePort, "6666");
    //while (USART_SUCESS != GSM_CallNumber((char*)Test_TelNumber));

	cmdLen = strlen(AT_CIPMODE_0);
    GSM_SendAT((char *) AT_CIPMODE_0, (char *) AT_OK, cmdLen);
	
    GSM_creg();
    GSM_cgatt();
    GSM_cstt();
    GSM_ciicr();
    GSM_cifsr();

#if 1
    while (USART_SUCESS != GPRS_LinkServer(&stNetCfg));

	while(1)
	{
    	if(USART_FAIL == GPRS_SendData(str, 17))
    	{
			printf("GPRS_SendData Fail\n");
    	}
		delay_ms(10000);
	}
#endif
}

void GSM_test(void)
{
    GSM_cgpsstatus();
}

void GSM_tcpip_test(void)
{

}

