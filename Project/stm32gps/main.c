/**
 ******************************************************************************
 * @file    USART/Printf/main.c
 * @author  MCD Application Team
 * @version V3.5.0
 * @date    08-April-2011
 * @brief   Main program body
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
#include "stm32f10x.h"
#include "stm32f10x_it_api.h"
#include "stm32gps_board.h"
#include "stm32gps_config.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "GSM_App.h"
#include "GPS_App.h"
#include "eelink.h"
#include "gpio.h"

//#define USE_DEBUG

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
	unsigned char status[2];    // dveice status
	unsigned char analog1[2]; 	// analog input 1
	unsigned char analog2[2];		// analog input 2
}ST_DEVICEDATA, *pST_DEVICEDATA;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#undef TEST_MACRO
#define BKP_DR_NUMBER              42
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LsiFreq = 40000;

// sim and gps information
uint8_t imeiBuf[IMEI_INFO_LEN];  // imei information buffer

// packet message
EELINK_SIM_PACKET_LOGIN loginMsg;
char loginBuf[PROTO_LOGIN_BUF];
EELINK_PACKET_GPS gpsMsg;
char gpsBuf[PROTO_GPS_BUF];

ST_SIMDATA g_simData;
ST_GPSDATA g_gpsData;
ST_DEVICEDATA g_deviceData;
ST_IMSIINFO g_imsiInfo;

uint16_t g_sequenceNum;  // packet's sequence
uint16_t g_successNum;

#if 1

uint16_t BKPDataReg[BKP_DR_NUMBER] =
  {
    BKP_DR1, BKP_DR2, BKP_DR3, BKP_DR4, BKP_DR5, BKP_DR6, BKP_DR7, BKP_DR8,
    BKP_DR9, BKP_DR10, BKP_DR11, BKP_DR12, BKP_DR13, BKP_DR14, BKP_DR15, BKP_DR16,
    BKP_DR17, BKP_DR18, BKP_DR19, BKP_DR20, BKP_DR21, BKP_DR22, BKP_DR23, BKP_DR24,
    BKP_DR25, BKP_DR26, BKP_DR27, BKP_DR28, BKP_DR29, BKP_DR30, BKP_DR31, BKP_DR32,
    BKP_DR33, BKP_DR34, BKP_DR35, BKP_DR36, BKP_DR37, BKP_DR38, BKP_DR39, BKP_DR40,
    BKP_DR41, BKP_DR42
  }; 
#endif

/* Private function prototypes -----------------------------------------------*/
uint8_t ProcessIMEI(uint8_t *pImei, uint8_t *pRst, int32_t imeilen, int32_t rstlen);
void loadLoginMsg(uint8_t *imei, uint16_t sequence);
void PackLoginMsg(void);
void LoadGpsMsg(uint16_t sequence);
void PackGpsMsg(void);
void PackAlarmMsg(void);
/* Private functions ---------------------------------------------------------*/


void ShowLoginMsg(void)
{
	uint32_t i;
	// imei
	printf("IMEI:");
	for(i = 0; i < IMEI_INFO_LEN; i++)
	{
		printf("%c-", imeiBuf[i]);
	}
	printf("\r\n");
	// login msg
	printf("LOGIN MSG:");
	for(i = 0; i < PROTO_LOGIN_BUF; i++)
	{
		printf("0x%x-", loginBuf[i]);
	}
	printf("\r\n");
}

void ShowGpsMsg(void)
{
	uint32_t i;
	// gps msg
	printf("GPS MSG:");
	for(i = 0; i < PROTO_GPS_BUF; i++)
	{
		printf("0x%x-", gpsBuf[i]);
	}
	printf("\r\n");
}

/*********************************************************************************************************
 ** Function name:       InitVariables()
 ** Descriptions:        初始化全局变量
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      NONE
 *********************************************************************************************************/
void InitVariables(void)
{
	uint32_t i;
	
	g_sequenceNum = 1;
	g_successNum = 0;
	
	memset(imeiBuf, sizeof(imeiBuf), 0);
	for(i = 0; i < PROTO_LOGIN_BUF; i++)
	{
		loginBuf[i] = 0;
	}
	for(i = 0; i < PROTO_GPS_BUF; i++)
	{
		gpsBuf[i] = 0;
	}

	memset(&loginMsg, sizeof(loginMsg), 0);
	memset(&gpsMsg, sizeof(gpsMsg), 0);

	memset(&g_simData, sizeof(g_simData), 0);
	memset(&g_gpsData, sizeof(g_gpsData), 0);
	memset(&g_deviceData, sizeof(g_deviceData), 0);
	
}

uint8_t getRemovalFlag(void)
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) 
{
//	int i = 0;
	uint8_t gpsRecvTimes = 0;
	uint8_t rst = GPS_FAIL;
    ST_GPSRMCINFO rmc;
	unsigned char errNum = 0;
	uint16_t removeNum = 0;
	uint16_t sendLen = 0;

	// Used for get GPRS Data
	char *pRecvBuf = NULL;
    uint32_t recvLen = 0;
	char *pfeed = NULL;

	/////////////////////////////////////////////////////////////////
	// Configure the GPIO ports and Power OFF GPS and GSM
	/////////////////////////////////////////////////////////////////
	MX_GPIO_Init();
#ifdef USE_STM32_GPS_BOARD_VB
	GPSPowerOff();
	GSM_PowerOff();
#endif
	/////////////////////////////////////////////////////////////////
	// Configure the SysTick
	/////////////////////////////////////////////////////////////////
	stm32gps_sys_tick_cfg();
	/////////////////////////////////////////////////////////////////
	// Configure PWR and BKP
	/////////////////////////////////////////////////////////////////
	/* Enable PWR and BKP clock */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  	/* Enable WKUP pin */
  	PWR_WakeUpPinCmd(ENABLE);
  	/* Allow access to BKP Domain */
  	PWR_BackupAccessCmd(ENABLE);
	/////////////////////////////////////////////////////////////////
	// Configure RCC
	/////////////////////////////////////////////////////////////////
	RCC_Configuration();
	/////////////////////////////////////////////////////////////////
	// Configure EXTI
	/////////////////////////////////////////////////////////////////
	EXTI_Configuration();
	/////////////////////////////////////////////////////////////////
	// Configure RTC
	/////////////////////////////////////////////////////////////////
	RTC_Configuration();
	RTC_NVIC_Configuration();
	//IWDG_Configuration();
	/////////////////////////////////////////////////////////////////
	// Configure TIMER
	/////////////////////////////////////////////////////////////////
	TIM2_Configuration();
	TIM2_NVIC_Configuration();

	/////////////////////////////////////////////////////////////////
	// Init BKP Register when PowerOn Reset
	/////////////////////////////////////////////////////////////////
#if 1
	/* Check if the Power On Reset flag is set */
	if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
	{
		/* Clear reset flags */
		RCC_ClearFlag();
		// Write BKP register
		BKP_WriteBackupRegister(BKPDataReg[BKP_SLEEP_TIME], (uint16_t)(SLEEP_NORMAL_SEC));
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_FLAG], (uint16_t)(BKP_FALSE));
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_YES], 0);
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_NOT], 0);
	}
#endif

	/////////////////////////////////////////////////////////////////
	// Configure LED and USART(GPS + GSM + DEBUG)
	/////////////////////////////////////////////////////////////////
    stm32gps_led_cfg();
	STM_EVAL_LEDOff(LED1);

    stm32gps_com_debug_cfg();

    usart_init(STM32_SIM908_GPS_COM);
    stm32gps_com_gps_cfg();

    usart_init(STM32_SIM908_GSM_COM);
    stm32gps_com_gsm_cfg();

	/////////////////////////////////////////////////////////////////
	// Tuen On TIMER
	/////////////////////////////////////////////////////////////////
	TIM2_Start();
	
	/////////////////////////////////////////////////////////////////
	// Init Variables
	/////////////////////////////////////////////////////////////////
	InitVariables();
	
	/////////////////////////////////////////////////////////////////
	// Check Remove Action
	/////////////////////////////////////////////////////////////////
#if 1
	// set remove flag
	if(BKP_FALSE == BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_FLAG]) 
		&& (BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_NOT]) > CHECK_REMOVE_TIMES))
	{
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_FLAG], (uint16_t)(BKP_TRUE));
		printf("setting bkp_flag true\n");
	}

	// reset remove flag
	if(BKP_TRUE == BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_FLAG]) 
		&& (BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_YES]) > CHECK_REMOVE_TIMES))
	{
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_FLAG], (uint16_t)(BKP_FALSE));
		printf("setting bkp_flag false\n");
	}
	
	// have removed
	if((uint8_t)Bit_SET == getRemovalFlag())
	{
		removeNum = BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_YES]);
		removeNum++;
		printf("YES removalnum = %d\n", removeNum);
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_YES], removeNum);
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_NOT], 0);
	}
	
	// not removed
	if((uint8_t)Bit_RESET == getRemovalFlag()) 
	{
		removeNum= BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_NOT]);
		
		removeNum++;
		printf("NOT removalnum = %d\n", removeNum);
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_NOT], removeNum);
		BKP_WriteBackupRegister(BKPDataReg[BKP_REMOVE_YES], 0);
	}
	
#endif


	/////////////////////////////////////////////////////////////////
	// First Power ON GPS
	/////////////////////////////////////////////////////////////////
	GPSPowerOn();
	while(1)
	{
		// delay 1sec
		delay_10ms(100);
		/////////////////////////////////////////////////////////////////
		// Receive GPS Data and Analyze
		/////////////////////////////////////////////////////////////////
		rst = GPSInfoAnalyze(&rmc);
		if( GPS_SUCCESS == rst)
		{
			STM_EVAL_LEDOff(LED1);
			DEBUG("GPS Recv Success!\n");
			break;
		}
		else if(GPS_INVALID == rst)
		{
			STM_EVAL_LEDOn(LED1);
			DEBUG("GPS Recv Invalid\n");
		}
		else
		{
			STM_EVAL_LEDOn(LED1);
			DEBUG("GPS Recv Error\n");
		}
		/////////////////////////////////////////////////////////////////
		// If GPS Receive Times Over then break, ~30sec
		/////////////////////////////////////////////////////////////////
		gpsRecvTimes++;
		if(gpsRecvTimes > GPS_RETERY_TIMES)
		{
			gpsRecvTimes = 0;
			break;
		}

		/////////////////////////////////////////////////////////////////
		// Set RTC Alarm to wake from STOP mode
		/////////////////////////////////////////////////////////////////
		/* Wait till RTC Second event occurs */
	    RTC_ClearFlag(RTC_FLAG_SEC);
	    while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

	    /* Alarm in 10 second */
	    RTC_SetAlarm(RTC_GetCounter()+ STOP_GPS_SEC);
	    /* Wait until last write operation on RTC registers has finished */
	    RTC_WaitForLastTask();
		
		/////////////////////////////////////////////////////////////////
		// Go Into STOP Mode
		/////////////////////////////////////////////////////////////////
		/* Request to enter STOP mode with regulator in low power mode*/
    	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

		/* Configures system clock after wake-up from STOP: enable HSE, PLL and select 
       		PLL as system clock source (HSE and PLL are disabled in STOP mode) */
    	SYSCLKConfig_STOP();
	}
	
#ifndef MACRO_FOR_TEST
	/////////////////////////////////////////////////////////////////
	// Second Power ON GSM
	/////////////////////////////////////////////////////////////////
	GSM_PowerOn();
	while(1)
	{
		// reset sequence to 1
		g_sequenceNum = 1;
		g_successNum = 0;
		errNum = 0;
		/////////////////////////////////////////////////////////////////
		// While loop to handshake with SIM800L module
		/////////////////////////////////////////////////////////////////
		GSM_Init();
		/////////////////////////////////////////////////////////////////
		// While loop to check sim card status
		/////////////////////////////////////////////////////////////////
		GSM_CheckSIMCard();
#ifdef USE_DEBUG
		//GetGsmData(&g_simData, g_imsiInfo);
		GSM_test();
#endif
//GSM_QueryNumber();
		/////////////////////////////////////////////////////////////////
		// While loop to check network registration status
		/////////////////////////////////////////////////////////////////
		GSM_CheckNetworkReg();
		/////////////////////////////////////////////////////////////////
		// While loop to check Attach or Detach from GPRS Service
		/////////////////////////////////////////////////////////////////
		GSM_CheckGPRSService();
		/////////////////////////////////////////////////////////////////
		// While loop to set network registration 
		// 2 Enable network registration unsolicited result code with
		// location information +CREG: <stat>[,<lac>,<ci>]
		/////////////////////////////////////////////////////////////////
		GSM_SetNetworkReg();
		/////////////////////////////////////////////////////////////////
		// While loop to Set CIPMODE
		/////////////////////////////////////////////////////////////////
		GSM_SetCIPMode(0);
		/////////////////////////////////////////////////////////////////
		// (While loop) to Start task and Set APN "CMNET"
		/////////////////////////////////////////////////////////////////
		GSM_StartTaskAndSetAPN();
		/////////////////////////////////////////////////////////////////
		// While loop to Bring Up Wireless Connection
		/////////////////////////////////////////////////////////////////
		GSM_BringUpConnect();
		/////////////////////////////////////////////////////////////////
		// While loop to Get Local IP Address
		/////////////////////////////////////////////////////////////////
		GSM_GetLocalIP();
		/////////////////////////////////////////////////////////////////
		// While loop to Start Up TCP or UDP Connection
		/////////////////////////////////////////////////////////////////
		GSM_StartUpConnect();

		/////////////////////////////////////////////////////////////////
		// Query SIM IMEI and Package LOGIN Message
		/////////////////////////////////////////////////////////////////
		while(USART_SUCESS != GSM_QueryImei(imeiBuf));
		loadLoginMsg(imeiBuf, g_sequenceNum);
		PackLoginMsg();

		/////////////////////////////////////////////////////////////////
		// While loop to Send LOGIN Message
		/////////////////////////////////////////////////////////////////
		while(USART_SUCESS != GPRS_SendData_rsp(loginBuf, EELINK_LOGIN_MSGLEN, &pRecvBuf, &recvLen))
		{
			STM_EVAL_LEDToggle(LED1);
			printf("GPRS_SendData LOGIN MSG Fail\n");
		}
#if 0
		// parse response data
		pfeed = strstr_len(pRecvBuf, "gg", recvLen);
		if(pfeed != NULL)
		{
			printf("\r\n 2= 0x%x-", *(pfeed+2));
		}
		printf("\r\n");
#endif
		/////////////////////////////////////////////////////////////////
		// Query SIM IMSI and Analyze
		/////////////////////////////////////////////////////////////////
		while(USART_SUCESS != GSM_QueryImsi(&g_imsiInfo));

		/////////////////////////////////////////////////////////////////
		// While loop to Send GPS Message
		/////////////////////////////////////////////////////////////////
	    while(1)
	    {
			/////////////////////////////////////////////////////////////////
			// Receive GPS Data and Analyze
			/////////////////////////////////////////////////////////////////
			rst = GPSInfoAnalyze(&rmc);
			if( GPS_SUCCESS == rst)
			{
				printf("GPS Recv Success!\n");
			}
			else if(GPS_INVALID == rst)
			{
				printf("GPS Recv Invalid\n");
			}
			else
			{
				memset(&rmc, sizeof(rmc), 0);
				printf("GPS Recv Error\n");
			}
			
			ParseGPSInfo(rmc, &g_gpsData);
			/////////////////////////////////////////////////////////////////
			// Get GSM related Data and Analyze, Package GPS Message
			/////////////////////////////////////////////////////////////////
			GetGsmData(&g_simData, g_imsiInfo);
			LoadGpsMsg(g_sequenceNum);

			// remove alarm
			if((BKP_TRUE == BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_FLAG]))
				&&((BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_YES]) > 0)))
			{
				PackAlarmMsg();
				sendLen = EELINK_ALARM_MSGLEN;
				printf("send alarmmsg\n");
			}
			else
			{
				PackGpsMsg();
				sendLen = EELINK_GPS_MSGLEN;
				printf("send gpsmsg\n");
			}

			/////////////////////////////////////////////////////////////////
			// Send GPS Message, 
			/////////////////////////////////////////////////////////////////
			if(USART_FAIL == GPRS_SendData(gpsBuf, sendLen))
			{
				STM_EVAL_LEDToggle(LED1);
				errNum++;
				if(errNum > GSM_RETERY_TIMES)
				{
					errNum = 0;
					GPRS_CloseLink();
					GPRS_CIPShut();
					break;
				}
				printf("GPRS_SendData Fail\n");
			}
			else
			{

				// Increase success number
				g_successNum++;
			}
			
#ifndef MACRO_FOR_TEST
			// if send ok then into sleep
			if(g_successNum > GSM_SUCCESS_TIMES)
			{
				break;
			}
#endif
#ifdef DBG_ENABLE_MACRO
		ShowGpsMsg();
#endif

			// Increase sequence number if overflow then re-init
			g_sequenceNum++;
			if(g_sequenceNum == 0)
			{
				g_sequenceNum = 1;
			}

			// delay 1 second
			delay_10ms(100);
	    }
		
		// if send ok then into sleep
		if(g_successNum > GSM_SUCCESS_TIMES)
		{
			break;
		}
	}
#endif
	/////////////////////////////////////////////////////////////////
	// This Process is Finished, Then goto sleep
	/////////////////////////////////////////////////////////////////
	DEBUG("this being in normal standby mode\n");
#ifndef MACRO_FOR_TEST
	//delay_10ms(100);
	/////////////////////////////////////////////////////////////////
	// Power OFF GPS and GSM before go into standby mode
	/////////////////////////////////////////////////////////////////
	GSM_TurnOnOff();
#ifdef USE_STM32_GPS_BOARD_VB
	GPSPowerOff();
	GSM_PowerOff();
#endif
	/* Wait till RTC Second event occurs */
	RTC_ClearFlag(RTC_FLAG_SEC);
	while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

	// normal working state
	if(BKP_TRUE == BKP_ReadBackupRegister(BKPDataReg[BKP_REMOVE_FLAG]))
	{
printf("normal working state\n");
		/* Set the RTC Alarm after xx s */
		RTC_SetAlarm(RTC_GetCounter()+ BKP_ReadBackupRegister(BKPDataReg[BKP_SLEEP_TIME]));
	}
	else
	{
printf("sleep working state\n");
		RTC_SetAlarm(RTC_GetCounter()+ SLEEP_NORMAL_SEC);
	}
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
	PWR_EnterSTANDBYMode();
#endif

	/////////////////////////////////////////////////////////////////
	// Should NOT run to here
	/////////////////////////////////////////////////////////////////
	while(1)
	{
//#ifndef MACRO_FOR_TEST
		STM_EVAL_LEDToggle(LED1);
//#endif
		delay_10ms(100);
		//delay_ms(1000);
#ifdef USE_DEBUG
		if( GPS_SUCCESS != GPSInfoAnalyze(&rmc))
		{
			printf("GPS Recv Error!\n");
		}
		else
		{
			STM_EVAL_LEDToggle(LED1);
		}
#endif
	}

}

/*********************************************************************************************************
 ** Function name:       ProcessIMEI()
 ** Descriptions:        将15个字节的IMEI字符串处理成8个字节的EELINK协议格式
 ** input parameters:    pImei 需要处理的15个字节的IMEI字符串
 **                      pRst 返回的字符串指针
 **                      imeilen IMEI字符串的长度
 **                      rstlen 返回的字符串的长度
 ** output parameters:   
 ** Returned value:      成功RST_OK 失败RST_FAIL
 *********************************************************************************************************/
uint8_t ProcessIMEI(uint8_t *pImei, uint8_t *pRst, int32_t imeilen, int32_t rstlen)
{
	uint32_t i;
	if(((imeilen/2 +1) < rstlen) || (imeilen != 15))
	{
		return RST_FAIL;
	}
	
	*(pRst+0) = (*(pImei+0))&0x0F;
	for(i = (imeilen/2); i > 0 ; i--)
	{
		*(pRst+i) = (((*(pImei+i*2-1)) << 4)&0xF0) + ((*(pImei+i*2)) &0x0F);
	}
	return RST_OK;
}

void loadLoginMsg(uint8_t *imei, uint16_t sequence)
{
	loginMsg.hdr.header[0] = PROTO_EELINK_HEADER;
	loginMsg.hdr.header[1] = PROTO_EELINK_HEADER;
	loginMsg.hdr.type = PACKET_EELINK_LOGIN;
	loginMsg.hdr.len[0] = 0x00;
	loginMsg.hdr.len[1] = 0x0C;
	loginMsg.hdr.seq[0] = (uint8_t)((sequence >> 8)& 0x00FF);
	loginMsg.hdr.seq[1] = (uint8_t)((sequence)& 0x00FF);
	if(RST_FAIL == ProcessIMEI(imei, loginMsg.imei, IMEI_INFO_LEN, 8))
	{
		// re-init imei buffer
		memset(loginMsg.imei, sizeof(loginMsg.imei), 0);
		printf("IMEI process ERROR!\n");
	}
	loginMsg.lang = EELINK_LANG; // english
	loginMsg.zone = EELINK_ZONE; // east 8
}

void PackLoginMsg(void)
{
	uint32_t i;
	uint32_t offset = 0;
	offset = 0;
	for(i = 0; i < 2; i++)
	{
		loginBuf[offset] = loginMsg.hdr.header[i];
		offset++;
	}
	loginBuf[offset] = loginMsg.hdr.type;
	offset++;
	for(i = 0; i < 2; i++)
	{
		loginBuf[offset] = loginMsg.hdr.len[i];
		offset++;
	}
	for(i = 0; i < 2; i++)
	{
		loginBuf[offset] = loginMsg.hdr.seq[i];
		offset++;
	}
	for(i = 0; i < 8; i++)
	{
		loginBuf[offset] = loginMsg.imei[i];
		offset++;
	}
	loginBuf[offset] = loginMsg.lang;
	offset++;
	loginBuf[offset] = loginMsg.zone;
	offset++;
	if(offset != (EELINK_LOGIN_MSGLEN))
	{
		printf("PackLoginMsg ERROR!\n");
	}
}

void LoadGpsMsg(uint16_t sequence)
{
	uint32_t i;
	gpsMsg.hdr.header[0] = PROTO_EELINK_HEADER;
	gpsMsg.hdr.header[1] = PROTO_EELINK_HEADER;
	gpsMsg.hdr.type = PACKET_EELINK_GPS;
	gpsMsg.hdr.len[0] = 0x00;
	gpsMsg.hdr.len[1] = 0x25;  // 37
	gpsMsg.hdr.seq[0] = (uint8_t)((sequence >> 8)& 0x00FF);
	gpsMsg.hdr.seq[1] = (uint8_t)((sequence)& 0x00FF);

	for(i = 0; i < 4; i++)
	{
		gpsMsg.utctime[i] = g_gpsData.utc.s[3 - i];
	}
	
	for(i = 0; i < 4; i++)
	{
		gpsMsg.lati[i] = g_gpsData.latitude.s[3 - i];
	}

	for(i = 0; i < 4; i++)
	{
		gpsMsg.longi[i] = g_gpsData.longitude.s[3 - i];
	}

	gpsMsg.speed = g_gpsData.speed;
	
	for(i = 0; i < 2; i++)
	{
		gpsMsg.course[i] = g_gpsData.course.s[1 - i];
	}

	for(i = 0; i < 9; i++)
	{
		gpsMsg.basestation[i] = g_simData.Station[i];
	}

	gpsMsg.gps_valid = g_gpsData.status;

	for(i = 0; i < 2; i++)
	{
		gpsMsg.dev_status[i] = g_deviceData.status[i];
	}
	
	for(i = 0; i < 2; i++)
	{
		gpsMsg.battery_voltage[i] = g_simData.Battery[i];
	}
	
	for(i = 0; i < 2; i++)
	{
		gpsMsg.signal_strength[i] = g_simData.Signal[i];
	}
	
	for(i = 0; i < 2; i++)
	{
		gpsMsg.analog_input1[i] = g_deviceData.analog1[i];
	}
	
	for(i = 0; i < 2; i++)
	{
		gpsMsg.analog_input2[i] = g_deviceData.analog2[i];
	}
}

void PackGpsMsg(void)
{
	uint32_t i;
	uint32_t offset = 0;
	offset = 0;
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.hdr.header[i];
		offset++;
	}
	gpsBuf[offset] = gpsMsg.hdr.type;
	offset++;
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.hdr.len[i];
		offset++;
	}
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.hdr.seq[i];
		offset++;
	}
	for(i = 0; i < 4; i++)
	{
		gpsBuf[offset] = gpsMsg.utctime[i];
		offset++;
	}
	for(i = 0; i < 4; i++)
	{
		gpsBuf[offset] = gpsMsg.lati[i];
		offset++;
	}
	for(i = 0; i < 4; i++)
	{
		gpsBuf[offset] = gpsMsg.longi[i];
		offset++;
	}
	gpsBuf[offset] = gpsMsg.speed;
	offset++;
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.course[i];
		offset++;
	}
	for(i = 0; i < 9; i++)
	{
		gpsBuf[offset] = gpsMsg.basestation[i];
		offset++;
	}
	gpsBuf[offset] = gpsMsg.gps_valid;
	offset++;
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.dev_status[i];
		offset++;
	}
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.battery_voltage[i];
		offset++;
	}
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.signal_strength[i];
		offset++;
	}
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.analog_input1[i];
		offset++;
	}
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.analog_input2[i];
		offset++;
	}
	
	if(offset != (EELINK_GPS_MSGLEN))
	{
		printf("PackGpsMsg ERROR!\n");
	}
}

void PackAlarmMsg(void)
{
	uint32_t i;
	uint32_t offset = 0;
	offset = 0;
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.hdr.header[i];
		offset++;
	}
	gpsBuf[offset] = gpsMsg.hdr.type;
	offset++;
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.hdr.len[i];
		offset++;
	}
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.hdr.seq[i];
		offset++;
	}
	for(i = 0; i < 4; i++)
	{
		gpsBuf[offset] = gpsMsg.utctime[i];
		offset++;
	}
	for(i = 0; i < 4; i++)
	{
		gpsBuf[offset] = gpsMsg.lati[i];
		offset++;
	}
	for(i = 0; i < 4; i++)
	{
		gpsBuf[offset] = gpsMsg.longi[i];
		offset++;
	}
	gpsBuf[offset] = gpsMsg.speed;
	offset++;
	for(i = 0; i < 2; i++)
	{
		gpsBuf[offset] = gpsMsg.course[i];
		offset++;
	}
	for(i = 0; i < 9; i++)
	{
		gpsBuf[offset] = gpsMsg.basestation[i];
		offset++;
	}
	gpsBuf[offset] = gpsMsg.gps_valid;
	offset++;
	gpsBuf[offset] = 0x71;  // remove alarm flag
	offset++;
	
	if(offset != (EELINK_ALARM_MSGLEN))
	{
		printf("PackAlarmMsg ERROR!\n");
	}
}

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
