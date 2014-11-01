#ifndef __GSM_APP_H_
#define __GSM_APP_H_


//#define USARTBUFSIZE    1024

#define CHINAMOBILE     3
#define CHINAUNICOM     4

typedef	struct
{
	char CMDType;
	void *pCMDData;
}ST_GSMCMD, *pST_GSMCMD;

typedef struct
{
  char PhoneNumber[12];
  char Name[6];
}ST_PHONEBOOKS, *pST_PHONEBOOKS;

typedef struct
{
	char RemoteIP[16];
	char RemotePort[5];
	char TransferMode[4];
	char APN[10];
}ST_NETWORKCONFIG, *pST_NETWORKCONFIG;

typedef struct
{
	unsigned char	BatStatus;		//电池状态，0没充电，1充电中，2已充满
	unsigned char BatPower;		  //电池电量，0-100
}ST_BATTERYSTATUS, *pST_BATTERYSTATUS;



extern unsigned char GSMSinal;			 
extern volatile unsigned char RingFlag;
extern volatile unsigned	int RingCount;
extern volatile unsigned char CallingRing;
extern volatile unsigned char SMSingRing;

char *strnchr(char *S, int C, int n);
unsigned char GSM_ChkRingSta(void);
void GSM_PowerOnOff(void);
void GSM_ClearBuffer(void);
unsigned char GSM_SendAT(char *pCMD, char *pCMDBack, uint32_t CMDLen);
unsigned char GSM_SendAT_rsp(char *pCMD, char *pCMDBack,
        uint32_t CMDLen, char **ppRecvBuf, uint32_t *pRecvLen);
unsigned char GSM_QueryNetType(void);
unsigned char GSM_QuerySignal(unsigned char *pSig);
unsigned char GSM_CallNumber(char *pNumber);
unsigned char GSM_ReadNumberFromSIM(unsigned char No, ST_PHONEBOOKS *pNumber);
unsigned char GSM_CallSIMNumber(unsigned char numberNo);
unsigned char GSM_QueryCallStatus(void);
unsigned char GSM_HangCall(void);
unsigned char GSM_AnswerCall(void);

typedef     struct
{       /* date and time components */
    signed char     Sec;
    signed char     Min;
    signed char     Hour;
    signed char     Day;
    signed char     Month;
    signed char     Year;
    signed char     Week;
    signed char     Timezone;
}ST_RTCTIME, *pST_RTCTIME;

unsigned char GSM_GetRTCTime(pST_RTCTIME prtctime);
unsigned char GSM_SetRTCTime(ST_RTCTIME rtctime);
unsigned char GSM_SendSMS(char *pNumb, char *pSMS, unsigned char type);

void GSM_Init(void);
void GPRS_Init(void);
unsigned char GPRS_LinkServer(pST_NETWORKCONFIG pnetconfig);
unsigned char GPRS_CloseLink(void);
unsigned char GPRS_SendData(char *pString, unsigned int len);
unsigned char GPRS_ReceiveData(char *pString);
unsigned char GSM_QueryBattery(pST_BATTERYSTATUS pSig);

void GSM_test_once(void);
void GSM_test(void);

#endif // __GSM_APP_H_

