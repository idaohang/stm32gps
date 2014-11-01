
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "stm32f10x.h"
#include "string_operator.h"

/*********************************************************************************************************
 ** Function name:       strnchr()
 ** Descriptions:        �����ַ����е�n���ַ���λ��
 ** input parameters:    S ��Ҫ���ҵ�Ŀ���ַ���
 **                      C ��Ҫ���ҵ��ַ�
 **                      n �ڼ���λ�õ�C
 ** output parameters:   ��Ҫ���ҵ��ַ�λ��ָ��
 ** Returned value:
 *********************************************************************************************************/
char *strnchr(char *S, int C, int n)
{
    char *pnchr = NULL;
    char *pStr = S;

    if (n > strlen(S))
    {
        return NULL;
    }
    while (n--)
    {
        pnchr = strchr((const char *) pStr, C);
        if (NULL == pnchr)
        {
            break;
        }
        else
        {
            pStr = pnchr + 1;
        }
    }
    return pnchr;
}

char *strstr_len(char *str, char *subStr, uint32_t strlenth)
{
    uint32_t subStrLen = strlen(subStr);
    int32_t cmpCnt = strlenth - strlen(subStr) + 1;

    char *retPtr = NULL;
    int32_t i;

    if (cmpCnt > 0)
    {
        for (i = 0; i < cmpCnt; i ++)
        {
            if (memcmp(str+i, subStr, subStrLen) == 0)
            {
                break;
            }
        }

        if (i < cmpCnt)
        {
            retPtr = str + i;
        }
    }

    return retPtr;
}

char *strnchr_len(char *S, int C, uint32_t n, uint32_t len)
{
    char *pnchr = NULL;
    char *pStr = S;
    uint32_t i;

    if (n > len)
    {
        return NULL;
    }
    while (n--)
    {
        for (i = 0; i < len; i ++)
        {
            if (*(pStr+i) == C)
            {
                break;
            }
        }

        if (i < len)
        {
            pnchr = pStr+i;
            len -= i + 1;
        }
        else {
            pnchr = NULL;
        }

        if (NULL == pnchr)
        {
            break;
        }
        else
        {
            pStr = pnchr + 1;
        }
    }
    return pnchr;
}

