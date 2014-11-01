
#ifndef __STRING_OPERATOR_H
#define __STRING_OPERATOR_H

#include "stm32f10x.h"

char *strnchr(char *S, int C, int n);
char *strstr_len(char *str, char *subStr, uint32_t strlenth);
char *strnchr_len(char *S, int C, uint32_t n, uint32_t len);

#endif // __STRING_OPERATOR_H

