#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart2;
static uint8_t c;

void vprint(const char *fmt, va_list argp)
{
    char string[256];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), HAL_MAX_DELAY); // send message via UART
    }
}

void uprintf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

uint8_t uget() {
	HAL_UART_Receive(&huart2, &c, sizeof(c), HAL_MAX_DELAY);
	return c;
}
