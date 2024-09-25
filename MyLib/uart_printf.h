#ifndef __UART_PRINTF_H
#define __UART_PRINTF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"  // Bao gồm các thư viện HAL liên quan

// Khai báo UART handler
extern UART_HandleTypeDef huart1;

// Định nghĩa hàm PUTCHAR_PROTOTYPE tùy vào trình biên dịch
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#ifdef __cplusplus
}
#endif

#endif /* __UART_PRINTF_H */
