/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x32
 * @date     14, June, 2017
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __TARGET_H__
#define __TARGET_H__

#include "NuMicro.h"
#include "ISP_USER.h"
#include "uart_transfer.h"

/* rename for uart_transfer.c */
#define UART_T                  UART0
#define UART_T_IRQHandler       UART0_IRQHandler
#define UART_T_IRQn             UART0_IRQn

/*
// UART_T define option
#define UART_T                  UART
#define UART_T                  UART0
#define UART_T                  UART1
*/

/*
// UART_T_IRQHandler define option
#define UART_T_IRQHandler       UART_IRQHandler
#define UART_T_IRQHandler       UART0_IRQHandler
#define UART_T_IRQHandler       UART1_IRQHandler

*/

/*
// UART_T_IRQn define option
#define UART_T_IRQn                 UART_IRQn
#define UART_T_IRQn                 UART0_IRQn
#define UART_T_IRQn                 UART1_IRQn
*/
#endif //__TARGET_H__
