/*************************************************************************//**
 *
 * Copyright(c) 2016 Nuvoton Technology Corp. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * This sample code simulate how to execute code in SPROM area, its obj file
 * should be placed on SPROM_BASE.
 * User can refer to scatter file(option->linker) to see how it works.
 *
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

static void SendChar_ToUART(int ch)
{

    while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    DEBUG_PORT->DAT = ch;

    if (ch == '\n')
    {
        while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

        DEBUG_PORT->DAT = '\r';
    }
}
/* User should always see "SPROM AREA" on uart console when Sprom_function is called */
const char c_acStringBuf[] = {'S', 'P', 'R', 'O', 'M', ' ', 'A', 'R', 'E', 'A', '\n'};
void SPROM_Function(void)
{
    char *ptr;
    /* printf() is defined in standard library in APROM,
       this message can not be showed when CPU is excuting APROM code in SPROM security/debug mode*/
    printf("*** You can see this message only when SPROM is in non-security mode ***\n");
    ptr = (char *)&c_acStringBuf[0];

    /* send message by uart to check that excution code in SPROM works normally */
    while (*ptr != '\n')
    {
        SendChar_ToUART(*ptr++);
    }

    SendChar_ToUART('\n');
}
