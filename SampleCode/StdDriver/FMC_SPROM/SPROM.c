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

/* User should always see "SPROM AREA" on UART console when SPROM_Function is called */
const char c_acStringBuf[] = { "SPROM AREA\n" };
void SPROM_Function(void)
{
    char *ptr;
    /* printf() is defined in standard library in APROM,
       this message can not be showed when CPU is executing APROM code in SPROM security/debug mode. */
    printf("*** You can see this message only when SPROM is in non-security mode ***\n");
    ptr = (char *)&c_acStringBuf[0];

    /* Send message by UART to check that execution code in SPROM works normally */
    while (*ptr != '\n')
    {
        SendChar_ToUART(*ptr++);
    }

    SendChar_ToUART('\n');
}
