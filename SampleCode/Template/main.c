/*************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       Software Development Template.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"


void SYS_Init(void)
{
    /* Enable IP clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;
} // SYS_Init()


int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("Simple Demo Code\n\n");
    printf("Please Input Any Key\n\n");

    do
    {
        int8_t ch;

        printf("Input: ");
        ch = getchar();
        printf("%c\n", ch);
    } while (1);
} // main()


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
