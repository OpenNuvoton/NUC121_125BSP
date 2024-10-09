/**************************************************************************//**
 * @file    main.c
 * @version V3.01
 * @brief   A sample code to show how to debug with semihost message print.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#if defined (__GNUC__) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Lock protected registers */
    SYS_LockReg();
} // SYS_Init()

/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main()
{
    /*
    This sample code is used to show how to print message/getchar on IDE debug environment.

    To enable semihost in KEIL MDK or IAR Workbench
        User must define "DEBUG_ENABLE_SEMIHOST" constant when building sample code.
        If defined DEBUG_ENABLE_SEMIHOST = 1 or 2 and ICE connected, the message will output to ICE.
        If defined DEBUG_ENABLE_SEMIHOST = 1 and ICE off line, the message will re-direct to UART debug port.
        If defined DEBUG_ENABLE_SEMIHOST = 2 and ICE off line, no any debug message output.

        In KEIL MDK, user need to open "View->Serial Window->UART #1" windows in debug mode.
        In IAR Workbench, user need to open "View->Terminal I/O" in debug mode.

        NOTE1: Hardfault_Handler is used for semihost. User cannot overwrite it when using semihost.
           If it is necessary to process hardfault, user can append code to ProcessHardfault of retarget.c
        NOTE2: Semihost only works with Nuvoton NuLink ICE Dongle in debug mode.
        NOTE3: The message will output to debug port if Nuvoton NuLink ICE Dongle is not connected.


        Semihost On/Off | NuLink Connected | Output Path
        ==============================================================
          1         |         1        |  ICE
          1         |         0        |  UART Debug Port / NULL when DEBUG_ENABLE_SEMIHOST=2
          0         |         1        |  UART Debug Port
          0         |         0        |  UART Debug Port
        --------------------------------------------------------------

    To enable semihost in NuEclipse IDE
        1. Call initialise_monitor_handles() before calling printf
        2. User must define "OS_USE_SEMIHOSTING" constant when building sample code.
        3. Check "Enable ARM semihosting" is enabled in
           Debug Configuration->GDB Nuvoton Nu-Link Debugging->Startup

        If defined OS_USE_SEMIHOSTING, message will output to NuEclipse IDE console.
        If not defined OS_USE_SEMIHOSTING, message will output to UART debug port.
    */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
#if defined (__GNUC__) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n Start SEMIHOST test: \n");

    while (1)
    {

        /* Get input character */
        int8_t item = getchar();

        /* Print input character back */
        printf("%c\n", item);
    }
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
