/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Implement a code and execute in SRAM to program embedded Flash.
 *           (Support KEIL MDK Only)
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define APROM_TEST_BASE     0x6000

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Update core clock */
    PllClock        = 0;                 // PLL
    SystemCoreClock = __HIRC;            // HCLK
    CyclesPerUs     = __HIRC / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set PA multi-function pins for CLKO(PA.15) */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk)) | SYS_GPA_MFPH_PA15MFP_CLKO;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int main()
{
    uint32_t u32Addr;
    uint32_t i;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Initial UART */
    UART0_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Check main function execution address (0x%08X).\n", (uint32_t)main);

    /*
       This sample code demonstrates how to implement code execution in SRAM.
       By configuring the linker script files:
       - KEIL: FMC_ExeInSRAM.scf
       - IAR:  FMC_ExeInSRAM.icf
       - GCC:  FMC_ExeInSRAM.ld
    */

    /* Enable FMC ISP functions */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
    /* Enable updating APROM */
    FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk;

    /* The ROM address for erase/write/read demo */
    u32Addr = APROM_TEST_BASE;

    /* Erase */
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = 0x1;
    __ISB();

    while (FMC->ISPTRG);

    for (i = 0; i < 0x100; i += 4)
    {
        uint32_t u32Data;

        /* Write Demo */
        u32Data = i + 0x12345678;
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = u32Addr + i;
        FMC->ISPDAT = u32Data;
        FMC->ISPTRG = 0x1;
        __ISB();

        while (FMC->ISPTRG);

        if ((i & 0xf) == 0)
            printf(".");

        /* Read Demo */
        FMC->ISPCMD = FMC_ISPCMD_READ;
        FMC->ISPADDR = u32Addr + i;
        FMC->ISPDAT = 0;
        FMC->ISPTRG = 0x1;
        __ISB();

        while (FMC->ISPTRG);

        uint32_t u32RData = FMC->ISPDAT;

        if (u32Data != u32RData)
        {
            printf("[Read/Write FAIL]\n");

            while (1);
        }
    }

    /* Disable FMC ISP function */
    FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
