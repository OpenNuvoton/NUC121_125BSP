/*************************************************************************//**
 * @file        main.c
 * @version     V3.00
 * @brief       Demonstrate how to set SPROM in security mode.
 *              When SPROM is in security mode, user only can see zero data in SPROM Area.
 *              (Support KEIL MDK Only)
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/* check CBS and flash lock before runing this sample code,
   BS should be APROM or APROM with IAP, flash lock should be disabled,
   please use target option -> Utilities -> setting -> config to check */

extern void SPROM_Function(void);

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
}

void UART_Init()
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

void SPROMDump(void)
{
    volatile uint32_t  addr;
    printf("Dump SPROM from 0x200000 to 0x2001FF\n");

    for (addr = FMC_SPROM_BASE; addr <= 0x2001FF; addr += 4)
    {
        if ((addr % 16) == 0)
            printf("\n0x%08x: ", addr);

        printf("0x%08x ", inpw(addr));
    }

    printf("\n");
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Init();

    /* Enable FMC ISP function and SPROM ,config update */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_CFGUEN_Msk | FMC_ISPCTL_SPUEN_Msk;

    printf("\n\n");

    /* read the last bytes of SPROM by FMC */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_SPROM_BASE + 0x1FC;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPDAT == 0xFFFFFFFF)
    {
        printf("======= SPROM is in non-security mode ========\n");
        printf("User should see valid data on SPROM with non-security mode\n");
        SPROMDump();
        SPROM_Function();
        printf("--------------------------------------------------------\n");
        printf("press any key to enter in SPROM security mode....\n");
        getchar();
        /* the program unit is 4 bytes, so we program 0x2001FC~0x2001FF to change mode.
           Write 0x0 on 0x2001FF to change mode to security mode,
           user also can write other data on this position except 0xFF(non-security mode) and 0xAA(debug mode) */
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_SPROM_BASE + 0x1FC;
        FMC->ISPDAT = 0x00FFFFFF;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

        /* setting SPROM to security mode would take effect after reset chip */
        SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
    }

    printf("======= SPROM is in security mode =======\n");
    printf("User should see all zero data on SPROM with security mode\n");
    SPROMDump();
    SPROM_Function();
    printf("--------------------------------------------------------\n");

    /* Enable FMC ISP function */
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("If user want to rerun this test,\nplease do flash again by ICE to erase SPROM area and flash back SPROM.o\n");

    while (1);
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
