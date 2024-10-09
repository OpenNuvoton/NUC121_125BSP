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

/* Check CBS and flash lock before runing this sample code,
   BS should be APROM or APROM with IAP, flash lock should be disabled,
   please use Target option -> Utilities -> Setting -> Config to check */

extern void SPROM_Function(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /*------------------------------------------------------------------------------------------------------*/
    /* Enable Module Clock                                                                                  */
    /*------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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

    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable SPROM update */
    FMC_EnableSPUpdate();

    /* Enable Config update */
    FMC_ENABLE_CFG_UPDATE();

    printf("\n\n");

    if (FMC_Read(FMC_SPROM_BASE + 0x1FC) == 0xFFFFFFFF)
    {
        printf("======= SPROM is in non-security mode ========\n");
        printf("User should see valid data on SPROM with non-security mode\n");
        SPROMDump();
        SPROM_Function();
        printf("--------------------------------------------------------\n");
        printf("Press any key to enter SPROM security mode....\n");
        getchar();
        /* The program unit is 4 bytes, hence we program 0x2001FC~0x2001FF to change mode.
           Write 0x0 on 0x2001FF to change mode to security mode,
           user also can write other data on this position except 0xFF(non-security mode) and 0xAA(debug mode) */
        FMC_Write(FMC_SPROM_BASE + 0x1FC, 0x00FFFFFF);
        /* Setting SPROM in security mode would take effect after reset chip */
        SYS_ResetChip();
    }

    printf("======= SPROM is in security mode =======\n");
    printf("User should see all zero data on SPROM with security mode\n");
    SPROMDump();
    SPROM_Function();
    printf("--------------------------------------------------------\n");

    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("Please flash this sample again to erase SPROM area to rerun test.\n");

    while (1);
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
