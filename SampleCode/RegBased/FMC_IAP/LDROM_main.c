/******************************************************************************
 * @file     LDROM_main.c
 * @version  V3.00
 * @brief    Show how to reboot to LDROM functions from APROM.
 *           This sample code set VECMAP to LDROM and reset to re-boot to LDROM.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC121.h"

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




int main()
{
    /* Unlock protected register */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Initial UART */
    UART_Init();

    printf("\n\n");
    printf("NUC121 FMC IAP Sample Code [LDROM code]\n");

    /* Enable FMC ISP function */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

    printf("\n\nPress any key to branch to APROM...\n");
    getchar();

    printf("\n\nChange VECMAP and branch to LDROM...\n");

    while (!(((UART0->FIFOSTS) & UART_FIFOSTS_TXEMPTYF_Msk) >> UART_FIFOSTS_TXEMPTYF_Pos))

        /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
        __set_PRIMASK(1);

    /* Change VECMAP for booting to APROM */
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPTRG = 0x1;
    __ISB();

    while (FMC->ISPTRG);             /* Waiting for ISP Done */

    /* Lock protected Register */
    SYS_LockReg();

    /* Software reset to boot to APROM */
    NVIC_SystemReset();

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
