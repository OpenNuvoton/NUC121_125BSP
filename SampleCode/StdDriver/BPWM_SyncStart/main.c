/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use BPWM counter synchronous start function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK       100000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Enable BPWM0 and BPWM1 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    CLK_EnableModuleClock(BPWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* BPWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.BPWM clock frequency is set equal to HCLK: select BPWM module clock source as PCLK */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL1_BPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(BPWM1_MODULE, CLK_CLKSEL1_BPWM1SEL_PCLK1, 0);

    /* case 2.BPWM clock frequency is set double to HCLK: select BPWM module clock source as PLL */
    //CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL1_BPWM0SEL_PLL, NULL);
    //CLK_SetModuleClock(BPWM1_MODULE, CLK_CLKSEL1_BPWM1SEL_PLL, NULL);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC_DIV2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Reset BPWM0 and BPWM1 module */
    SYS_ResetModule(BPWM0_RST);
    SYS_ResetModule(BPWM1_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD);

    /* Set multi-function pins for BPWM0 Channel0~5 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB7MFP_BPWM0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB6MFP_BPWM0_CH1;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB5MFP_BPWM0_CH2;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_BPWM0_CH3;
    SYS->GPD_MFPH = (SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD11MFP_Msk));
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD11MFP_BPWM0_CH4;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA11MFP_Msk));
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA11MFP_BPWM0_CH5;

    /* Set multi-function pins for BPWM1 Channel0~5 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk));
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_BPWM1_CH0;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk));
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB8MFP_BPWM1_CH1;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk));
    SYS->GPF_MFPL |= SYS_GPF_MFPL_PF3MFP_BPWM1_CH2;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk));
    SYS->GPF_MFPL |= SYS_GPF_MFPL_PF2MFP_BPWM1_CH3;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD5MFP_Msk));
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD5MFP_BPWM1_CH4;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk));
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD4MFP_BPWM1_CH5;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM0 and BPWM1 channel 0~5 at the same time.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin:  BPWM0_CH0(PB.7), BPWM0_CH1(PB.6), BPWM0_CH2(PB.5), BPWM0_CH3(PB.4), BPWM0_CH4(PD.11), BPWM0_CH5(PA.11)\n");
    printf("                          BPWM1_CH0(PB.14), BPWM1_CH1(PB.8), BPWM1_CH2(PF.3), BPWM1_CH3(PF.2), BPWM1_CH4(PD.5), BPWM1_CH5(PD.4)\n");


    /* BPWM0 and BPWM1 channel 0~5 frequency and duty configuration are as follows */
    BPWM_ConfigOutputChannel(BPWM0, 0, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 1, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 2, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 3, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 4, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 5, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 0, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 1, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 2, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 3, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 4, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 5, 1000, 50);

    /* Enable counter synchronous start function for BPWM0 and BPWM1 channel 0~5 */
    BPWM_ENABLE_TIMER_SYNC(BPWM0, 0x3F, BPWM_SSCTL_SSRC_BPWM0);
    BPWM_ENABLE_TIMER_SYNC(BPWM1, 0x3F, BPWM_SSCTL_SSRC_BPWM0);

    /* Enable output of BPWM0 and BPWM1 channel 0~5 */
    BPWM_EnableOutput(BPWM0, 0x3F);
    BPWM_EnableOutput(BPWM1, 0x3F);

    printf("Press any key to start.\n");
    getchar();

    /* Trigger BPWM counter synchronous start by BPWM0 */
    BPWM_TRIGGER_SYNC_START(BPWM0);

    printf("Done.");

    while (1);

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
