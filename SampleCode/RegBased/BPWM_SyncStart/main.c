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
#define PLLCTL_SETTING  CLK_PLLCTL_100MHz_HIRC_DIV2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Enable PLL */
    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for PLL clock ready */
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    /*Select HCLK clock source as PLL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable BPWM0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_BPWM0CKEN_Msk;

    /* Enable BPWM1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_BPWM1CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock divider as 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    /* BPWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.BPWM clock frequency is set equal to HCLK: select BPWM module clock source as PCLK */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_BPWM0SEL_Msk) | CLK_CLKSEL1_BPWM0SEL_PCLK0;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_BPWM1SEL_Msk) | CLK_CLKSEL1_BPWM1SEL_PCLK1;

    /* case 2.BPWM clock frequency is set double to HCLK: select BPWM module clock source as PLL */
    //CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_BPWM0SEL_Msk) | CLK_CLKSEL1_BPWM0SEL_PLL;
    //CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_BPWM1SEL_Msk) | CLK_CLKSEL1_BPWM1SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC_DIV2 and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UARTSEL_Msk) | CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Reset BPWM0 module */
    SYS->IPRST1 |= SYS_IPRST1_BPWM0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_BPWM0RST_Msk;

    /* Reset BPWM1 module */
    SYS->IPRST1 |= SYS_IPRST1_BPWM1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_BPWM1RST_Msk;

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
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
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
    printf("BPWM0 clock is from %s\n", (CLK->CLKSEL1 & CLK_CLKSEL1_BPWM0SEL_Msk) ? "CPU" : "PLL");
    printf("BPWM1 clock is from %s\n", (CLK->CLKSEL1 & CLK_CLKSEL1_BPWM1SEL_Msk) ? "CPU" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM0 and BPWM1 channel 0~5 at the same time.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0_CH0(PB.7), BPWM0_CH1(PB.6), BPWM0_CH2(PB.5), BPWM0_CH3(PB.4), BPWM0_CH4(PD.11), BPWM0_CH5(PA.11)\n");
    printf("                         BPWM1_CH0(PB.14), BPWM1_CH1(PB.8), BPWM1_CH2(PF.3), BPWM1_CH3(PF.2), BPWM1_CH4(PD.5), BPWM1_CH5(PD.4)\n");

    /* BPWM0 and BPWM1 channel 0~5 frequency and duty configuration are as follows */

    /*
      Configure BPWM0 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (1 * (399 + 1)) = 125000 Hz
      Duty ratio = (200) / (399 + 1) = 50%
    */

    /* Set BPWM to up counter type */
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;

    /* BPWM0 channel 0 frequency and duty configuration */
    /* Set BPWM Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 0); /* Divided by 1 */

    /* Set BPWM Timer period */
    BPWM_SET_CNR(BPWM0, 0, 399);
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM0, 0, 200);
    BPWM_SET_CMR(BPWM0, 1, 200);
    BPWM_SET_CMR(BPWM0, 2, 200);
    BPWM_SET_CMR(BPWM0, 3, 200);
    BPWM_SET_CMR(BPWM0, 4, 200);
    BPWM_SET_CMR(BPWM0, 5, 200);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);

    /* Enable output of BPWM0 channel 0 ~ 5 */
    BPWM0->POEN |= 0x3F;

    /* Set BPWM to up counter type */
    BPWM1->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;

    /* BPWM1 channel 0 frequency and duty configuration */
    /* Set BPWM Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM1, 0, 0); /* Divided by 1 */
    /* Set BPWM Timer period */
    BPWM_SET_CNR(BPWM1, 0, 399);
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM1, 0, 200);
    BPWM_SET_CMR(BPWM1, 1, 200);
    BPWM_SET_CMR(BPWM1, 2, 200);
    BPWM_SET_CMR(BPWM1, 3, 200);
    BPWM_SET_CMR(BPWM1, 4, 200);
    BPWM_SET_CMR(BPWM1, 5, 200);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM1, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);

    /* Enable output of BPWM1 channel 0 ~ 5 */
    BPWM1->POEN |= 0x3F;

    /* Enable counter synchronous start function for BPWM0 and BPWM1 channel 0~5 */
    BPWM0->SSCTL = 0x3F | BPWM_SSCTL_SSRC_BPWM0;
    BPWM1->SSCTL = 0x3F | BPWM_SSCTL_SSRC_BPWM0;

    printf("Press any key to start.\n");
    getchar();

    /* Trigger BPWM counter synchronous start by BPWM0 */
    BPWM0->SSTRG = BPWM_SSTRG_CNTSEN_Msk;

    printf("Done.");

    while (1);

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
