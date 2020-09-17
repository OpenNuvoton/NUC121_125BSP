/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use BPWM output waveform.
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
    /* BPWM clock frequency configuration                                                                       */
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
    printf("  This sample code will output waveform with BPWM0 and BPWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0_CH0(PB.7), BPWM0_CH1(PB.6), BPWM0_CH2(PB.5), BPWM0_CH3(PB.4), BPWM0_CH4(PD.11), BPWM0_CH5(PA.11)\n");
    printf("                         BPWM1_CH0(PB.14), BPWM1_CH1(PB.8), BPWM1_CH2(PF.3), BPWM1_CH3(PF.2), BPWM1_CH4(PD.5), BPWM1_CH5(PD.4)\n");

    /* BPWM0 and BPWM1 channel 0~5 frequency and duty configuration are as follows */

    /* Set BPWM to up counter type */
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;

    /*
      Configure BPWM0 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (1 * (199 + 1)) = 250000 Hz
      Duty ratio = (20) / (199 + 1) = 10%
    */
    /* BPWM0 channel 0 frequency and duty configuration */
    /* Set BPWM Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 0); /* Divided by 1 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM0, 0, 20);
    /* Set BPWM Timer period */
    BPWM_SET_CNR(BPWM0, 0, 199);

    /*
      Configure BPWM0 channel 1 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (1 * (199 + 1)) = 250000 Hz
      Duty ratio = (40) / (199 + 1) = 20%
    */
    /* BPWM0 channel 1 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 1 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM0, 1, 40);

    /*
      Configure BPWM0 channel 2 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (1 * (199 + 1)) = 250000 Hz
      Duty ratio = (60) / (199 + 1) = 30%
    */
    /* BPWM0 channel 2 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 2 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM0, 2, 60);

    /*
      Configure BPWM0 channel 3 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (1 * (199 + 1)) = 250000 Hz
      Duty ratio = (80) / (199 + 1) = 40%
    */
    /* BPWM0 channel 3 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 3 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM0, 3, 80);

    /*
      Configure BPWM0 channel 4 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (1 * (199 + 1)) = 250000 Hz
      Duty ratio = (100) / (199 + 1) = 50%
    */
    /* BPWM0 channel 4 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 4 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM0, 4, 100);

    /*
      Configure BPWM0 channel 5 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (1 * (199 + 1)) = 250000 Hz
      Duty ratio = (120) / (199 + 1) = 60%
    */
    /* BPWM0 channel 5 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 5 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM0, 5, 120);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);

    /* Enable output of BPWM0 channel 0 ~ 5 */
    BPWM0->POEN |= 0x3F;

    /* Set BPWM to up counter type */
    BPWM1->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;

    /*
      Configure BPWM1 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (40) / (399 + 1) = 10%
    */
    /* BPWM1 channel 0 frequency and duty configuration */
    /* Set BPWM Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM1, 0, 1); /* Divided by 2 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM1, 0, 40);
    /* Set BPWM Timer period */
    BPWM_SET_CNR(BPWM1, 0, 399);

    /*
      Configure BPWM1 channel 1 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (80) / (399 + 1) = 20%
    */
    /* BPWM1 channel 1 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 1 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM1, 1, 80);

    /*
      Configure BPWM1 channel 2 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (120) / (399 + 1) = 30%
    */
    /* BPWM1 channel 2 frequency and duty configuration */
    /* Set BPWM Timer clock prescaler */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 2 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM1, 2, 120);

    /*
      Configure BPWM1 channel 3 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (160) / (399 + 1) = 30%
    */
    /* BPWM1 channel 3 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 3 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM1, 3, 160);

    /*
      Configure BPWM1 channel 4 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (200) / (399 + 1) = 50%
    */
    /* BPWM1 channel 4 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 4 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM1, 4, 200);

    /*
      Configure BPWM1 channel 5 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (240) / (399 + 1) = 60%
    */
    /* BPWM1 channel 5 frequency and duty configuration */
    /* BPWM Timer clock prescaler and BPWM Period of Channel 5 is share with Channel 0 */
    /* Set BPWM Timer duty */
    BPWM_SET_CMR(BPWM1, 5, 240);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM1, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);

    /* Enable output of BPWM1 channel 0 ~ 5 */
    BPWM1->POEN |= 0x3F;

    /* Start BPWM0 counter */
    BPWM0->CNTEN = 0x1;
    /* Start BPWM1 counter */
    BPWM1->CNTEN = 0x1;

    printf("Press any key to stop.\n");
    getchar();

    /* Stop BPWM0 counter */
    BPWM0->CNTEN &= ~0x1;
    /* Stop BPWM1 counter */
    BPWM1->CNTEN &= ~0x1;

    printf("Done.\n");

    while (1);

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
