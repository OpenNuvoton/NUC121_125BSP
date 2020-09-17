/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use PWM output waveform.
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

/**
 * @brief       PWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 interrupt event
 */
void PWM0_IRQHandler(void)
{

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Enable PLL */
    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for PLL clock ready */
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    /*Select HCLK clock source as PLL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable PWM0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_PWM0CKEN_Msk;

    /* Enable PWM1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_PWM1CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock divider as 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);


    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_PWM0SEL_Msk) | CLK_CLKSEL1_PWM0SEL_PCLK0;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_PWM1SEL_Msk) | CLK_CLKSEL1_PWM1SEL_PCLK1;

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_PWM0SEL_Msk) | CLK_CLKSEL1_PWM0SEL_PLL;
    //CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_PWM1SEL_Msk) | CLK_CLKSEL1_PWM1SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC_DIV2 and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UARTSEL_Msk) | CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Reset PWM0 module */
    SYS->IPRST1 |= SYS_IPRST1_PWM0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_PWM0RST_Msk;

    /* Reset PWM1 module */
    SYS->IPRST1 |= SYS_IPRST1_PWM1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_PWM1RST_Msk;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD);

    /* Set PC multi-function pins for PWM0 Channel0~5 */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC10MFP_PWM0_CH0;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC11MFP_PWM0_CH1;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC12MFP_PWM0_CH2;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC13MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC13MFP_PWM0_CH3;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC8MFP_PWM0_CH4;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC9MFP_PWM0_CH5;

    /* Set PC multi-function pins for PWM1 Channel0~5 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_PWM1_CH0;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC1MFP_PWM1_CH1;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC2MFP_PWM1_CH2;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC3MFP_PWM1_CH3;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC4MFP_PWM1_CH4;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC5MFP_PWM1_CH5;

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
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL1 & CLK_CLKSEL1_PWM0SEL_Msk) ? "CPU" : "PLL");
    printf("PWM1 clock is from %s\n", (CLK->CLKSEL1 & CLK_CLKSEL1_PWM1SEL_Msk) ? "CPU" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with PWM0 and PWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PC.10), PWM0_CH1(PC.11), PWM0_CH2(PC.12), PWM0_CH3(PC.13), PWM0_CH4(PC.8), PWM0_CH5(PC.9)\n");
    printf("                         PWM1_CH0(PC.0),  PWM1_CH1(PC.1),  PWM1_CH2(PC.2),  PWM1_CH3(PC.3),  PWM1_CH4(PC.4), PWM1_CH5(PC.5)\n");

    /* PWM0 and PWM1 channel 0~5 frequency and duty configuration are as follows */

    /* Set PWM to up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /*
      Configure PWM0 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (200) / (399 + 1) = 50%
    */
    /* PWM0 channel 0 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 1); /* Divided by 2 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM0, 0, 200);
    /* Set PWM Timer period */
    PWM_SET_CNR(PWM0, 0, 399);

    /*
      Configure PWM0 channel 1 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (100) / (399 + 1) = 25%
    */
    /* PWM0 channel 1 frequency and duty configuration */
    /* PWM Timer clock prescaler and PWM Period of Channel 1 is share with Channel 0 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM0, 1, 100);

    /*
      Configure PWM0 channel 2 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (799 + 1)) = 31250 Hz
      Duty ratio = (400) / (799 + 1) = 50%
    */
    /* PWM0 channel 2 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 2, 1); /* Divided by 2 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM0, 2, 400);
    /* Set PWM Timer period */
    PWM_SET_CNR(PWM0, 2, 799);

    /*
      Configure PWM0 channel 3 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (799 + 1)) = 31250 Hz
      Duty ratio = (200) / (799 + 1) = 25%
    */
    /* PWM0 channel 3 frequency and duty configuration */
    /* PWM Timer clock prescaler and PWM Period of Channel 3 is share with Channel 2 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM0, 3, 200);

    /*
      Configure PWM0 channel 4 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (1599 + 1)) = 15625 Hz
      Duty ratio = (800) / (1599 + 1) = 50%
    */
    /* PWM0 channel 4 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 4, 1); /* Divided by 2 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM0, 4, 800);
    /* Set PWM Timer period */
    PWM_SET_CNR(PWM0, 4, 1599);

    /*
      Configure PWM0 channel 5 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (1599 + 1)) = 15625 Hz
      Duty ratio = (400) / (1599 + 1) = 25%
    */
    /* PWM0 channel 5 frequency and duty configuration */
    /* PWM Timer clock prescaler and PWM Period of Channel 5 is share with Channel 4 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM0, 5, 400);

    /* Set waveform generation */
    PWM_SET_OUTPUT_LEVEL(PWM0, 0x3F, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 0 ~ 5 */
    PWM0->POEN |= 0x3F;

    /* Set PWM to up counter type */
    PWM1->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /*
       Configure PWM1 channel 0 init period and duty(up counter type).
       Period is PLL / (prescaler * (CNR + 1))
       Duty ratio = (CMR) / (CNR + 1)
       Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
       Duty ratio = (40) / (399 + 1) = 10%
     */
    /* PWM1 channel 0 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    PWM_SET_PRESCALER(PWM1, 0, 1); /* Divided by 2 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM1, 0, 40);
    /* Set PWM Timer period */
    PWM_SET_CNR(PWM1, 0, 399);

    /*
      Configure PWM1 channel 1 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
     Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (80) / (399 + 1) = 20%
    */
    /* PWM1 channel 1 frequency and duty configuration */
    /* PWM Timer clock prescaler and PWM Period of Channel 1 is share with Channel 0 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM1, 1, 80);

    /*
      Configure PWM1 channel 2 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (120) / (399 + 1) = 30%
    */
    /* PWM1 channel 2 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    PWM_SET_PRESCALER(PWM1, 2, 1); /* Divided by 2 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM1, 2, 120);
    /* Set PWM Timer period */
    PWM_SET_CNR(PWM1, 2, 399);

    /*
      Configure PWM1 channel 3 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (160) / (399 + 1) = 40%
    */
    /* PWM1 channel 3 frequency and duty configuration */
    /* PWM Timer clock prescaler and PWM Period of Channel 3 is share with Channel 2 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM1, 3, 160);

    /*
      Configure PWM1 channel 4 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (200) / (399 + 1) = 50%
    */
    /* PWM1 channel 4 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    PWM_SET_PRESCALER(PWM1, 4, 1); /* Divided by 2 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM1, 4, 200);
    /* Set PWM Timer period */
    PWM_SET_CNR(PWM1, 4, 399);

    /*
      Configure PWM1 channel 5 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (2 * (399 + 1)) = 62500 Hz
      Duty ratio = (240) / (399 + 1) = 60%
    */
    /* PWM1 channel 5 frequency and duty configuration */
    /* PWM Timer clock prescaler and PWM Period of Channel 5 is share with Channel 4 */
    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM1, 5, 240);

    /* Set waveform generation */
    PWM_SET_OUTPUT_LEVEL(PWM1, 0x3F, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM1 channel 0 ~ 5 */
    PWM1->POEN |= 0x3F;

    /* Start PWM0 counter */
    PWM0->CNTEN = 0x3F;
    /* Start PWM1 counter */
    PWM1->CNTEN = 0x3F;

    printf("Press any key to stop.\n");
    getchar();

    /* Stop PWM0 counter */
    PWM0->CNTEN &= ~0x3F;
    /* Stop PWM1 counter */
    PWM1->CNTEN &= ~0x3F;

    printf("Done.\n");

    while (1);

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
