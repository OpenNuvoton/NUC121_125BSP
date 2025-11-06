/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief   Demonstrate how to use PWM Dead Zone function.
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
    static uint32_t u32Cnt;
    static uint32_t u32Out;

    /* Clear channel 0 period interrupt flag */
    PWM0->INTSTS0 = PWM_INTSTS0_PIF0_Msk;

    /* Channel 0 frequency is 5000Hz, every 1 second enter this IRQ handler 5000 times. */
    if (++u32Cnt == 5000)
    {
        if (u32Out)
            PWM0->POEN |= (0xF);
        else
            PWM0->POEN &= ~(0xF);

        u32Out ^= 1;
        u32Cnt = 0;
    }

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

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock divider as 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_PWM0SEL_Msk) | CLK_CLKSEL1_PWM0SEL_PCLK0;

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_PWM0SEL_Msk) | CLK_CLKSEL1_PWM0SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC_DIV2 and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UARTSEL_Msk) | CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Reset PWM0 module */
    SYS->IPRST1 |= SYS_IPRST1_PWM0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_PWM0RST_Msk;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD);

    /* Set PC multi-function pins for PWM0 Channel0~3 */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC10MFP_PWM0_CH0;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC11MFP_PWM0_CH1;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC12MFP_PWM0_CH2;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC13MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC13MFP_PWM0_CH3;
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
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output PWM0 channel 0~3 with different\n");
    printf("  frequency and duty, enable dead zone function of all PWM0 pairs.\n");
    printf("  And also enable/disable PWM output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PC.10), PWM0_CH1(PC.11), PWM0_CH2(PC.12), PWM0_CH3(PC.13)\n");

    /*---------------------------------------------------------------------------------------------------------*/
    /* Up counter type                                                                                         */
    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM0 channel0 frequency is 5000Hz, duty 30% */
    /* Assume PWM output frequency is 5000Hz and duty ratio is 30%, user can calculate PWM settings by follows.
       up counter type:
       duty ratio = (CMR)/(CNR+1)
       cycle time = CNR+1
       High level = CMR
       PWM clock source frequency = PLL = 50000000
       (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
               = 50000000/2/5000 = 5000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 4999
       duty ratio = 30% ==> (CMR)/(CNR+1) = 30% ==> CMR = (CNR+1)*0.3 = 5000*30/100
       CMR = 1500
       Prescale value is 1 : prescaler= 2
    */

    /* Set Pwm mode as complementary mode */
    PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);

    /* Set PWM Timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 1); /* Divided by 2 */

    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM0, 0, 1500);
    PWM_SET_CMR(PWM0, 1, 1500);

    /* Set PWM Timer period */
    PWM_SET_CNR(PWM0, 0, 4999);

    /* Set waveform generation */
    //PWM0->WGCTL0 = 0xAA;
    //PWM0->WGCTL1 = 0x55;
    PWM_SET_OUTPUT_LEVEL(PWM0, 0xF, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable and configure dead zone */
    SYS_UnlockReg();
    PWM0->DTCTL0_1 = 40;
    PWM0->DTCTL0_1 |= PWM_DTCTL0_1_DTEN_Msk;
    SYS_LockReg();

    /* PWM0 channel2 frequency is 2500Hz, duty 50% */
    /* Assume PWM output frequency is 2500Hz and duty ratio is 50%, user can calculate PWM settings by follows.
       up counter type:
       duty ratio = (CMR)/(CNR+1)
       cycle time = CNR+1
       High level = CMR
       PWM clock source frequency = PLL = 50000000
       (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
               = 50000000/2/2500 = 10000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 9999
       duty ratio = 50% ==> (CMR)/(CNR+1) = 50% ==> CMR = (CNR+1)*0.5 = 10000*50/100
       CMR = 5000
       Prescale value is 1 : prescaler = 2
    */

    /* Set PWM0 Timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 2, 1); /* Divided by 2 */

    /* Set PWM0 Timer duty */
    PWM_SET_CMR(PWM0, 2, 5000);
    PWM_SET_CMR(PWM0, 3, 5000);

    /* Set PWM0 Timer period */
    PWM_SET_CNR(PWM0, 2, 9999);

    /* Enable and configure dead zone */
    SYS_UnlockReg();
    PWM0->DTCTL2_3 = 20;
    PWM0->DTCTL2_3 |= PWM_DTCTL2_3_DTEN_Msk;
    SYS_LockReg();

    /* Enable output of PWM0 channel0~3 */
    PWM0->POEN |= (0xF);

    /* Enable PWM0 channel 0 period interrupt, use channel 0 to measure time. */
    PWM0->INTEN0 = (PWM0->INTEN0 & ~PWM_INTEN0_PIEN0_Msk) | PWM_INTEN0_PIEN0_Msk;
    NVIC_EnableIRQ(PWM0_IRQn);

    /* Start */
    PWM0->CNTEN = 0xF;

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
