/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use the timer2 capture function to capture timer2 counter value.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


/**
  * @brief      Timer2 IRQ
  *
  * @param      None
  *
  * @return     None
  *
  * @details    The Timer2 default IRQ, declared in startup_M0564.s.
  */
void TMR2_IRQHandler(void)
{
    if (TIMER_GetCaptureIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER2);

        g_au32TMRINTCount[2]++;
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

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /*------------------------------------------------------------------------------------------------------*/
    /* Enable Module Clock                                                                                  */
    /*------------------------------------------------------------------------------------------------------*/

    /* Enable peripheral clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk |
                   CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR2CKEN_Msk | CLK_APBCLK0_TMR3CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_HIRC_DIV2 |
                   CLK_CLKSEL1_TMR0SEL_HIRC_DIV2 | CLK_CLKSEL1_TMR2SEL_PCLK1 | CLK_CLKSEL1_TMR3SEL_HIRC_DIV2;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* PB.0 is UART0_RX */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB0MFP_Msk;
    SYS->GPB_MFPL |= (1 << SYS_GPB_MFPL_PB0MFP_Pos);
    /* PB.1 is UART0_TX */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB1MFP_Msk;
    SYS->GPB_MFPL |= (1 << SYS_GPB_MFPL_PB1MFP_Pos);

    /* set PA.11 as TM0 */
    SYS->GPA_MFPH &= ~SYS_GPA_MFPH_PA11MFP_Msk;
    SYS->GPA_MFPH |= (5 << SYS_GPA_MFPH_PA11MFP_Pos);

    /* set PC.0 as TM2 */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC0MFP_Msk;
    SYS->GPC_MFPL |= (5 << SYS_GPC_MFPL_PC0MFP_Pos);

    /* set PB.2 as TM2_EXT */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB2MFP_Msk;
    SYS->GPB_MFPL |= (2 << SYS_GPB_MFPL_PB2MFP_Pos);

    /* set PB.5 as TM3 */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB5MFP_Msk;
    SYS->GPB_MFPL |= (5 << SYS_GPB_MFPL_PB5MFP_Pos);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32InitCount;
    uint32_t au32CAPValue[10], u32CAPDiff;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer2 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");
    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HIRC/2\n");
    printf("    - Time-out frequency is 1000 Hz    			\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HIRC/2\n");
    printf("    - Time-out frequency is 2 Hz    			\n");
    printf("    - Toggle-output mode and frequency is 1 Hz	\n");
    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HCLK              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect T0(PA.11) toggle-output pin to T2(PC.0) event counter pin.\n");
    printf("# Connect T3(PB.5) toggle-output pin to T2_EXT(PB.2) external capture pin.\n\n");

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);

    /* Start Timer0 counting and output T0 frequency to 500 Hz*/
    TIMER0->CMP = (__HIRC_DIV2 / 1000);
    TIMER0->CTL = TIMER_CTL_CNTEN_Msk | TIMER_TOGGLE_MODE;

    /* Start Timer3 counting and output T3 frequency to 1 Hz */
    TIMER3->CMP = __HIRC_DIV2 / 2;
    TIMER3->CTL = TIMER_CTL_CNTEN_Msk | TIMER_TOGGLE_MODE;

    /* case 1. */
    printf("# Period between two falling edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[2] = 0;

    /* Enable Timer2 event counter input and external capture function */
    TIMER2->CMP = 0xFFFFFF;
    TIMER2->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_CTL_EXTCNTEN_Msk | TIMER_CONTINUOUS_MODE;
    TIMER2->EXTCTL = TIMER_EXTCTL_CAPEN_Msk | TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_FALLING_EDGE | TIMER_EXTCTL_CAPIEN_Msk;

    /* Check Timer2 capture trigger interrupt counts */
    while ((g_au32TMRINTCount[2] <= 10) && (u32InitCount < 10))
    {
        if (g_au32TMRINTCount[2] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER2->CAP;

            if (u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount], u32CAPDiff);

                if (u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }

            u32InitCount = g_au32TMRINTCount[2];
        }
    }

    printf("*** PASS ***\n\n");

    /* case 2. */
    TIMER2->EXTCTL &= ~TIMER_EXTCTL_CAPEN_Msk;
    TIMER_Stop(TIMER2);

    while (TIMER_IS_ACTIVE(TIMER2));

    TIMER_ClearIntFlag(TIMER2);
    TIMER_ClearCaptureIntFlag(TIMER2);

    printf("# Get first low duration should be 250 counts.\n");
    printf("# And follows duration between two rising edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[2] = 0;

    /* Enable Timer2 event counter input and external capture function */
    TIMER2->CMP = 0xFFFFFF;
    TIMER2->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_CTL_EXTCNTEN_Msk | TIMER_CONTINUOUS_MODE;
    TIMER2->EXTCTL = TIMER_EXTCTL_CAPEN_Msk | TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_RISING_EDGE | TIMER_EXTCTL_CAPIEN_Msk;

    /* Check Timer2 capture trigger interrupt counts */
    while ((g_au32TMRINTCount[2] <= 10) && (u32InitCount < 10))
    {
        if (g_au32TMRINTCount[2] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER2);

            if (u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount], u32CAPDiff);

                if (u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }

            u32InitCount = g_au32TMRINTCount[2];
        }
    }

    /* Stop Timer0, Timer2 and Timer3 counting */
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER2);
    TIMER_Stop(TIMER3);

    printf("*** PASS ***\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
