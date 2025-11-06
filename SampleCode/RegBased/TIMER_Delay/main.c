/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use timer0 to create various delay time.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

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
                   CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_HIRC_DIV2 |
                   CLK_CLKSEL1_TMR0SEL_PCLK0 | CLK_CLKSEL1_TMR1SEL_HIRC_DIV2;

    /* Update System Core Clock */
    PllClock        = 0;                 // PLL
    SystemCoreClock = __HIRC;            // HCLK
    CyclesPerUs     = __HIRC / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* PB.0 is UART0_RX */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB0MFP_Msk;
    SYS->GPB_MFPL |= (1 << SYS_GPB_MFPL_PB0MFP_Pos);
    /* PB.1 is UART0_TX */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB1MFP_Msk;
    SYS->GPB_MFPL |= (1 << SYS_GPB_MFPL_PB1MFP_Pos);
}

uint32_t CLK_GetPCLK0Freq(void)
{
    SystemCoreClockUpdate();

    if (CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk)
        return SystemCoreClock >> 1;
    else
        return SystemCoreClock;
}

uint32_t CLK_GetPCLK1Freq(void)
{
    SystemCoreClockUpdate();

    if (CLK->CLKSEL0 & CLK_CLKSEL0_PCLK1SEL_Msk)
        return SystemCoreClock >> 1;
    else
        return SystemCoreClock;
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

uint32_t TIMER_GetModuleClock(TIMER_T *timer)
{
    uint32_t u32Src, u32Clk;
    const uint32_t au32Clk[] = {__HXT, __LXT, 0, 0, 0, __LIRC, 0, __HIRC_DIV2};

    if (timer == TIMER0)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR0SEL_Msk) >> CLK_CLKSEL1_TMR0SEL_Pos;
    else if (timer == TIMER1)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR1SEL_Msk) >> CLK_CLKSEL1_TMR1SEL_Pos;
    else if (timer == TIMER2)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR2SEL_Msk) >> CLK_CLKSEL1_TMR2SEL_Pos;
    else if (timer == TIMER3)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR3SEL_Msk) >> CLK_CLKSEL1_TMR3SEL_Pos;
    else
        return 0;

    if (u32Src == 2UL)
    {
        if ((timer == TIMER0) || (timer == TIMER1))
        {
            u32Clk = CLK_GetPCLK0Freq();
        }
        else
        {
            u32Clk = CLK_GetPCLK1Freq();
        }
    }
    else
    {
        u32Clk = au32Clk[u32Src];
    }

    return u32Clk;
}

int32_t TIMER_Delay(TIMER_T *timer, uint32_t u32Usec)
{
    uint32_t u32Clk = TIMER_GetModuleClock(timer);
    uint32_t u32Prescale = 0UL, u32Delay;
    uint32_t u32Cmpr, u32Cntr, u32NsecPerTick, i = 0UL;

    // Clear current timer configuration/
    timer->CTL = 0;
    timer->EXTCTL = 0;

    if (u32Clk <= 1000000)   // min delay is 1000 us if timer clock source is <= 1 MHz
    {
        if (u32Usec < 1000)
            u32Usec = 1000;

        if (u32Usec > 1000000)
            u32Usec = 1000000;
    }
    else
    {
        if (u32Usec < 100)
            u32Usec = 100;

        if (u32Usec > 1000000)
            u32Usec = 1000000;
    }

    if (u32Clk <= 1000000)
    {
        u32Prescale = 0;
        u32NsecPerTick = 1000000000 / u32Clk;
        u32Cmpr = (u32Usec * 1000) / u32NsecPerTick;
    }
    else
    {
        if (u32Clk > 64000000)
        {
            u32Prescale = 7;    // real prescaler value is 8
            u32Clk >>= 3;
        }
        else if (u32Clk > 32000000)
        {
            u32Prescale = 3;    // real prescaler value is 4
            u32Clk >>= 2;
        }
        else if (u32Clk > 16000000)
        {
            u32Prescale = 1;    // real prescaler value is 2
            u32Clk >>= 1;
        }

        if (u32Usec < 250)
        {
            u32Cmpr = (u32Usec * u32Clk) / 1000000;
        }
        else if (u32Clk % 1000000 == 0)
        {
            u32Cmpr = (u32Clk / 1000000) * u32Usec;
        }
        else
        {
            u32NsecPerTick = 1000000000 / u32Clk;
            u32Cmpr = (u32Usec * 1000) / u32NsecPerTick;
        }
    }

    timer->CMP = u32Cmpr;
    timer->CTL = TIMER_CTL_CNTEN_Msk | TIMER_ONESHOT_MODE | u32Prescale;

    /* When system clock is faster than timer clock, it is possible timer active bit cannot set
       in time while we check it. And the while loop below return immediately, so put a tiny
       delay larger than 1 ECLK here allowing timer start counting and raise active flag. */
    for (u32Delay = (SystemCoreClock / u32Clk) + 1UL; u32Delay > 0UL; u32Delay--)
    {
        __NOP();
    }

    while (timer->CTL & TIMER_CTL_ACTSTS_Msk)
    {
        /* Bailed out if timer stop counting e.g. Some interrupt handler close timer clock source. */
        if (u32Cntr == timer->CNT)
        {
            if (i++ > u32Delay)
            {
                return TIMER_TIMEOUT_ERR;
            }
        }
        else
        {
            i = 0;
            u32Cntr = timer->CNT;
        }
    }

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32DelayTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------+\n");
    printf("|    Timer Delay API Sample Code    |\n");
    printf("+-----------------------------------+\n\n");

    printf("# This sample code is using Timer1 to check Timer0 TIMER_Delay API delay time is reasonable or not.\n");
    printf("# Delay time includes 100 ms, 200 ms, 300 ms, 400 ms and 500 ms.\n\n");

    /* Start Timer1 to measure delay period of TIMER_Delay API is reasonable or not */
    /* since we choose HIRC/2 as Timer1 source, timer1 clk = 24MHz/240 = 100000Hz */
    TIMER1->CTL = TIMER_PERIODIC_MODE | (240 - 1);
    TIMER1->CMP = 0xFFFFFF;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;
    /* delay 100000us */
    TIMER_Delay(TIMER0, 100000);
    /* Stop Timer1 */
    TIMER1->CTL &= ~TIMER_CTL_CNTEN_Msk;
    /* timer clk = 100000HZ = 100 times/ms */
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-1 is %d ms .... ", u32DelayTime);

    if (u32DelayTime == 100)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    /* reset timer1 */
    TIMER1->CMP = 0xFFFFFF;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;
    /* delay 200000us */
    TIMER_Delay(TIMER0, 200000);
    /* Stop Timer1 */
    TIMER1->CTL &= ~TIMER_CTL_CNTEN_Msk;
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-2 is %d ms .... ", u32DelayTime);

    if (u32DelayTime == 200)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    /* reset timer1 */
    TIMER1->CMP = 0xFFFFFF;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;
    /* delay 300000us */
    TIMER_Delay(TIMER0, 300000);
    /* Stop Timer1 */
    TIMER1->CTL &= ~TIMER_CTL_CNTEN_Msk;
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-3 is %d ms .... ", u32DelayTime);

    if (u32DelayTime == 300)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    /* reset timer1 */
    TIMER1->CMP = 0xFFFFFF;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;
    /* delay 400000us */
    TIMER_Delay(TIMER0, 400000);
    /* Stop Timer1 */
    TIMER1->CTL &= ~TIMER_CTL_CNTEN_Msk;
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-4 is %d ms .... ", u32DelayTime);

    if (u32DelayTime == 400)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    /* reset timer1 */
    TIMER1->CMP = 0xFFFFFF;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;
    /* delay 500000us */
    TIMER_Delay(TIMER0, 500000);
    /* Stop Timer1 */
    TIMER1->CTL &= ~TIMER_CTL_CNTEN_Msk;
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-5 is %d ms .... ", u32DelayTime);

    if (u32DelayTime == 500)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    printf("\n*** Check TIMER_Delay API delay time done ***\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
