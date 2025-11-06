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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /*------------------------------------------------------------------------------------------------------*/
    /* Enable Module Clock                                                                                  */
    /*------------------------------------------------------------------------------------------------------*/

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC_DIV2, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

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

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
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

    printf("# This sample code use Timer1 to check whether the TIMER_Delay API delay time of Timer0 is reasonable or not.\n");
    printf("# Delay time includes 100 ms, 200 ms, 300 ms, 400 ms and 500 ms.\n\n");

    /* Start Timer1 to measure delay period of TIMER_Delay API is reasonable or not */
    /* since we choose HIRC/2 as Timer1 source, timer1 clk = 24MHz/240 = 100000Hz */
    TIMER1->CTL = TIMER_PERIODIC_MODE | (240 - 1);
    TIMER1->CMP = 0xFFFFFF;
    TIMER_Start(TIMER1);
    /* delay 100000us */
    TIMER_Delay(TIMER0, 100000);
    TIMER_Stop(TIMER1);

    /* timer clk = 100000HZ = 100 times/ms */
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-1 is %u ms .... ", u32DelayTime);

    if (u32DelayTime == 100)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    /* reset timer1 */
    TIMER1->CMP = 0xFFFFFF;
    TIMER_Start(TIMER1);
    /* delay 200000us */
    TIMER_Delay(TIMER0, 200000);
    TIMER_Stop(TIMER1);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-2 is %u ms .... ", u32DelayTime);

    if (u32DelayTime == 200)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    /* reset timer1 */
    TIMER1->CMP = 0xFFFFFF;
    TIMER_Start(TIMER1);
    /* delay 300000us */
    TIMER_Delay(TIMER0, 300000);
    TIMER_Stop(TIMER1);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-3 is %u ms .... ", u32DelayTime);

    if (u32DelayTime == 300)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    /* reset timer1 */
    TIMER1->CMP = 0xFFFFFF;
    TIMER_Start(TIMER1);
    /* delay 400000us */
    TIMER_Delay(TIMER0, 400000);
    TIMER_Stop(TIMER1);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-4 is %u ms .... ", u32DelayTime);

    if (u32DelayTime == 400)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    /* reset timer1 */
    TIMER1->CMP = 0xFFFFFF;
    TIMER_Start(TIMER1);
    /* delay 500000us */
    TIMER_Delay(TIMER0, 500000);
    TIMER_Stop(TIMER1);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 100;
    printf("    Check DelayTime-5 is %u ms .... ", u32DelayTime);

    if (u32DelayTime == 500)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    printf("\n*** Check TIMER_Delay API delay time done ***\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
