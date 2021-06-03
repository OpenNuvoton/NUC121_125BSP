/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use WDT time-out interrupt event to wake-up system.

 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint8_t g_u8IsWDTWakeupINT = 0;


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power-down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    printf("System enter to power-down mode.\n\n");

    /* To check if all the debug messages are finished */
    while (IsDebugFifoEmpty() == 0);

    SCB->SCR = 4;

    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PWRCTL = (CLK->PWRCTL & ~(CLK_PWRCTL_PDEN_Msk)) | CLK_PWRCTL_PDWKIEN_Msk;
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    __WFI();
}

/**
 * @brief       IRQ Handler for WDT and WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT and WWDT.
 */
void WDT_IRQHandler(void)
{
    if ((WDT_GET_TIMEOUT_INT_FLAG() == 1) && (WDT_GET_TIMEOUT_WAKEUP_FLAG() == 1))
    {
        /* Clear WDT time-out interrupt and wake-up flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();

        g_u8IsWDTWakeupINT = 1;

        printf("WDT time-out wake-up interrupt occurred.\n");
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

    /* Select HCLK clock source as HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Enable LIRC */
    CLK->PWRCTL |= CLK_PWRCTL_LIRCEN_Msk;

    /* Waiting for clock ready */
    while (!(CLK->STATUS & CLK_STATUS_LIRCSTB_Msk));

    /* Enable peripheral clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_WDTCKEN_Msk ;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_HIRC_DIV2 | CLK_CLKSEL1_WDTSEL_LIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
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
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("CPU @ %u Hz\n", SystemCoreClock);
    printf("+----------------------------------------------+\n");
    printf("|    WDT Power-down and Wake-up Sample Code    |\n");
    printf("+----------------------------------------------+\n\n");

    printf("# WDT Settings:\n");
    printf("    - Clock source is 10 kHz                \n");
    printf("    - Time-out interval is 2^14 * WDT clock \n");
    printf("      (around 1.6384 second)                \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Wake-up function enable               \n");
    printf("# System will generate a WDT time-out interrupt event after around 1.6384 second, \n");
    printf("    and will be wake up from power-down mode also.\n");
    printf("    (Use PA.12 low period time to check WDT time-out interval)\n\n");

    /* Use PA.12 to check time-out period time */
    PA->MODE = 0xFDFFFFFF;
    PA12 = 1;
    PA12 = 0;

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Enable WDT wake-up function and select time-out interval to 2^14 * WDT clock then start WDT counting */
    g_u8IsWDTWakeupINT = 0;
    WDT->CTL = WDT_TIMEOUT_2POW14 | WDT_CTL_INTEN_Msk | WDT_CTL_WKEN_Msk | WDT_CTL_WDTEN_Msk ;

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* System entry into Power-down Mode */
    PowerDownFunction();

    /* Check if WDT time-out interrupt and wake-up interrupt flag occurred */
    while (1)
    {
        if (((CLK->PWRCTL & CLK_PWRCTL_PDWKIF_Msk) == CLK_PWRCTL_PDWKIF_Msk) && (g_u8IsWDTWakeupINT == 1))
            break;
    }

    PA12 = 1;

    printf("System has been wake-up done.\n");

    /* Clear Power-down wake-up interrupt flag */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;

    /* Disable WDT counting */
    WDT_Close();

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
