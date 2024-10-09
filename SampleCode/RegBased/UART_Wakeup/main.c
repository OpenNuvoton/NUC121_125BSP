/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up system from Power-down mode by UART interrupt.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define UART_TIMEOUT            (SystemCoreClock >> 2)

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void);
void UART_CTSWakeUp(void);
void UART_PowerDown_TestItem(void);
void UART_PowerDownWakeUpTest(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UUART_WAIT_TX_EMPTY(UUART0);

    /* Set the processor is deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled*/
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

}

void SYS_Init(void)
{
    volatile int32_t i32TimeoutCnt = UART_TIMEOUT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Enable UART and USCI module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;
    CLK->APBCLK1 |= CLK_APBCLK1_USCI0CKEN_Msk;

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC_DIV2;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk)) | CLK_CLKDIV0_UART(1);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PC multi-function pins for UART0 RXD(PC.4) and TXD(PC.5) */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_UART0_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_UART0_TXD;

    /* Set PC multi-function pins for UART0 RTS(PC.3) and CTS(PC.2) */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_UART0_nRTS;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk)) | SYS_GPC_MFPL_PC2MFP_UART0_nCTS;

    /* Set PB multi-function pins for USCI0_DAT0(PB.4) and USCI0_DAT1(PB.5) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_USCI0_DAT0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_USCI0_DAT1;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 9600);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS->IPRST2 |=  SYS_IPRST2_USCI0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_USCI0RST_Msk;

    /* Configure USCI0 as UART mode */
    UUART0->CTL = (2 << UUART_CTL_FUNMODE_Pos);                                 /* Set UART function mode */
    UUART0->LINECTL = UUART_WORD_LEN_8 | UUART_LINECTL_LSB_Msk;                 /* Set UART line configuration */
    UUART0->DATIN0 = (2 << UUART_DATIN0_EDGEDET_Pos);                           /* Set falling edge detection */
    UUART0->BRGEN = ((34 << UUART_BRGEN_CLKDIV_Pos) | (5 << UUART_BRGEN_DSCNT_Pos) |
                     (1 << UUART_BRGEN_PDSCNT_Pos));                             /* Set UART baud rate as 115200bps */
    UUART0->PROTCTL |= UUART_PROTCTL_PROTEN_Msk;                                /* Enable UART protocol */
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for test */
    UART0_Init();

    /* Init USCI0 for printf */
    USCI0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART Power-down and Wake-up sample function */
    UART_PowerDownWakeUpTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntSts = UART0->INTSTS;
    uint32_t u32Data;
    volatile int32_t i32TimeoutCnt = UART_TIMEOUT;

    if (u32IntSts & UART_INTSTS_WKINT_Msk)              /* UART wake-up interrupt flag */
    {
        uint32_t u32WkSts = UART0->WKSTS;

        UART0->WKSTS = u32WkSts;
        printf("UART wake-up.\n");
        UUART_WAIT_TX_EMPTY(UUART0);
    }
    else if (u32IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) /* UART receive data available flag */
    {
        i32TimeoutCnt = UART_TIMEOUT;

        while ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            u32Data = UART0->DAT;
            printf("Data: 0x%X\n", u32Data);

            if (--i32TimeoutCnt <= 0)
            {
                break;
            }
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_CTSWakeUp(void)
{
    /* Enable UART nCTS wake-up frunction */
    UART0->WKCTL |= UART_WKCTL_WKCTSEN_Msk;

    printf("System enter to Power-down mode.\n");
    printf("Toggle UART0 nCTS to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void)
{
    /* Enable UART data wake-up frunction */
    UART0->WKCTL |= UART_WKCTL_WKDATEN_Msk;

    /* Set UART data wake-up start bit compensation value */
    UART0->DWKCOMP = 488;

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to UART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Power-down and wake-up test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    /* Enable UART wake-up and receive data available interrupt */
    UART0->INTEN |= (UART_INTEN_WKIEN_Msk | UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART0_IRQn);

    UART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);

    switch (u32Item)
    {
        case '1':
            UART_CTSWakeUp();
            break;

        case '2':
            UART_DataWakeUp();
            break;

        default:
            break;
    }

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n\n");
    getchar();

    /* Disable UART wake-up function */
    UART0->WKCTL = 0;

    /* Disable UART Interrupt */
    UART0->INTEN = 0;
    NVIC_DisableIRQ(UART0_IRQn);

    printf("UART Sample Program End.\n");
}
