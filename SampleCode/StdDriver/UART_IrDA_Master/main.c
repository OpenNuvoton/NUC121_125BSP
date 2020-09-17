/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data in UART IrDA mode.
 *           This sample code needs to work with UART_IrDA_Slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTxTest(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Transmit Test                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTxTest()
{
    uint8_t u8OutChar;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                          _______  |\n");
    printf("| |      |                                        |       | |\n");
    printf("| |Master|--UART0_TXD(PC.5) <==> UART0_RXD(PC.4)--|Slave  | |\n");
    printf("| |      |                                        |       | |\n");
    printf("| |______|                                        |_______| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Test                                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send data based on    |\n");
    printf("|    terminal input and Slave will printf received data on  |\n");
    printf("|    terminal screen.                                       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        UUART0 is set to debug port and connect with PC firstly.
        The IrDA sample code needs two module board to execute.
        Set the master board is IrDA TX Mode and the other is IrDA Rx mode.
        Inputting char on terminal will be sent to the UUART0 of master.
        After the master receiving, the inputting char will send to UUART0 again.
        At the same time, it also sends to UART0 TX pin by IrDA mode.
        Slave will print received char after UART0 send out.
        Note that IrDA mode is ONLY used when baud rate equation is selected mode 0.

    */

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Mode Test (Master)                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Input char by UART0 terminal.                         |\n");
    printf("| 2). UART0 will send a char according to step 1.           |\n");
    printf("| 3). Return step 1. (Press '0' to exit)                    |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\nIrDA Sample Code Start. \n");

    /* Set IrDA Tx Mode, Baud Rate configuration must be used MODE0 */
    UART_SelectIrDAMode(UART0, 57600, UART_IRDA_TXEN);

    /* Wait Terminal input to send data to UART0 TX pin */
    do
    {
        u8OutChar = getchar();
        printf("   Input: %c , Send %c out\n", u8OutChar, u8OutChar);
        UART_WRITE(UART0, u8OutChar);
    } while (u8OutChar != '0');

    printf("\nIrDA Sample Code End.\n");

}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable USCI module clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PC multi-function pins for UART0 RXD(PC.4) and TXD(PC.5) */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_UART0_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_UART0_TXD;

    /* Set PB multi-function pins for USCI0_DAT0(PB.4) and USCI0_DAT1(PB.5) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_USCI0_DAT0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_USCI0_DAT1;

}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(USCI0_RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for IrDA */
    UART0_Init();

    /* Init USCI0 for printf */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART sample IrDA Master function */
    IrDA_FunctionTxTest();

    while (1);

}

