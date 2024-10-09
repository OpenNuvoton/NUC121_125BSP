/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * @brief    Configure SPI0 as Slave mode and demonstrate how to communicate
 *           with an off-chip SPI Master device with FIFO mode. This sample
 *           code needs to work with SPI_MasterFifoMode sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT          16
#define SPI_TIMEOUT         (SystemCoreClock >> 2)

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    volatile uint32_t u32TxDataCount, u32RxDataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART for print message */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|           SPI Slave Mode Sample Code (NUC121 DN/DE only)               |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0:\n");
    printf("    SPI0_SS (PC00), SPI0_CLK (PC01)\n");
    printf("    SPI0_MISO (PC02), SPI0_MOSI (PC03)\n");
    printf("SPI controller will enable FIFO mode and transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for (u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32TxDataCount] = 0x00AA0000 + u32TxDataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.\n");
    getchar();
    printf("\n");

    /* Access TX and RX FIFO */
    while (u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if (((SPI0->STATUS & SPI_STATUS_TXFULL_Msk) == 0) && (u32TxDataCount < TEST_COUNT))
            SPI0->TX = g_au32SourceData[u32TxDataCount++]; /* Write to TX FIFO */

        /* Check RX EMPTY flag */
        if ((SPI0->STATUS & SPI_STATUS_RXEMPTY_Msk) == 0)
            g_au32DestinationData[u32RxDataCount++] = SPI0->RX; /* Read RX FIFO */
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32RxDataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 peripheral clock */
    CLK->APBCLK0 &= (~CLK_APBCLK0_SPI0CKEN_Msk);

    while (1);
}

void SYS_Init(void)
{
    volatile int32_t i32TimeoutCnt = SPI_TIMEOUT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /*------------------------------------------------------------------------------------------------------*/
    /* Enable Module Clock                                                                                  */
    /*------------------------------------------------------------------------------------------------------*/
    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Select PCLK0 as the clock source of SPI0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK0;

    /* Enable UART0 and SPI0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_SPI0CKEN_Msk;

    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

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

    /* PC.0 is SPI_SS */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC0MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC0MFP_Pos);
    /* PC.1 is SPI_CLK */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC1MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC1MFP_Pos);
    /* PC.2 is SPI_MISO */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC2MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC2MFP_Pos);
    /* PC.3 is SPI_MOSI */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC3MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC3MFP_Pos);
}

void UART_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* UART peripheral clock rate 12MHz; UART bit rate 115200 bps. */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI0->CTL = SPI_SLAVE | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Configure SPI0 as a low level active device. */
    SPI0->SSCTL = 0;
    /* Set IP clock divider. SPI peripheral clock rate = f_PCLK0 */
    SPI0->CLKDIV = 0;
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
