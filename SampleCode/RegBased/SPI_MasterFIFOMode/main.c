/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Configure SPI0 as Master mode and demonstrate how to communicate
 *           with an off-chip SPI Slave device with FIFO mode. This sample
 *           code needs to work with SPI_SlaveFifoMode sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define TEST_COUNT 16
#define SPI_TIMEOUT (SystemCoreClock >> 2)

//------------------------------------------------------------------------------
uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

//------------------------------------------------------------------------------
/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount;
    volatile int32_t i32TimeoutCnt = SPI_TIMEOUT;

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
    printf("|             SPI Master Mode Sample Code (NUC121 DN/DE only)          |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0:\n");
    printf("    SPI0_SS (PC00), SPI0_CLK (PC01)\n");
    printf("    SPI0_MISO (PC02), SPI0_MOSI (PC03)\n");
    printf("SPI controller will enable FIFO mode and transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The SPI master configuration is ready.\n");

    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x00550000 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.\n");
    getchar();
    printf("\n");

    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFOCTL = (2 << SPI_FIFOCTL_TXTH_Pos) | SPI_FIFOCTL_RXTOIEN_Msk | SPI_FIFOCTL_TXTHIEN_Msk;
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(SPI0_IRQn);

    i32TimeoutCnt = SPI_TIMEOUT;

    /* Wait for transfer done */
    while (g_u32RxDataCount < TEST_COUNT)
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }

    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFOCTL = 0;
    NVIC_DisableIRQ(SPI0_IRQn);
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

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI0->CTL = SPI_MASTER | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI0->SSCTL = SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk;
    /* Set IP clock divider. SPI clock rate = f_PCLK0 / (5+1) */
    SPI0->CLKDIV = (SPI0->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | 5;
}

void SPI0_IRQHandler(void)
{
    /* Check RX EMPTY flag */
    while ((SPI0->STATUS & SPI_STATUS_RXEMPTY_Msk) == 0)
    {
        /* Read RX FIFO */
        g_au32DestinationData[g_u32RxDataCount++] = SPI0->RX;
    }

    /* Check TX FULL flag and TX data count */
    while (((SPI0->STATUS & SPI_STATUS_TXFULL_Msk) == 0) && (g_u32TxDataCount < TEST_COUNT))
    {
        /* Write to TX FIFO */
        SPI0->TX = g_au32SourceData[g_u32TxDataCount++];
    }

    if (g_u32TxDataCount >= TEST_COUNT)
        SPI0->FIFOCTL &= (~SPI_FIFOCTL_TXTHIEN_Msk); /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if (SPI0->STATUS & SPI_FIFOCTL_RXTOIEN_Msk)
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while ((SPI0->STATUS & SPI_STATUS_RXEMPTY_Msk) == 0)
            g_au32DestinationData[g_u32RxDataCount++] = SPI0->RX;
    }
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
