/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    NUC121 SPI Driver Sample Code
 *           This is a I2S demo for playing and recording data with PDMA function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define I2S_TX_DMA_CH   1
#define I2S_RX_DMA_CH   2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN        4
#define I2S_TIMEOUT     (SystemCoreClock >> 2)

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

DESC_TABLE_T g_asDescTable_TX[2], g_asDescTable_RX[2];

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);

/* Global variable declaration */
volatile uint8_t g_u8TxIdx = 0, g_u8RxIdx = 0;
volatile uint32_t g_u32PlayReady = 0, g_u32RecReady = 0;
uint32_t g_au32PcmRxBuff[2][BUFF_LEN] = {0};
uint32_t g_au32PcmTxBuff[2][BUFF_LEN] = {0};

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t u8Id)
{
    g_asDescTable_TX[u8Id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_TX[u8Id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t u8Id)
{
    g_asDescTable_RX[u8Id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_RX[u8Id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32InitValue, u32DataCount;
    volatile int32_t i32TimeoutCnt = I2S_TIMEOUT;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    printf("\n");
    printf("+----------------------------------------------+\n");
    printf("|    I2S + PDMA  Play and Record Sample Code   |\n");
    printf("+----------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX 1/2 value: 0x50005000/0xA000A000, 0x50015001/0xA001A001, ... \n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S0_LRCLK (PC00)\n      I2S0_BCLK(PC01)\n");
    printf("      I2S0_DI (PC02)\n      I2S0_DO (PC03)\n\n");
    printf("      This sample code will transmit then receive data with PDMA transfer.\n");
    printf("      Connect I2S_DI and I2S_DO to check if the transmitted data is the same\nwith the received data.\n");
    printf("      After PDMA transfer is finished, the transmitted and received values\nin each buffer will be printed.\n");

    /* Select PCLK as the clock source of SPI0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK0;

    /* Reset SPI/I2S */
    SYS->IPRST1 |= SYS_IPRST1_SPI0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_SPI0RST_Msk;

    /* Master mode, 16-bit word width, stereo mode, I2S format. */
    SPI0->I2SCTL = I2S_MODE_MASTER | I2S_DATABIT_16 | I2S_STEREO | I2S_FORMAT_I2S;
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    SPI0->I2SCLK = (SPI0->I2SCLK & ~SPI_I2SCLK_BCLKDIV_Msk) | (11 << SPI_I2SCLK_BCLKDIV_Pos);
    /* Enable I2S */
    SPI0->I2SCTL |= SPI_I2SCTL_I2SEN_Msk;

    /* Data initiation */
    u32InitValue = 0x50005000;

    for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        g_au32PcmTxBuff[0][u32DataCount] = u32InitValue;
        g_au32PcmTxBuff[1][u32DataCount] = u32InitValue + 0x50005000;
        u32InitValue += 0x00010001;
    }

    /* Enable PDMA channels */
    PDMA->DSCT[1].CTL = 0;
    PDMA->DSCT[2].CTL = 0;
    PDMA->CHCTL |= (1 << I2S_TX_DMA_CH) | (1 << I2S_RX_DMA_CH);

    /* Tx(Play) description */
    g_asDescTable_TX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[0].SA = (uint32_t)&g_au32PcmTxBuff[0];
    g_asDescTable_TX[0].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[0].FIRST = (uint32_t)&g_asDescTable_TX[1] - (PDMA->SCATBA);

    g_asDescTable_TX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[1].SA = (uint32_t)&g_au32PcmTxBuff[1];
    g_asDescTable_TX[1].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[1].FIRST = (uint32_t)&g_asDescTable_TX[0] - (PDMA->SCATBA);   //link to first description

    /* Rx(Record) description */
    g_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[0].DA = (uint32_t)&g_au32PcmRxBuff[0];
    g_asDescTable_RX[0].FIRST = (uint32_t)&g_asDescTable_RX[1] - (PDMA->SCATBA);

    g_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[1].DA = (uint32_t)&g_au32PcmRxBuff[1];
    g_asDescTable_RX[1].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);   //link to first description

    /* Configure PDMA transfer mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 8)) | (PDMA_SPI0_TX << 8);
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 16)) | (PDMA_SPI0_RX << 16);

    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[1].FIRST = (uint32_t)&g_asDescTable_TX[0] - (PDMA->SCATBA);

    PDMA->DSCT[2].CTL = (PDMA->DSCT[2].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[2].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA->INTEN |= (1 << 1) | (1 << 2);

    NVIC_EnableIRQ(PDMA_IRQn);

    /* Clear RX FIFO */
    I2S_CLR_RX_FIFO(SPI0);

    /* Enable RX function and TX function */
    SPI0->I2SCTL |= (SPI_I2SCTL_RXEN_Msk | SPI_I2SCTL_TXEN_Msk);
    /* Enable RX PDMA and TX PDMA function */
    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);

    i32TimeoutCnt = I2S_TIMEOUT;

    while (g_u32PlayReady <= 2 || g_u32RecReady <= 2)
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }


    /* Disable RX function and TX function */
    SPI0->I2SCTL &= ~(SPI_I2SCTL_RXEN_Msk | SPI_I2SCTL_TXEN_Msk);

    /* Disable RX PDMA and TX PDMA function */
    SPI0->PDMACTL &= ~(SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);

    NVIC_DisableIRQ(PDMA_IRQn);


    /* Print the transmitted data */
    printf("\nTX Buffer 1\tTX Buffer 2\n");


    for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        printf("0x%X\t0x%X\n", g_au32PcmTxBuff[0][u32DataCount], g_au32PcmTxBuff[1][u32DataCount]);
    }

    g_u32PlayReady = 0;

    /* Print the received data */
    printf("\nRX Buffer 1\tRX Buffer 2\n");

    for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        printf("0x%X\t0x%X\n", g_au32PcmRxBuff[0][u32DataCount], g_au32PcmRxBuff[1][u32DataCount]);
    }

    g_u32RecReady = 0;

    PDMA->CHCTL = 0;
    printf("\n\nExit I2S sample code.\n");

    while (1);
}

void SYS_Init(void)
{
    volatile int32_t i32TimeoutCnt = I2S_TIMEOUT;

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

    /* Select HIRC/2 as the clock source of UART */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Enable peripheral clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_SPI0CKEN_Msk;

    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* PB.0 is UART0_RX */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB0MFP_Msk;
    SYS->GPB_MFPL |= (1 << SYS_GPB_MFPL_PB0MFP_Pos);
    /* PB.1 is UART0_TX */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB1MFP_Msk;
    SYS->GPB_MFPL |= (1 << SYS_GPB_MFPL_PB1MFP_Pos);

    /* PC.0 is I2S_LRCLK */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC0MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC0MFP_Pos);
    /* PC.1 is I2S_BCLK */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC1MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC1MFP_Pos);
    /* PC.2 is I2S_DI */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC2MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC2MFP_Pos);
    /* PC.3 is I2S_DO */
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

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & 0x1)   /* abort */
    {
        if (PDMA_GET_ABORT_STS() & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF1_Msk);

        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (u32Status & 0x2)
    {
        if (PDMA_GET_TD_STS() & 0x2)            /* channel 1 done */
        {
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetTxSGTable(g_u8TxIdx);
            g_u8TxIdx ^= 1;
            g_u32PlayReady ++;
        }

        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF1_Msk);

        if (PDMA_GET_TD_STS() & 0x4)            /* channel 2 done */
        {
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetRxSGTable(g_u8RxIdx);
            g_u8RxIdx ^= 1;
            g_u32RecReady ++;
        }

        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
