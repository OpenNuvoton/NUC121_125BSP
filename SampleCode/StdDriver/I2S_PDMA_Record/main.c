/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    NUC121 SPI Driver Sample Code
 *           This is a I2S demo for recording data and demonstrate how I2S works with PDMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define I2S_TX_DMA_CH 1
#define I2S_RX_DMA_CH 2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN 16
#define TX_BUFF_LEN 32

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

DESC_TABLE_T g_asDescTable_TX[1], g_asDescTable_RX[2];

/* Function prototype declaration */
void SYS_Init(void);

/* Global variable declaration */
volatile uint8_t g_u8RxIdx = 0;
volatile uint32_t g_au32PcmRxBuff[2][BUFF_LEN] = {0};
volatile uint32_t g_au32PcmTxBuff[1][TX_BUFF_LEN] = {0};

/* Since ping-pong buffer would record infinitely, in this case we only want to make sure the first buffer of RX Buffer1 and Buffer2 are correct.
   Hence use  g_au32PcmRxBuff1 and g_u8Count to check and record the first buffer of RX Buffer1 and Buffer2*/
volatile uint32_t g_au32PcmRxBuff1[2][BUFF_LEN] = {0};
volatile uint8_t g_u8Count = 0;

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
    volatile int32_t i32TimeoutCount = SystemCoreClock;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+----------------------------------------------+\n");
    printf("|        I2S + PDMA  Record Sample Code        |\n");
    printf("+----------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x50005000, 0x50015001, ... \n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S0_LRCLK (PC00)\n      I2S0_BCLK(PC01)\n");
    printf("      I2S0_DI (PC02)\n      I2S0_DO (PC03)\n\n");
    printf("      This sample code will transmit and receive data with PDMA transfer.\n");
    printf("      Connect I2S_DI and I2S_DO to check if the data which stored in two receive\n buffers are the same with the transmitted values.\n");
    printf("      After PDMA transfer is finished, the received values will be printed.\n\n");

    /* Select PCLK as the clock source of SPI1 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable I2S TX and RX functions */
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    /* Master mode, 16-bit word width, stereo mode, I2S format. */
    I2S_Open(SPI0, I2S_MODE_MASTER, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);
    /* In I2S_Open() function, TX would be enabled first, this may cause RX get zero data since clk is working, disable it first */
    I2S_DISABLE_TX(SPI0);

    /* Data initiation */
    u32InitValue = 0x50005000;

    /* Here we use a TX Buffer with TX_BUFF_LEN size which is twice as much as RX Buffer size to be a playing source,
       to simulate recording case with ping-pong buffer, ie. RX record samples that are played by TX */
    printf("expected record samples:\n");

    for (u32DataCount = 0; u32DataCount < TX_BUFF_LEN; u32DataCount++)
    {
        g_au32PcmTxBuff[0][u32DataCount] = u32InitValue;
        u32InitValue += 0x00010001;
        printf("0x%8X\n", g_au32PcmTxBuff[0][u32DataCount]);
    }

    /* Enable PDMA channels */
    PDMA_Open((1 << I2S_TX_DMA_CH) | (1 << I2S_RX_DMA_CH));

    /* Tx description */
    PDMA_SetTransferCnt(1, PDMA_WIDTH_32, TX_BUFF_LEN - 1);
    PDMA_SetTransferAddr(1, (uint32_t)&g_au32PcmTxBuff[0], PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    PDMA_SetTransferMode(1, PDMA_SPI0_TX, FALSE, 0);
    PDMA_SetBurstType(1, PDMA_REQ_SINGLE, 0);

    /* Rx(Record) description */
    g_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[0].DA = (uint32_t)&g_au32PcmRxBuff[0];
    g_asDescTable_RX[0].FIRST = (uint32_t)&g_asDescTable_RX[1] - (PDMA->SCATBA);

    g_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[1].DA = (uint32_t)&g_au32PcmRxBuff[1];
    g_asDescTable_RX[1].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);   //link to first description

    PDMA_SetTransferMode(2, PDMA_SPI0_RX, 1, (uint32_t)&g_asDescTable_RX[0]);

    /* Enable PDMA channel 2 interrupt */
    PDMA_EnableInt(2, PDMA_INT_TRANS_DONE);

    NVIC_EnableIRQ(PDMA_IRQn);

    /* Clear RX FIFO */
    I2S_CLR_RX_FIFO(SPI0);

    /* Enable TX function and TX PDMA function */
    I2S_ENABLE_TX(SPI0);
    I2S_ENABLE_TXDMA(SPI0);

    // Enable RX function and RX PDMA function for receiving data
    I2S_ENABLE_RX(SPI0);

    // Reset timeout count for checking RX FIFO level
    i32TimeoutCount = SystemCoreClock;

    // Wait until the RX FIFO level is greater than 0 or timeout occurs
    while ((I2S_GET_RX_FIFO_LEVEL(SPI0) == 0) && (--i32TimeoutCount >= 0)) {}

    // Clear the RX FIFO to ensure no stale data remains
    I2S_CLR_RX_FIFO(SPI0);

    // Enable RX DMA function for receiving data via DMA
    I2S_ENABLE_RXDMA(SPI0);

    /* Once I2S is enabled, CLK would be sent immediately, we still have chance to get zero data at beginning,
       in this case we only need to make sure the on-going data is correct */
    printf("RX Buffer 1\tRX Buffer 2\n");

    for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        printf("0x%X\t\t0x%X\n", g_au32PcmRxBuff1[0][u32DataCount], g_au32PcmRxBuff1[1][u32DataCount]);
    }

    PDMA_Close();
    printf("\n\nExit I2S sample code.\n");

    while (1);

}

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

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Enable IP peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable PDMA peripheral clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    SYS->IPRST1 |= SYS_IPRST1_SPI0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_SPI0RST_Msk;

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

void PDMA_IRQHandler(void)
{
    uint32_t u32DataCount = 0;
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & 0x1)   /* abort */
    {
        if (PDMA_GET_ABORT_STS() & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (u32Status & 0x2)
    {
        if (PDMA_GET_TD_STS() & 0x4)            /* channel 2 done */
        {
            /* record the first buffer */
            if (g_u8Count == 0)
            {
                for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
                    g_au32PcmRxBuff1[0][u32DataCount] = g_au32PcmRxBuff[0][u32DataCount];
            }
            else if (g_u8Count == 1)
            {
                for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
                    g_au32PcmRxBuff1[1][u32DataCount] = g_au32PcmRxBuff[1][u32DataCount];
            }

            ++g_u8Count;
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetRxSGTable(g_u8RxIdx);
            g_u8RxIdx ^= 1;
        }

        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
