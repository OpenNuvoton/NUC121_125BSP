/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate SPI data transfer with PDMA.
 *           SPI0 will be configured as Master mode.
 *           Both TX PDMA function and RX PDMA function will be enabled.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SPI_MASTER_TX_DMA_CH    0
#define SPI_MASTER_RX_DMA_CH    1
#define SPI_SLAVE_TX_DMA_CH     2
#define SPI_SLAVE_RX_DMA_CH     3

#define TEST_COUNT              64
#define SPI_TIMEOUT             (SystemCoreClock >> 2)

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);
void SPI_LoopTestWithPDMA(void);

/* Global variable declaration */
uint32_t g_au32MasterToSlaveTestPattern[TEST_COUNT];
uint32_t g_au32SlaveToMasterTestPattern[TEST_COUNT];
uint32_t g_au32MasterRxBuffer[TEST_COUNT];
uint32_t g_au32SlaveRxBuffer[TEST_COUNT];

int main(void)
{
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
    printf("+--------------------------------------------------------------+\n");
    printf("|                  SPI + PDMA Sample Code                      |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master and SPI1 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0 loopback:\n");
    printf("    SPI0_SS (PC00), SPI0_CLK (PC01)\n");
    printf("    SPI0_MISO (PC02) <--> SPI0_MOSI (PC03)\n");
    printf("Please press any key to start transmission ...");
    printf("\n");
    getchar();


    SPI_LoopTestWithPDMA();

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 and SPI1 peripheral clock */
    CLK->APBCLK0 &= ~(CLK_APBCLK0_SPI0CKEN_Msk);

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
    /* Configure SPI0 */
    /* Configure SPI0 as a master, clock idle low, TX on falling clock edge, RX on rising edge and 32-bit transaction. */
    SPI0->CTL = SPI_MASTER | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI0->SSCTL = SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk;
    /* Set SPI0 clock divider. SPI clock rate = PCLK / (9+1) */
    SPI0->CLKDIV = (SPI0->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (9 << SPI_CLKDIV_DIVIDER_Pos);
}

void SPI_LoopTestWithPDMA(void)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;


    printf("\nSPI0/1 Loop test with PDMA ");

    /* Source data initiation */
    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        g_au32MasterToSlaveTestPattern[u32DataCount] = 0x55000000 | (u32DataCount + 1);

    /* SPI master PDMA TX channel configuration */
    PDMA->CHCTL |= (1 << SPI_MASTER_TX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128   | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE  | /* Single request type */
        PDMA_OP_BASIC;     /* Basic mode */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].SA = (uint32_t)g_au32MasterToSlaveTestPattern;
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].DA = (uint32_t)&SPI0->TX;
#if(SPI_MASTER_TX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (SPI_MASTER_TX_DMA_CH % 4))))) |
                      ((0 + 16) << (8 * (SPI_MASTER_TX_DMA_CH % 4))); /* SPIx_TX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (SPI_MASTER_TX_DMA_CH % 4))))) |
                    ((0 + 16) << (8 * (SPI_MASTER_TX_DMA_CH % 4))); /* SPIx_TX */
#endif

    /* SPI master PDMA RX channel configuration */
    PDMA->CHCTL |= (1 << SPI_MASTER_RX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].SA = (uint32_t)&SPI0->RX;
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].DA = (uint32_t)g_au32MasterRxBuffer;
#if(SPI_MASTER_RX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (SPI_MASTER_RX_DMA_CH % 4))))) |
                      ((0 + 17) << (8 * (SPI_MASTER_RX_DMA_CH % 4))); /* SPIx_RX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (SPI_MASTER_RX_DMA_CH % 4))))) |
                    ((0 + 17) << (8 * (SPI_MASTER_RX_DMA_CH % 4))); /* SPIx_RX */
#endif

    /* Enable SPI master DMA function */
    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);

    i32Err = 0;

    for (u32TestCycle = 0; u32TestCycle < 10000; u32TestCycle++)
    {
        if ((u32TestCycle & 0x1FF) == 0)
        {
            putchar('.');
            fflush(stdout);
        }

        while (1)
        {
            u32RegValue = PDMA->INTSTS;

            /* Check the DMA transfer done interrupt flag */
            if (u32RegValue & PDMA_INTSTS_TDIF_Msk)
            {
                if ((PDMA->TDSTS & ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))) ==
                        ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH)))
                {
                    /* Clear the DMA transfer done flag */
                    PDMA->TDSTS = ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH));
                    /* Disable SPI master's DMA transfer function */
                    SPI0->PDMACTL = 0;

                    /* Check the transfer data */
                    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                    {
                        if (g_au32MasterToSlaveTestPattern[u32DataCount] != g_au32MasterRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                    }

                    if (u32TestCycle >= 10000)
                        break;

                    /* Source data initiation */
                    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                        g_au32MasterToSlaveTestPattern[u32DataCount]++;

                    /* Re-trigger */
                    /* Master PDMA TX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_MASTER_TX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Master PDMA RX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_MASTER_RX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Enable master's DMA transfer function */
                    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);
                    break;
                }
            }

            /* Check the DMA transfer abort interrupt flag */
            if (u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA->ABTSTS;
                /* Clear the target abort flag */
                PDMA->ABTSTS = u32Abort;
                i32Err = 1;
                break;
            }

            /* Check the DMA time-out interrupt flag */
            if (u32RegValue & 0x00000300)
            {
                /* Clear the time-out flag */
                PDMA->INTSTS = u32RegValue & 0x00000300;
                i32Err = 1;
                break;
            }
        }

        if (i32Err)
            break;
    }

    /* Disable PDMA clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_PDMACKEN_Msk;

    if (i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
