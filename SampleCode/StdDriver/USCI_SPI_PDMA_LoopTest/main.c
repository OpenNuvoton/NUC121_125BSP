/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief   Demonstrate USCI SPI data transfer with PDMA.
 *          USCI_SPI0 (USPI0) will be configured as Master mode and transfer data loopback to self by connect MOSI pin to MISO pin.
 *          Both TX PDMA function and RX PDMA function will be enabled.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT  64
#define TEST_CYCLE  10000

/*----------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                               */
/*----------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void USCI_SPI_Init(void);
void USCI_SPI_LoopTestWithPDMA(void);
void UART0_Init(void);

/*----------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
#define SPI_MASTER_TX_DMA_CH    0
#define SPI_MASTER_RX_DMA_CH    1

uint32_t g_au32MasterToSlaveTestPattern[TEST_COUNT];
uint32_t g_au32MasterRxBuffer[TEST_COUNT];


int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Initiate UART0 to 115200-8n1 for print message */
    UART0_Init();

    /*------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                          */
    /*------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Initiate USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                  SPI + PDMA Sample Code                      |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a master.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0 loop back:\n");
    printf("    USCI_SPI0_SS (PC.1) <--> NULL\n    USCI_SPI0_CLK (PC.0) <--> NULL\n");
    printf("    USCI_SPI0_MISO (PC.2) <--> USCI_SPI0_MOSI (PC.3)\n\n");
    printf("Please connect USPI0_MISO with USPI0_MOSI before starting the test ...\n");
    printf("Press any key to start transmission ...");
    printf("\n");
    getchar();


    USCI_SPI_LoopTestWithPDMA();

    printf("\n\nExit SPI driver sample code.\n");

    /* Close USPI0 */
    USPI_Close(USPI0);

    while (1);
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

    /*------------------------------------------------------------------------------------------------------*/
    /* Enable Module Clock                                                                                  */
    /*------------------------------------------------------------------------------------------------------*/
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Enable USCI0 clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initiate I/O Multi-function                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set USCI_SPI0 multi-function pins */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk))  | (SYS_GPC_MFPL_PC1MFP_USCI0_CTL0);   /* PC.1  */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk))  | (SYS_GPC_MFPL_PC0MFP_USCI0_CLK);    /* PC.0  */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk))  | (SYS_GPC_MFPL_PC3MFP_USCI0_DAT0);   /* PC.3  */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk))  | (SYS_GPC_MFPL_PC2MFP_USCI0_DAT1);   /* PC.2  */
}

void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Initiate USCI_SPI0                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a master, USCI_SPI0 clock rate 2 MHz,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 16, 2000000);
    /* Enable the automatic hardware slave selection function and configure USCI_SPI_SS pin as low-active. */
    USPI_EnableAutoSS(USPI0, 0, USPI_SS_ACTIVE_LOW);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void USCI_SPI_LoopTestWithPDMA(void)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;

    printf("\nUSCI_SPI0 Loop test with PDMA ");

    /* Source data initiation */
    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32MasterToSlaveTestPattern[u32DataCount] = 0x55000000 | (u32DataCount + 1);
    }

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Enable PDMA channels */
    PDMA_Open((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH));

    /*=======================================================================
      USPI master PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 16 bits
        Transfer Count = TEST_COUNT
        Source = g_au32MasterToSlaveTestPattern
        Source Address = Increasing
        Destination = USPI0->TXDAT
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (16 bits) and transfer count */
    PDMA_SetTransferCnt(SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_16, TEST_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(SPI_MASTER_TX_DMA_CH, (uint32_t)g_au32MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&USPI0->TXDAT, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(SPI_MASTER_TX_DMA_CH, PDMA_USCI0_TX, FALSE, 0);
    /* Single request type. USPI only support PDMA single request type. */
    PDMA_SetBurstType(SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA_DisableInt(SPI_MASTER_TX_DMA_CH, PDMA_INT_TEMPTY);

    /*=======================================================================
      USPI master PDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 16 bits
        Transfer Count = TEST_COUNT
        Source = USPI0->RXDAT
        Source Address = Fixed
        Destination = g_au32MasterRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (16 bits) and transfer count */
    PDMA_SetTransferCnt(SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_16, TEST_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(SPI_MASTER_RX_DMA_CH, (uint32_t)&USPI0->RXDAT, PDMA_SAR_FIX, (uint32_t)g_au32MasterRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(SPI_MASTER_RX_DMA_CH, PDMA_USCI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA_DisableInt(SPI_MASTER_RX_DMA_CH, PDMA_INT_TEMPTY);

    /* Enable USPI master DMA function */
    USPI_ENABLE_PDMA(USPI0);

    i32Err = 0;

    for (u32TestCycle = 0; u32TestCycle < TEST_CYCLE; u32TestCycle++)
    {
        if ((u32TestCycle & 0x1FF) == 0)
        {
            putchar('.');
            fflush(stdout);
        }

        while (1)
        {
            /* Get interrupt status */
            u32RegValue = PDMA_GET_INT_STATUS();

            /* Check the PDMA transfer done interrupt flag */
            if (u32RegValue & PDMA_INTSTS_TDIF_Msk)
            {
                /* Check the PDMA transfer done flags */
                if ((PDMA_GET_TD_STS() & ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))) ==
                        ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH)))
                {
                    /* Clear the PDMA transfer done flags */
                    PDMA_CLR_TD_FLAG((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH));

                    /* Disable SPI master's PDMA transfer function */
                    USPI_DISABLE_PDMA(USPI0);

                    /* Check the transfer data */
                    for (u32DataCount = 0; u32DataCount < (TEST_COUNT / 2); u32DataCount++)
                    {
                        /* printf("data[%d] Tx = 0x%X, Rx = 0x%X\n", u32DataCount, g_au32MasterToSlaveTestPattern[u32DataCount], g_au32MasterRxBuffer[u32DataCount]); */
                        if (g_au32MasterToSlaveTestPattern[u32DataCount] != g_au32MasterRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                    }

                    if (u32TestCycle >= TEST_CYCLE)
                        break;

                    /* Source data initiation */
                    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                    {
                        g_au32MasterToSlaveTestPattern[u32DataCount]++;
                    }

                    /* Re-trigger */
                    /* Master PDMA TX channel configuration */
                    /* Set transfer width (16 bits) and transfer count */
                    PDMA_SetTransferCnt(SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_16, TEST_COUNT);
                    /* Set request source; set basic mode. */
                    PDMA_SetTransferMode(SPI_MASTER_TX_DMA_CH, PDMA_USCI0_TX, FALSE, 0);

                    /* Master PDMA RX channel configuration */
                    /* Set transfer width (16 bits) and transfer count */
                    PDMA_SetTransferCnt(SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_16, TEST_COUNT);
                    /* Set request source; set basic mode. */
                    PDMA_SetTransferMode(SPI_MASTER_RX_DMA_CH, PDMA_USCI0_RX, FALSE, 0);

                    /* Enable master's DMA transfer function */
                    USPI_ENABLE_PDMA(USPI0);
                    break;
                }
            }

            /* Check the DMA transfer abort interrupt flag */
            if (u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS();
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(u32Abort);
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

    /* Disable all PDMA channels */
    PDMA_Close();

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
