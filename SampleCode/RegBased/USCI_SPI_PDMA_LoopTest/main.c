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

//------------------------------------------------------------------------------
/*----------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
#define SPI_MASTER_TX_DMA_CH    0
#define SPI_MASTER_RX_DMA_CH    1

#define TEST_COUNT              64
#define TEST_CYCLE              10000
#define USPI_TIMEOUT            (SystemCoreClock >> 2)

/*----------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                               */
/*----------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void USCI_SPI_Init(void);
void USCI_SPI_LoopTestWithPDMA(void);
void UART0_Init(void);

//------------------------------------------------------------------------------
uint32_t g_au32MasterToSlaveTestPattern[TEST_COUNT];
uint32_t g_au32MasterRxBuffer[TEST_COUNT];

//------------------------------------------------------------------------------
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
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;

    while (1);
}

void SYS_Init(void)
{
    volatile int32_t i32TimeoutCnt = USPI_TIMEOUT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
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

    /* Enable USCI0 clock */
    CLK->APBCLK1 |= CLK_APBCLK1_USCI0CKEN_Msk;

    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Update System Core Clock */
    PllClock        = 0;                 // PLL
    SystemCoreClock = __HIRC;            // HCLK
    CyclesPerUs     = __HIRC / 1000000;  // For CLK_SysTickDelay()

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

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void USCI_SPI_Init(void)
{
    uint32_t u32ClkDiv = 0;
    uint32_t u32Pclk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initiate USCI_SPI0                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a master, USCI_SPI0 clock rate 2 MHz,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 16, 2000000); */

    /* Enable USPI protocol */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI0->CTL = 1 << USPI_CTL_FUNMODE_Pos;

    /* Data format configuration : 16-bit */
    USPI0->LINECTL &= ~USPI_LINECTL_DWIDTH_Msk;
    USPI0->LINECTL |= (0 << USPI_LINECTL_DWIDTH_Pos);

    /* MSB data format */
    USPI0->LINECTL &= ~USPI_LINECTL_LSB_Msk;

    /* Set slave selection signal active low : for master */
    USPI0->LINECTL |= USPI_LINECTL_CTLOINV_Msk;

    /* Set operating mode and transfer timing : for master and mode 0 */
    USPI0->PROTCTL &= ~(USPI_PROTCTL_SCLKMODE_Msk | USPI_PROTCTL_AUTOSS_Msk | USPI_PROTCTL_SLAVE_Msk);
    USPI0->PROTCTL |= (USPI_MASTER | USPI_MODE_0);

    /* Set USPI bus clock : 2000000Hz */
    if (CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk)
        u32Pclk = SystemCoreClock >> 1;
    else
        u32Pclk = SystemCoreClock;

    u32ClkDiv = (((((u32Pclk / 2) * 10) / (2000000)) + 5) / 10) - 1; /* Compute proper divider for USPI clock */
    USPI0->BRGEN &= ~USPI_BRGEN_CLKDIV_Msk;
    USPI0->BRGEN |= (u32ClkDiv << USPI_BRGEN_CLKDIV_Pos);

    USPI0->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;

    /* Enable the automatic hardware slave selection function and configure USCI_SPI_SS pin as low-active. */
    USPI0->LINECTL = (USPI0->LINECTL & ~USPI_LINECTL_CTLOINV_Msk) | USPI_SS_ACTIVE_LOW;
    USPI0->PROTCTL |= USPI_PROTCTL_AUTOSS_Msk;
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
    SYS->IPRST0 |= SYS_IPRST0_PDMARST_Msk;
    SYS->IPRST0 &= (~SYS_IPRST0_PDMARST_Msk);

    /* Enable PDMA channels */
    PDMA->CHCTL |= ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH));

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
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk);
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= (PDMA_WIDTH_16 | ((TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos));

    /* Set source/destination address and attributes */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].SA = (uint32_t)g_au32MasterToSlaveTestPattern;
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].DA = (uint32_t)&USPI0->TXDAT;
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk);
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= (PDMA_SAR_INC | PDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Msk) | PDMA_USCI0_TX;
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_OPMODE_Msk);
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= (PDMA_OP_BASIC);

    /* Single request type. USPI only support PDMA single request type. */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXTYPE_Msk | PDMA_DSCT_CTL_BURSIZE_Msk);
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= (PDMA_REQ_SINGLE | 0);

    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;


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
    /* PDMA_SetTransferCnt(SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_16, TEST_COUNT); */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk);
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= (PDMA_WIDTH_16 | ((TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos));

    /* Set source/destination address and attributes */
    /* PDMA_SetTransferAddr(SPI_MASTER_RX_DMA_CH, (uint32_t)&USPI0->RXDAT, PDMA_SAR_FIX, (uint32_t)g_au32MasterRxBuffer, PDMA_DAR_INC); */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].SA = (uint32_t)&USPI0->RXDAT;
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].DA = (uint32_t)g_au32MasterRxBuffer;
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk);
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= (PDMA_SAR_FIX | PDMA_DAR_INC);

    /* Set request source; set basic mode. */
    /* PDMA_SetTransferMode(SPI_MASTER_RX_DMA_CH, PDMA_USCI0_RX, FALSE, 0); */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_USCI0_RX << PDMA_REQSEL0_3_REQSRC1_Pos);
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_OPMODE_Msk);
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= (PDMA_OP_BASIC);

    /* Single request type. SPI only support PDMA single request type. */
    /* PDMA_SetBurstType(SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0); */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXTYPE_Msk | PDMA_DSCT_CTL_BURSIZE_Msk);
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= (PDMA_REQ_SINGLE | 0);

    /* Disable table interrupt */
    /* PDMA_DisableInt(SPI_MASTER_RX_DMA_CH, PDMA_INT_TEMPTY); */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Enable USPI master DMA function */
    /* USPI_ENABLE_PDMA(USPI0); */
    USPI0->PDMACTL |= (USPI_PDMACTL_TXPDMAEN_Msk | USPI_PDMACTL_RXPDMAEN_Msk | USPI_PDMACTL_PDMAEN_Msk);

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
            /* u32RegValue = PDMA_GET_INT_STATUS(); */
            u32RegValue = PDMA->INTSTS;

            /* Check the PDMA transfer done interrupt flag */
            if (u32RegValue & PDMA_INTSTS_TDIF_Msk)
            {
                /* Check the PDMA transfer done flags */
                /* if((PDMA_GET_TD_STS() & ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) )) ==
                        ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))) */
                if ((PDMA->TDSTS & ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))) ==
                        ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH)))
                {
                    /* Clear the PDMA transfer done flags */
                    /* PDMA_CLR_TD_FLAG((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH)); */
                    PDMA->TDSTS = ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH));

                    /* Disable SPI master's PDMA transfer function */
                    /* USPI_DISABLE_PDMA(USPI0); */
                    USPI0->PDMACTL &= ~(USPI_PDMACTL_TXPDMAEN_Msk | USPI_PDMACTL_RXPDMAEN_Msk | USPI_PDMACTL_PDMAEN_Msk);

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
                    /* PDMA_SetTransferCnt(SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_16, TEST_COUNT); */
                    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk);
                    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= (PDMA_WIDTH_16 | ((TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos));

                    /* Set request source; set basic mode. */
                    /* PDMA_SetTransferMode(SPI_MASTER_TX_DMA_CH, PDMA_USCI0_TX, FALSE, 0); */
                    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Msk) | PDMA_USCI0_TX;
                    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_OPMODE_Msk);
                    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= (PDMA_OP_BASIC);

                    /* Master PDMA RX channel configuration */
                    /* Set transfer width (16 bits) and transfer count */
                    /* PDMA_SetTransferCnt(SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_16, TEST_COUNT); */
                    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk);
                    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= (PDMA_WIDTH_16 | ((TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos));

                    /* Set request source; set basic mode. */
                    /* PDMA_SetTransferMode(SPI_MASTER_RX_DMA_CH, PDMA_USCI0_RX, FALSE, 0); */
                    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_USCI0_RX << PDMA_REQSEL0_3_REQSRC1_Pos);
                    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_OPMODE_Msk);
                    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= (PDMA_OP_BASIC);

                    /* Enable master's DMA transfer function */
                    /* USPI_ENABLE_PDMA(USPI0); */
                    USPI0->PDMACTL |= (USPI_PDMACTL_TXPDMAEN_Msk | USPI_PDMACTL_RXPDMAEN_Msk | USPI_PDMACTL_PDMAEN_Msk);
                    break;
                }
            }

            /* Check the DMA transfer abort interrupt flag */
            if (u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                /* u32Abort = PDMA_GET_ABORT_STS(); */
                u32Abort = PDMA->ABTSTS;

                /* Clear the target abort flag */
                /* PDMA_CLR_ABORT_FLAG(u32Abort); */
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

    /* Disable all PDMA channels */
    PDMA->CHCTL = 0;

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
