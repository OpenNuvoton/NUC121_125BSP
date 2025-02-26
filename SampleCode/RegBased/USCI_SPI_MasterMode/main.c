/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief   Configure USCI_SPI0 as Master mode and demonstrate how to communicate with an off-chip SPI Slave device.
 *          This sample code needs to work with USCI_SPI_SlaveMode sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define TEST_COUNT              16
#define USPI_TIMEOUT            (SystemCoreClock >> 2)

/*----------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                               */
/*----------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void USCI_SPI_Init(void);
void UART0_Init(void);

/*----------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

void USCI_IRQHandler(void)
{
    uint32_t u32RxData;
    volatile int32_t i32TimeoutCnt = USPI_TIMEOUT;

    /* Clear TX end interrupt flag */
    USPI0->PROTSTS = USPI_PROTSTS_TXENDIF_Msk;

    /* Check RX EMPTY flag */
    while ((USPI0->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) == 0)
    {
        /* Read RX Buffer */
        u32RxData = USPI0->RXDAT;

        g_au32DestinationData[g_u32RxDataCount++] = u32RxData;

        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    /* Check TX data count */
    if (g_u32TxDataCount < TEST_COUNT)
    {
        /* Write to TX Buffer */
        USPI0->TXDAT = g_au32SourceData[g_u32TxDataCount++];
    }
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Module Clock                                                                                     */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Enable USCI0 clock */
    CLK->APBCLK1 |= CLK_APBCLK1_USCI0CKEN_Msk;

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

/*----------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                            */
/*----------------------------------------------------------------------------------------------------------*/
int main()
{
    uint32_t u32DataCount;
    volatile int32_t i32TimeoutCnt = USPI_TIMEOUT;

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
    printf("+--------------------------------------------------------+\n");
    printf("|             USCI_SPI Master Mode Sample Code           |\n");
    printf("+--------------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a master.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0:\n");
    printf("    USCI_SPI0_SS (PC.1)\n    USCI_SPI0_CLK (PC.0)\n");
    printf("    USCI_SPI0_MISO (PC.2)\n    USCI_SPI0_MOSI (PC.3)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The USCI_SPI master configuration is ready.\n");

    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x5500 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready.\n");
    printf("Press any key to start the transfer.");
    getchar();
    printf("\n");

    /* Enable TX end interrupt */
    USPI0->INTEN |= USPI_INTEN_TXENDIEN_Msk;

    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;

    NVIC_EnableIRQ(USCI_IRQn);

    /* Write to TX Buffer */
    USPI0->TXDAT = g_au32SourceData[g_u32TxDataCount++];

    i32TimeoutCnt = USPI_TIMEOUT;

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

    /* Disable TX end interrupt */
    USPI0->INTEN &= (~USPI_INTEN_TXENDIEN_Msk);

    NVIC_DisableIRQ(USCI_IRQn);

    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Close USCI_SPI0 */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
