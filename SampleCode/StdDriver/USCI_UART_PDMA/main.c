/***************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive USCI_UART data with PDMA.
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define UUART_RX_DMA_CH 0
#define UUART_TX_DMA_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t g_i32UUartTestLength = 64;
uint8_t g_au8SrcArray[64];
uint8_t g_au8DestArray[64];
volatile int32_t g_i32IntCnt;
volatile int32_t g_i32IsTestOver;
volatile uint32_t g_u32TwoChannelPdmaTest = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Clear buffer funcion                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern)
{
    uint8_t *pu8Ptr;
    uint32_t u32Idxi;

    pu8Ptr = (uint8_t *)u32Addr;

    for (u32Idxi = 0; u32Idxi < u32Length; u32Idxi++)
    {
        *pu8Ptr++ = u8Pattern;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Bulid Src Pattern function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t u32Idxi = 0, u32Idxj, u32Loop;
    uint8_t *pAddr;

    pAddr = (uint8_t *)u32Addr;

    do
    {
        if (u32Length > 256)
            u32Loop = 256;
        else
            u32Loop = u32Length;

        u32Length = u32Length - u32Loop;

        for (u32Idxj = 0; u32Idxj < u32Loop; u32Idxj++)
            *pAddr++ = (uint8_t)(u32Idxj + u32Idxi);

        u32Idxi++;
    } while ((u32Loop != 0) || (u32Length != 0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UUART Tx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UUART_TxTest(void)
{
    /* UUART Tx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(UUART_TX_DMA_CH, PDMA_WIDTH_8, g_i32UUartTestLength);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(UUART_TX_DMA_CH, (uint32_t)g_au8SrcArray, PDMA_SAR_INC, (uint32_t)&UUART0->TXDAT, PDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(UUART_TX_DMA_CH, PDMA_USCI0_TX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(UUART_TX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA->DSCT[UUART_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* UUART Rx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UUART_RxTest(void)
{
    /* UUART Rx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(UUART_RX_DMA_CH, PDMA_WIDTH_8, g_i32UUartTestLength);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(UUART_RX_DMA_CH, (uint32_t)&UUART0->RXDAT, PDMA_SAR_FIX, (uint32_t)g_au8DestArray, PDMA_DAR_INC);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(UUART_RX_DMA_CH, PDMA_USCI0_RX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(UUART_RX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA->DSCT[UUART_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Callback_0(void)
{
    printf("\tTransfer Done %d!\r", ++g_i32IntCnt);

    /* Use PDMA to do UUART loopback test 10 times */
    if (g_i32IntCnt < 10)
    {
        /* UUART Tx and Rx PDMA configuration */
        PDMA_UUART_TxTest();
        PDMA_UUART_RxTest();

        /* Enable UUART Tx and Rx PDMA function */
        UUART0->PDMACTL |= (UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_TXPDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        g_i32IsTestOver = TRUE;
    }
}

void PDMA_Callback_1(void)
{
    int32_t i32Idxi;

    printf("\tTransfer Done %d!\t", ++g_i32IntCnt);

    /* Show UUART Rx data */
    for (i32Idxi = 0; i32Idxi < g_i32UUartTestLength; i32Idxi++)
        printf(" 0x%x(%c),", inpb(((uint32_t)g_au8DestArray + i32Idxi)), inpb(((uint32_t)g_au8DestArray + i32Idxi)));

    printf("\n");

    /* Use PDMA to do UUART Rx test 10 times */
    if (g_i32IntCnt < 10)
    {
        /* UUART Rx PDMA configuration */
        PDMA_UUART_RxTest();

        /* Enable UUART Rx PDMA function */
        UUART0->PDMACTL |= (UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        g_i32IsTestOver = TRUE;
    }
}

void PDMA_IRQHandler(void)
{
    /* Get PDMA interrupt status */
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)   /* Target Abort */
    {
        uint32_t u32AbSts;

        g_i32IsTestOver = 2;
        u32AbSts = PDMA->ABTSTS;
        PDMA->ABTSTS = u32AbSts;
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)     /* Transfer Done */
    {
        /* UUART Tx PDMA transfer done interrupt flag */
        if (PDMA_GET_TD_STS() & (1 << UUART_TX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG((1 << UUART_TX_DMA_CH));

            /* Disable UUART Tx PDMA function */
            UUART0->PDMACTL &= ~UUART_PDMACTL_TXPDMAEN_Msk;
        }

        /* UUART Rx PDMA transfer done interrupt flag */
        if (PDMA_GET_TD_STS() & (1 << UUART_RX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG((1 << UUART_RX_DMA_CH));

            /* Disable UUART Rx PDMA function */
            UUART0->PDMACTL &= ~UUART_PDMACTL_RXPDMAEN_Msk;

            /* Handle PDMA transfer done interrupt event */
            if (g_u32TwoChannelPdmaTest == 1)
            {
                PDMA_Callback_0();
            }
            else if (g_u32TwoChannelPdmaTest == 0)
            {
                PDMA_Callback_1();
            }
        }
    }
    else
    {
        printf("unknown interrupt, status=0x%x !!\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    /* Get UART0 Rx data and send the data to UUART0 Tx */
    if (UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAIF_Msk))
        UUART0->TXDAT = UART0->DAT;
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Sample Code:                                                                                       */
/*         i32option : ['1'] UUART0 TX/RX PDMA Loopback                                                     */
/*                     [Others] UUART0 RX PDMA test                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UUART(int32_t i32option)
{
    /* Source data initiation */
    BuildSrcPattern((uint32_t)g_au8SrcArray, g_i32UUartTestLength);
    ClearBuf((uint32_t)g_au8DestArray, g_i32UUartTestLength, 0xFF);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    if (i32option == '1')
    {
        printf("  [Using TWO PDMA channel].\n");
        printf("  This sample code will use PDMA to do UUART0 loopback test 10 times.\n");
        printf("  Please connect UUART0_RXD(PB.4) <--> UUART0_TXD(PB.5) before testing.\n");
        printf("  After connecting PB.4 <--> PB.5, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 1;
        getchar();
    }
    else
    {
        g_i32UUartTestLength = 2;     /* Test Length */
        printf("  [Using ONE PDMA channel].\n");
        printf("  This sample code will use PDMA to do UUART0 Rx test 10 times.\n");
        printf("  Please connect UUART0_RXD(PB.4) <--> UUART0_TXD(PB.5) before testing.\n");
        printf("  After connecting PB.4 <--> PB.5, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 0;
        getchar();
        printf("  Please input %d bytes to trigger PDMA one time.(Ex: Press 'a''b')\n", g_i32UUartTestLength);
    }

    if (g_u32TwoChannelPdmaTest == 1)
    {
        /* Enable PDMA channel */
        PDMA_Open((1 << UUART_RX_DMA_CH) | (1 << UUART_TX_DMA_CH));

        /* UUART Tx and Rx PDMA configuration */
        PDMA_UUART_TxTest();
        PDMA_UUART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(UUART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
        PDMA_EnableInt(UUART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    }
    else
    {
        /* Enable PDMA channel */
        PDMA_Open((1 << UUART_RX_DMA_CH));

        /* UUART Rx PDMA configuration */
        PDMA_UUART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(UUART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    }

    /* Enable PDMA Transfer Done Interrupt */
    g_i32IntCnt = 0;
    g_i32IsTestOver = FALSE;
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Enable UART0 RDA interrupt */
    if (g_u32TwoChannelPdmaTest == 0)
    {
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    }

    /* Enable UUART Tx and Rx PDMA function */
    if (g_u32TwoChannelPdmaTest == 1)
        UUART0->PDMACTL |= (UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_TXPDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);
    else
        UUART0->PDMACTL &= ~UUART_PDMACTL_TXPDMAEN_Msk;

    UUART0->PDMACTL |= (UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);

    /* Wait for PDMA operation finish */
    while (g_i32IsTestOver == FALSE);

    /* Check PDMA status */
    if (g_i32IsTestOver == 2)
        printf("target abort...\n");

    /* Disable UUART Tx and Rx PDMA function */
    UUART0->PDMACTL &= ~(UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_TXPDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);

    /* Disable PDMA channel */
    PDMA_Close();

    /* Disable PDMA Interrupt */
    PDMA_DisableInt(UUART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_DisableInt(UUART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(PDMA_IRQn);

    /* Disable UART0 RDA interrupt */
    UART_DisableInt(UART0, UART_INTEN_RDAIEN_Msk);
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

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable USCI module clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set PB multi-function pins for USCI0_DAT0(PB.4), USCI0_DAT1(PB.5) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_USCI0_DAT0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_USCI0_DAT1;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint8_t unItem;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init USCI0 for test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUUART PDMA Sample Program");

    /* UART PDMA sample function */
    do
    {
        printf("\n\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|                      UUART PDMA Driver Sample Code                     |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("| [1] Using TWO PDMA channel to test. < TX1(CH1)-->RX1(CH0) >            |\n");
        printf("| [2] Using ONE PDMA channel to test. < TX1-->RX1(CH0) >                 |\n");
        printf("+------------------------------------------------------------------------+\n");
        unItem = getchar();

        g_i32IsTestOver = FALSE;

        if ((unItem == '1') || (unItem == '2'))
        {
            PDMA_UUART(unItem);
            printf("\n\n  UUART PDMA sample code is complete.\n");
        }

    } while (unItem != 27);

    while (1);

}
