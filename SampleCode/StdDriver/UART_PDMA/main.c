/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive UART data with PDMA.
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define UART_RX_DMA_CH 0
#define UART_TX_DMA_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t g_i32UartTestLength = 64;
uint8_t g_au8SrcArray[64];
uint8_t g_au8DestArray[64];
volatile int32_t g_i32IntCnt;
volatile int32_t g_i32IsTestOver;
volatile uint32_t g_u32TwoChannelPdmaTest = 0;

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
/* UART Tx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_TxTest(void)
{
    /* UART Tx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(UART_TX_DMA_CH, PDMA_WIDTH_8, g_i32UartTestLength);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(UART_TX_DMA_CH, (uint32_t)g_au8SrcArray, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(UART_TX_DMA_CH, PDMA_UART0_TX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(UART_TX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA->DSCT[UART_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Rx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_RxTest(void)
{
    /* UART Rx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(UART_RX_DMA_CH, PDMA_WIDTH_8, g_i32UartTestLength);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(UART_RX_DMA_CH, (uint32_t)&UART0->DAT, PDMA_SAR_FIX, (uint32_t)g_au8DestArray, PDMA_DAR_INC);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(UART_RX_DMA_CH, PDMA_UART0_RX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(UART_RX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA->DSCT[UART_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Callback_0(void)
{
    printf("\tTransfer Done %d!\r", ++g_i32IntCnt);

    /* Use PDMA to do UART loopback test 10 times */
    if (g_i32IntCnt < 10)
    {
        /* UART Tx and Rx PDMA configuration */
        PDMA_UART_TxTest();
        PDMA_UART_RxTest();

        /* Enable UART Tx and Rx PDMA function */
        UART0->INTEN |= (UART_INTEN_RXPDMAEN_Msk | UART_INTEN_TXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        g_i32IsTestOver = TRUE;
    }
}

void PDMA_Callback_1(void)
{
    int32_t u32Idxi;

    printf("\tTransfer Done %d!\t", ++g_i32IntCnt);

    /* Show UART Rx data */
    for (u32Idxi = 0; u32Idxi < g_i32UartTestLength; u32Idxi++)
        printf(" 0x%x(%c),", inpb(((uint32_t)g_au8DestArray + u32Idxi)), inpb(((uint32_t)g_au8DestArray + u32Idxi)));

    printf("\n");

    /* Use PDMA to do UART Rx test 10 times */
    if (g_i32IntCnt < 10)
    {
        /* UART Rx PDMA configuration */
        PDMA_UART_RxTest();

        /* Enable UART Rx PDMA function */
        UART0->INTEN |= UART_INTEN_RXPDMAEN_Msk;
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
        u32AbSts = PDMA->ABTSTS ;
        PDMA->ABTSTS = u32AbSts ;
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)     /* Transfer Done */
    {
        /* UART Tx PDMA transfer done interrupt flag */
        if (PDMA_GET_TD_STS() & (1 << UART_TX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG((1 << UART_TX_DMA_CH));

            /* Disable UART Tx PDMA function */
            UART0->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;
        }

        /* UART Rx PDMA transfer done interrupt flag */
        if (PDMA_GET_TD_STS() & (1 << UART_RX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG((1 << UART_RX_DMA_CH));

            /* Disable UART Rx PDMA function */
            UART0->INTEN &= ~UART_INTEN_RXPDMAEN_Msk;

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
/* ISR to handle USCI interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_IRQHandler(void)
{
    uint32_t u32IntSts = UUART0->PROTSTS;

    /* Get UUART0 Rx data and send the data to UART0 Tx */
    if (u32IntSts & UUART_PROTSTS_RXENDIF_Msk)
    {

        /* Cleare RX end interrupt flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);
        UART0->DAT = UUART_READ(UUART0);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{

    uint32_t u32IntSts = UART0->INTSTS;

    if (u32IntSts & UART_INTSTS_HWRLSIF_Msk)
    {
        uint32_t u32DAT;

        if (UART0->FIFOSTS & UART_FIFOSTS_BIF_Msk)
            printf("\n BIF \n");

        if (UART0->FIFOSTS & UART_FIFOSTS_FEF_Msk)
            printf("\n FEF \n");

        if (UART0->FIFOSTS & UART_FIFOSTS_PEF_Msk)
            printf("\n PEF \n");

        u32DAT = UART0->DAT; // read out data
        printf("\n Error Data is '0x%x' \n", u32DAT);
        UART0->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Sample Code:                                                                                       */
/*         i32option : ['1'] UART0 TX/RX PDMA Loopback                                                     */
/*                     [Others] UART0 RX PDMA test                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART(int32_t i32Option)
{
    /* Source data initiation */
    BuildSrcPattern((uint32_t)g_au8SrcArray, g_i32UartTestLength);
    ClearBuf((uint32_t)g_au8DestArray, g_i32UartTestLength, 0xFF);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    if (i32Option == '1')
    {
        printf("  [Using TWO PDMA channel].\n");
        printf("  This sample code will use PDMA to do UART0 loopback test 10 times.\n");
        printf("  Please connect UART0_RXD(PB.0) <--> UART0_TXD(PB.1) before testing.\n");
        printf("  After connecting PB.0 <--> PB.1, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 1;
        getchar();
    }
    else
    {
        g_i32UartTestLength = 2;      /* Test Length */
        printf("  [Using ONE PDMA channel].\n");
        printf("  This sample code will use PDMA to do UART0 Rx test 10 times.\n");
        printf("  Please connect UART0_RXD(PB.0) <--> UART0_TXD(PB.1) before testing.\n");
        printf("  After connecting PB.0 <--> PB.1, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 0;
        getchar();
        printf("  Please input %d bytes to trigger PDMA one time.(Ex: Press 'a''b')\n", g_i32UartTestLength);
    }

    if (g_u32TwoChannelPdmaTest == 1)
    {
        /* Enable PDMA channel */
        PDMA_Open((1 << UART_RX_DMA_CH) | (1 << UART_TX_DMA_CH));

        /* UART Tx and Rx PDMA configuration */
        PDMA_UART_TxTest();
        PDMA_UART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
        PDMA_EnableInt(UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    }
    else
    {
        /* Enable PDMA channel */
        PDMA_Open((1 << UART_RX_DMA_CH));

        /* UART Rx PDMA configuration */
        PDMA_UART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    }

    /* Enable PDMA Transfer Done Interrupt */
    g_i32IntCnt = 0;
    g_i32IsTestOver = FALSE;
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Enable UUART0 receive interrupt */
    if (g_u32TwoChannelPdmaTest == 0)
    {
        UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
        NVIC_EnableIRQ(USCI_IRQn);
    }

    /* Enable UART Tx and Rx PDMA function */
    if (g_u32TwoChannelPdmaTest == 1)
        UART_PDMA_ENABLE(UART0, UART_INTEN_TXPDMAEN_Msk);
    else
        UART_PDMA_DISABLE(UART0, UART_INTEN_TXPDMAEN_Msk);

    UART_ENABLE_INT(UART0, UART_INTEN_RLSIEN_Msk);
    UART_PDMA_ENABLE(UART0, UART_INTEN_RXPDMAEN_Msk);
    /* Enable UART0 IRQ */
    NVIC_EnableIRQ(UART0_IRQn);

    /* Wait for PDMA operation finish */
    while (g_i32IsTestOver == FALSE);

    /* Check PDMA status */
    if (g_i32IsTestOver == 2)
        printf("target abort...\n");

    /* Disable UART Tx and Rx PDMA function */
    UART_PDMA_DISABLE(UART0, UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);

    /* Disable PDMA channel */
    PDMA_Close();

    /* Disable PDMA Interrupt */
    PDMA_DisableInt(UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_DisableInt(UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(PDMA_IRQn);

    /* Disable UART0 IRQ */
    NVIC_DisableIRQ(UART0_IRQn);

    /* Disable UUART0 RDA interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_DisableIRQ(USCI_IRQn);
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

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable USCI module clock */
    CLK_EnableModuleClock(USCI0_MODULE);

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

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint8_t u8Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for test */
    UART0_Init();

    /* Init USCI0 for printf and test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART PDMA Sample Program");

    /* UART PDMA sample function */
    do
    {
        printf("\n\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|                      UART PDMA Driver Sample Code                      |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("| [1] Using TWO PDMA channel to test. < TX0(CH1)-->RX0(CH0) >            |\n");
        printf("| [2] Using ONE PDMA channel to test. < TX0-->RX0(CH0) >                 |\n");
        printf("+------------------------------------------------------------------------+\n");
        u8Item = getchar();

        g_i32IsTestOver = FALSE;

        if ((u8Item == '1') || (u8Item == '2'))
        {
            PDMA_UART(u8Item);
            printf("\n\n  UART PDMA sample code is complete.\n");
        }

    } while (u8Item != 27);

    while (1);

}
