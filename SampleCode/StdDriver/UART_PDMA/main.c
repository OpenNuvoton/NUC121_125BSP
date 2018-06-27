/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive UART data with PDMA.
 *
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define UART_RX_DMA_CH 0
#define UART_TX_DMA_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t UART_TEST_LENGTH = 64;
uint8_t SrcArray[64];
uint8_t DestArray[64];
volatile int32_t IntCnt;
volatile int32_t IsTestOver;
volatile uint32_t g_u32TwoChannelPdmaTest = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Clear buffer funcion                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern)
{
    uint8_t *pu8Ptr;
    uint32_t i;

    pu8Ptr = (uint8_t *)u32Addr;

    for (i = 0; i < u32Length; i++)
    {
        *pu8Ptr++ = u8Pattern;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Bulid Src Pattern function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t i = 0, j, loop;
    uint8_t *pAddr;

    pAddr = (uint8_t *)u32Addr;

    do
    {
        if (u32Length > 256)
            loop = 256;
        else
            loop = u32Length;

        u32Length = u32Length - loop;

        for (j = 0; j < loop; j++)
            *pAddr++ = (uint8_t)(j + i);

        i++;
    }
    while ((loop != 0) || (u32Length != 0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Tx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_TxTest(void)
{
    /* UART Tx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(UART_TX_DMA_CH, PDMA_WIDTH_8, UART_TEST_LENGTH);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(UART_TX_DMA_CH, (uint32_t)SrcArray, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);

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
    PDMA_SetTransferCnt(UART_RX_DMA_CH, PDMA_WIDTH_8, UART_TEST_LENGTH);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(UART_RX_DMA_CH, (uint32_t)&UART0->DAT, PDMA_SAR_FIX, (uint32_t)DestArray, PDMA_DAR_INC);

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
    printf("\tTransfer Done %d!\r", ++IntCnt);

    /* Use PDMA to do UART loopback test 10 times */
    if (IntCnt < 10)
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
        IsTestOver = TRUE;
    }
}

void PDMA_Callback_1(void)
{
    int32_t i ;

    printf("\tTransfer Done %d!\t", ++IntCnt);

    /* Show UART Rx data */
    for (i = 0; i < UART_TEST_LENGTH; i++)
        printf(" 0x%x(%c),", inpb(((uint32_t)DestArray + i)), inpb(((uint32_t)DestArray + i)));

    printf("\n");

    /* Use PDMA to do UART Rx test 10 times */
    if (IntCnt < 10)
    {
        /* UART Rx PDMA configuration */
        PDMA_UART_RxTest();

        /* Enable UART Rx PDMA function */
        UART0->INTEN |= UART_INTEN_RXPDMAEN_Msk;
    }
    else
    {
        /* Test is over */
        IsTestOver = TRUE;
    }
}

void PDMA_IRQHandler(void)
{
    /* Get PDMA interrupt status */
    uint32_t status = PDMA_GET_INT_STATUS();

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* Target Abort */
    {
        IsTestOver = 2;
        PDMA->ABTSTS = (PDMA->ABTSTS);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* Transfer Done */
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
        printf("unknown interrupt, status=0x%x !!\n", status);
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
    uint32_t u32DAT;
    uint32_t u32IntSts = UART0->INTSTS;

    if (u32IntSts & UART_INTSTS_HWRLSIF_Msk)
    {
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
void PDMA_UART(int32_t i32option)
{
    /* Source data initiation */
    BuildSrcPattern((uint32_t)SrcArray, UART_TEST_LENGTH);
    ClearBuf((uint32_t)DestArray, UART_TEST_LENGTH, 0xFF);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    if (i32option == '1')
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
        UART_TEST_LENGTH = 2;      /* Test Length */
        printf("  [Using ONE PDMA channel].\n");
        printf("  This sample code will use PDMA to do UART0 Rx test 10 times.\n");
        printf("  Please connect UART0_RXD(PB.0) <--> UART0_TXD(PB.1) before testing.\n");
        printf("  After connecting PB.0 <--> PB.1, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 0;
        getchar();
        printf("  Please input %d bytes to trigger PDMA one time.(Ex: Press 'a''b')\n", UART_TEST_LENGTH);
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
    IntCnt = 0;
    IsTestOver = FALSE;
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Enable UUART0 receive interrupt */
    if (g_u32TwoChannelPdmaTest == 0)
    {
        UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
        NVIC_EnableIRQ(USCI_IRQn);
    }

    /* Enable UART Tx and Rx PDMA function */
    if (g_u32TwoChannelPdmaTest == 1)
        UART0->INTEN |= UART_INTEN_TXPDMAEN_Msk;
    else
        UART0->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;

    UART0->INTEN |= UART_INTEN_RXPDMAEN_Msk | UART_INTEN_RLSIEN_Msk;

    NVIC_EnableIRQ(UART0_IRQn);

    /* Wait for PDMA operation finish */
    while (IsTestOver == FALSE);

    /* Check PDMA status */
    if (IsTestOver == 2)
        printf("target abort...\n");

    /* Disable UART Tx and Rx PDMA function */
    UART0->INTEN &= ~(UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);

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

    uint8_t unItem;

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
        unItem = getchar();

        IsTestOver = FALSE;

        if ((unItem == '1') || (unItem == '2'))
        {
            PDMA_UART(unItem);
            printf("\n\n  UART PDMA sample code is complete.\n");
        }

    }
    while (unItem != 27);

    while (1);

}
