/****************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement a USB dual virtual COM port device.(UART + UUART)
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "cdc_serial.h"

#define CRYSTAL_LESS    1
#define HIRC_AUTO_TRIM  (SYS_IRCTCTL_REFCKSEL_Msk | 0x2);   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT       (SYS_BASE+0x110)
#define TRIM_THRESHOLD  16      /* Each value is 0.125%, max 2% */

#if CRYSTAL_LESS
    static volatile uint32_t s_u32DefaultTrim, s_u32LastTrim;
#endif

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_sLineCoding0 = {115200, 0, 0, 8};   /* UART0: Baud rate : 115200    */
STR_VCOM_LINE_CODING g_sLineCoding1 = {115200, 0, 0, 8};   /* USCI0_UART(UUART):Baud rate : 115200 */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t g_u16CtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t g_u16CtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/

#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* TX buffer size */

#define TX_FIFO_SIZE_0      16  /* UART: TX Hardware FIFO size */
#define TX_FIFO_SIZE_1      1  /* USCI_UART(UUART): TX Hardware FIFO size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
volatile uint8_t  g_au8ComRbuf0[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes0 = 0;
volatile uint16_t g_u16ComRhead0 = 0;
volatile uint16_t g_u16ComRtail0 = 0;

volatile uint8_t  g_au8ComTbuf0[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes0 = 0;
volatile uint16_t g_u16ComThead0 = 0;
volatile uint16_t g_u16ComTtail0 = 0;

uint8_t g_au8RxBuf0[64] = {0};
volatile uint8_t *g_pu8RxBuf0 = 0;
volatile uint32_t g_u32RxSize0 = 0;
volatile uint32_t g_u32TxSize0 = 0;

volatile int8_t g_i8BulkOutReady0 = 0;

/* UART1 */
volatile uint8_t  g_au8ComRbuf1[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes1 = 0;
volatile uint16_t g_u16ComRhead1 = 0;
volatile uint16_t g_u16ComRtail1 = 0;

volatile uint8_t  g_au8ComTbuf1[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes1 = 0;
volatile uint16_t g_u16ComThead1 = 0;
volatile uint16_t g_u16ComTtail1 = 0;

uint8_t g_au8RxBuf1[64] = {0};
volatile uint8_t *g_pu8RxBuf1 = 0;
volatile uint32_t g_u32RxSize1 = 0;
volatile uint32_t g_u32TxSize1 = 0;

volatile int8_t g_i8BulkOutReady1 = 0;


/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal HIRC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN);

    /* Waiting for Internal HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

#if (CRYSTAL_LESS)
    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select module clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));
#else
    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_48MHZ);

    /* Select module clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(2));
#endif

    SystemCoreClockUpdate();
    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set PB multi-function pins for USCI0_DAT0(PB.4) and USCI0_DAT1(PB.5) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_USCI0_DAT0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_USCI0_DAT1;

}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    UART0->INTEN = UART_INTEN_TOCNTEN_Msk | UART_INTEN_RDAIEN_Msk;
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
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;

    u32IntStatus = UART0->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART0->DAT;

            /* Check if buffer full */
            if (g_u16ComRbytes0 < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8ComRbuf0[g_u16ComRtail0++] = bInChar;

                if (g_u16ComRtail0 >= RXBUFSIZE)
                    g_u16ComRtail0 = 0;

                g_u16ComRbytes0++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if (u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if (g_u16ComTbytes0 && (UART0->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            int32_t size = g_u16ComTbytes0;

            if (size >= TX_FIFO_SIZE_0)
            {
                size = TX_FIFO_SIZE_0;
            }

            while (size)
            {
                if (g_u16ComThead0 >= TXBUFSIZE)
                    g_u16ComThead0 = 0;

                bInChar = g_au8ComTbuf0[g_u16ComThead0++];
                UART0->DAT = bInChar;

                g_u16ComTbytes0--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART0->INTEN &= (~UART_INTEN_THREIEN_Msk);
        }
    }

}








void USCI_IRQHandler(void)
{
    uint32_t u32IntStatus;

    u32IntStatus = UUART0->PROTSTS;

    if (u32IntStatus & UUART_PROTSTS_RXENDIF_Msk)
    {
        /* Cleare RX end interrupt flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        /* Get all the input characters */
        while (!UUART_IS_RX_EMPTY(UUART0))
        {
            /* Get the character from UART Buffer */
            uint8_t bInChar = UUART_READ(UUART0);

            /* Check if buffer full */
            if (g_u16ComRbytes1 < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8ComRbuf1[g_u16ComRtail1++] = bInChar;

                if (g_u16ComRtail1 >= RXBUFSIZE)
                    g_u16ComRtail1 = 0;

                g_u16ComRbytes1++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

}

void VCOM_TransferData(void)
{
    uint32_t i, u32Len;

    /* Check whether USB is ready for next packet or not */
    if (g_u32TxSize0 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if (g_u16ComRbytes0)
        {
            u32Len = g_u16ComRbytes0;

            if (u32Len > EP2_MAX_PKT_SIZE)
                u32Len = EP2_MAX_PKT_SIZE;

            for (i = 0; i < u32Len; i++)
            {
                if (g_u16ComRhead0 >= RXBUFSIZE)
                    g_u16ComRhead0 = 0;

                g_au8RxBuf0[i] = g_au8ComRbuf0[g_u16ComRhead0++];
            }

            __set_PRIMASK(1);
            g_u16ComRbytes0 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize0 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)g_au8RxBuf0, u32Len);
            USBD_SET_PAYLOAD_LEN(EP2, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP2);

            if (u32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }


    if (g_u32TxSize1 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if (g_u16ComRbytes1)
        {
            u32Len = g_u16ComRbytes1;

            if (u32Len > EP7_MAX_PKT_SIZE)
                u32Len = EP7_MAX_PKT_SIZE;

            for (i = 0; i < u32Len; i++)
            {
                if (g_u16ComRhead1 >= RXBUFSIZE)
                    g_u16ComRhead1 = 0;

                g_au8RxBuf1[i] = g_au8ComRbuf1[g_u16ComRhead1++];
            }

            __set_PRIMASK(1);
            g_u16ComRbytes1 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize1 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)g_au8RxBuf1, u32Len);
            USBD_SET_PAYLOAD_LEN(EP7, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP7);

            if (u32Len == EP7_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP7, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (g_i8BulkOutReady0 && (g_u32RxSize0 <= TXBUFSIZE - g_u16ComTbytes0))
    {
        for (i = 0; i < g_u32RxSize0; i++)
        {
            g_au8ComTbuf0[g_u16ComTtail0++] = g_pu8RxBuf0[i];

            if (g_u16ComTtail0 >= TXBUFSIZE)
                g_u16ComTtail0 = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes0 += g_u32RxSize0;
        __set_PRIMASK(0);

        g_u32RxSize0 = 0;
        g_i8BulkOutReady0 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    if (g_i8BulkOutReady1 && (g_u32RxSize1 <= TXBUFSIZE - g_u16ComTbytes1))
    {
        for (i = 0; i < g_u32RxSize1; i++)
        {
            g_au8ComTbuf1[g_u16ComTtail1++] = g_pu8RxBuf1[i];

            if (g_u16ComTtail1 >= TXBUFSIZE)
                g_u16ComTtail1 = 0;
        }

        //__set_PRIMASK(1);//no need protection in the same thread
        g_u16ComTbytes1 += g_u32RxSize1;
        //__set_PRIMASK(0);

        g_u32RxSize1 = 0;
        g_i8BulkOutReady1 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if (g_u16ComTbytes0)
    {
        /* Check if Tx is working */
        if ((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            if (g_u16ComThead0 >= TXBUFSIZE)
            {
                g_u16ComThead0 = 0;
            }

            /* Send one bytes out */
            UART0->DAT = g_au8ComTbuf0[g_u16ComThead0++];

            g_u16ComTbytes0--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }



    /* Check if data received from USB_OUT are not been transfered to USCI_UART(UUART)*/
    if (g_u16ComTbytes1)
    {
        uint32_t size;

        size = g_u16ComTbytes1;

        while (size--)
        {
            uint8_t bInChar;

            while (UUART_IS_TX_FULL(UUART0)); /* Wait Tx is not full to transmit data */

            /* Send one bytes out */
            bInChar = g_au8ComTbuf1[g_u16ComThead1++];
            UUART_WRITE(UUART0, bInChar);

            if (g_u16ComThead1 >= TXBUFSIZE)
                g_u16ComThead1 = 0;

            //__set_PRIMASK(1);//no need protection in the same thread
            g_u16ComTbytes1--;
            //__set_PRIMASK(0);
        }
    }


}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    //NUC121 is with UART0 only so we use UUART for alternative
    USCI0_Init();

    printf("\n\n");
    printf("+---------------------------------------------------------------+\n");
    printf("| NuMicro USB Virtual COM Dual Port Sample Code(UART,USCI_UART) |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("Set PB.0 as UART RX pin and PB.1 as UART TX pin\n");
    printf("Set PB.4 as UUART RX pin and PB.5 as UUART TX pin\n");

    /* Open USB controller */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    /* Start USB device */
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);

    /* Enable USCI UART receive and transmit end interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_EnableIRQ(USCI_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim value */
    s_u32DefaultTrim = M32(TRIM_INIT);
    s_u32LastTrim = s_u32DefaultTrim;

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
#endif

    while (1)
    {
#if CRYSTAL_LESS

        /* Start USB trim function if it is not enabled. */
        if ((SYS->IRCTCTL & SYS_IRCTCTL_FREQSEL_Msk) != 0x2)
        {
            /* Start USB trim only when USB signal arrived */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /* Enable USB clock trim function */
                SYS->IRCTCTL = HIRC_AUTO_TRIM;
            }
        }

        /* Disable USB Trim when any error found */
        if (SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk))
        {
            /* Last TRIM */
            M32(TRIM_INIT) = s_u32LastTrim;

            /* Disable USB clock trim function */
            SYS->IRCTCTL = 0;

            /* Clear trim error flags */
            SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

        }

        /* Check trim value whether it is over the threshold */
        if ((M32(TRIM_INIT) > (s_u32DefaultTrim + TRIM_THRESHOLD)) || (M32(TRIM_INIT) < (s_u32DefaultTrim - TRIM_THRESHOLD)))
        {
            /* Write updated value */
            M32(TRIM_INIT) = s_u32LastTrim;
        }
        else
        {
            /* Backup trim value */
            s_u32LastTrim =  M32(TRIM_INIT);
        }

#endif

        VCOM_TransferData();
    }
}



/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

