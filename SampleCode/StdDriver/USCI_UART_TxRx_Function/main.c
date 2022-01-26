/***************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_au8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void USCI_UART_TEST_HANDLE(void);
void USCI_UART_FunctionTest(void);


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

    /* Enable USCI module clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

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

/*---------------------------------------------------------------------------------------------------------*/
/* USCI UART Test Sample                                                                                   */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init USCI0 for printf and test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART sample function */
    USCI_UART_FunctionTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI UART interrupt event                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_IRQHandler(void)
{
    USCI_UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* USCI UART Callback function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_TEST_HANDLE()
{

    uint32_t u32IntSts = UUART0->PROTSTS;

    if (u32IntSts & UUART_PROTSTS_RXENDIF_Msk)
    {

        /* Cleare RX end interrupt flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        printf("\nInput:");

        /* Get all the input characters */
        while (!UUART_IS_RX_EMPTY(UUART0))
        {

            /* Get the character from USCI UART Buffer */
            uint32_t u8InChar = UUART_READ(UUART0);

            printf("%c ", u8InChar);

            if (u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if (g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
            }
        }

        printf("\nTransmission Test:");
    }

    if (u32IntSts & UUART_PROTSTS_TXENDIF_Msk)
    {

        uint32_t u32Tmp;
        u32Tmp = g_u32comRtail;

        /* Cleare TX end interrupt flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_TXENDIF_Msk);

        if (g_u32comRhead != u32Tmp)
        {
            uint32_t u8InChar = g_au8RecData[g_u32comRhead];

            while (UUART_IS_TX_FULL(UUART0)); /* Wait Tx is not full to transmit data */

            UUART_WRITE(UUART0, u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI UART Function Test                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_FunctionTest()
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI UART Function Test                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect USCI-UART0 and PC.
        USCI-UART0 is set to debug port. USCI-UART0 is enable RX and TX end interrupt.
        When inputting char to terminal screen, RX end interrupt will happen and
        USCI-UART0 will print the received char on screen.
    */

    /* Enable USCI UART receive and transmit end interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk | UUART_INTEN_TXENDIEN_Msk);
    NVIC_EnableIRQ(USCI_IRQn);

    while (g_bWait);

    /* Disable USCI UART receive and transmit end interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk | UUART_INTEN_TXENDIEN_Msk);
    NVIC_DisableIRQ(USCI_IRQn);
    g_bWait = TRUE;
    printf("\nUSCI UART Sample Demo End.\n");
}
