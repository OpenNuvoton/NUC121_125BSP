/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data in UART RS485 mode.
 *           This sample code needs to work with UART_RS485_Slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define MATCH_ADDRSS1       0xC0
#define MATCH_ADDRSS2       0xA2
#define UNMATCH_ADDRSS1     0xB1
#define UNMATCH_ADDRSS2     0xD3

#define UART_TIMEOUT        (SystemCoreClock >> 2)

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void RS485_FunctionTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control  (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                        */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    /* Set UART parity as MARK */
    UART0->LINE = (UART_WORD_LEN_8 | UART_PARITY_MARK | UART_STOP_BIT_1);

    /* Send data */
    UART0->DAT = u8data;
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;
    volatile int32_t i32TimeoutCnt = UART_TIMEOUT;

    /* Set UART parity as SPACE */
    UART0->LINE = (UART_WORD_LEN_8 | UART_PARITY_SPACE | UART_STOP_BIT_1);

    /* Send data */
    for (u32Count = 0; u32Count != u32WriteBytes; u32Count++)
    {
        i32TimeoutCnt = UART_TIMEOUT;

        while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk))   /* Wait Tx empty */
        {
            if (--i32TimeoutCnt <= 0)
            {
                break;
            }
        }

        UART0->DAT = pu8TxBuf[u32Count]; /* Send UART Data from buffer */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeMaster()
{
    volatile int32_t i32;
    uint8_t g_u8SendDataGroup1[10] = {0};
    uint8_t g_u8SendDataGroup2[10] = {0};
    uint8_t g_u8SendDataGroup3[10] = {0};
    uint8_t g_u8SendDataGroup4[10] = {0};

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               RS485 9-bit Master Test                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function will send different address with 10 data     |\n");
    printf("| bytes to test RS485 9-bit mode. Please connect TX/RX to   |\n");
    printf("| another board and wait its ready to receive.              |\n");
    printf("| Press any key to start...                                 |\n");
    printf("+-----------------------------------------------------------+\n\n");
    getchar();

    /* Select UART RS485 function mode */
    UART0->FUNCSEL = UART_FUNCSEL_RS485;

    /* Set RS485-Master as AUD mode */
    /* Enable AUD mode to HW control RTS pin automatically */
    /* It also can use GPIO to control RTS pin for replacing AUD mode */
    UART0->ALTCTL = UART_ALTCTL_RS485AUD_Msk;

    /* Set RTS pin active level as high level active */
    UART0->MODEM = (UART0->MODEM & (~UART_MODEM_RTSACTLV_Msk)) | UART_RTS_IS_HIGH_LEV_ACTIVE;

    /* Set TX delay time */
    UART0->TOUT = 0x2000;

    /* Prepare data to transmit*/
    for (i32 = 0; i32 < 10; i32++)
    {
        g_u8SendDataGroup1[i32] = i32;
        g_u8SendDataGroup2[i32] = i32 + 10;
        g_u8SendDataGroup3[i32] = i32 + 20;
        g_u8SendDataGroup4[i32] = i32 + 30;
    }

    /* Send different address and data for test */
    printf("Send Address %x and data 0~9\n", MATCH_ADDRSS1);
    RS485_SendAddressByte(MATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup1, 10);

    printf("Send Address %x and data 10~19\n", UNMATCH_ADDRSS1);
    RS485_SendAddressByte(UNMATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup2, 10);

    printf("Send Address %x and data 20~29\n", MATCH_ADDRSS2);
    RS485_SendAddressByte(MATCH_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup3, 10);

    printf("Send Address %x and data 30~39\n", UNMATCH_ADDRSS2);
    RS485_SendAddressByte(UNMATCH_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup4, 10);

    printf("Transfer Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest()
{
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      RS485 Function Test IO Setting                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART0_TXD(PC.5)  <==>  UART0_RXD(PC.4)--|Slave| |\n");
    printf("| |      |--UART0_nRTS(PC.3) <==> UART0_nRTS(PC.3)--|     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");


    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.
        Master:
            1.Set AUD mode and HW will control RTS pin. RTSACTLV is set to '0'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'. (Set UART_LINE = 0x2B)
            4.Data bytes : the parity bit should be '0'. (Set UART_LINE = 0x3B)
            5.RTS pin is low in idle state. When master is sending,
              RTS pin will be pull high.

        Slave:
            1.Set AAD and AUD mode firstly. RTSACTLV is set to '0'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".  (Default)
            4.AAD: The slave will ignore any data until ADDRESS match address match value.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check if RS485 address byte detect flag is set and read RX FIFO data to clear ADDRESS stored in RX FIFO.

              NMM: The slave will ignore data byte until RXOFF is disabled.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check the ADDRESS is match or not by user in UART_IRQHandler.
              If the ADDRESS is match, clear RXOFF bit to receive data byte.
              If the ADDRESS is not match, set RXOFF bit to avoid data byte stored in FIFO.
    */

    RS485_9bitModeMaster();

}

void SYS_Init(void)
{
    volatile int32_t i32TimeoutCnt = UART_TIMEOUT;

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

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Enable UART and USCI module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;
    CLK->APBCLK1 |= CLK_APBCLK1_USCI0CKEN_Msk;

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC_DIV2;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk)) | CLK_CLKDIV0_UART(1);

    /* Update System Core Clock */
    PllClock        = 0;                 // PLL
    SystemCoreClock = __HIRC;            // HCLK
    CyclesPerUs     = __HIRC / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PC multi-function pins for UART0 RXD(PC.4) and TXD(PC.5) */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_UART0_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_UART0_TXD;

    /* Set PC multi-function pins for UART0 RTS(PC.3) */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_UART0_nRTS;

    /* Set PB multi-function pins for USCI0_DAT0(PB.4) and USCI0_DAT1(PB.5) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_USCI0_DAT0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_USCI0_DAT1;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS->IPRST2 |=  SYS_IPRST2_USCI0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_USCI0RST_Msk;

    /* Configure USCI0 as UART mode */
    UUART0->CTL = (2 << UUART_CTL_FUNMODE_Pos);                                 /* Set UART function mode */
    UUART0->LINECTL = UUART_WORD_LEN_8 | UUART_LINECTL_LSB_Msk;                 /* Set UART line configuration */
    UUART0->DATIN0 = (2 << UUART_DATIN0_EDGEDET_Pos);                           /* Set falling edge detection */
    UUART0->BRGEN = ((34 << UUART_BRGEN_CLKDIV_Pos) | (5 << UUART_BRGEN_DSCNT_Pos) |
                     (1 << UUART_BRGEN_PDSCNT_Pos));                             /* Set UART baud rate as 115200bps */
    UUART0->PROTCTL |= UUART_PROTCTL_PROTEN_Msk;                                /* Enable UART protocol */
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for test */
    UART0_Init();

    /* Init USCI0 for printf */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART RS485 sample master function */
    RS485_FunctionTest();

    while (1);

}
