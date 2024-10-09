/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show a Master how to access 10-bit address Slave
 *           This sample code needs to work with USCI_I2C_Master_10bit
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/;
volatile uint8_t g_u8SlvData[256];
volatile uint32_t g_u32SlaveBuffAddr;
volatile uint8_t g_au8SlvRxData[4];
volatile uint16_t g_u16SlvRcvAddr;
volatile uint8_t g_u8SlvDataLen;

enum UI2C_SLAVE_EVENT volatile s_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

static volatile UI2C_FUNC s_UI2C0HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_IRQHandler(void)
{
    uint32_t u32Status;

    //USCI0 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (s_UI2C0HandlerFn != NULL)
        s_UI2C0HandlerFn(u32Status);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SlaveTRx(uint32_t u32Status)
{
    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        s_Event = SLAVE_H_RD_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (s_Event == SLAVE_H_WR_ADDRESS_ACK)
        {
            g_u8SlvDataLen = 0;

            s_Event = SLAVE_L_WR_ADDRESS_ACK;
            g_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_H_RD_ADDRESS_ACK)
        {
            g_u8SlvDataLen = 0;

            UI2C_SET_DATA(UI2C0, g_u8SlvData[g_u32SlaveBuffAddr]);
            g_u32SlaveBuffAddr++;
            g_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_L_WR_ADDRESS_ACK)
        {
            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                UI2C_SET_DATA(UI2C0, g_u8SlvData[g_u32SlaveBuffAddr]);
                g_u32SlaveBuffAddr++;
            }
            else
            {
                s_Event = SLAVE_GET_DATA;
            }

            g_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_L_RD_ADDRESS_ACK)
        {
            UI2C_SET_DATA(UI2C0, g_u8SlvData[g_u32SlaveBuffAddr]);
            g_u32SlaveBuffAddr++;
        }
        else if (s_Event == SLAVE_GET_DATA)
        {
            g_au8SlvRxData[g_u8SlvDataLen] = (uint8_t)UI2C_GET_DATA(UI2C0);
            g_u8SlvDataLen++;

            if (g_u8SlvDataLen == 2)
            {
                g_u32SlaveBuffAddr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
            }

            if (g_u8SlvDataLen == 3)
            {
                g_u8SlvData[g_u32SlaveBuffAddr] = g_au8SlvRxData[2];
                g_u8SlvDataLen = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        s_Event = SLAVE_H_WR_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        s_Event = SLAVE_H_WR_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN;

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

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
    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set UI2C0 PC multi-function pins */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC3MFP_USCI0_DAT0 | SYS_GPC_MFPL_PC0MFP_USCI0_CLK);

    /* I2C pins enable schmitt trigger */
    PC->SMTEN |= GPIO_SMTEN_SMTEN0_Msk | GPIO_SMTEN_SMTEN3_Msk;

}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    uint32_t u32Clkdiv;

    u32Clkdiv = (((((SystemCoreClock / 2) * 10) / (u32ClkSpeed)) + 5) / 10) - 1;

    if (u32Clkdiv < 8)
        u32Clkdiv = 8;

    /* Open USCI_I2C0 and set clock to 100k */
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C0->CTL = 4 << UI2C_CTL_FUNMODE_Pos;

    //Data format configuration
    // 8 bit data length
    UI2C0->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C0->LINECTL |= 8 << UI2C_LINECTL_DWIDTH_Pos;

    // MSB data format
    UI2C0->LINECTL &= ~UI2C_LINECTL_LSB_Msk;

    /* Set UI2C0 clock divider */
    UI2C0->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C0->BRGEN |= (u32Clkdiv << UI2C_BRGEN_CLKDIV_Pos);

    /* Enable 10-bit mode */
    UI2C0->PROTCTL |= UI2C_PROTCTL_ADDR10EN_Msk;

    /* Set UI2C0 Slave Addresses */
    UI2C0->DEVADDR0 = 0x115;
    UI2C0->DEVADDR1 = 0x135;

    /* Set UI2C0 Slave Addresses Msk */
    UI2C0->ADDRMSK0 = 0x01;
    UI2C0->ADDRMSK1 = 0x04;

    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);

    /* Enable UI2C0 protocol */
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
}

void UART_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* UART peripheral clock rate 24MHz; UART bit rate 115200 bps. */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                        Main function                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    uint32_t u32Index;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Master access        |\n");
    printf("|  10-bit address Slave (Slave)                         |\n");
    printf("|  UI2C0(Master)  <----> UI2C0(Slave)                   |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a Slave\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PC.3), UI2C0_SCL(PC.0)\n\n");

    /* Init USCI_I2C0 */
    UI2C0_Init(100000);

    s_Event = SLAVE_H_WR_ADDRESS_ACK;

    /* Trigger USCI_I2C0 */
    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    for (u32Index = 0; u32Index < 0x100; u32Index++)
        g_u8SlvData[u32Index] = 0;

    /* I2C0 function to Slave receive/transmit data */
    s_UI2C0HandlerFn = UI2C_SlaveTRx;

    printf("UI2C0 Slave is running...\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
