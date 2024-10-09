/*************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use USCI_I2C interface to access EEPROM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLenM;
volatile uint8_t g_u8EndFlagM = 0;
enum UI2C_MASTER_EVENT volatile m_Event;

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
    {
        s_UI2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Tx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_EEPROM_MasterTx(uint32_t u32Status)
{
    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);                    /* Clear START INT Flag */
        UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x00);                        /* Write SLA+W to Register TXDAT */
        m_Event = MASTER_SEND_ADDRESS;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);                     /* Clear ACK INT Flag */

        if (m_Event == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8TxData[g_u8DataLenM++]);                     /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            m_Event = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (m_Event == MASTER_SEND_DATA)
        {
            if (g_u8DataLenM != 3)
            {
                UI2C_SET_DATA(UI2C0, g_au8TxData[g_u8DataLenM++]);                  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                g_u8EndFlagM = 1;
                m_Event = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));        /* Send STOP signal */
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);                     /* Clear NACK INT Flag */
        g_u8EndFlagM = 0;

        if (m_Event == MASTER_SEND_ADDRESS)
        {
            /* SLA+W has been transmitted and NACK has been received */
            m_Event = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));            /* Send START signal */
        }
        else if (m_Event == MASTER_SEND_DATA)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            m_Event = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
        }
        else
        {
            printf("Get Wrong NACK Event\n");
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Rx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_EEPROM_MasterRx(uint32_t u32Status)
{
    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk); /* Clear START INT Flag */

        if (m_Event == MASTER_SEND_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x00); /* Write SLA+W to Register TXDAT */
            m_Event = MASTER_SEND_ADDRESS;
        }
        else if (m_Event == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
            m_Event = MASTER_SEND_H_RD_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */

        if (m_Event == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8TxData[g_u8DataLenM++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            m_Event = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (m_Event == MASTER_SEND_DATA)
        {
            if (g_u8DataLenM != 2)
            {
                UI2C_SET_DATA(UI2C0, g_au8TxData[g_u8DataLenM++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                m_Event = MASTER_SEND_REPEAT_START;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send repeat START signal */
            }
        }
        else if (m_Event == MASTER_SEND_H_RD_ADDRESS)
        {
            m_Event = MASTER_READ_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */

        if (m_Event == MASTER_SEND_ADDRESS)
        {
            m_Event = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
        }
        else if (m_Event == MASTER_READ_DATA)
        {
            g_u8RxData = (unsigned char) UI2C_GET_DATA(UI2C0) & 0xFF;
            g_u8EndFlagM = 1;
            m_Event = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
        {
            printf("Get Wrong NACK Event\n");
        }
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
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;
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
    {
        u32Clkdiv = 8;
    }

    /* Open USCI_I2C0 and set clock to 100k */
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C0->CTL = 4 << UI2C_CTL_FUNMODE_Pos;
    //Data format configuration
    // 8 bit data length
    UI2C0->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C0->LINECTL |= 8 << UI2C_LINECTL_DWIDTH_Pos;
    // MSB data format
    UI2C0->LINECTL &= ~UI2C_LINECTL_LSB_Msk;
    UI2C0->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C0->BRGEN |= (u32Clkdiv << UI2C_BRGEN_CLKDIV_Pos);
    /* Set UI2C1 Slave Addresses */
    UI2C0->DEVADDR0 = 0x15;   /* Slave Address : 0x15 */
    UI2C0->DEVADDR1 = 0x35;   /* Slave Address : 0x35 */
    /* Set UI2C1 Slave Addresses Msk */
    UI2C0->ADDRMSK0 = 0x1;   /* Slave Address : 0x1 */
    UI2C0->ADDRMSK1 = 0x4;   /* Slave Address : 0x4 */
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

int main()
{
    uint32_t u32Index, u32TimeOutCnt;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Init();
    /*
        This sample code sets I2C bus clock to 100kHz. Then, accesses EEPROM 24LC64 with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+-------------------------------------------------------+\n");
    printf("|     USCI_I2C Driver Sample Code with EEPROM 24LC64    |\n");
    printf("+-------------------------------------------------------+\n");
    printf("\n");
    printf("Configure UI2C0 as a Master\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PC.3), UI2C0_SCL(PC.0)\n\n");
    /* Init USCI_I2C0 to access EEPROM */
    UI2C0_Init(100000);
    g_u8DeviceAddr = 0x50;

    for (u32Index = 0; u32Index < 0x100; u32Index++)
    {
        uint8_t u8Tmp;
        g_au8TxData[0] = (uint8_t)((u32Index & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(u32Index & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);
        g_u8DataLenM = 0;
        g_u8EndFlagM = 0;
        /* USCI_I2C function to write data to slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_EEPROM_MasterTx;
        /* USCI_I2C as master sends START signal */
        m_Event = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);
        /* Wait USCI_I2C Tx Finish */
        u32TimeOutCnt = SystemCoreClock;

        while (g_u8EndFlagM == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait USCI_I2C Tx Finish Timeout\n");
                return -1;
            }
        }

        g_u8EndFlagM = 0;
        /* USCI_I2C function to read data from slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_EEPROM_MasterRx;
        g_u8DataLenM = 0;
        g_u8DeviceAddr = 0x50;
        m_Event = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);
        /* Wait USCI_I2C Rx Finish */
        u32TimeOutCnt = SystemCoreClock;

        while (g_u8EndFlagM == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait USCI_I2C Rx Finish Timeout\n");
                return -1;
            }
        }

        g_u8EndFlagM = 0;
        /* Compare data */
        u8Tmp = g_au8TxData[2];

        if (g_u8RxData != u8Tmp)
        {
            printf("USCI_I2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
            return -1;
        }
    }

    printf("USCI_I2C Access EEPROM Test OK\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
