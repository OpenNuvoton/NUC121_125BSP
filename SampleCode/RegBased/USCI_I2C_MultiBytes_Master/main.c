/******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to Read and Write multi bytes data to Slave.
 *           Needs to work with USCI_I2C_Slave sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
**********************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
uint8_t au8TxBuf[256] = {0}, au8rDataBuf[256] = {0};

/*-------------------------------------------------------------------------------------------*/
/* Write multi byte                                                                          */
/*-----------------------------------==------------------------------------------------------*/
uint32_t UI2C_WriteMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t *pu8Data, uint32_t u32wLen)
{
    uint8_t u8Xfering = 1, u8Err = 0, u8Ctrl = 0;
    uint32_t u32txLen = 0;
    enum UI2C_MASTER_EVENT eEvent = MASTER_SEND_START;

    UI2C_START(ui2c);                                                           /* Send START */

    while (u8Xfering && (u8Err == 0))
    {
        while (!(UI2C_GET_PROT_STATUS(ui2c) & 0x3F00));                     /* Wait UI2C new status occur */

        switch (UI2C_GET_PROT_STATUS(ui2c) & 0x3F00)
        {
            case UI2C_PROTSTS_STARIF_Msk:
                UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);         /* Clear START INT Flag */
                UI2C_SET_DATA(ui2c, (u8SlaveAddr << 1) | 0x00);                 /* Write SLA+W to Register UI2C_TXDAT */
                eEvent = MASTER_SEND_ADDRESS;
                u8Ctrl = UI2C_CTL_PTRG;                                         /* Clear SI */
                break;

            case UI2C_PROTSTS_ACKIF_Msk:
                UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);          /* Clear ACK INT Flag */

                if (eEvent == MASTER_SEND_ADDRESS)
                {
                    UI2C_SET_DATA(ui2c, (uint8_t)(u16DataAddr & 0xFF00) >> 8);  /* Write Hi byte data address to UI2C_TXDAT */
                    eEvent = MASTER_SEND_H_WR_ADDRESS;
                }
                else if (eEvent == MASTER_SEND_H_WR_ADDRESS)
                {
                    UI2C_SET_DATA(ui2c, (uint8_t)(u16DataAddr & 0xFF));         /* Write Lo byte data address to UI2C_TXDAT */
                    eEvent = MASTER_SEND_L_ADDRESS;
                }
                else
                {
                    if (u32txLen < u32wLen)
                        UI2C_SET_DATA(ui2c, pu8Data[u32txLen++]);                  /* Write data to UI2C_TXDAT */
                    else
                    {
                        u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                /* Clear SI and send STOP */
                        u8Xfering = 0;
                    }
                }

                break;

            case UI2C_PROTSTS_NACKIF_Msk:
                UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);         /* Clear NACK INT Flag */
                u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                        /* Clear SI and send STOP */
                u8Err = 1;
                break;

            case UI2C_PROTSTS_ARBLOIF_Msk:                                      /* Arbitration Lost */
            default:                                                            /* Unknow status */
                u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                        /* Clear SI and send STOP */
                u8Err = 1;
                break;
        }

        UI2C_SET_CONTROL_REG(ui2c, u8Ctrl);                                     /* Write controlbit to UI2C_CTL register */
    }

    return u32txLen;                                                            /* Return bytes length that have been transmitted */
}

/*-------------------------------------------------------------------------------------------*/
/* Read multi byte                                                                           */
/*-----------------------------------==------------------------------------------------------*/
uint32_t UI2C_ReadMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *pu8rData, uint32_t u32rLen)
{
    uint8_t u8Xfering = 1, u8Err = 0, u8Ctrl = 0;
    uint32_t u32rxLen = 0;
    enum UI2C_MASTER_EVENT eEvent = MASTER_SEND_START;

    UI2C_START(ui2c);                                                       /* Send START */

    while (u8Xfering && (u8Err == 0))
    {
        while (!(UI2C_GET_PROT_STATUS(ui2c) & 0x3F00));                     /* Wait UI2C new status occur */

        switch (UI2C_GET_PROT_STATUS(ui2c) & 0x3F00)
        {
            case UI2C_PROTSTS_STARIF_Msk:
                UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);     /* Clear START INT Flag */

                if (eEvent == MASTER_SEND_START)
                {
                    UI2C_SET_DATA(ui2c, (u8SlaveAddr << 1) | 0x00);         /* Write SLA+W to Register UI2C_TXDAT */
                    eEvent = MASTER_SEND_ADDRESS;
                }
                else if (eEvent == MASTER_SEND_REPEAT_START)
                {
                    UI2C_SET_DATA(UI2C0, (u8SlaveAddr << 1) | 0x01);        /* Write SLA+R to Register TXDAT */
                    eEvent = MASTER_SEND_H_RD_ADDRESS;
                }

                u8Ctrl = UI2C_CTL_PTRG;
                break;

            case UI2C_PROTSTS_ACKIF_Msk:
                UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);      /* Clear ACK INT Flag */

                if (eEvent == MASTER_SEND_ADDRESS)
                {
                    UI2C_SET_DATA(ui2c, (uint8_t)(u16DataAddr & 0xFF00) >> 8);  /* Write Hi byte address of register */
                    eEvent = MASTER_SEND_H_WR_ADDRESS;
                }
                else if (eEvent == MASTER_SEND_H_WR_ADDRESS)
                {
                    UI2C_SET_DATA(ui2c, (uint8_t)(u16DataAddr & 0xFF));       /* Write Lo byte address of register */
                    eEvent = MASTER_SEND_L_ADDRESS;
                }
                else if (eEvent == MASTER_SEND_L_ADDRESS)
                {
                    u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STA);                /* Send repeat START signal */
                    eEvent = MASTER_SEND_REPEAT_START;
                }
                else if (eEvent == MASTER_SEND_H_RD_ADDRESS)
                {
                    u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_AA);
                    eEvent = MASTER_READ_DATA;
                }
                else
                {
                    pu8rData[u32rxLen++] = (uint8_t) UI2C_GET_DATA(ui2c);      /* Receive Data */

                    if (u32rxLen < u32rLen - 1)
                        u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_AA);
                    else
                        u8Ctrl = UI2C_CTL_PTRG;
                }

                break;

            case UI2C_PROTSTS_NACKIF_Msk:
                UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);     /* Clear NACK INT Flag */

                if ((eEvent == MASTER_SEND_H_RD_ADDRESS) || (eEvent == MASTER_SEND_H_WR_ADDRESS) || (eEvent == MASTER_SEND_L_ADDRESS))
                {
                    u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);
                    u8Err = 1;
                }
                else
                {
                    pu8rData[u32rxLen++] = (uint8_t) UI2C_GET_DATA(ui2c);                  /* Receive Data */
                    u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);
                    u8Xfering = 0;
                }

                break;

            case UI2C_PROTSTS_ARBLOIF_Msk:                                  /* Arbitration Lost */
            default:                                                        /* Unknow status */
                u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                    /* Clear SI and send STOP */
                u8Err = 1;
                break;
        }

        UI2C_SET_CONTROL_REG(ui2c, u8Ctrl);                                 /* Write controlbit to UI2C_PROTCTL register */
    }

    return u32rxLen;                                                        /* Return bytes length that have been received */
}

/*---------------------------------------------------------------------------------------------------------*/
/* System Initial                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
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

    /* Open UI2C0 and set clock to 100k */
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C0->CTL = 4 << UI2C_CTL_FUNMODE_Pos;

    //Data format configuration
    // 8 bit data length
    UI2C0->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C0->LINECTL |= 8 << UI2C_LINECTL_DWIDTH_Pos;

    // MSB data format
    UI2C0->LINECTL &= ~UI2C_LINECTL_LSB_Msk;

    /* Set UI2C0 Clock divider */
    UI2C0->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C0->BRGEN |= (u32Clkdiv << UI2C_BRGEN_CLKDIV_Pos);


    /* Set UI2C0 Slave Addresses */
    UI2C0->DEVADDR0 = 0x15;   /* Slave Address : 0x15 */
    UI2C0->DEVADDR1 = 0x35;   /* Slave Address : 0x35 */

    /* Set UI2C0 Slave Addresses Msk */
    UI2C0->ADDRMSK0 = 0x1;   /* Slave Address : 0x1 */
    UI2C0->ADDRMSK1 = 0x4;   /* Slave Address : 0x4 */

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
int32_t main(void)
{
    uint32_t u32Index;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART_Init();

    /* Init USCI_I2C0 */
    UI2C0_Init(100000);

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets UI2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+---------------------------------------------------------+\n");
    printf("| UI2C Sample Code for Multi Bytes Read/Write Test        |\n");
    printf("| Needs to work with USCI_I2C_Slave sample code           |\n");
    printf("|                                                         |\n");
    printf("|      UI2C Master (I2C0) <---> UI2C Slave (I2C0)         |\n");
    printf("| !! This sample code requires two boards to test !!      |\n");
    printf("+---------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a Master\n");
    printf("The I/O connection to UI2C0:\n");
    printf("UI2C0_SDA(PC.3), UI2C0_SCL(PC.0)\n");
    printf("Press any key to continue\n");
    getchar();

    /* Slave address */
    g_u8DeviceAddr = 0x15;

    /* Prepare data for transmission */
    for (u32Index = 0; u32Index < 256; u32Index++)
    {
        au8TxBuf[u32Index] = (uint8_t) u32Index + 3;
    }

    for (u32Index = 0; u32Index < 256; u32Index += 32)
    {
        /* Write 32 bytes data to Slave */
        while (UI2C_WriteMultiBytesTwoRegs(UI2C0, g_u8DeviceAddr, u32Index, &au8TxBuf[u32Index], 32) < 32);
    }

    printf("Multi bytes Write access Pass.....\n");

    printf("\n");

    /* Use Multi Bytes Read from Slave (Two Registers) */
    while (UI2C_ReadMultiBytesTwoRegs(UI2C0, g_u8DeviceAddr, 0x0000, au8rDataBuf, 256) < 256);

    /* Compare TX data and RX data */
    for (u32Index = 0; u32Index < 256; u32Index++)
    {
        if (au8TxBuf[u32Index] != au8rDataBuf[u32Index])
            printf("Data compare fail... R[%u] Data: 0x%X\n", u32Index, au8rDataBuf[u32Index]);
    }

    printf("Multi bytes Read access Pass.....\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
