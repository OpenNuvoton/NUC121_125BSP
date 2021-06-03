/******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to Read and Write multi bytes data to Slave.
 *           Needs to work with I2C_Slave sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *********************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

uint8_t au8TxBuf[256] = {0}, au8rDataBuf[256] = {0};
/*---------------------------------------------------------------------------------------------------------*/
/* Init System                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Enable UART module clock and I2C controller */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

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

    /* Set I2C PC multi-function pins */
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC11MFP_Msk | SYS_GPC_MFPH_PC12MFP_Msk);
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC11MFP_I2C0_SDA | SYS_GPC_MFPH_PC12MFP_I2C0_SCL);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void I2C0_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C0 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->CTL |= I2C_CTL_I2CEN_Msk;

    /* I2C0 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C0->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", (SystemCoreClock / (((I2C0->CLKDIV) + 1) << 2)));

    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->ADDR0 = (I2C0->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x15 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->ADDR1 = (I2C0->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x35 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->ADDR2 = (I2C0->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x55 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->ADDR3 = (I2C0->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x75 << I2C_ADDR3_ADDR_Pos);
}

void I2C0_Close(void)
{
    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Write Multi Bytes                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t I2C_WriteMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t *pu8Data, uint32_t u32wLen)
{
    uint8_t u8Xfering = 1, u8Err = 0, u8Addr = 1, u8Ctrl = 0;
    uint32_t u32txLen = 0;

    I2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0))
    {
        I2C_WAIT_READY(i2c);

        switch (I2C_GET_STATUS(i2c))
        {
            case 0x08:
                I2C_SET_DATA(i2c, (u8SlaveAddr << 1 | 0x00));               /* Write SLA+W to Register I2CDAT */
                u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x18:                                                      /* Slave Address ACK */
                I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF00) >> 8);    /* Write Hi byte address of register */
                break;

            case 0x20:                                                      /* Slave Address NACK */
            case 0x30:                                                      /* Master transmit data NACK */
                u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1;
                break;

            case 0x28:
                if (u8Addr)
                {
                    I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF));       /* Write Lo byte address of register */
                    u8Addr = 0;
                }
                else if ((u32txLen < u32wLen) && (u8Addr == 0))
                    I2C_SET_DATA(i2c, pu8Data[u32txLen++]);                           /* Write data to Register I2CDAT*/
                else
                {
                    u8Ctrl = I2C_CTL_STO_SI;                              /* Clear SI and send STOP */
                    u8Xfering = 0;
                }

                break;

            case 0x38:                                                      /* Arbitration Lost */
            default:                                                        /* Unknow status */
                u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1;
                break;
        }

        I2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to I2C_CTL register */
    }

    return u32txLen;                                                        /* Return bytes length that have been transmitted */
}

/*---------------------------------------------------------------------------------------------------------*/
/* Read Multi Bytes                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t I2C_ReadMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *pu8rData, uint32_t u32rLen)
{
    uint8_t u8Xfering = 1, u8Err = 0, u8Addr = 1, u8Ctrl = 0;
    uint32_t u32rxLen = 0;

    I2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0))
    {
        I2C_WAIT_READY(i2c);

        switch (I2C_GET_STATUS(i2c))
        {
            case 0x08:
                I2C_SET_DATA(i2c, (u8SlaveAddr << 1 | 0x00));               /* Write SLA+W to Register I2CDAT */
                u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x18:                                                      /* Slave Address ACK */
                I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF00) >> 8);    /* Write Hi byte address of register */
                break;

            case 0x20:                                                      /* Slave Address NACK */
            case 0x30:                                                      /* Master transmit data NACK */
                u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1;
                break;

            case 0x28:
                if (u8Addr)
                {
                    I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF));       /* Write Lo byte address of register */
                    u8Addr = 0;
                }
                else
                    u8Ctrl = I2C_CTL_STA_SI;                              /* Clear SI and send repeat START */

                break;

            case 0x10:
                I2C_SET_DATA(i2c, ((u8SlaveAddr << 1) | 0x01));             /* Write SLA+R to Register I2CDAT */
                u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x40:                                                      /* Slave Address ACK */
                u8Ctrl = I2C_CTL_SI_AA;                                   /* Clear SI and set ACK */
                break;

            case 0x48:                                                      /* Slave Address NACK */
                u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1;
                break;

            case 0x50:
                pu8rData[u32rxLen++] = (unsigned char) I2C_GET_DATA(i2c);      /* Receive Data */

                if (u32rxLen < (u32rLen - 1))
                    u8Ctrl = I2C_CTL_SI_AA;                               /* Clear SI and set ACK */
                else
                    u8Ctrl = I2C_CTL_SI;                                  /* Clear SI */

                break;

            case 0x58:
                pu8rData[u32rxLen++] = (unsigned char) I2C_GET_DATA(i2c);      /* Receive Data */
                u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Xfering = 0;
                break;

            case 0x38:                                                      /* Arbitration Lost */
            default:                                                        /* Unknow status */
                u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1;
                break;
        }

        I2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to I2C_CTL register */
    }

    return u32rxLen;                                                        /* Return bytes length that have been received */
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Index;
    uint8_t u8Err;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+--------------------------------------------------------+\n");
    printf("| I2C Sample Code for Multi Bytes Read/Write Test        |\n");
    printf("| Needs to work with I2C_Slave sample code               |\n");
    printf("|                                                        |\n");
    printf("|      I2C Master (I2C0) <---> I2C Slave (I2C0)          |\n");
    printf("| !! This sample code requires two boards to test !!     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master\n");
    printf("The I/O connection to I2C0\n");
    printf("I2C0_SDA(PC.11), I2C0_SCL(PC.12)\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Slave address */
    g_u8DeviceAddr = 0x15;

    u8Err = 0;

    /* Prepare data for transmission */
    for (u32Index = 0; u32Index < 256; u32Index++)
    {
        au8TxBuf[u32Index] = (uint8_t) u32Index + 3;
    }

    for (u32Index = 0; u32Index < 256; u32Index += 32)
    {
        /* Write 32 bytes data to Slave */
        while (I2C_WriteMultiBytesTwoRegs(I2C0, g_u8DeviceAddr, u32Index, &au8TxBuf[u32Index], 32) < 32);
    }

    printf("Multi bytes Write access Pass.....\n");

    printf("\n");

    /* Use Multi Bytes Read from Slave (Two Registers) */
    while (I2C_ReadMultiBytesTwoRegs(I2C0, g_u8DeviceAddr, 0x0000, au8rDataBuf, 256) < 256);

    /* Compare TX data and RX data */
    for (u32Index = 0; u32Index < 256; u32Index++)
    {
        if (au8TxBuf[u32Index] != au8rDataBuf[u32Index])
        {
            u8Err = 1;
            printf("Data compare fail... R[%u] Data: 0x%X\n", u32Index, au8rDataBuf[u32Index]);
        }
    }

    if (u8Err)
        printf("Multi bytes Read access Fail.....\n");
    else
        printf("Multi bytes Read access Pass.....\n");

    while (1);
}



