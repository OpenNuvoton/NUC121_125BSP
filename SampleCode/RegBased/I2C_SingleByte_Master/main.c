/******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to Read and Write single byte data to Slave.
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

    /* I2C pins enable schmitt trigger */
    PC->SMTEN |= GPIO_SMTEN_SMTEN11_Msk | GPIO_SMTEN_SMTEN12_Msk;

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

}

void I2C0_Close(void)
{
    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Write Single Byte                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t I2C_WriteByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t u8Data)
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
                u8Ctrl = I2C_CTL_SI;                                        /* Clear SI */
                break;

            case 0x18:                                                      /* Slave Address ACK */
                I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF00) >> 8);    /* Write Hi byte address of register */
                break;

            case 0x20:                                                      /* Slave Address NACK */
            case 0x30:                                                      /* Master transmit data NACK */
                u8Ctrl = I2C_CTL_STO_SI;                                    /* Clear SI and send STOP */
                u8Err = 1;
                break;

            case 0x28:
                if (u8Addr)
                {
                    I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF));       /* Write Lo byte address of register */
                    u8Addr = 0;
                }
                else if ((u32txLen < 1) && (u8Addr == 0))
                {
                    I2C_SET_DATA(i2c, u8Data);
                    u32txLen++;
                }
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

    return (u8Err | u8Xfering);                                             /* return (Success)/(Fail) status */
}

/*---------------------------------------------------------------------------------------------------------*/
/* Read Single Byte                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t I2C_ReadByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr)
{
    uint8_t u8Xfering = 1, u8Err = 0, u8rData = 0, u8Addr = 1, u8Ctrl = 0;

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
                u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x48:                                                      /* Slave Address NACK */
                u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1;
                break;

            case 0x58:
                u8rData = (unsigned char) I2C_GET_DATA(i2c);                  /* Receive Data */
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

    if (u8Err)
        u8rData = 0;                                                          /* If occurs error, return 0 */

    return u8rData;                                                           /* Return read data */
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
    printf("| I2C Sample Code for Single Byte Read/Write Test        |\n");
    printf("| Needs to work with I2C_Slave sample code               |\n");
    printf("|                                                        |\n");
    printf("|      I2C Master (I2C0) <---> I2C Slave (I2C0)          |\n");
    printf("| !! This sample code requires two boards to test !!     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PC.11), I2C0_SCL(PC.12)\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Slave Address */
    g_u8DeviceAddr = 0x15;

    u8Err = 0;

    for (u32Index = 0; u32Index < 256; u32Index++)
    {
        uint8_t u8Data, u8Tmp;

        u8Tmp = (uint8_t)u32Index + 3;

        /* Single Byte Write (Two Registers) */
        while (I2C_WriteByteTwoRegs(I2C0, g_u8DeviceAddr, u32Index, u8Tmp));

        /* Single Byte Read (Two Registers) */
        u8Data = I2C_ReadByteTwoRegs(I2C0, g_u8DeviceAddr, u32Index);

        if (u8Data != u8Tmp)
        {
            u8Err = 1;
            printf("%03u: Single byte write data fail,  W(0x%X)/R(0x%X) \n", u32Index, u8Tmp, u8Data);
        }
    }

    printf("\n");

    if (u8Err)
        printf("Single byte Read/Write access Fail.....\n");
    else
        printf("Single byte Read/Write access Pass.....\n");

    while (1);
}



