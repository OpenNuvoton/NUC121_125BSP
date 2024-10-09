/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to set I2C in Slave mode and receive the data from Master.
 *           This sample code needs to work with I2C_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8SlvDataLen;
uint32_t g_u32SlaveBuffAddr;
uint8_t g_au8SlvData[256];
uint8_t g_au8SlvRxData[3];
volatile uint8_t g_u8SlvTRxAbortFlag = 0;
volatile uint8_t g_u8TimeoutFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

volatile static I2C_FUNC s_I2C0HandlerFn = NULL;


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C0->STATUS;

    if (I2C0->TOCTL & I2C_TOCTL_TOIF_Msk)
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
        g_u8TimeoutFlag = 1;
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
        {
            s_I2C0HandlerFn(u32Status);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8Data;

    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8Data = (unsigned char) I2C_GET_DATA(I2C0);

        if (g_u8SlvDataLen < 2)
        {
            g_au8SlvRxData[g_u8SlvDataLen++] = u8Data;
            g_u32SlaveBuffAddr = (uint32_t)(g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        else
        {
            g_au8SlvData[g_u32SlaveBuffAddr++] = u8Data;

            if (g_u32SlaveBuffAddr == 256)
            {
                g_u32SlaveBuffAddr = 0;
            }
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[g_u32SlaveBuffAddr]);
        g_u32SlaveBuffAddr++;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xB8)                 /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[g_u32SlaveBuffAddr++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        printf("[SlaveTRx] Status [0x%x] Unexpected abort!!\n", u32Status);

        if (u32Status == 0x68)              /* Slave receive arbitration lost, clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else if (u32Status == 0xB0)         /* Address transmit arbitration lost, clear SI  */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else                                /* Slave bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }

        g_u8SlvTRxAbortFlag = 1;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
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
    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->ADDR0 = (I2C0->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x15 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->ADDR1 = (I2C0->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x35 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->ADDR2 = (I2C0->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x55 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->ADDR3 = (I2C0->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x75 << I2C_ADDR3_ADDR_Pos);
    /* Set I2C0 4 Slave Addresses Mask Bits*/
    /* Slave Address Mask Bits: 0x01 */
    I2C0->ADDRMSK0 = (I2C0->ADDRMSK0 & ~I2C_ADDRMSK0_ADDRMSK_Msk) | (0x01 << I2C_ADDRMSK0_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->ADDRMSK1 = (I2C0->ADDRMSK1 & ~I2C_ADDRMSK1_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK1_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x01 */
    I2C0->ADDRMSK2 = (I2C0->ADDRMSK2 & ~I2C_ADDRMSK2_ADDRMSK_Msk) | (0x01 << I2C_ADDRMSK2_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->ADDRMSK3 = (I2C0->ADDRMSK3 & ~I2C_ADDRMSK3_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK3_ADDRMSK_Pos);
    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->CTL &= ~I2C_CTL_INTEN_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);
    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Index, u32TimeOutCnt;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
    /* Init UART0 for printf */
    UART0_Init();
    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+-------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code(Slave) for access Slave        |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
    printf("+-------------------------------------------------------+\n");
    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PC.11), I2C0_SCL(PC.12)\n");
    /* Init I2C0 */
    I2C0_Init();
    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    for (u32Index = 0; u32Index < 0x100; u32Index++)
    {
        g_au8SlvData[u32Index] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_I2C0HandlerFn = I2C_SlaveTRx;
    printf("\n");
    printf("I2C Slave Mode is Running.\n");
    g_u8TimeoutFlag = 0;

    while (1)
    {
        /* Handle Slave timeout condition */
        if (g_u8TimeoutFlag)
        {
            printf(" SlaveTRx time out, press any key to reset IP\n");
            getchar();
            SYS_UnlockReg();
            SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
            SYS->IPRST1 = 0;
            SYS_LockReg();
            I2C0_Init();
            g_u8TimeoutFlag = 0;
            g_u8SlvTRxAbortFlag = 1;
        }

        /* When I2C abort, clear SI to enter non-addressed SLV mode*/
        if (g_u8SlvTRxAbortFlag)
        {
            g_u8SlvTRxAbortFlag = 0;
            u32TimeOutCnt = SystemCoreClock;

            while (I2C0->CTL & I2C_CTL_SI_Msk)
            {
                if (--u32TimeOutCnt == 0)
                {
                    break;
                }
            }

            printf("I2C Slave re-start. status[0x%x]\n", I2C0->STATUS);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
    }
}
