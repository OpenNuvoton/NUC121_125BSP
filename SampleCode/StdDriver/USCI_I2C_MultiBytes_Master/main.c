/*******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to set UI2C use Multi bytes API Read and Write data to Slave.
 *           Needs to work with USCI_I2C_Slave sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ***********************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;

/*---------------------------------------------------------------------------------------------------------*/
/* System initial                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable IP clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

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

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}


void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);

    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x35, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */

    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                    /* Slave Address : 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                    /* Slave Address : 0x4 */
}

void UI2C0_Close(void)
{
    /* Disable UI2C0 and close USCI clock */
    UI2C_Close(UI2C0);
    CLK_DisableModuleClock(USCI0_MODULE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Index;
    uint8_t au8TxBuf[256] = {0}, au8rDataBuf[256] = {0};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init USCI_I2C0 */
    UI2C0_Init(100000);

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets UI2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+---------------------------------------------------------+\n");
    printf("| UI2C Driver Sample Code for Multi Bytes Read/Write Test |\n");
    printf("| Needs to work with USCI_I2C_Slave sample code           |\n");
    printf("|                                                         |\n");
    printf("|      UI2C Master (I2C0) <---> UI2C Slave (I2C0)         |\n");
    printf("| !! This sample code requires two borads to test !!      |\n");
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



