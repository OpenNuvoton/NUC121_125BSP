/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use I2C Signle byte API Read and Write data to Slave
 *           Needs to work with I2C_Slave sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;

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

    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set I2C0 PC multi-function pins */
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC11MFP_Msk | SYS_GPC_MFPH_PC12MFP_Msk);
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC11MFP_I2C0_SDA | SYS_GPC_MFPH_PC12MFP_I2C0_SCL);

    /* I2C pins enable schmitt trigger */
    PC->SMTEN |= GPIO_SMTEN_SMTEN11_Msk | GPIO_SMTEN_SMTEN12_Msk;

}

void I2C0_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

int32_t main(void)
{
    uint32_t u32Index;
    uint8_t u8Err;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+--------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for Single Byte Read/Write Test |\n");
    printf("| Needs to work with I2C_Slave sample code               |\n");
    printf("|                                                        |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                |\n");
    printf("| !! This sample code requires two boards to test !!     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");

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



