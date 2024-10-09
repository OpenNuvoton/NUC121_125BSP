/***************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show a Master how to access 10-bit address Slave
 *           This sample code need works with USCI_I2C_Slave_10bit sample code
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SLV_10BIT_ADDR (0x1E<<2)             //1111+0xx+r/w

/* For NUC121: The macros are used to set STO/PTRG without clearing the other field in PROTCTL register.   */
#define UI2C_SET_CONTROL_REG_STOP(ui2c) ((ui2c)->PROTCTL = ((ui2c)->PROTCTL | UI2C_PROTCTL_STO_Msk))
#define UI2C_SET_CONTROL_REG_PTRG(ui2c) ((ui2c)->PROTCTL = ((ui2c)->PROTCTL | UI2C_PROTCTL_PTRG_Msk))

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceHAddr;
volatile uint8_t g_u8DeviceLAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint8_t g_u8MstDataLen;

enum UI2C_MASTER_EVENT volatile m_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);
static volatile UI2C_FUNC s_UI2C0HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_IRQHandler(void)
{
    uint32_t u32Status;
    //UI2C0 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (s_UI2C0HandlerFn != NULL)
    {
        s_UI2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 Master Rx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_MasterRx(uint32_t u32Status)
{
    if ((UI2C0->PROTSTS & UI2C_PROTSTS_TOIF_Msk) == UI2C_PROTSTS_TOIF_Msk)
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_TOIF_Msk);
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);  /* Clear START INT Flag */

        if (m_Event == MASTER_SEND_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceHAddr << 1) | 0x00); /* Write SLA+W to Register TXDAT */
            m_Event = MASTER_SEND_H_WR_ADDRESS;
        }
        else if (m_Event == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceHAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
            m_Event = MASTER_SEND_H_RD_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */

        if (m_Event == MASTER_SEND_H_WR_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_u8DeviceLAddr);
            m_Event = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (m_Event == MASTER_SEND_L_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            m_Event = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (m_Event == MASTER_SEND_DATA)
        {
            if (g_u8MstDataLen != 2)
            {
                UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
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
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);      /* Clear NACK INT Flag */

        if (m_Event == MASTER_SEND_H_WR_ADDRESS)
        {
            m_Event = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
        }
        else if (m_Event == MASTER_SEND_L_ADDRESS)
        {
            m_Event = MASTER_STOP;
#if 0
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
#else
            /* NUC121 Limitation: STOP and PTRG should be set respectively */
            UI2C_SET_CONTROL_REG_STOP(UI2C0);                                       /* Send STOP signal */
            UI2C_SET_CONTROL_REG_PTRG(UI2C0);
#endif
        }
        else if (m_Event == MASTER_READ_DATA)
        {
            g_u8MstRxData = (uint8_t) UI2C_GET_DATA(UI2C0);
            g_u8MstEndFlag = 1;
            m_Event = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
            /* TO DO */
        {
            printf("Status 0x%x is NOT processed\n", u32Status);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 Master Tx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_MasterTx(uint32_t u32Status)
{
    if ((UI2C0->PROTSTS & UI2C_PROTSTS_TOIF_Msk) == UI2C_PROTSTS_TOIF_Msk)
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_TOIF_Msk);
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);               /* Clear START INT Flag */
        UI2C_SET_DATA(UI2C0, (g_u8DeviceHAddr << 1) | 0x00);     /* Write SLA+W to Register TXDAT */
        m_Event = MASTER_SEND_H_WR_ADDRESS;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);   /* Clear ACK INT Flag */

        /* Event process */
        if (m_Event == MASTER_SEND_H_WR_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_u8DeviceLAddr);
            m_Event = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (m_Event == MASTER_SEND_L_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            m_Event = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (m_Event == MASTER_SEND_DATA)
        {
            if (g_u8MstDataLen != 3)
            {
                UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                g_u8MstEndFlag = 1;
                m_Event = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));        /* Send STOP signal */
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);  /* Clear NACK INT Flag */
        g_u8MstEndFlag = 0;

        if (m_Event == MASTER_SEND_H_WR_ADDRESS)
        {
            /* SLA+W has been transmitted and NACK has been received */
            m_Event = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));            /* Send START signal */
        }
        else if (m_Event == MASTER_SEND_L_ADDRESS)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            m_Event = MASTER_STOP;
#if 0
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
#else
            /* NUC121 Limitation: STOP and PTRG should be set respectively */
            UI2C_SET_CONTROL_REG_STOP(UI2C0);                                       /* Send STOP signal */
            UI2C_SET_CONTROL_REG_PTRG(UI2C0);
#endif
        }
        else if (m_Event == MASTER_SEND_DATA)
        {
            /* Data has been transmitted and NACK has been received */
            m_Event = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
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
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);
    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));
    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x015, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x035, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                    /* Slave Address : 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                    /* Slave Address : 0x4 */
    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);
}

int32_t UI2C_ReadWriteSlave(uint16_t u16SlvAddr)
{
    uint32_t u32Index, u32TimeOutCnt;
    /* Init Send 10-bit Addr */
    g_u8DeviceHAddr = (u16SlvAddr >> 8) | SLV_10BIT_ADDR;
    g_u8DeviceLAddr = u16SlvAddr & 0xFF;

    for (u32Index = 0; u32Index < 0x100; u32Index++)
    {
        uint8_t u8Tmp;
        g_au8MstTxData[0] = (uint8_t)((u32Index & 0xFF00) >> 8);
        g_au8MstTxData[1] = (uint8_t)(u32Index & 0x00FF);
        g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);
        g_u8MstDataLen = 0;
        g_u8MstEndFlag = 0;
        /* USCI_I2C function to write data to slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_MasterTx;
        /* USCI_I2C as master sends START signal */
        m_Event = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);
        /* Wait USCI_I2C Tx Finish */
        u32TimeOutCnt = SystemCoreClock;

        while (g_u8MstEndFlag == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait USCI_I2C Tx Finish Timeout\n");
                return -1;
            }
        }

        g_u8MstEndFlag = 0;
        /* USCI_I2C function to read data from slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_MasterRx;
        g_u8MstDataLen = 0;
        m_Event = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);
        /* Wait USCI_I2C Rx Finish */
        u32TimeOutCnt = SystemCoreClock;

        while (g_u8MstEndFlag == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait USCI_I2C Rx Finish Timeout\n");
                return -1;
            }
        }

        g_u8MstEndFlag = 0;
        /* Compare data */
        u8Tmp = g_au8MstTxData[2];

        if (g_u8MstRxData != u8Tmp)
        {
            printf("USCI_I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
            return -1;
        }
    }

    printf("Master Access Slave (0x%X) Test OK\n", u16SlvAddr);
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*                        Main function                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
    /* Init UART for print message */
    UART_Open(UART0, 115200);
    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+-------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for (Master) access      |\n");
    printf("|  10-bit address Slave. This sample code needs work    |\n");
    printf("|  with USCI_I2C_Slave_10bit sample code.              |\n");
    printf("|      UI2C_Master(UI2C0) < - > UI2C_Slave(UI2C0)       |\n");
    printf("|  This sample code requires two boards for testing     |\n");
    printf("+-------------------------------------------------------+\n");
    printf("\n");
    printf("Configure UI2C0 as a Master\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PC.3), UI2C0_SCL(PC.0)\n");
    /* Init USCI_I2C0 */
    UI2C0_Init(100000);
    /* Master Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
    UI2C_ReadWriteSlave(0x115);
    UI2C_ReadWriteSlave(0x135);
    printf("SLAVE Address test OK.\n");
    /* Master Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    UI2C_ReadWriteSlave(0x115 & ~0x01);
    UI2C_ReadWriteSlave(0x135 & ~0x04);
    printf("SLAVE Address Mask test OK.\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
