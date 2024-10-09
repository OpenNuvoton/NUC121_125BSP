/*************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up MCU in Power-down mode through I2C interface.
 *           This sample code needs to work with USCI_I2C_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8SlvData[256];
volatile uint32_t g_u32SlaveBuffAddr = 0;
volatile uint8_t g_au8RxData[4];
volatile uint16_t g_u16RecvAddr = 0;
volatile uint8_t g_u8DataLenS = 0;
volatile uint8_t g_u8SlvPWRDNWK = 0;
volatile uint8_t g_u8SlvI2CWK = 0;
volatile uint32_t g_u32WKfromAddr = 0;

enum UI2C_SLAVE_EVENT volatile s_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

static volatile UI2C_FUNC s_UI2C0HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    /* Check system power down mode wake-up interrupt flag */
    if (((CLK->PWRCTL) & CLK_PWRCTL_PDWKIF_Msk) != 0)
    {
        /* Clear system power down wake-up interrupt flag */
        CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;
        g_u8SlvPWRDNWK = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_IRQHandler(void)
{
    uint32_t u32Status;

    //UI2C0 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (s_UI2C0HandlerFn != NULL)
        s_UI2C0HandlerFn(u32Status);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UI2C0 toggle wake-up                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SlaveToggleWakeup(uint32_t u32Status)
{
    if ((UI2C0->WKSTS & UI2C_WKSTS_WKF_Msk) == UI2C_WKSTS_WKF_Msk)
    {
        printf("T\n");
        g_u32WKfromAddr = 0;
        g_u8SlvI2CWK = 1;

        /* Clear WKF INT Flag */
        UI2C_CLR_WAKEUP_FLAG(UI2C0);
        return;
    }

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (s_Event == SLAVE_ADDRESS_ACK)
        {
            g_u8DataLenS = 0;

            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                s_Event = SLAVE_SEND_DATA;
                UI2C_SET_DATA(UI2C0, g_au8SlvData[g_u32SlaveBuffAddr]);
                g_u32SlaveBuffAddr++;
            }
            else
            {
                s_Event = SLAVE_GET_DATA;
            }

            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_GET_DATA)
        {
            g_au8RxData[g_u8DataLenS] = (uint8_t)UI2C_GET_DATA(UI2C0);
            g_u8DataLenS++;

            if (g_u8DataLenS == 2)
            {
                /* Address has been received; ACK has been returned*/
                g_u32SlaveBuffAddr = (g_au8RxData[0] << 8) + g_au8RxData[1];
            }

            if (g_u8DataLenS == 3)
            {
                g_au8SlvData[g_u32SlaveBuffAddr] = g_au8RxData[2];
                g_u8DataLenS = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);

        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UI2C0 address match wake-up                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SlaveAddressWakeup(uint32_t u32Status)
{
    if ((UI2C0->WKSTS & UI2C_WKSTS_WKF_Msk) == UI2C_WKSTS_WKF_Msk)
    {
        printf("A\n");
        g_u32WKfromAddr = 1;
        g_u8SlvI2CWK = 1;

        /* Waiting for Wakeup done */
        while ((UI2C0->PROTSTS & UI2C_PROTSTS_WKAKDONE_Msk) == 0) {};

        /* Clear WK flag */
        UI2C_CLR_WAKEUP_FLAG(UI2C0);

        UI2C0->PROTSTS = UI2C_PROTSTS_WKAKDONE_Msk;

        return;
    }

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (s_Event == SLAVE_ADDRESS_ACK)
        {
            g_u8DataLenS = 0;

            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                s_Event = SLAVE_SEND_DATA;
                UI2C_SET_DATA(UI2C0, g_au8SlvData[g_u32SlaveBuffAddr]);
                g_u32SlaveBuffAddr++;
            }
            else
            {
                s_Event = SLAVE_GET_DATA;
            }

            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_GET_DATA)
        {
            g_au8RxData[g_u8DataLenS] = (uint8_t)UI2C_GET_DATA(UI2C0);
            g_u8DataLenS++;

            if (g_u8DataLenS == 2)
            {
                /* Address has been received; ACK has been returned*/
                g_u32SlaveBuffAddr = (g_au8RxData[0] << 8) + g_au8RxData[1];
            }

            if (g_u8DataLenS == 3)
            {
                g_au8SlvData[g_u32SlaveBuffAddr] = g_au8RxData[2];
                g_u8DataLenS = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);

        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

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

    /* Enable GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOCCKEN_Msk;

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

    /* Calculate UI2C clock divider */
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

    /* Set UI2C0 clock divider */
    UI2C0->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C0->BRGEN |= (u32Clkdiv << UI2C_BRGEN_CLKDIV_Pos);


    /* Set UI2C0 Slave Addresses */
    UI2C0->DEVADDR0 = 0x15;   /* Slave Address : 0x15 */
    UI2C0->DEVADDR1 = 0x35;   /* Slave Address : 0x35 */

    /* Set UI2C0 Slave Addresses Msk */
    UI2C0->ADDRMSK0 = 0x1;   /* Slave Address : 0x1 */
    UI2C0->ADDRMSK1 = 0x4;   /* Slave Address : 0x4 */

    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);

    /* Enable UI2C protocol */
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
    uint32_t u32Index;
    uint8_t  u8Char;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART for print message */
    UART_Init();

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+-------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for 7-bit address Slave  |\n");
    printf("|  Wake-up from power down mode. This sample code needs |\n");
    printf("|  work with USCI_I2C_Wakeup_Master sample code         |\n");
    printf("|          UI2C0(Master)  <----> UI2C0(Slave)           |\n");
    printf("|  This sample code requires two boards for testing     |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a Slave\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PC.3), UI2C0_SCL(PC.0)\n\n");

    /* Init UI2C0 100KHz */
    UI2C0_Init(100000);

    printf("[T] I/O Toggle Wake-up Mode\n");
    printf("[A] Address Match Wake-up Mode\n");
    printf("Select: ");
    u8Char =  getchar();

    if ((u8Char == 'T') || (u8Char == 't'))
    {
        printf("(T)oggle\n");

        /* Enable UI2C0 toggle mode wake-up */
        UI2C0->WKCTL &= ~UI2C_WKCTL_WKADDREN_Msk;

        s_Event = SLAVE_ADDRESS_ACK;

        /* I2C function to Slave receive/transmit data */
        s_UI2C0HandlerFn = UI2C_SlaveToggleWakeup;
    }
    else
    {
        /* Default Mode*/
        printf("(A)ddress math\n");

        /* Enable UI2C0 address match mode wake-up */
        UI2C0->WKCTL |= UI2C_WKCTL_WKADDREN_Msk;

        s_Event = SLAVE_GET_DATA;

        /* UI2C0 function to Slave receive/transmit data */
        s_UI2C0HandlerFn = UI2C_SlaveAddressWakeup;
    }

    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    for (u32Index = 0; u32Index < 0x100; u32Index++)
    {
        g_au8SlvData[u32Index] = 0;
    }

    /* Enable power wake-up interrupt */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_EnableIRQ(PWRWU_IRQn);

    /* Enable Ui2C0 Wake-up */
    UI2C0->WKCTL |= UI2C_WKCTL_WKEN_Msk;

    /* System power down enable */
    printf("Enter PD 0x%x 0x%x\n", UI2C0->PROTCTL, UI2C_GET_PROT_STATUS(UI2C0));
    printf("\nCHIP enter power down status.\n");

    /* Waiting for UART printf finish*/
    while ((UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

    /* Clear flage before enter power-down mode */
    if (UI2C0->PROTSTS != 0)
        UI2C0->PROTSTS = UI2C_GET_PROT_STATUS(UI2C0);

    /* Set the processor uses power-down sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled*/
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

    while (g_u8SlvPWRDNWK == 0);

    while (g_u8SlvI2CWK == 0);

    if (g_u32WKfromAddr)
        printf("UI2C0 [A]ddress match Wake-up from power down\n");
    else
        printf("UI2C0 [T]oggle Wake-up from power down\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
