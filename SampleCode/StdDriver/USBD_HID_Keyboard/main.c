/******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Demonstrate how to implement a USB keyboard device.
 *           It supports to use GPIO to simulate key input.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC121.h"
#include "hid_kb.h"

/*--------------------------------------------------------------------------*/
uint8_t volatile g_u8EP2Ready = 0;



void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal HIRC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;


}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void HID_UpdateKbData(void)
{
    int32_t i;
    uint8_t *buf;
    uint32_t key = 0xF;
    static uint32_t preKey;

    if (g_u8EP2Ready) {
        buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));

        /* If PB.15 = 0, just report it is key 'a' */
        key = (PB->PIN & (1 << 15)) ? 0 : 1;

        if (key == 0) {
            for (i = 0; i < 8; i++) {
                buf[i] = 0;
            }

            if (key != preKey) {
                /* Trigger to note key release */
                USBD_SET_PAYLOAD_LEN(EP2, 8);
            }
        } else {
            preKey = key;
            buf[2] = 0x04; /* Key A */
            USBD_SET_PAYLOAD_LEN(EP2, 8);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|          NuMicro USB HID Keyboard Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");
    printf("If PB.15 = 0, just report it is key 'a'.\n");

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);

    /* start to IN data */
    g_u8EP2Ready = 1;

    while (1) {
        HID_UpdateKbData();
    }
}



/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

