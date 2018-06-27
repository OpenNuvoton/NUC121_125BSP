/******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Demonstrate how to implement a composite device. (HID Transfer and keyboard)
 *           Transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "HID_Transfer_and_Keyboard.h"


/*--------------------------------------------------------------------------*/
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


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /*
        This sample code demonstrate how to use HID interface to transfer data
        between PC and USB device.
        A demo window tool is also included in "WindowsTool" directory with this
        sample code. User can use it to test data transfer with this sample code.
    */

    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init system and multi-function I/O */
    SYS_Init();

    /* Init UART for debug message */
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|        NuMicro USB Composite Device Sample Code        |\n");
    printf("|               (USB HID Transfer + Keyboard)            |\n");
    printf("+--------------------------------------------------------+\n");
    printf("If PB.15 = 0, just report key 'a'.\n");
    /* Set PB.15 as Quasi-bidirectional mode */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE15_Msk) | (GPIO_MODE_QUASI << GPIO_MODE_MODE15_Pos);

    /* Open USB controller */
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration for HID */
    HID_Init();

    /* Start USB device */
    USBD_Start();

    /* Enable USB device interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

    while (1)
    {
        HID_UpdateKbData();
    }
}



/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

