/******************************************************************************//**
 * @file     main.c
 * @version  V3.01
 * @brief
 *           Demonstrate how to update chip flash data through USB HID interface
 *           between chip USB device and PC.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip USB device and assign update file
 *           of Flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "hid_transfer.h"

#define HIRC_AUTO_TRIM      (SYS_IRCTCTL_REFCKSEL_Msk | 0x2);   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x110)
#define TRIM_THRESHOLD      16      /* Each value is 0.125%, max 2% */

static volatile uint32_t s_u32DefaultTrim, s_u32LastTrim;

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal HIRC 48 MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    /* Enable IP clock */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();
    /* Init system and multi-funcition I/O */
    SYS_Init();
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;    // (1ul << 0)
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    while (DetectPin == 0)
    {
        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);
        /*Init Endpoint configuration for HID */
        HID_Init();
        /* Start USB device */
        USBD_Start();
        /* Enable USB device interrupt */
        NVIC_EnableIRQ(USBD_IRQn);

        /* Backup default trim value */
        s_u32DefaultTrim = M32(TRIM_INIT);
        s_u32LastTrim = s_u32DefaultTrim;

        /* Clear SOF */
        USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

        while (DetectPin == 0)
        {
            /* Start USB trim function if it is not enabled. */
            if ((SYS->IRCTCTL & SYS_IRCTCTL_FREQSEL_Msk) != 0x2)
            {
                /* Start USB trim only when USB signal arrived */
                if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
                {
                    /* Clear SOF */
                    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                    /* Enable USB clock trim function */
                    SYS->IRCTCTL = HIRC_AUTO_TRIM;
                }
            }

            /* Disable USB Trim when any error found */
            if (SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk))
            {
                /* Last TRIM */
                M32(TRIM_INIT) = s_u32LastTrim;

                /* Disable USB clock trim function */
                SYS->IRCTCTL = 0;

                /* Clear trim error flags */
                SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk;

                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

            }

            /* Check trim value whether it is over the threshold */
            if ((M32(TRIM_INIT) > (s_u32DefaultTrim + TRIM_THRESHOLD)) || (M32(TRIM_INIT) < (s_u32DefaultTrim - TRIM_THRESHOLD)))
            {
                /* Write updated value */
                M32(TRIM_INIT) = s_u32LastTrim;
            }
            else
            {
                /* Backup trim value */
                s_u32LastTrim =  M32(TRIM_INIT);
            }

            if (bUsbDataReady == TRUE)
            {
                WDT->CTL &= ~(WDT_CTL_WDTEN_Msk | WDT_CTL_ICEDEBUG_Msk);
                WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_CTL_RSTEN_Msk);
                ParseCmd((uint8_t *)usb_rcvbuf, EP3_MAX_PKT_SIZE);
                EP2_Handler();
                bUsbDataReady = FALSE;
            }
        }

        goto _APROM;
    }

    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

_APROM:
    outpw(&SYS->RSTSTS, 3); // Clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}
