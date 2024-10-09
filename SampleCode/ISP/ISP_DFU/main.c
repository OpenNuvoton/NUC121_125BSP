/******************************************************************************//**
 * @file     main.c
 * @version  V3.01
 * @brief
 *           Demonstrate how to upgrade firmware between USB device and PC through USB DFU (Device Firmware Upgrade) class.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "dfu_transfer.h"

#define DetectPin           PB0
#define HIRC_AUTO_TRIM      (SYS_IRCTCTL_REFCKSEL_Msk | 0x2);   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x110)
#define TRIM_THRESHOLD      16      /* Each value is 0.125%, max 2% */
#define HCLK_DIV            1
#define USBD_DIV            1
#define PLL_CLOCK           48000000

static volatile uint32_t s_u32DefaultTrim, s_u32LastTrim;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal HIRC 48 MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 , USB clock source divide 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(HCLK_DIV);
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBDIV_Msk) | CLK_CLKDIV0_USB(USBD_DIV);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
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

    /* Enable FMC ISP and APROM update function */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_ISPFF_Msk;

    /* Open USB controller */
    USBD_Open(&gsInfo, DFU_ClassRequest, NULL);

    /* Init Endpoint configuration for DFU */
    DFU_Init();

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
    }

    /* Reset to boot from APROM */
    FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;
    // Wait system reset
    NVIC_SystemReset();
}
