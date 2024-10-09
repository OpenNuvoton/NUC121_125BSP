/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Perform A/D Conversion with ADC continuous scan mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /*------------------------------------------------------------------------------------------------------*/
    /* Enable Module Clock                                                                                  */
    /*------------------------------------------------------------------------------------------------------*/

    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Enable ADC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_ADCCKEN_Msk;

    /* Set ADC clock source to HIRC=48MHz, set divider to 3, ADC clock will be 16 MHz */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_ADCDIV_Msk) | (((3) - 1) << CLK_CLKDIV0_ADCDIV_Pos);
    CLK->CLKSEL1 &= (~CLK_CLKSEL1_ADCSEL_Msk);
    CLK->CLKSEL1 |= CLK_CLKSEL1_ADCSEL_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*------------------------------------------------------------------------------------------------------*/
    /* Initiate I/O Multi-function                                                                          */
    /*------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set PD.0 ~ PD.3 to input mode */
    PD->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Set PD0 ~ PD3 to ADC mode for ADC input channel 0 ~ 3 */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_ADC_CH0 | SYS_GPD_MFPL_PD1MFP_ADC_CH1 | SYS_GPD_MFPL_PD2MFP_ADC_CH2 | SYS_GPD_MFPL_PD3MFP_ADC_CH3);

    /* Disable the digital input paths of ADC analog pins */
    PD->DINOFF |= 0x000F0000;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: ADC_GetConversionRate                                                                         */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   Return the A/D conversion rate (sample/second)                                                        */
/*                                                                                                         */
/* Description:                                                                                            */
/*   The conversion rate depends on the clock source of ADC clock.                                         */
/*   It only needs 21 ADC clocks to complete an A/D conversion.                                            */
/*---------------------------------------------------------------------------------------------------------*/
static __INLINE uint32_t ADC_GetConversionRate()
{
    uint32_t u32AdcClkSrcSel;
    uint32_t au32ClkTbl[4] = {__HXT, 0, 0, __HIRC};

    /* Set the PLL clock frequency */
    au32ClkTbl[1] = PllClock;
    /* Set the system core clock frequency */
    au32ClkTbl[2] = SystemCoreClock;
    /* Get the clock source setting */
    u32AdcClkSrcSel = ((CLK->CLKSEL1 & CLK_CLKSEL1_ADCSEL_Msk) >> CLK_CLKSEL1_ADCSEL_Pos);
    /* Return the ADC conversion rate */
    return ((au32ClkTbl[u32AdcClkSrcSel]) / (((CLK->CLKDIV0 & CLK_CLKDIV0_ADCDIV_Msk) >> CLK_CLKDIV0_ADCDIV_Pos) + 1) / 21);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcContScanModeTest                                                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC continuous scan mode test.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AdcContScanModeTest(void)
{
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;

    printf("\n\nConversion rate: %u samples/second\n", ADC_GetConversionRate());
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC continuous scan mode sample code                 |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 2 cycles of conversion result from the specified channels.\n");

    while (1)
    {
        uint8_t  u8Option;

        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit continuous scan mode test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set the ADC operation mode as continuous scan, input mode as single-end and
                 enable the analog input channel 0, 1, 2 and 3 */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                        ADC_ADCR_DIFFEN_SINGLE_END |
                        ADC_ADCR_ADMD_CONTINUOUS;
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0xF);

            /* Power on ADC module */
            ADC->ADCR |= ADC_ADCR_ADEN_Msk;

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait conversion done */
            while (!(ADC->ADSR0 & ADC_ADF_INT));

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            for (u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[u32ChannelCount] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;

                printf("Conversion result of channel %u: 0x%X (%i)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Wait conversion done */
            while (!(ADC->ADSR0 & ADC_ADF_INT));

            /* Stop A/D conversion */
            ADC->ADCR &= (~ADC_ADCR_ADST_Msk);

            for (u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[u32ChannelCount] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;

                printf("Conversion result of channel %u: 0x%X (%i)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;
        }
        else if (u8Option == '2')
        {
            /* Set the ADC operation mode as continuous scan, input mode as differential and
               enable analog input channel 0 and 2 */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                        ADC_ADCR_DIFFEN_DIFFERENTIAL |
                        ADC_ADCR_ADMD_CONTINUOUS;
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0x5);

            /* Power on ADC module */
            ADC->ADCR |= ADC_ADCR_ADEN_Msk;

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait conversion done */
            while (!(ADC->ADSR0 & ADC_ADF_INT));

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            for (u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[u32ChannelCount * 2] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;

                printf("Conversion result of differential input pair %u: 0x%X (%i)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Wait conversion done */
            while (!(ADC->ADSR0 & ADC_ADF_INT));

            /* Stop A/D conversion */
            ADC->ADCR &= (~ADC_ADCR_ADST_Msk);

            for (u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[u32ChannelCount * 2] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;

                printf("Conversion result of differential input pair %u: 0x%X (%i)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;
        }
        else
            return ;

    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Initiate UART0 to 115200-8n1 for print message */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %u Hz", SystemCoreClock);

    /* Continuous scan mode test */
    AdcContScanModeTest();

    /* Disable ADC module */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= (~CLK_APBCLK0_ADCCKEN_Msk);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("\nExit ADC sample code\n");

    while (1);
}
