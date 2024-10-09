/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Perform A/D Conversion with ADC burst mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                               */
/*----------------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


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

    /* Set PD.2 and PD.3 to input mode */
    PD->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Set PD2 and PD3 to ADC mode for ADC input channel 2 and 3 */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD2MFP_ADC_CH2 | SYS_GPD_MFPL_PD3MFP_ADC_CH3);

    /* Disable the digital input paths of ADC analog pins */
    PD->DINOFF |= 0x000C0000;
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

/*----------------------------------------------------------------------------------------------------------*/
/* Function: AdcBurstModeTest                                                                              */
/*                                                                                                          */
/* Parameters:                                                                                              */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Returns:                                                                                                 */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Description:                                                                                             */
/*   ADC burst mode test.                                                                                  */
/*----------------------------------------------------------------------------------------------------------*/
void AdcBurstModeTest(void)
{
    uint32_t u32ConversionCount;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADC burst mode sample code                     |\n");
    printf("+----------------------------------------------------------------------+\n");

    while (1)
    {
        uint8_t  u8Option;

        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set the ADC operation mode as burst, input mode as single-end and enable the analog input channel 2 */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                        ADC_ADCR_DIFFEN_SINGLE_END |
                        ADC_ADCR_ADMD_BURST;
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0x1 << 2);

            /* Power on ADC module */
            ADC->ADCR |= ADC_ADCR_ADEN_Msk;

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Enable the ADC interrupt */
            ADC->ADCR |= ADC_ADCR_ADIE_Msk;

            NVIC_EnableIRQ(ADC_IRQn);

            /* Reset the ADC interrupt indicator and Start A/D conversion */
            g_u32AdcIntFlag = 0;

            /* ADC_START_CONV(ADC); */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) when more than 8 samples in buffer */
            while (g_u32AdcIntFlag == 0);

            /* Always read conversion data from channel "0" in burst mode. */
            for (u32ConversionCount = 0; u32ConversionCount < 32; u32ConversionCount++)
            {
                i32ConversionData = (ADC->ADDR[0] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;

                printf("Conversion result of channel 2 count %u: 0x%X (%i)\n", u32ConversionCount, i32ConversionData, i32ConversionData);
            }

            /* Stop burst mode conversion */
            ADC->ADCR &= (~ADC_ADCR_ADST_Msk);

            /* Disable the ADC interrupt */
            ADC->ADCR &= (~ADC_ADCR_ADIE_Msk);

            /* Close and reset ADC engine */
            SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
            SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

        }
        else if (u8Option == '2')
        {
            /* Set the ADC operation mode as burst, input mode as differential and
               enable analog input channel 2 for differential input channel pair 1 */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                        ADC_ADCR_DIFFEN_DIFFERENTIAL |
                        ADC_ADCR_ADMD_BURST;
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0x1 << 2);

            /* Power on ADC module */
            ADC->ADCR |= ADC_ADCR_ADEN_Msk;

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Enable the ADC interrupt */
            ADC->ADCR |= ADC_ADCR_ADIE_Msk;

            /* NVIC_EnableIRQ(ADC_IRQn); */
            NVIC->ISER[0] = (1 << ((uint32_t)(ADC_IRQn) & 0x1F));

            /* Reset the ADC interrupt indicator and Start A/D conversion */
            g_u32AdcIntFlag = 0;

            /* Start burst mode conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) when more than 8 samples in buffer */
            while (g_u32AdcIntFlag == 0);

            /* Always read conversion data from channel "0" in burst mode. */
            for (u32ConversionCount = 0; u32ConversionCount < 32; u32ConversionCount++)
            {
                i32ConversionData = (ADC->ADDR[0] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;

                printf("Conversion result of channel pair 1 count %u: 0x%X (%d)\n", u32ConversionCount, i32ConversionData, i32ConversionData);
            }

            /* Stop burst mode conversion */
            ADC->ADCR &= (~ADC_ADCR_ADST_Msk);

            /* Disable the ADC interrupt */
            ADC->ADCR &= (~ADC_ADCR_ADIE_Msk);

            /* Close and reset ADC engine */
            SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
            SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;
        }
        else
            return ;
    }
}



/*----------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    /* clear the A/D conversion flag */
    ADC->ADSR0 = ADC_ADF_INT;
}


/*----------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                            */
/*----------------------------------------------------------------------------------------------------------*/
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

    /*------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                          */
    /*------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %u Hz", SystemCoreClock);

    /* Burst Mode test */
    AdcBurstModeTest();

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
