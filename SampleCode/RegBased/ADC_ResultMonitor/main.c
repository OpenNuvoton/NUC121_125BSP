/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Monitor the conversion result of channel 2 by the digital compare function.
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
volatile uint32_t g_u32AdcCmp0IntFlag;
volatile uint32_t g_u32AdcCmp1IntFlag;

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

    /* Set PD.2 to input mode */
    PD->MODE &= ~(GPIO_MODE_MODE2_Msk);

    /* Set PD2 to ADC mode for ADC input channel 2 */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD2MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD2MFP_ADC_CH2);

    /* Disable the digital input paths of ADC analog pins */
    PD->DINOFF |= 0x00040000;
}

void UART0_Init(void)
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
/* Function: AdcResultMonitorTest                                                                           */
/*                                                                                                          */
/* Parameters:                                                                                              */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Returns:                                                                                                 */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Description:                                                                                             */
/*   ADC result monitor function test.                                                                      */
/*----------------------------------------------------------------------------------------------------------*/
void AdcResultMonitorTest(void)
{
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|           ADC compare function (result monitor) sample code          |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Set the ADC operation mode as continuous scan, input mode as single-end and enable the analog input channel 2 */
    ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                ADC_ADCR_DIFFEN_SINGLE_END |
                ADC_ADCR_ADMD_CONTINUOUS;
    ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0x1 << 2);

    /* Power on ADC module */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Enable ADC comparator 0. Compare condition: conversion result < 0x800; match Count=5. */
    printf("   Set the compare condition of comparator 0: channel 2 is less than 0x800; match count is 5.\n");
    ADC->ADCMPR[0] = (2 << ADC_ADCMPR_CMPCH_Pos) | ADC_ADCMPR_CMPCOND_LESS_THAN |
                     (0x800 << ADC_ADCMPR_CMPD_Pos) | ((5 - 1) << ADC_ADCMPR_CMPMATCNT_Pos) |
                     ADC_ADCMPR_CMPEN_Msk;

    /* Enable ADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5. */
    printf("   Set the compare condition of comparator 1: channel 2 is greater than or equal to 0x800; match count is 5.\n");
    ADC->ADCMPR[1] = (2 << ADC_ADCMPR_CMPCH_Pos) | ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL |
                     (0x800 << ADC_ADCMPR_CMPD_Pos) | ((5 - 1) << ADC_ADCMPR_CMPMATCNT_Pos) |
                     ADC_ADCMPR_CMPEN_Msk;

    /* Clear the ADC comparator 0 interrupt flag for safe */
    ADC->ADSR0 = ADC_CMP0_INT;

    /* Enable ADC comparator 0 interrupt */
    ADC->ADCMPR[0] |= ADC_ADCMPR_CMPIE_Msk;

    /* Clear the ADC comparator 1 interrupt flag for safe */
    ADC->ADSR0 = ADC_CMP1_INT;

    /* Enable ADC comparator 1 interrupt */
    ADC->ADCMPR[1] |= ADC_ADCMPR_CMPIE_Msk;

    NVIC_EnableIRQ(ADC_IRQn);

    g_u32AdcCmp0IntFlag = 0;
    g_u32AdcCmp1IntFlag = 0;

    /* Clear the ADC interrupt flag */
    ADC->ADSR0 = ADC_ADF_INT;

    /* Start A/D conversion */
    ADC->ADCR |= ADC_ADCR_ADST_Msk;

    /* Wait ADC compare interrupt */
    while ((g_u32AdcCmp0IntFlag == 0) && (g_u32AdcCmp1IntFlag == 0));

    /* Stop A/D conversion */
    ADC->ADCR &= (~ADC_ADCR_ADST_Msk);

    /* Disable ADC comparator interrupt */
    ADC->ADCMPR[0] &= (~ADC_ADCMPR_CMPIE_Msk);

    ADC->ADCMPR[1] &= (~ADC_ADCMPR_CMPIE_Msk);

    /* Disable compare function */
    ADC->ADCMPR[0] = 0;

    /* ADC_DISABLE_CMP1(ADC); */
    ADC->ADCMPR[1] = 0;

    if (g_u32AdcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 2 is less than 0x800\n");
    }
    else if (g_u32AdcCmp1IntFlag == 1)
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 2 is greater than or equal to 0x800\n");
    }
    else
    {
        printf("Both Comparator 0 and 1 have no interrupt occurs.\n");
    }
}


/*----------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
void ADC_IRQHandler(void)
{
    if ((ADC->ADSR0 & ADC_CMP0_INT) != 0)
    {
        g_u32AdcCmp0IntFlag = 1;
        /* clear the A/D compare flag 0 */
        ADC->ADSR0 = ADC_CMP0_INT;
    }

    if ((ADC->ADSR0 & ADC_CMP1_INT) != 0)
    {
        g_u32AdcCmp1IntFlag = 1;
        /* clear the A/D compare flag 1 */
        ADC->ADSR0 = ADC_CMP1_INT;
    }
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

    /* Result monitor test */
    AdcResultMonitorTest();

    /* Disable ADC module */
    /* ADC_Close(ADC); */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= (~CLK_APBCLK0_ADCCKEN_Msk);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("\nExit ADC sample code\n");

    while (1);
}
