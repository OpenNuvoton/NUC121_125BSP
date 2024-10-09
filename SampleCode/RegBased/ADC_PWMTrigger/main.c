/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger ADC by PWM.
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

    /* Enable PWM0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_PWM0CKEN_Msk;

    /* Select PWM0 module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_PWM0SEL_Msk) | CLK_CLKSEL1_PWM0SEL_PCLK0;

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

    /* Configure the PC10 as PWM0 output pin */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk)) | SYS_GPC_MFPH_PC10MFP_PWM0_CH0;
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
/* Function: ADC_PWMTrigTest_SingleOpMode                                                                   */
/*                                                                                                          */
/* Parameters:                                                                                              */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Returns:                                                                                                 */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Description:                                                                                             */
/*   ADC hardware trigger test.                                                                             */
/*----------------------------------------------------------------------------------------------------------*/
void ADC_PWMTrigTest_SingleOpMode(void)
{
    printf("\n<<< PWM trigger test (Single mode) >>>\n");

    /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 2 */
    ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                ADC_ADCR_DIFFEN_SINGLE_END |
                ADC_ADCR_ADMD_SINGLE;
    ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0x1 << 2);

    /* Power on ADC module */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Configure the hardware trigger condition and enable hardware trigger; PWM trigger delay: (4*10) system clock cycles*/
    ADC->ADCR &= ~(ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGCOND_Msk | ADC_ADCR_TRGEN_Msk);
    ADC->ADTDCR = (ADC->ADTDCR & ~ADC_ADTDCR_PTDT_Msk) | 0;
    ADC->ADCR |= ADC_ADCR_TRGS_PWM | ADC_ADCR_TRGEN_Msk;

    /* Clear the A/D interrupt flag for safe */
    ADC->ADSR0 = ADC_ADF_INT;

    /* Center-aligned type */
    PWM_SET_ALIGNED_TYPE(PWM0, PWM_CH_0_MASK, PWM_CENTER_ALIGNED);

    /* Clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 1);

    /* PWM counter value */ /* PWM frequency = PWM clock source/(clock prescaler setting + 1)/(CNR+1) */

    PWM0->PERIOD[0] = 5;

    /* PWM compare value */
    PWM0->CMPDAT[0] = 1;

    /* Enable PWM0 to trigger ADC */
    PWM0->ADCTS0 &= ~((PWM_ADCTS0_TRGSEL0_Msk) << (0 * 8));
    PWM0->ADCTS0 |= ((PWM_ADCTS0_TRGEN0_Msk | PWM_TRIGGER_ADC_EVEN_PERIOD_POINT) << (0 * 8));

    /* PWM0 pin output enabled */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING);

    /* PWM_EnableOutput(PWM0, PWM_CH_0_MASK); */
    PWM0->POEN |= PWM_CH_0_MASK;

    /* Start PWM module */
    PWM0->CNTEN |= PWM_CH_0_MASK;

    /* wait for one cycle */
    while ((PWM0->INTSTS0 & PWM_INTSTS0_PIF0_Msk) == 0)

        while ((PWM0->INTSTS0 & PWM_INTSTS0_ZIF0_Msk) == 0)

            //PWM_ClearPeriodIntFlag(PWM0, 0);
            PWM0->INTSTS0 = PWM_INTSTS0_PIF0_Msk;

    //PWM_ClearZeroIntFlag(PWM0, 0);
    PWM0->INTSTS0 = PWM_INTSTS0_ZIF0_Msk;

    /* Stop PWM generation */
    PWM0->CNTEN &= ~PWM_CH_0_MASK;

    /* Wait conversion done */
    while (!(ADC->ADSR0 & ADC_ADF_INT));

    /* Clear the ADC interrupt flag */
    ADC->ADSR0 = ADC_ADF_INT;

    printf("Channel 2: 0x%X\n", (uint32_t)(ADC->ADDR[2] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos);

    /* Disable ADC */
    ADC->ADCR &= (~ADC_ADCR_ADEN_Msk);
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

    /* ADC hardware trigger test */
    ADC_PWMTrigTest_SingleOpMode();

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
