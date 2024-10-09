/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Perform A/D Conversion with ADC single cycle scan mode and transfer result by PDMA.
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
#define PDMA_CH 2
#define ADC_TEST_COUNT 32

uint32_t g_au32RxPDMADestination[ADC_TEST_COUNT];
uint32_t au32AdcData[ADC_TEST_COUNT];

volatile uint32_t g_u32PdmaTDoneInt;
volatile uint32_t g_u32PdmaTAbortInt;


void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA->INTSTS;

    uint32_t u32TDStatus = PDMA->TDSTS;

    if (u32Status & PDMA_INTSTS_TDIF_Msk)
    {
        if (u32TDStatus & PDMA_TDSTS_TDIF0_Msk)   /* CH0 */
        {
            g_u32PdmaTDoneInt = 1;
            PDMA->TDSTS = PDMA_TDSTS_TDIF0_Msk;
        }
        else if (u32TDStatus & PDMA_TDSTS_TDIF1_Msk)     /* CH1 */
        {
            g_u32PdmaTDoneInt = 2;
            PDMA->TDSTS = PDMA_TDSTS_TDIF1_Msk;
        }
        else if (u32TDStatus & PDMA_TDSTS_TDIF2_Msk)     /* CH2 */
        {
            g_u32PdmaTDoneInt = 3;
            PDMA->TDSTS = PDMA_TDSTS_TDIF2_Msk;
        }
        else if (u32TDStatus & PDMA_TDSTS_TDIF3_Msk)     /* CH3 */
        {
            g_u32PdmaTDoneInt = 4;
            PDMA->TDSTS = PDMA_TDSTS_TDIF3_Msk;
        }
        else if (u32TDStatus & PDMA_TDSTS_TDIF4_Msk)     /* CH4 */
        {
            g_u32PdmaTDoneInt = 5;
            PDMA->TDSTS = PDMA_TDSTS_TDIF4_Msk;
        }
    }
}


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
    /* Init I/O Multi-function                                                                              */
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
/* Function: PDMA_Init                                                                                      */
/*                                                                                                          */
/* Parameters:                                                                                              */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Returns:                                                                                                 */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Description:                                                                                             */
/*   Configure PDMA channel for ADC continuous scan mode test.                                              */
/*----------------------------------------------------------------------------------------------------------*/
void PDMA_Init(void)
{
    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Open Channel 2 */
    PDMA->CHCTL |= (1 << PDMA_CH);

    /* Transfer configuration of Channel 2 */
    PDMA->DSCT[PDMA_CH].CTL = \
                              ((ADC_TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is ADC_TEST_COUNT */ \
                              PDMA_WIDTH_32 |  /* Transfer width is 32 bits(one word) */ \
                              PDMA_SAR_FIX |   /* Source increment size is no increment(fixed) */ \
                              PDMA_DAR_INC |   /* Destination increment size is 32 bits(one word) */ \
                              PDMA_REQ_SINGLE | /* Transfer type is burst transfer type */ \
                              PDMA_BURST_1 |   /* Burst size is 1. No effect in single transfer type */ \
                              PDMA_OP_BASIC;   /* Operation mode is basic mode */

    /* Configure source address */
    PDMA->DSCT[PDMA_CH].SA = (uint32_t)&ADC->ADPDMA;

    /* Configure destination address */
    PDMA->DSCT[PDMA_CH].DA = (uint32_t)&g_au32RxPDMADestination;

    /* Configure PDMA channel 2 as memory to memory transfer */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Pos) | (PDMA_ADC_RX << PDMA_REQSEL0_3_REQSRC2_Pos);

    /* Enable transfer done interrupt */
    PDMA->INTEN |= (1 << PDMA_CH);

    /* Enable PDMA IRQ */
    /* NVIC_EnableIRQ(PDMA_IRQn); */
    NVIC->ISER[0] = (1 << ((uint32_t)(PDMA_IRQn) & 0x1F));
}


/*----------------------------------------------------------------------------------------------------------*/
/* Function: PDMA_ConfigReload                                                                              */
/*                                                                                                          */
/* Parameters:                                                                                              */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Returns:                                                                                                 */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Description:                                                                                             */
/*   Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test.            */
/*----------------------------------------------------------------------------------------------------------*/
void PDMA_ConfigReload()
{
    /* Transfer configuration of Channel 2 */
    PDMA->DSCT[PDMA_CH].CTL |= \
                               ((ADC_TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is ADC_TEST_COUNT */ \
                               PDMA_OP_BASIC;   /* Operation mode is basic mode */
}


/*----------------------------------------------------------------------------------------------------------*/
/* Function: AdcSingleCycleScanModePDMATest                                                                 */
/*                                                                                                          */
/* Parameters:                                                                                              */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Returns:                                                                                                 */
/*   None.                                                                                                  */
/*                                                                                                          */
/* Description:                                                                                             */
/*   ADC single cycle scan mode with PDMA test.                                                             */
/*----------------------------------------------------------------------------------------------------------*/
void AdcSingleCycleScanModePDMATest(void)
{
    uint32_t u32DataCount;
    uint32_t u32ErrorCount;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC single cycle scan mode with PDMA sample code     |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Init PDMA channel to transfer ADC conversion results */
    PDMA_Init();

    while (1)
    {
        uint8_t  u8Option;

        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit single cycle scan mode with PDMA test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test. */
            PDMA_ConfigReload();

            /* Set the ADC operation mode as single-cycle, input mode as single-end and
                 enable the analog input channel 0, 1, 2 and 3 */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                        ADC_ADCR_DIFFEN_SINGLE_END |
                        ADC_ADCR_ADMD_SINGLE_CYCLE;
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | 0xF;

            /* Enable PDMA transfer */
            ADC->ADCR |= ADC_ADCR_PTEN_Msk;

            /* Power on ADC module */
            ADC->ADCR |= ADC_ADCR_ADEN_Msk;

            /* Clear destination buffer */
            for (u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
                g_au32RxPDMADestination[u32DataCount] = 0;

            u32DataCount = 0;
            u32ErrorCount = 0;
            g_u32PdmaTDoneInt = 0;

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            while (1)
            {

                if (ADC->ADSR0 & ADC_ADF_INT)
                {
                    /* Clear the ADC interrupt flag */
                    ADC->ADSR0 = ADC_ADF_INT;

                    uint32_t u32Ch;

                    for (u32Ch = 0; u32Ch < 4; u32Ch++)
                    {
                        au32AdcData[u32DataCount++] = (ADC->ADDR[u32Ch] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;

                        if (u32DataCount >= ADC_TEST_COUNT)
                            break;
                    }

                    if (u32DataCount < ADC_TEST_COUNT)
                        /* Start A/D conversion */
                        ADC->ADCR |= ADC_ADCR_ADST_Msk;
                    else
                        break;
                }
            }

            /* Wait for PDMA transfer done */
            while (g_u32PdmaTDoneInt == 0);

            /* Compare the log of ADC conversion data register with the content of PDMA target buffer */
            for (u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
            {
                if (au32AdcData[u32DataCount] != (g_au32RxPDMADestination[u32DataCount] & 0xFFF))
                {
                    printf("*** Count %u, conversion result: 0x%X, PDMA result: 0x%X.\n",
                           u32DataCount, au32AdcData[u32DataCount], g_au32RxPDMADestination[u32DataCount]);
                    u32ErrorCount++;
                }
            }

            if (u32ErrorCount == 0)
                printf("\n    PASS!\n");
            else
                printf("\n    FAIL!\n");
        }
        else if (u8Option == '2')
        {
            /* Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test. */
            PDMA_ConfigReload();

            /* Set the ADC operation mode as single-cycle, input mode as differential and
               enable analog input channel 0 and 2 */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                        ADC_ADCR_DIFFEN_DIFFERENTIAL |
                        ADC_ADCR_ADMD_SINGLE_CYCLE;
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | 0x5;

            /* Enable PDMA transfer */
            ADC->ADCR |= ADC_ADCR_PTEN_Msk;

            /* Power on ADC module */
            ADC->ADCR |= ADC_ADCR_ADEN_Msk;

            /* Clear destination buffer */
            for (u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
                g_au32RxPDMADestination[u32DataCount] = 0;

            u32DataCount = 0;
            u32ErrorCount = 0;
            g_u32PdmaTDoneInt = 0;

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            while (1)
            {

                if (ADC->ADSR0 & ADC_ADF_INT)
                {
                    /* Clear the ADC interrupt flag */
                    ADC->ADSR0 = ADC_ADF_INT;

                    uint32_t u32Ch;

                    for (u32Ch = 0; u32Ch < 4; u32Ch += 2)
                    {
                        au32AdcData[u32DataCount++] = (ADC->ADDR[u32Ch] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;

                        if (u32DataCount >= ADC_TEST_COUNT)
                            break;
                    }

                    if (u32DataCount < ADC_TEST_COUNT)
                        /* Start A/D conversion */
                        ADC->ADCR |= ADC_ADCR_ADST_Msk;
                    else
                        break;
                }
            }

            /* Wait for PDMA transfer done */
            while (g_u32PdmaTDoneInt == 0);

            /* Compare the log of ADC conversion data register with the content of PDMA target buffer */
            for (u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
            {
                if (au32AdcData[u32DataCount] != (g_au32RxPDMADestination[u32DataCount] & 0xFFF))
                {
                    printf("*** Count %u, conversion result: 0x%X, PDMA result: 0x%X.\n",
                           u32DataCount, au32AdcData[u32DataCount], g_au32RxPDMADestination[u32DataCount]);
                    u32ErrorCount++;
                }
            }

            if (u32ErrorCount == 0)
                printf("\n    PASS!\n");
            else
                printf("\n    FAIL!\n");
        }
        else
            return ;

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

    /* Single cycle scan mode test */
    AdcSingleCycleScanModePDMATest();

    /* Disable ADC module */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= (~CLK_APBCLK0_ADCCKEN_Msk);

    /* Disable PDMA IP clock */
    CLK->AHBCLK &= (~CLK_AHBCLK_PDMACKEN_Msk);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_DisableIRQ(PDMA_IRQn);

    printf("\nExit ADC sample code\n");

    while (1);
}
