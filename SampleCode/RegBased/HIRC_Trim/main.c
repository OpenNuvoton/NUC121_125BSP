
/******************************************************************************
 * @file        main.c
 * @version     V3.00
 * @brief       How to use HIRC trim by LXT.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define  u32ClkDiv        1   /*u32ClkDiv is between 0 and 15*/

void CLKDIRC_IRQHandler(void)
{

    if (((SYS->IRCTISTS & SYS_IRCTISTS_CLKERRIF_Msk) == SYS_IRCTISTS_CLKERRIF_Msk) || ((SYS->IRCTISTS & SYS_IRCTISTS_TFAILIF_Msk) == SYS_IRCTISTS_TFAILIF_Msk))
    {
        printf("Trim Fail, SYS->IRCTCTL[%8x], SYS->IRCTISTS[%8x]\n ", SYS->IRCTCTL, SYS->IRCTISTS) ;
        /* Clear IRCTRIM INT flag */
        SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk ;
    }
}

void SYS_Init(void)
{
    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN ;

    /* Wait for HIRC stable */
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) == 0) ;

    /* Select HIRC to HCLK */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    /* Enable LXT */
    CLK->PWRCTL = (CLK->PWRCTL & ~CLK_PWRCTL_XTLEN_Msk) | CLK_PWRCTL_LXTEN ;

    /* Wait for LXT stable */
    while (!(CLK->STATUS & CLK_STATUS_LXTSTB_Msk)) ;

    /* Enable CKO clock source */
    CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

    /* Select CKO clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_CLKOSEL_Msk)) | CLK_CLKSEL2_CLKOSEL_HIRC ;

    /*Output CKO frequency = HIRC / 2^(u32ClkDiv + 1)*/
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv ;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD, TXD and CLKO */
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_CLKO ;

    /* Set XT1_OUT(PF.0) and XT1_IN(PF.1) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk);

    /* Disable Digital Input Path of PF.0 and PF.1 */
    GPIO_DISABLE_DIGITAL_PATH(PF, BIT0 | BIT1);

    /* Set PF multi-function pins for X32_OUT(PF.0) and X32_IN(PF.1) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_XT_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_XT_IN;

} // SYS_Init()

void Trim_Init()
{
    uint32_t u32TmpCtl ;
    uint32_t u32IRCSTS ;

    //Clear status
    u32IRCSTS = SYS->IRCTISTS ;
    SYS->IRCTISTS = u32IRCSTS ;

    u32TmpCtl = (0 << SYS_IRCTCTL_REFCKSEL_Pos) |      // Ref clock is from LXT
                (1 << SYS_IRCTCTL_CESTOPEN_Pos) |      // Stop when clock inaccuracy
                (3 << SYS_IRCTCTL_RETRYCNT_Pos) |      // Retry 512 times
                (3 << SYS_IRCTCTL_LOOPSEL_Pos) |       // Every 32 clock updates trim value.
                (2 << SYS_IRCTCTL_FREQSEL_Pos) ;       // Enable trim

    /* Setting HIRCTrim control register */
    SYS->IRCTCTL = u32TmpCtl ;

    /* Enable interrupt */
    SYS->IRCTIEN = SYS_IRCTIEN_CLKEIEN_Msk | SYS_IRCTIEN_TFAILIEN_Msk ;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                  NUC121 HIRC Trim Sample Code                          |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("PB.12 for Clock Output \n\n") ;

    /* Enable Interrupt */
    NVIC_EnableIRQ(CLKDIRC_IRQn) ;

    printf("Trim Start, ") ;
    /* Enable HIRC Trim, set HIRC clock to 48Mhz and enable interrupt */
    Trim_Init() ;
    printf("SYS->IRCTCTL[0x%8x]\n", SYS->IRCTCTL) ;

    /* Waiting for HIRC Frequency Lock */
    while ((SYS->IRCTISTS & SYS_IRCTISTS_FREQLOCK_Msk) != SYS_IRCTISTS_FREQLOCK_Msk) ;

    printf("Trim Lock. \n") ;

    while (1) ;

}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
