/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Change duty cycle and period of output waveform by BPWM Double Buffer function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK       100000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{
    static int i8Toggle = 0;

    /* Clear channel 0 period interrupt flag */
    BPWM_ClearPeriodIntFlag(BPWM0, 0);

    /* Update BPWM0 channel 0 period and duty */
    if (i8Toggle == 0)
    {
        BPWM_SET_CNR(BPWM0, 0, 199);
        BPWM_SET_CMR(BPWM0, 0, 40);
        BPWM_SET_CMR(BPWM0, 1, 40);
        BPWM_SET_CMR(BPWM0, 2, 40);
        BPWM_SET_CMR(BPWM0, 3, 40);
        BPWM_SET_CMR(BPWM0, 4, 40);
        BPWM_SET_CMR(BPWM0, 5, 40);
    }
    else
    {
        BPWM_SET_CNR(BPWM0, 0, 399);
        BPWM_SET_CMR(BPWM0, 0, 200);
        BPWM_SET_CMR(BPWM0, 1, 200);
        BPWM_SET_CMR(BPWM0, 2, 200);
        BPWM_SET_CMR(BPWM0, 3, 200);
        BPWM_SET_CMR(BPWM0, 4, 200);
        BPWM_SET_CMR(BPWM0, 5, 200);
    }

    i8Toggle ^= 1;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Enable BPWM0 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* BPWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.BPWM clock frequency is set equal to HCLK: select BPWM module clock source as PCLK */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL1_BPWM0SEL_PCLK0, 0);

    /* case 2.BPWM clock frequency is set double to HCLK: select BPWM module clock source as PLL */
    //CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL1_BPWM0SEL_PLL, NULL);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC_DIV2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Reset BPWM0 module */
    SYS_ResetModule(BPWM0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD);

    /* Set multi-function pins for BPWM0 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB7MFP_BPWM0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB6MFP_BPWM0_CH1;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB5MFP_BPWM0_CH2;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_BPWM0_CH3;
    SYS->GPD_MFPH = (SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD11MFP_Msk));
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD11MFP_BPWM0_CH4;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA11MFP_Msk));
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA11MFP_BPWM0_CH5;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                        Six channels use one counter                    |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0 channel 0(PB.7)\n");
    printf("    waveform output pin: BPWM0 channel 1(PB.6)\n");
    printf("    waveform output pin: BPWM0 channel 2(PB.5)\n");
    printf("    waveform output pin: BPWM0 channel 3(PB.4)\n");
    printf("    waveform output pin: BPWM0 channel 4(PD.11)\n");
    printf("    waveform output pin: BPWM0 channel 5(PA.11)\n");
    printf("\nUse double buffer feature.\n");

    /*
        BPWM0 channel 0 waveform of this sample shown below(down counter type):

        |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
                       |<-  CMR clk ->|  CMR = 200 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 199 + 1 CLKs
                                               |<-CMR->|  CMR = 160 CLKs

                        ______________              ____
        |_____200______|     200      |_____160____| 40 |____ BPWM waveform

    */

    /*
      Configure BPWM0 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 50 MHz / (1 * (199 + 1)) = 250000 Hz
      Duty ratio = (100) / (199 + 1) = 50%
    */
    /* BPWM0 channel 0 frequency is 250000Hz, duty 50%, */
    BPWM_ConfigOutputChannel(BPWM0, 0, 250000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 1, 250000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 2, 250000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 3, 250000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 4, 250000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 5, 250000, 50);

    /* Enable output of BPWM0 channel */
    BPWM_EnableOutput(BPWM0, 0x3F);

    /* Enable BPWM0 channel 0 period interrupt. */
    BPWM_EnablePeriodInt(BPWM0, 0, 0);
    NVIC_EnableIRQ(BPWM0_IRQn);

    /* Start */
    BPWM_Start(BPWM0, 0x3F);

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
