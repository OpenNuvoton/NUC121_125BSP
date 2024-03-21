/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC       0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0

#define GPIO_P0_TO_P15      0xFFFFFFFF
#define GPIOF_P0_TO_P15     0xFFFFFFF0

void PowerDownFunction(void);
void GPAB_IRQHandler(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void SYS_Init(void);
void UART0_Init(void);

void GPAB_IRQHandler(void)
{
    /* To check if PB.2 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("PB.2 INT occurred.\n");
    }
    else
    {
        uint32_t u32Status;

        /* Un-expected interrupt. Just clear all PB interrupts */
        u32Status =  PB->INTSRC;
        PB->INTSRC = u32Status;
        printf("Un-expected interrupts.\n");
    }
}

void PorSetting(void)
{
    if (SET_POR == 0)
    {
        SYS_DISABLE_POR();
    }
    else
    {
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LIRC == 0)
    {
        CLK->PWRCTL &= ~CLK_PWRCTL_LIRCEN_Msk;

        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LIRCSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK->PWRCTL |= CLK_PWRCTL_LIRCEN_Msk;

        if ((CLK->STATUS & CLK_STATUS_LIRCSTB_Msk) == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC enable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LXT == 0)
    {
        CLK->PWRCTL &= ~CLK_PWRCTL_LXTEN;

        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LXTSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK->PWRCTL |= CLK_PWRCTL_LXTEN;

        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        if ((CLK->STATUS & CLK_STATUS_LXTSTB_Msk) != CLK_STATUS_LXTSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT enable time-out!\n");
                return -1;
            }
        }

        CLK->PWRCTL &= ~CLK_PWRCTL_HXTEN;

        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while ((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) == CLK_STATUS_HXTSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for HXT disable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32HIRCTRIMCTL;

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enable Power-down mode wake-up interrupt */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIEN_Msk;

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled */
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk);

    /* Store HIRC control register */
    u32HIRCTRIMCTL = SYS->IRCTCTL;

    /* Disable HIRC auto trim */
    SYS->IRCTCTL &= (~SYS_IRCTCTL_FREQSEL_Msk);

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

    /* Restore HIRC control register */
    SYS->IRCTCTL = u32HIRCTRIMCTL;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void BOD_IRQHandler(void)
{
    /* Clear BOD Interrupt Flag */
    SYS_CLEAR_BOD_INT_FLAG();

    printf("Brown Out is Detected.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Power-down Mode Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    /* Check system power down mode wake-up interrupt status flag */
    if (CLK->PWRCTL & CLK_PWRCTL_PDWKIF_Msk)
    {
        /* Clear system power down wake-up interrupt flag */
        CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;

        printf("System wake-up from Power-down mode.\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Set XT1_OUT(PF.0) and XT1_IN(PF.1) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) == 0);

    /* Apply HCLK source divider as 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk));

    /* Select HCLK clock source as HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Enable all GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOACKEN_Msk | CLK_AHBCLK_GPIOBCKEN_Msk | CLK_AHBCLK_GPIOCCKEN_Msk | CLK_AHBCLK_GPIODCKEN_Msk |
                   CLK_AHBCLK_GPIOECKEN_Msk | CLK_AHBCLK_GPIOFCKEN_Msk;

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC/2 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Update core clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.2 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("|  Operating sequence                                               |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                         |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable analog function, e.g. POR module                      |\n");
    printf("|  4. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  5. Enter to Power-Down                                           |\n");
    printf("|  6. Wait for PB.2 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /* To measure Power-down current on NuTiny-SDK-NUC121 board, remove Nu-Link-Me and R10*/

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFPH = 0;
    SYS->GPB_MFPL = (SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD);
    SYS->GPB_MFPH = 0;
    SYS->GPC_MFPL = 0;
    SYS->GPC_MFPH = 0;
    SYS->GPD_MFPL = 0;
    SYS->GPD_MFPH = 0;
    SYS->GPE_MFPL = 0;
    SYS->GPF_MFPL = 0;

    /* Configure all GPIO as Quasi-bidirectional Mode */
    PA->MODE = PA->MODE | GPIO_P0_TO_P15;
    PB->MODE = PB->MODE | GPIO_P0_TO_P15;
    PC->MODE = PC->MODE | GPIO_P0_TO_P15;
    PD->MODE = PD->MODE | GPIO_P0_TO_P15;
    PE->MODE = PE->MODE | GPIO_P0_TO_P15;
    PF->MODE = PF->MODE | GPIOF_P0_TO_P15;

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if (LircSetting() < 0) goto lexit;

    /* LXT setting */
    if (LxtSetting() < 0) goto lexit;

    /* Configure PB.2 as Input mode and enable interrupt by falling edge trigger */
    PB->INTTYPE = (PB->INTTYPE & (~GPIO_INTTYPE_TYPE2_Msk)) | (GPIO_INTTYPE_EDGE << GPIO_INTTYPE_TYPE2_Pos);
    PB->INTEN |= GPIO_INTEN_RHIEN2_Msk;

    NVIC_EnableIRQ(GPAB_IRQn);

    PowerDownFunction();

    /* Waiting for PB.2 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while (1);

}

