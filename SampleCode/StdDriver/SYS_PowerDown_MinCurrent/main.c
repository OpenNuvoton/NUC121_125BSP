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

#define GPIO_P0_TO_P15      0xFFFF
#define GPIOF_P0_TO_P15     0xFFFC

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
        CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
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
        CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

        if (CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LXT == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LXTEN);
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
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN);

        if (CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
        }

        CLK_DisableXtalRC(CLK_PWRCTL_HXTEN);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_HXTSTB_Msk)
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
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enable Power-down mode wake-up interrupt */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIEN_Msk;

    /* Enter to Power-down mode */
    CLK_PowerDown();
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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable all GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOACKEN_Msk | CLK_AHBCLK_GPIOBCKEN_Msk | CLK_AHBCLK_GPIOCCKEN_Msk | CLK_AHBCLK_GPIODCKEN_Msk |
                   CLK_AHBCLK_GPIOECKEN_Msk | CLK_AHBCLK_GPIOFCKEN_Msk;

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

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
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIOF_P0_TO_P15, GPIO_MODE_QUASI);

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if (LircSetting() < 0) goto lexit;

    /* LXT setting */
    if (LxtSetting() < 0) goto lexit;

    /* Configure PB.2 as Input mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);
    GPIO_EnableInt(PB, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPAB_IRQn);

    PowerDownFunction();

    /* Waiting for PB.2 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while (1);

}

