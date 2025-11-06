/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO interrupt function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// Function prototype
void GPAB_IRQHandler(void);
void GPCDEF_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);


/**
 * @brief       PortA/PortB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortA/PortB default IRQ, declared in startup_NUC121.s.
 */
void GPAB_IRQHandler(void)
{
    /* To check if PB.14 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT14))
    {
        GPIO_CLR_INT_FLAG(PB, BIT14);
        printf("PB.14 INT occurred.\n");
    }
    else
    {
        uint32_t u32Status;

        /* Un-expected interrupt. Just clear all PORTA, PORTB interrupts */
        u32Status =  PA->INTSRC;
        PA->INTSRC = u32Status;
        u32Status =  PB->INTSRC;
        PB->INTSRC = u32Status;
        printf("Un-expected interrupts.\n");
    }
}

/**
 * @brief       PortC/PortD/PortE/PortF IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortC/PortD/PortE/PortF default IRQ, declared in startup_NUC121.s.
 */
void GPCDEF_IRQHandler(void)
{
    /* To check if PD.11 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PD, BIT11))
    {
        GPIO_CLR_INT_FLAG(PD, BIT11);
        printf("PD.11 INT occurred.\n");
    }
    else
    {
        uint32_t u32Status;

        /* Un-expected interrupt. Just clear all PORTC, PORTD, PORTE and PORTF interrupts */
        u32Status =  PC->INTSRC;
        PC->INTSRC = u32Status;
        u32Status =  PD->INTSRC;
        PD->INTSRC = u32Status;
        u32Status =  PE->INTSRC;
        PE->INTSRC = u32Status;
        u32Status =  PF->INTSRC;
        PF->INTSRC = u32Status;
        printf("Un-expected interrupts.\n");
    }
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    GPIO PB.14 and PD.11 Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("PB.14 and PD.11 are used to test interrupt ......\n");

    /* Configure PB.14 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT14, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 14, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPAB_IRQn);

    /*  Configure PD.11 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PD, BIT11, GPIO_MODE_QUASI);
    GPIO_EnableInt(PD, 11, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPCDEF_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT14);
    GPIO_ENABLE_DEBOUNCE(PD, BIT11);

    /* Waiting for interrupts */
    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
