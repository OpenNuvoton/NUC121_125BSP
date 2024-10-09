/**************************************************************************//**
 * @file     startup_NUC121.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro NUC121
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void);
__NO_RETURN void Default_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)    __attribute__((weak));
void SVC_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)      __attribute__((weak, alias("Default_Handler")));

/* External Interrupts */
void BOD_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void WDT_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void EINT024_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void EINT135_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void GPAB_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void GPCDEF_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void PWM0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void PWM1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TMR0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TMR1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TMR2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TMR3_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void UART0_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void SPI0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void BPWM0_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void BPWM1_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void USCI_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void USBD_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void PWM_BRAKE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PDMA_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void PWRWU_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void ADC_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void CLKDIRC_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));


/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
#if defined ( __GNUC__ )
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[];
const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE =
{
    (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*       Initial Stack Pointer                            */
    Reset_Handler,                            /*       Reset Handler                                    */
    NMI_Handler,                              /*   -14 NMI Handler                                      */
    HardFault_Handler,                        /*   -13 Hard Fault Handler                               */
    0,                                        /*   -12 Reserved                                         */
    0,                                        /*   -11 Reserved                                         */
    0,                                        /*   -10 Reserved                                         */
    0,                                        /*    -9 Reserved                                         */
    0,                                        /*    -8 Reserved                                         */
    0,                                        /*    -7 Reserved                                         */
    0,                                        /*    -6 Reserved                                         */
    SVC_Handler,                              /*    -5 SVC Handler                                      */
    0,                                        /*    -4 Reserved                                         */
    0,                                        /*    -3 Reserved                                         */
    PendSV_Handler,                           /*    -2 PendSV Handler Handler                           */
    SysTick_Handler,                          /*    -1 SysTick Handler                                  */

    /* Interrupts */
    BOD_IRQHandler,                           /*    0: Brown-Out low voltage detected                   */
    WDT_IRQHandler,                           /*    1: Watchdog Timer                                   */
    EINT024_IRQHandler,                       /*    2: External interrupt from PB.14, PC.13, PC.12, PE.0*/
    EINT135_IRQHandler,                       /*    3: External interrupt from PD.11, PE.2, PB.15       */
    GPAB_IRQHandler,                          /*    4: External signal interrupt from GPA/GPB           */
    GPCDEF_IRQHandler,                        /*    5: External interrupt from GPC/GPD/GPE/GPF          */
    PWM0_IRQHandler,                          /*    6: PWM0 interrupt                                   */
    PWM1_IRQHandler,                          /*    7: PWM1 interrupt                                   */
    TMR0_IRQHandler,                          /*    8: Timer 0                                          */
    TMR1_IRQHandler,                          /*    9: Timer 1                                          */
    TMR2_IRQHandler,                          /*    10: Timer 2                                         */
    TMR3_IRQHandler,                          /*    11: Timer 3                                         */
    UART0_IRQHandler,                         /*    12: URAT0                                           */
    Default_Handler,                          /*    13: Reserved                                        */
    SPI0_IRQHandler,                          /*    14: SPI0                                            */
    Default_Handler,                          /*    15: Reserved                                        */
    Default_Handler,                          /*    16: Reserved                                        */
    Default_Handler,                          /*    17: Reserved                                        */
    I2C0_IRQHandler,                          /*    18: I2C0                                            */
    I2C1_IRQHandler,                          /*    19: I2C1                                            */
    BPWM0_IRQHandler,                         /*    20: BPWM0                                           */
    BPWM1_IRQHandler,                         /*    21: BPWM1                                           */
    USCI_IRQHandler,                          /*    22: USCI0                                           */
    USBD_IRQHandler,                          /*    23: USB Device                                      */
    Default_Handler,                          /*    24: Reserved                                        */
    PWM_BRAKE_IRQHandler,                     /*    25: PWM Brake                                       */
    PDMA_IRQHandler,                          /*    26: PDMA                                            */
    Default_Handler,                          /*    27: Reserved                                        */
    PWRWU_IRQHandler,                         /*    28: Clock controller interrupt for chip wake-up     */
    ADC_IRQHandler,                           /*    29: ADC                                             */
    CLKDIRC_IRQHandler                       /*    30: Clock fail detect and IRC TRIM                  */
};

#if defined ( __GNUC__ )
    #pragma GCC diagnostic pop
#endif

__WEAK void Reset_Handler_PreInit(void)
{
    // Empty function
}

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    __set_PSP((uint32_t)(&__INITIAL_SP));

    Reset_Handler_PreInit();

    SYS_UnlockReg();            /* Unlock protected registers */
    SYS->PORCTL = 0x5AA5;       /* Init POR */
    SystemInit();               /* CMSIS System Initialization */
    SYS_LockReg();              /* Lock protected registers */
    __PROGRAM_START();          /* Enter PreMain (C library entry point) */
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*---------------------------------------------------------------------------
  Hard Fault Handler
 *---------------------------------------------------------------------------*/
__WEAK void HardFault_Handler(void)
{
    while (1);
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#endif
