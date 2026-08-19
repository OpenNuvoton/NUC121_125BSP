/******************************************************************************
 * @file     NUC121.h
 * @version  V3.00
 * @brief    NUC121/NUC125 Series Peripheral Access Layer Header File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/**
  \mainpage NuMicro NUC121/NUC125 Series CMSIS BSP Driver Reference
  *
  * <b>Introduction</b>
  *
  * This user manual describes the usage of NUC121/NUC125 Series MCU device driver
  *
  * <b>Disclaimer</b>
  *
  * The Software is furnished "AS IS", without warranty as to performance or results, and
  * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
  * warranties, express, implied or otherwise, with regard to the Software, its use, or
  * operation, including without limitation any and all warranties of merchantability, fitness
  * for a particular purpose, and non-infringement of intellectual property rights.
  *
  * <b>Important Notice</b>
  *
  * Nuvoton Products are neither intended nor warranted for usage in systems or equipment,
  * any malfunction or failure of which may cause loss of human life, bodily injury or severe
  * property damage. Such applications are deemed, "Insecure Usage".
  *
  * Insecure usage includes, but is not limited to: equipment for surgical implementation,
  * atomic energy control instruments, airplane or spaceship instruments, the control or
  * operation of dynamic, brake or safety systems designed for vehicular use, traffic signal
  * instruments, all types of safety devices, and other applications intended to support or
  * sustain life.
  *
  * All Insecure Usage shall be made at customer's risk, and in the event that third parties
  * lay claims to Nuvoton as a result of customer's Insecure Usage, customer shall indemnify
  * the damages and liabilities thus incurred by Nuvoton.
  *
  * Please note that all data and specifications are subject to change without notice. All the
  * trademarks of products and companies mentioned in this datasheet belong to their respective
  * owners.
  *
  * <b>Copyright Notice</b>
  *
 * SPDX-License-Identifier: Apache-2.0
  * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
  */

#ifndef __NUC121_H__
#define __NUC121_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

/** @addtogroup MCU_CMSIS Device Definitions for CMSIS
    Interrupt Number Definition and Configurations for CMSIS
    @{
*/

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT           0U       /*!< armikcmu does not provide a MPU present or not       */
#define __NVIC_PRIO_BITS        2U       /*!< armikcmu Supports 2 Bits for the Priority Levels     */
#define __Vendor_SysTickConfig  0U       /*!< Set to 1 if different SysTick Config is used         */

/** @} end of group MCU_CMSIS */


#include <stdint.h>
#include "irqn.h"                       /*!< Interrupt Number Definition                          */
#include "core_cm0.h"                   /*!< Cortex-M0 processor and core peripherals             */
#include "system_NUC121.h"              /*!< NUC121 System                                        */

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup REGISTER Control Register
    @{
*/

#include "adc_reg.h"
#include "bpwm_reg.h"
#include "clk_reg.h"
#include "fmc_reg.h"
#include "gpio_reg.h"
#include "i2c_reg.h"
#include "int_reg.h"
#include "pdma_reg.h"
#include "pwm_reg.h"
#include "spi_reg.h"
#include "sys_reg.h"
#include "timer_reg.h"
#include "uart_reg.h"
#include "ui2c_reg.h"
#include "uspi_reg.h"
#include "uuart_reg.h"
#include "usbd_reg.h"
#include "wdt_reg.h"
#include "wwdt_reg.h"

/** @} end of REGISTER group */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup PERIPHERAL_BASE Peripheral Memory Base
    Memory Mapped Structure for Series Peripheral
    @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE          ((uint32_t)0x00000000UL)
#define SRAM_BASE           ((uint32_t)0x20000000UL)
#define AHB_BASE            ((uint32_t)0x50000000UL)
#define APB1_BASE           ((uint32_t)0x40000000UL)
#define APB2_BASE           ((uint32_t)0x40100000UL)

/* Peripheral memory map */
/* AHB */
#define SYS_BASE            (AHB_BASE       + 0x00000UL)                  /*!< System Global Controller Base Address            */
#define CLK_BASE            (AHB_BASE       + 0x00200UL)                  /*!< System Clock Controller Base Address             */
#define INT_BASE            (AHB_BASE       + 0x00300UL)                  /*!< Interrupt Source Controller Base Address         */
#define GPIO_BASE           (AHB_BASE       + 0x04000UL)                  /*!< GPIO Base Address                                */
#define PDMA_BASE           (AHB_BASE       + 0x08000UL)                  /*!< PDMA Base Address                                */
#define FMC_BASE            (AHB_BASE       + 0x0C000UL)                  /*!< Flash Memory Controller Base Address             */

#define PA_BASE             (GPIO_BASE                 )                  /*!< GPIO PA Base Address                             */
#define PB_BASE             (GPIO_BASE      + 0x00040UL)                  /*!< GPIO PB Base Address                             */
#define PC_BASE             (GPIO_BASE      + 0x00080UL)                  /*!< GPIO PC Base Address                             */
#define PD_BASE             (GPIO_BASE      + 0x000C0UL)                  /*!< GPIO PD Base Address                             */
#define PE_BASE             (GPIO_BASE      + 0x00100UL)                  /*!< GPIO PE Base Address                             */
#define PF_BASE             (GPIO_BASE      + 0x00140UL)                  /*!< GPIO PF Base Address                             */
#define GPIO_DBCTL_BASE     (GPIO_BASE      + 0x00180UL)                  /*!< GPIO De-bounce Cycle Control Base Address        */
#define GPIO_PIN_DATA_BASE  (GPIO_BASE      + 0x00200UL)                  /*!< GPIO Pin Data Input/Output Control Base Address  */

/* APB1 */
#define WDT_BASE            (APB1_BASE      + 0x04000UL)                  /*!< Watch Dog Timer Base Address                     */
#define WWDT_BASE           (APB1_BASE      + 0x04100UL)                  /*!< Window Watch Dog Timer Base Address              */
#define TIMER0_BASE         (APB1_BASE      + 0x10000UL)                  /*!< Timer0 Base Address                              */
#define TIMER1_BASE         (APB1_BASE      + 0x10020UL)                  /*!< Timer1 Base Address                              */
#define I2C0_BASE           (APB1_BASE      + 0x20000UL)                  /*!< I2C0 Base Address                                */
#define SPI0_BASE           (APB1_BASE      + 0x30000UL)                  /*!< SPI0 Base Address                                */
#define BPWM0_BASE          (APB1_BASE      + 0x40000UL)                  /*!< BPWM0 Base Address                               */
#define PWM0_BASE           (APB1_BASE      + 0x44000UL)                  /*!< PWM0 Base Address                                */
#define UART0_BASE          (APB1_BASE      + 0x50000UL)                  /*!< UART0 Base Address                               */
#define USBD_BASE           (APB1_BASE      + 0x60000UL)                  /*!< USB Device Base Address                          */
#define USCI0_BASE          (APB1_BASE      + 0x70000UL)
#define ADC_BASE            (APB1_BASE      + 0xE0000UL)                  /*!< ADC Base Address                                 */

/* APB2 */
#define TIMER2_BASE         (APB2_BASE      + 0x10000UL)                  /*!< Timer2 Base Address                              */
#define TIMER3_BASE         (APB2_BASE      + 0x10020UL)                  /*!< Timer3 Base Address                              */
#define I2C1_BASE           (APB2_BASE      + 0x20000UL)                  /*!< I2C1 Base Address                                */
#define BPWM1_BASE          (APB2_BASE      + 0x40000UL)                  /*!< BPWM1 Base Address                               */
#define PWM1_BASE           (APB2_BASE      + 0x44000UL)                  /*!< PWM1 Base Address                                */

/** @} end of group PERIPHERAL_BASE */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/

/** @addtogroup PMODULE Peripheral Pointer
    The Declaration of Peripheral Pointer
    @{
 */
#define ADC                 ((ADC_T *) ADC_BASE)                        /*!< ADC Configuration Struct                         */

#define BPWM0               ((BPWM_T *) BPWM0_BASE)                     /*!< BPWM0 Configuration Struct                       */
#define BPWM1               ((BPWM_T *) BPWM1_BASE)                     /*!< BPWM1 Configuration Struct                       */

#define CLK                 ((CLK_T *) CLK_BASE)                        /*!< System Clock Controller Configuration Struct     */

#define FMC                 ((FMC_T *) FMC_BASE)                        /*!< Flash Memory Controller */

#define GPIO                ((GPIO_DBCTL_T *) GPIO_DBCTL_BASE)          /*!< Interrupt De-bounce Cycle Control Configuration Struct */
#define PA                  ((GPIO_T *) PA_BASE)                        /*!< GPIO PORTA Configuration Struct                        */
#define PB                  ((GPIO_T *) PB_BASE)                        /*!< GPIO PORTB Configuration Struct                        */
#define PC                  ((GPIO_T *) PC_BASE)                        /*!< GPIO PORTC Configuration Struct                        */
#define PD                  ((GPIO_T *) PD_BASE)                        /*!< GPIO PORTD Configuration Struct                        */
#define PE                  ((GPIO_T *) PE_BASE)                        /*!< GPIO PORTE Configuration Struct                        */
#define PF                  ((GPIO_T *) PF_BASE)                        /*!< GPIO PORTF Configuration Struct                        */

#define I2C0                ((I2C_T *) I2C0_BASE)                       /*!< I2C0 Configuration Struct                        */
#define I2C1                ((I2C_T *) I2C1_BASE)                       /*!< I2C1 Configuration Struct                        */

#define INT                 ((INT_T *) INT_BASE)                        /*!< Interrupt Source Controller Configuration Struct */

#define PDMA                ((PDMA_T *) PDMA_BASE)                      /*!< PDMA Configuration Struct                        */

#define PWM0                ((PWM_T *)  PWM0_BASE)                      /*!< PWM0 Configuration Struct                        */
#define PWM1                ((PWM_T *)  PWM1_BASE)                      /*!< PWM1 Configuration Struct                        */

#define SPI0                ((SPI_T *) SPI0_BASE)                       /*!< SPI0 Configuration Struct                        */

#define SYS                 ((SYS_T *) SYS_BASE)                        /*!< System Global Controller Configuration Struct    */

#define TIMER0              ((TIMER_T *) TIMER0_BASE)                   /*!< TIMER0 Configuration Struct                      */
#define TIMER1              ((TIMER_T *) TIMER1_BASE)                   /*!< TIMER1 Configuration Struct                      */
#define TIMER2              ((TIMER_T *) TIMER2_BASE)                   /*!< TIMER2 Configuration Struct                      */
#define TIMER3              ((TIMER_T *) TIMER3_BASE)                   /*!< TIMER3 Configuration Struct                      */

#define UART0               ((UART_T *) UART0_BASE)                     /*!< UART0 Configuration Struct                       */

#define UI2C0               ((UI2C_T *) USCI0_BASE)                     /*!< UI2C0 Configuration Struct                       */
#define USPI0               ((USPI_T *) USCI0_BASE)                     /*!< USPI0 Configuration Struct                       */
#define UUART0              ((UUART_T *) USCI0_BASE)                    /*!< UUART0 Configuration Struct                      */

#define USBD                ((USBD_T *) USBD_BASE)                      /*!< USB Device Configuration Struct                  */

#define WDT                 ((WDT_T *) WDT_BASE)                        /*!< Watch Dog Timer Configuration Struct             */

#define WWDT                ((WWDT_T *) WWDT_BASE)                      /*!< Window Watch Dog Timer Configuration Struct      */

/** @} end of group PMODULE */


//=============================================================================
typedef volatile uint8_t  vu8;        ///< Define 8-bit unsigned volatile data type
typedef volatile uint16_t vu16;       ///< Define 16-bit unsigned volatile data type
typedef volatile uint32_t vu32;       ///< Define 32-bit unsigned volatile data type
typedef volatile uint64_t vu64;       ///< Define 64-bit unsigned volatile data type

#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

#define outpw(port,value)   (*((volatile unsigned int *)(port))=(value))
#define inpw(port)          ((*((volatile unsigned int *)(port))))
#define outpb(port,value)   (*((volatile unsigned char *)(port))=(value))
#define inpb(port)          ((*((volatile unsigned char *)(port))))
#define outps(port,value)   (*((volatile unsigned short *)(port))=(value))
#define inps(port)          ((*((volatile unsigned short *)(port))))

#define outp32(port,value)  (*((volatile unsigned int *)(port))=(value))
#define inp32(port)         ((*((volatile unsigned int *)(port))))
#define outp8(port,value)   (*((volatile unsigned char *)(port))=(value))
#define inp8(port)          ((*((volatile unsigned char *)(port))))
#define outp16(port,value)  (*((volatile unsigned short *)(port))=(value))
#define inp16(port)         ((*((volatile unsigned short *)(port))))

#define E_SUCCESS   (0)
#ifndef NULL
    #define NULL    (0)
#endif

#define TRUE        (1UL)
#define FALSE       (0UL)

#define ENABLE      (1UL)
#define DISABLE     (0UL)

/* Bit Mask Definitions */
#define BIT0    (0x00000001UL)
#define BIT1    (0x00000002UL)
#define BIT2    (0x00000004UL)
#define BIT3    (0x00000008UL)
#define BIT4    (0x00000010UL)
#define BIT5    (0x00000020UL)
#define BIT6    (0x00000040UL)
#define BIT7    (0x00000080UL)
#define BIT8    (0x00000100UL)
#define BIT9    (0x00000200UL)
#define BIT10   (0x00000400UL)
#define BIT11   (0x00000800UL)
#define BIT12   (0x00001000UL)
#define BIT13   (0x00002000UL)
#define BIT14   (0x00004000UL)
#define BIT15   (0x00008000UL)
#define BIT16   (0x00010000UL)
#define BIT17   (0x00020000UL)
#define BIT18   (0x00040000UL)
#define BIT19   (0x00080000UL)
#define BIT20   (0x00100000UL)
#define BIT21   (0x00200000UL)
#define BIT22   (0x00400000UL)
#define BIT23   (0x00800000UL)
#define BIT24   (0x01000000UL)
#define BIT25   (0x02000000UL)
#define BIT26   (0x04000000UL)
#define BIT27   (0x08000000UL)
#define BIT28   (0x10000000UL)
#define BIT29   (0x20000000UL)
#define BIT30   (0x40000000UL)
#define BIT31   (0x80000000UL)

/* Byte Mask Definitions */
#define BYTE0_Msk               (0x000000FF)
#define BYTE1_Msk               (0x0000FF00)
#define BYTE2_Msk               (0x00FF0000)
#define BYTE3_Msk               (0xFF000000)

#define _GET_BYTE0(u32Param)    (((u32Param) & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define _GET_BYTE1(u32Param)    (((u32Param) & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define _GET_BYTE2(u32Param)    (((u32Param) & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define _GET_BYTE3(u32Param)    (((u32Param) & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "adc.h"
#include "bpwm.h"
#include "clk.h"
#include "fmc.h"
#include "gpio.h"
#include "i2c.h"
#include "pdma.h"
#include "pwm.h"
#include "spi_i2s.h"
#include "sys.h"
#include "timer.h"
#include "uart.h"
#include "usbd.h"
#include "usci_i2c.h"
#include "usci_spi.h"
#include "usci_uart.h"
#include "wdt.h"
#include "wwdt.h"

#endif /*__NUC121_H__*/
