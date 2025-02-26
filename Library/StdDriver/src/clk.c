/**************************************************************************//**
 * @file     clk.c
 * @version  V3.00
 * @brief    NUC121 series CLK driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"
/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

int32_t g_CLK_i32ErrCode = 0;    /*!< CLK global error code */

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Disable frequency output function
  * @details    This function disable frequency output function.
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
    CLK->APBCLK0 &= (~CLK_APBCLK0_CLKOCKEN_Msk);
}

/**
  * @brief      This function enable frequency divider module clock.
  *             enable frequency divider clock function and configure frequency divider.
  * @param[in]  u32ClkSrc is frequency divider function clock source. Including :
  *             - \ref CLK_CLKSEL2_CLKOSEL_HXT
  *             - \ref CLK_CLKSEL2_CLKOSEL_LXT
  *             - \ref CLK_CLKSEL2_CLKOSEL_HCLK
  *             - \ref CLK_CLKSEL2_CLKOSEL_HIRC_DIV2
  *             - \ref CLK_CLKSEL2_CLKOSEL_HIRC
  *             - \ref CLK_CLKSEL2_CLKOSEL_SOF
  * @param[in]  u32ClkDiv is divider output frequency selection.
  * @param[in]  u32ClkDivBy1En is frequency divided by one enable.
  *
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv.
  *             The formula is:
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1)
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves.
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv | (u32ClkDivBy1En << CLK_CLKOCTL_DIV1EN_Pos);

    /* Enable CKO clock source */
    CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

    /* Select CKO clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_CLKOSEL_Msk)) | (u32ClkSrc);

}

/**
  * @brief      Enter to Power-down mode
  * @details    This function is used to let system enter to Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_PowerDown(void)
{
    uint32_t u32HIRCTRIMCTL;

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

    /* Clear deep sleep mode selection */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Restore HIRC control register */
    SYS->IRCTCTL = u32HIRCTRIMCTL;
}

/**
  * @brief      Enter to Idle mode
  * @details    This function let system enter to Idle mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Set chip in idle mode because of WFI command */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Get external high speed crystal clock frequency
  * @return     External high frequency crystal frequency
  * @details    This function get external high frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHXTFreq(void)
{
    if (CLK->PWRCTL & CLK_PWRCTL_HXTEN)
        return __HXT;
    else
        return 0;
}

/**
  * @brief      Get external low speed crystal clock frequency
  * @return     External low speed crystal clock frequency
  * @details    This function get external low frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetLXTFreq(void)
{
    if (CLK->PWRCTL & CLK_PWRCTL_LXTEN)
        return __LXT;
    else
        return 0;
}


/**
  * @brief      Get HCLK frequency
  * @return     HCLK frequency
  * @details    This function get HCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Get PCLK0 frequency
  * @return     PCLK0 frequency
  * @details    This function get PCLK0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK0Freq(void)
{
    SystemCoreClockUpdate();

    if (CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk)
        return SystemCoreClock >> 1;
    else
        return SystemCoreClock;
}

/**
  * @brief      Get PCLK1 frequency
  * @return     PCLK1 frequency
  * @details    This function get PCLK1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK1Freq(void)
{
    SystemCoreClockUpdate();

    if (CLK->CLKSEL0 & CLK_CLKSEL0_PCLK1SEL_Msk)
        return SystemCoreClock >> 1;
    else
        return SystemCoreClock;
}

/**
  * @brief      Get CPU frequency
  * @return     CPU frequency
  * @details    This function get CPU frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Set HCLK frequency
  * @param[in]  u32Hclk is HCLK frequency. The range of u32Hclk is 25 MHz ~ 50 MHz.
  * @return     HCLK frequency
  * @details    This function is used to set HCLK frequency. The frequency unit is Hz. \n
  *             It would configure PLL frequency to 50MHz ~ 100MHz,
  *             set HCLK clock divider as 2 and switch HCLK clock source to PLL. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* The range of u32Hclk is 25 MHz ~ 50 MHz */
    if (u32Hclk > FREQ_50MHZ)
        u32Hclk = FREQ_50MHZ;

    if (u32Hclk < FREQ_25MHZ)
        u32Hclk = FREQ_25MHZ;

    /* Switch HCLK clock source to HIRC clock for safe */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKDIV0 &= (~CLK_CLKDIV0_HCLKDIV_Msk);

    /* Configure PLL setting if HXT clock is stable */
    if (CLK->STATUS & CLK_STATUS_HXTSTB_Msk)
        u32Hclk = CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, (u32Hclk << 1));

    /* Configure PLL setting if HXT clock is not stable */
    else
    {
        u32Hclk = CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC_DIV2, (u32Hclk << 1));

        /* Read HIRC clock source stable flag */
        u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;
    }

    /* Select HCLK clock source to PLL,
       Select HCLK clock source divider as 2
       and update system core clock
    */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Disable HIRC if HIRC is disabled before setting core clock */
    if (u32HIRCSTB == 0)
        CLK->PWRCTL &= ~CLK_PWRCTL_HIRCEN_Msk;

    /* Return actually HCLK frequency is PLL frequency divide 2 */
    return u32Hclk >> 1;
}

/**
  * @brief      Set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *             - \ref CLK_CLKSEL0_HCLKSEL_HXT
  *             - \ref CLK_CLKSEL0_HCLKSEL_LXT
  *             - \ref CLK_CLKSEL0_HCLKSEL_PLL
  *             - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *             - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  *             - \ref CLK_CLKSEL0_HCLKSEL_PLL_DIV2
  *             - \ref CLK_CLKSEL0_HCLKSEL_HIRC_DIV2
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *             - \ref CLK_CLKDIV0_HCLK(x)
  * @details    This function set HCLK clock source and HCLK clock divider.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Switch to HIRC for Safe. Avoid HCLK too high when applying new divider. */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Apply new Divider */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | u32ClkDiv;

    /* Switch HCLK to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | u32ClkSrc;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Disable HIRC if HIRC is disabled before switching HCLK source */
    if (u32HIRCSTB == 0)
        CLK->PWRCTL &= ~CLK_PWRCTL_HIRCEN_Msk;
}
/**
  * @brief      This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @details    Valid parameter combinations listed in following table:
  *
  * |Module index        |Clock source                          |Divider                 |
  * | :----------------  | :----------------------------------- | :--------------------- |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  | x                      |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_LXT           | x                      |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_LIRC          | x                      |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_HXT           |\ref CLK_CLKDIV0_ADC(x) |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_PLL           |\ref CLK_CLKDIV0_ADC(x) |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_PCLK0         |\ref CLK_CLKDIV0_ADC(x) |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_HIRC          |\ref CLK_CLKDIV0_ADC(x) |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HXT          | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_LXT          | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_PCLK0        | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_EXT_TRG      | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_LIRC         | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HIRC_DIV2    | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HXT          | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_LXT          | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_PCLK0        | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_EXT_TRG      | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_LIRC         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HIRC_DIV2    | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_HXT          | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_LXT          | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_PCLK1        | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_EXT_TRG      | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_LIRC         | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_HIRC_DIV2    | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_HXT          | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_LXT          | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_PCLK1        | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_EXT_TRG      | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_LIRC         | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_HIRC_DIV2    | x                      |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UARTSEL_HXT          |\ref CLK_CLKDIV0_UART(x)|
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UARTSEL_PLL          |\ref CLK_CLKDIV0_UART(x)|
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UARTSEL_LXT          |\ref CLK_CLKDIV0_UART(x)|
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UARTSEL_HIRC_DIV2    |\ref CLK_CLKDIV0_UART(x)|
  * |\ref BPWM0_MODULE   |\ref CLK_CLKSEL1_BPWM0SEL_PLL         | x                      |
  * |\ref BPWM0_MODULE   |\ref CLK_CLKSEL1_BPWM0SEL_PCLK0       | x                      |
  * |\ref BPWM1_MODULE   |\ref CLK_CLKSEL1_BPWM1SEL_PLL         | x                      |
  * |\ref BPWM1_MODULE   |\ref CLK_CLKSEL1_BPWM1SEL_PCLK1       | x                      |
  * |\ref PWM0_MODULE    |\ref CLK_CLKSEL1_PWM0SEL_PLL          | x                      |
  * |\ref PWM0_MODULE    |\ref CLK_CLKSEL1_PWM0SEL_PCLK0        | x                      |
  * |\ref PWM1_MODULE    |\ref CLK_CLKSEL1_PWM1SEL_PLL          | x                      |
  * |\ref PWM1_MODULE    |\ref CLK_CLKSEL1_PWM1SEL_PCLK1        | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_HXT          | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_LXT          | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_HCLK         | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_HIRC_DIV2    | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_HIRC         | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_SOF          | x                      |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL2_WWDTSEL_HCLK_DIV2048 | x                      |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL2_WWDTSEL_LIRC         | x                      |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL2_SPI0SEL_HXT          | x                      |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL2_SPI0SEL_PLL          | x                      |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL2_SPI0SEL_PCLK0        | x                      |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL2_SPI0SEL_HIRC         | x                      |
  * |\ref USBD_MODULE    |\ref CLK_CLKSEL3_USBDSEL_HIRC         | x                      |
  * |\ref USBD_MODULE    |\ref CLK_CLKSEL3_USBDSEL_PLL          |\ref CLK_CLKDIV0_USB(x) |
  */
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    if (MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        uint32_t u32div;
        const uint32_t u32DivTbl[] = {0x0};

        /* Get clock divider control register address */
        u32div = (uint32_t)&CLK->CLKDIV0 + (u32DivTbl[MODULE_CLKDIV(u32ModuleIdx)]);
        /* Apply new divider */
        M32(u32div) = (M32(u32div) & (~(MODULE_CLKDIV_Msk(u32ModuleIdx) << MODULE_CLKDIV_Pos(u32ModuleIdx)))) | u32ClkDiv;
    }

    if (MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        uint32_t u32sel;
        const uint32_t u32SelTbl[] = {0x0, 0x4, 0xC, 0x24};

        /* Get clock select control register address */
        u32sel = (uint32_t)&CLK->CLKSEL0 + (u32SelTbl[MODULE_CLKSEL(u32ModuleIdx)]);
        /* Set new clock selection setting */
        M32(u32sel) = (M32(u32sel) & (~(MODULE_CLKSEL_Msk(u32ModuleIdx) << MODULE_CLKSEL_Pos(u32ModuleIdx)))) | u32ClkSrc;
    }
}

/**
  * @brief      Set SysTick clock source
  * @param[in]  u32ClkSrc is module clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_HXT
  *             - \ref CLK_CLKSEL0_STCLKSEL_LXT
  *             - \ref CLK_CLKSEL0_STCLKSEL_HXT_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV4
  * @details    This function set SysTick clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc;
}

/**
  * @brief      Enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_HXTEN
  *             - \ref CLK_PWRCTL_LXTEN
  *             - \ref CLK_PWRCTL_HIRCEN
  *             - \ref CLK_PWRCTL_LIRCEN
  * @details    This function enable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  *             Notice that HXT and LXT are using commmon pin,
  *             that is the two clock(HXT, LXT) sources are mutual exclusive.
  *             So parameters, CLK_PWRCTL_HXTEN and CLK_PWRCTL_LXTEN, can not be applied at the same time.
  *             In other word, user should make sure that LXT is disabled if user want to enable HXT.
  *             user should disable HXT if user want to enable LXT.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    if (((CLK->PWRCTL | u32ClkMask) & CLK_PWRCTL_XTLEN_Msk) == CLK_PWRCTL_XTLEN_Msk)
        return;

    CLK->PWRCTL |= u32ClkMask;
}

/**
  * @brief      Disable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_HXTEN
  *             - \ref CLK_PWRCTL_LXTEN
  *             - \ref CLK_PWRCTL_HIRCEN
  *             - \ref CLK_PWRCTL_LIRCEN
  * @details    This function disable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL &= ~u32ClkMask;
}

/**
  * @brief      This function enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref ISP_MODULE
  *             - \ref GPIOA_MODULE
  *             - \ref GPIOB_MODULE
  *             - \ref GPIOC_MODULE
  *             - \ref GPIOD_MODULE
  *             - \ref GPIOE_MODULE
  *             - \ref GPIOF_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref UART0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref BPWM1_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref PWM1_MODULE
  *             - \ref ADC_MODULE
  *             - \ref USBD_MODULE
  *             - \ref PDMA_MODULE
  *             - \ref USCI0_MODULE
  * @details    This function enable module clock.
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    const uint32_t u32ClkTbl[3] = {0x0, 0x4, 0x2C};

    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK + (u32ClkTbl[MODULE_APBCLK(u32ModuleIdx)]))  |= 1 << MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief      This function disable module clock
  * @param[in]  u32ModuleIdx is module index
  *             - \ref ISP_MODULE
  *             - \ref GPIOA_MODULE
  *             - \ref GPIOB_MODULE
  *             - \ref GPIOC_MODULE
  *             - \ref GPIOD_MODULE
  *             - \ref GPIOE_MODULE
  *             - \ref GPIOF_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref UART0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref BPWM1_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref PWM1_MODULE
  *             - \ref ADC_MODULE
  *             - \ref USBD_MODULE
  *             - \ref PDMA_MODULE
  *             - \ref USCI0_MODULE
  * @details    This function disable module clock.
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    const uint32_t u32ClkTbl[3] = {0x0, 0x4, 0x2C};

    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK + (u32ClkTbl[MODULE_APBCLK(u32ModuleIdx)]))  &= ~(1 << MODULE_IP_EN_Pos(u32ModuleIdx));
}


/**
  * @brief      Set PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *             - \ref CLK_PLLCTL_PLLSRC_HXT
  *             - \ref CLK_PLLCTL_PLLSRC_HIRC_DIV2
  * @param[in]  u32PllFreq is PLL frequency.
  * @return     PLL frequency
  * @details    This function is used to configure PLLCTL register to set specified PLL frequency. \n
  *             The register write-protection function should be disabled before using this function.
  *             The PLL output is least 50MHz at least if BP(CLK_PLLCTL[17]) is not set.
  */
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq)
{
    uint32_t u32PllSrcClk, u32NR, u32NF, u32NO, u32CLK_SRC;
    uint32_t u32Tmp, u32Tmp2, u32Tmp3, u32Min, u32MinNF, u32MinNR;

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* PLL source clock is from HXT */
    if (u32PllClkSrc == CLK_PLLCTL_PLLSRC_HXT)
    {
        /* Enable HXT clock */
        CLK->PWRCTL = (CLK->PWRCTL & ~CLK_PWRCTL_XTLEN_Msk) | CLK_PWRCTL_HXTEN;

        /* Wait for HXT clock ready */
        CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

        /* Select PLL source clock from HXT */
        u32CLK_SRC = CLK_PLLCTL_PLLSRC_HXT;
        u32PllSrcClk = __HXT;

        /* u32NR start from 2 */
        u32NR = 2;
    }

    /* PLL source clock is from HIRC_DIV2 */
    else
    {
        /* Enable HIRC_DIV2 clock */
        CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

        /* Wait for HIRC clock ready */
        CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

        /* Select PLL source clock from HIRC */
        u32CLK_SRC = CLK_PLLCTL_PLLSRC_HIRC_DIV2;
        u32PllSrcClk = __HIRC_DIV2;

        /* u32NR start from 4 when FIN = 24MHz to avoid calculation overflow */
        u32NR = 4;
    }

    /* Select "NO" according to request frequency */
    if ((u32PllFreq <= FREQ_500MHZ) && (u32PllFreq > FREQ_250MHZ))
    {
        u32NO = 0;
    }
    else if ((u32PllFreq <= FREQ_250MHZ) && (u32PllFreq > FREQ_125MHZ))
    {
        u32NO = 1;
        u32PllFreq = u32PllFreq << 1;
    }
    else if ((u32PllFreq <= FREQ_125MHZ) && (u32PllFreq >= FREQ_50MHZ))
    {
        u32NO = 3;
        u32PllFreq = u32PllFreq << 2;
    }
    else
    {
        /* Wrong frequency request. Just return default setting. */
        goto lexit;
    }

    /* Find best solution */
    u32Min = (uint32_t) - 1;
    u32MinNR = 0;
    u32MinNF = 0;

    for (; u32NR <= 33; u32NR++)
    {
        u32Tmp = u32PllSrcClk / u32NR;

        if ((u32Tmp > 1600000) && (u32Tmp < 16000000))
        {
            for (u32NF = 2; u32NF <= 513; u32NF++)
            {
                u32Tmp2 = u32Tmp * u32NF;

                if ((u32Tmp2 >= 200000000) && (u32Tmp2 <= 500000000))
                {
                    u32Tmp3 = (u32Tmp2 > u32PllFreq) ? u32Tmp2 - u32PllFreq : u32PllFreq - u32Tmp2;

                    if (u32Tmp3 < u32Min)
                    {
                        u32Min = u32Tmp3;
                        u32MinNR = u32NR;
                        u32MinNF = u32NF;

                        /* Break when get good results */
                        if (u32Min == 0)
                            break;
                    }
                }
            }
        }
    }

    /* Enable and apply new PLL setting. */
    CLK->PLLCTL = u32CLK_SRC | (u32NO << 14) | ((u32MinNR - 2) << 9) | (u32MinNF - 2);

    /* Wait for PLL clock stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Return actual PLL output clock frequency */
    return u32PllSrcClk / ((u32NO + 1) * u32MinNR) * u32MinNF;

lexit:

    /* Apply default PLL setting and return */
    if (u32PllClkSrc == CLK_PLLCTL_PLLSRC_HXT)
        CLK->PLLCTL = CLK_PLLCTL_96MHz_HXT; /* 96MHz */
    else
        CLK->PLLCTL = CLK_PLLCTL_96MHz_HIRC_DIV2; /* 96MHz */

    /* Wait for PLL clock stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    return CLK_GetPLLClockFreq();

}

/**
  * @brief      Disable PLL
  * @details    This function set PLL in Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisablePLL(void)
{
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
}

/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_HXTSTB_Msk
  *             - \ref CLK_STATUS_LXTSTB_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_PLLSTB_Msk
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  * @details    To wait for clock ready by specified clock source stable flag or timeout (~250ms)
  * @note       This function sets g_CLK_i32ErrCode to CLK_TIMEOUT_ERR if clock source status is not stable
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    uint32_t u32TimeOutCnt = (SystemCoreClock >> 2);
    uint32_t u32Ret = 1U;

    g_CLK_i32ErrCode = 0;

    while ((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if (--u32TimeOutCnt == 0)
        {
            u32Ret = 0U;
            break;
        }
    }

    if (u32TimeOutCnt == 0)
        g_CLK_i32ErrCode = CLK_TIMEOUT_ERR;

    return u32Ret;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_HXT
  *             - \ref CLK_CLKSEL0_STCLKSEL_LXT
  *             - \ref CLK_CLKSEL0_STCLKSEL_HXT_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV4
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0;

    /* Set System Tick clock source */
    if (u32ClkSrc == CLK_CLKSEL0_STCLKSEL_HCLK)
    {
        /* Select System Tick clock source from core */
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    }
    else
    {
        /* Select System Tick external reference clock source */
        CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc;

        /* Select System Tick clock source from external reference clock */
        SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
    }

    /* Set System Tick reload value */
    SysTick->LOAD = u32Count;

    /* Clear System Tick current value and counter flag */
    SysTick->VAL = 0;

    /* Set System Tick interrupt enabled and counter enabled */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief      Disable System Tick counter
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0;
}

/**
  * @brief      This function execute delay function.
  * @param[in]  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                             50MHz => 335544us, 48MHz => 349525us, 28MHz => 699050us ...
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  *             User can use SystemCoreClockUpdate() to calculate CyclesPerUs automatically before using this function.
  */
void CLK_SysTickDelay(uint32_t us)
{
    uint32_t u32TargetValue, u32TargetInt, u32TargetRem, u32DelayCycles;

    /* Systick function is using and clock source is core clock */
    if ((SysTick->CTRL & (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk)) == (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk))
    {
        u32DelayCycles = us * CyclesPerUs;

        if (u32DelayCycles > SysTick->LOAD)
        {
            /* Calculate re-load cycles with current SysTick->LOAD */
            u32TargetInt = u32DelayCycles / SysTick->LOAD;

            /* Calculate remainder delay cycles */
            u32TargetRem = u32DelayCycles % SysTick->LOAD;
        }
        else
        {
            u32TargetInt = 0;
            u32TargetRem = u32DelayCycles;
        }

        if (u32TargetRem > SysTick->VAL)
        {
            u32TargetValue = SysTick->LOAD;
            u32TargetValue = u32TargetValue - (u32TargetRem - SysTick->VAL);
            u32TargetInt++;
        }
        else
        {
            u32TargetValue = SysTick->VAL - u32TargetRem;
        }

        while (u32TargetInt > 0)
        {
            /* Waiting for down-count to zero */
            while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
            {
            }

            u32TargetInt--;
        }

        /* Waiting for down-count to target */
        while (SysTick->VAL > u32TargetValue)
        {
        }
    }
    else
    {
        SysTick->LOAD = us * CyclesPerUs;
        SysTick->VAL  = 0x0UL;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
        {
        }

        /* Disable SysTick counter */
        SysTick->CTRL = 0UL;
    }
}


/** @} end of group CLK_EXPORTED_FUNCTIONS */

/** @} end of group CLK_Driver */

/** @} end of group Standard_Driver */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
