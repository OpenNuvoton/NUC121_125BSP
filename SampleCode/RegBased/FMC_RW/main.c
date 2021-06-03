/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to read/program embedded flash by ISP function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             0x6000
#define DATA_FLASH_TEST_BASE        0x7000
#define DATA_FLASH_TEST_END         0x8000

#define TEST_PATTERN                0x5A5A5A5A

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

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* Update core clock */
    PllClock        = 0;                 // PLL
    SystemCoreClock = __HIRC;            // HCLK
    CyclesPerUs     = __HIRC / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

    /* Set PA multi-function pins for CLKO(PA.15) */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk)) | SYS_GPA_MFPH_PA15MFP_CLKO;
}

void UART_Init()
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


static int  SetDataFlashBase(uint32_t u32DFBA)
{
    uint32_t au32Config[2], i;

    /* Read current User Configuration */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    au32Config[0] = FMC->ISPDAT;

    /* Just return when Data Flash has been enabled */
    if (!(au32Config[0] & 0x1))
        return 0;

    /* Enable User Configuration Update */
    FMC->ISPCTL |= FMC_ISPCTL_CFGUEN_Msk;

    /* Erase User Configuration */
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    /* Write User Configuration to Enable Data Flash */
    au32Config[0] &= ~0x1;
    au32Config[1] = u32DFBA;

    for (i = 0; i < 2; i++)
    {
        /* write back config */
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_CONFIG_BASE + i * 4;
        FMC->ISPDAT = au32Config[i];
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

        /* verify config */
        FMC->ISPCMD = FMC_ISPCMD_READ;
        FMC->ISPADDR = FMC_CONFIG_BASE + i * 4;
        FMC->ISPDAT = 0;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

        uint32_t u32Data = FMC->ISPDAT;

        if (u32Data != au32Config[i])
            return -1;
    }

    printf("\nSet Data Flash base as 0x%x.\n", FMC->DFBA);

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;

    return 0;
}


int32_t FillDataPattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = u32Addr;
        FMC->ISPDAT = u32Pattern;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    }

    return 0;
}


int32_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC->ISPCMD = FMC_ISPCMD_READ;
        FMC->ISPADDR = u32Addr;
        FMC->ISPDAT = 0;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

        uint32_t u32Data = FMC->ISPDAT;

        if (u32Data != u32Pattern)
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data, u32Pattern);
            return -1;
        }
    }

    return 0;
}


int32_t  FlashTest(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Erase page */
        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

        /* Verify if page contents are all 0xFFFFFFFF */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        /* Write test pattern to fill the whole page */
        if (FillDataPattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all equal to test pattern */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }

        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

        /* Verify if page contents are all 0xFFFFFFFF */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }
    }

    printf("\r    Flash Test Passed.          \n");
    return 0;
}


int main()
{
    uint32_t i, u32Data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system clock and multi-function I/O */
    SYS_Init();

    /* Init UART */
    UART_Init();

    /*
        This sample code is used to show how to use StdDriver API to implement ISP functions.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|          NUC121 FMC Sample Code          |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

    if (SetDataFlashBase(DATA_FLASH_TEST_BASE) < 0)
    {
        printf("Failed to set Data Flash base address!\n");
        goto lexit;
    }

    /* Read Boot Select */
    printf("  Boot Mode ............................. ");

    if (((FMC->ISPCTL & FMC_ISPCTL_BS_Msk) >> FMC_ISPCTL_BS_Pos) == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CID;
    FMC->ISPADDR = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    u32Data = FMC->ISPDAT;
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    FMC->ISPCMD = FMC_ISPCMD_READ_DID;
    FMC->ISPADDR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    u32Data = FMC->ISPDAT;
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    for (i = 0; i < 3; i++)
    {
        FMC->ISPCMD = FMC_ISPCMD_READ_UID;
        FMC->ISPADDR = (i << 2);
        FMC->ISPDAT = 0;
        FMC->ISPTRG = 0x1;
        __ISB();

        while (FMC->ISPTRG);

        u32Data = FMC->ISPDAT;
        printf("  Unique ID %u ........................... [0x%08x]\n", i, u32Data);
    }

    for (i = 0; i < 4; i++)
    {
        FMC->ISPCMD = FMC_ISPCMD_READ_UID;
        FMC->ISPADDR = (0x04 * i) + 0x10;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

        u32Data = FMC->ISPDAT;
        printf("  Unique Customer ID %u .................. [0x%08x]\n", i, u32Data);
    }

    /* Read User Configuration */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    u32Data = FMC->ISPDAT;
    printf("  User Config 0 ......................... [0x%08x]\n", u32Data);
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE + 4;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    u32Data = FMC->ISPDAT;
    printf("  User Config 1 ......................... [0x%08x]\n", u32Data);

    /* Read Data Flash base address */
    u32Data = FMC->DFBA;
    printf("  Data Flash Base Address ............... [0x%08x]\n", u32Data);

    printf("\n\nLDROM test =>\n");
    /* Enable LDROM update */
    FMC->ISPCTL |= FMC_ISPCTL_LDUEN_Msk;

    if (FlashTest(FMC_LDROM_BASE, FMC_LDROM_BASE + FMC_LDROM_SIZE, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");
        goto lexit;
    }

    /* Disable LDROM update */
    FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;

    printf("\n\nAPROM test =>\n");
    /* Enable APROM update */
    FMC->ISPCTL |= FMC_ISPCTL_APUEN_Msk;

    if (FlashTest(APROM_TEST_BASE, DATA_FLASH_TEST_BASE, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");
        goto lexit;
    }

    /* Disable APROM update */
    FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk;

    printf("\n\nData Flash test =>\n");

    if (FlashTest(DATA_FLASH_TEST_BASE, DATA_FLASH_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nUHB test failed!\n");
        goto lexit;
    }

lexit:

    /* Disable FMC ISP function */
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
