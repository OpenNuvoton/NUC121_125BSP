/******************************************************************************
 * @file     APROM_main.c
 * @version  V3.00
 * @brief    Show how to reboot to LDROM functions from APROM.
 *           This sample code set VECMAP to LDROM and reset to re-boot to LDROM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

typedef void (FUNC_PTR)(void);

extern uint32_t  loaderImage1Base, loaderImage1Limit;


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


static int SetIAPBoot(void)
{
    uint32_t au32Config[2];
    uint32_t u32CBS;

    /* Read current boot mode */
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;

    if (u32CBS & 1)
    {
        uint32_t i;

        /* Modify User Configuration when it is not in IAP mode */
        for (i = 0; i < 2; i++)
        {
            FMC->ISPCMD = FMC_ISPCMD_READ;
            FMC->ISPADDR = FMC_CONFIG_BASE + i * 4;
            FMC->ISPDAT = 0;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            __ISB();

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

            au32Config[i] = FMC->ISPDAT;
        }

        if (au32Config[0] & 0x40)
        {
            /* Enable updating config */
            FMC->ISPCTL |= FMC_ISPCTL_CFGUEN_Msk;
            au32Config[0] &= ~0x40;
            /* Do page erase */
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = FMC_CONFIG_BASE;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            __ISB();

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

            /* write back config */
            for (i = 0; i < 2; i++)
            {
                FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
                FMC->ISPADDR = FMC_CONFIG_BASE + i * 4;
                FMC->ISPDAT = au32Config[i];
                FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
                __ISB();

                while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
            }
        }

        /* Perform chip reset to make new User Config take effect */
        SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
    }

    return 0;
}

static int  LoadImage(uint32_t u32ImageBase, uint32_t u32ImageLimit, uint32_t u32FlashAddr, uint32_t u32MaxSize)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    (void)(u32ImageLimit);
    u32ImageSize = u32MaxSize;

    printf("Program image to flash address 0x%x...", u32FlashAddr);
    pu32Loader = (uint32_t *)u32ImageBase;

    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        /* Do Page Erase */
        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADDR = u32FlashAddr + i;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

        /* Program image to LDROM */
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
            FMC->ISPADDR = u32FlashAddr + i + j;
            FMC->ISPDAT = pu32Loader[(i + j) / 4];
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            __ISB();

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
        }
    }

    printf("OK.\n");

    printf("Verify ...");

    /* Verify loader */
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC->ISPCMD = FMC_ISPCMD_READ;
            FMC->ISPADDR = u32FlashAddr + i + j;
            FMC->ISPDAT = 0;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            __ISB();

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

            u32Data = FMC->ISPDAT;

            if (u32Data != pu32Loader[(i + j) / 4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", u32FlashAddr + i + j, u32Data, pu32Loader[(i + j) / 4]);
                return -1;
            }

            if (i + j >= u32ImageSize)
                break;
        }
    }

    printf("OK.\n");
    return 0;
}


int main()
{
    uint8_t u8Item;
    uint32_t u32Data;
    char *acBootMode[] = {"LDROM+IAP", "LDROM", "APROM+IAP", "APROM"};
    uint32_t u32CBS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system clock and multi-function I/O */
    SYS_Init();

    /* Init UART */
    UART_Init();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|      NUC121 FMC IAP Sample Code        |\n");
    printf("|           [APROM code]                 |\n");
    printf("+----------------------------------------+\n");


    /* Enable FMC ISP function */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

    if (SetIAPBoot() < 0)
    {
        printf("Failed to set IAP boot mode!\n");
        goto lexit;
    }

    /* Get boot mode */
    printf("  Boot Mode ............................. ");
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    printf("[%s]\n", acBootMode[u32CBS]);

    /* Read Company ID */
    FMC->ISPCMD = FMC_ISPCMD_READ_CID;
    FMC->ISPADDR = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    u32Data = FMC->ISPDAT;
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Read Product ID */
    FMC->ISPCMD = FMC_ISPCMD_READ_DID;
    FMC->ISPADDR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    u32Data = FMC->ISPDAT;
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

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

    do
    {
        printf("\n\n\n");
        printf("+----------------------------------------+\n");
        printf("|               Select                   |\n");
        printf("+----------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM             |\n");
        printf("| [1] Run IAP program (in LDROM)         |\n");
        printf("+----------------------------------------+\n");
        printf("Please select...");
        u8Item = getchar();
        printf("%c\n", u8Item);

        switch (u8Item)
        {
            case '0':
                FMC->ISPCTL |= FMC_ISPCTL_LDUEN_Msk;

                if (LoadImage((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                              FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
                {
                    printf("Load image to LDROM failed!\n");
                    goto lexit;
                }

                FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;
                break;

            case '1':
                printf("\n\nChange VECMAP and branch to LDROM...\n");
                UART_WAIT_TX_EMPTY(UART0); /* To make sure all message has been print out */

                /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
                __set_PRIMASK(1);

                /* Set VECMAP to LDROM for booting from LDROM */
                FMC->ISPCMD = FMC_ISPCMD_VECMAP;
                FMC->ISPADDR = FMC_LDROM_BASE;
                FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
                __ISB();

                while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

                /* Software reset to boot to LDROM */
                NVIC_SystemReset();

                break;

            default :
                break;
        }
    } while (1);


lexit:

    /* Disable FMC ISP function */
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
