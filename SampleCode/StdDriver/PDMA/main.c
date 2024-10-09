/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use PDMA channel 2 to transfer data from memory to memory.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
    #pragma data_alignment=4
    uint8_t au8SrcArray[256];
    uint8_t au8DestArray[256];
#else
    __attribute__((aligned(4))) uint8_t au8SrcArray[256];
    __attribute__((aligned(4))) uint8_t au8DestArray[256];
#endif

uint32_t PDMA_TEST_LENGTH = 64;
uint32_t volatile g_u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ.
 */
void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 2 has abort error */
        if (PDMA_GET_ABORT_STS() & PDMA_ABTSTS_ABTIF2_Msk)
        {
            g_u32IsTestOver = 2;
        }

        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if (PDMA_GET_TD_STS() & PDMA_TDSTS_TDIF2_Msk)
        {
            g_u32IsTestOver = 1;
        }

        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF2_Msk);
    }
    else
    {
        printf("unknown interrupt !!\n");
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    /* Update clock */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART module clock source as HIRC_DIV2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));
    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;
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
int main(void)
{
    uint32_t i, u32TimeOutCnt;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();
    /* Init UART for printf */
    UART0_Init();
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|    PDMA Memory to Memory Driver Sample Code          | \n");
    printf("+------------------------------------------------------+ \n");
    /* Reset PDMA module */
    SYS_UnlockReg();
    SYS_ResetModule(PDMA_RST);
    SYS_LockReg();

    for (i = 0; i < 0x100; i++)
    {
        au8SrcArray[i] = (uint8_t) i;
        au8DestArray[i] = 0;
    }

    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray                         au8DestArray
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      PDMA transfer configuration:

        Channel = 2
        Operation mode = basic mode
        Request source = PDMA_MEM(memory to memory)
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    /* Open Channel 2 */
    PDMA_Open(PDMA_CHCTL_CHEN2_Msk);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(2, (uint32_t)au8SrcArray, PDMA_SAR_INC, (uint32_t)au8DestArray, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(2, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(2, PDMA_REQ_BURST, PDMA_BURST_4);
    /* Enable interrupt */
    PDMA_EnableInt(2, PDMA_INT_TRANS_DONE);
    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;
    /* Generate a software request to trigger transfer with PDMA channel 2  */
    PDMA_Trigger(2);
    /* Waiting for transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            break;
        }
    }

    /* Check transfer result */
    if (g_u32IsTestOver == 1)
    {
        printf("test done...\n");

        /* Compare data */
        for (i = 0; i < 0x100; i++)
        {
            if (au8SrcArray[i] != au8DestArray[i])
            {
                printf(" - Compare data fail\n");
                break;
            }
        }
    }
    else if (g_u32IsTestOver == 2)
    {
        printf("target abort...\n");
    }

    /* Close channel 2 */
    PDMA_Close();

    while (1);
}
