/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief Use PDMA channel 4 to transfer data from memory to memory by scatter-gather mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#ifdef __ICCARM__
    #pragma data_alignment=4
    uint8_t au8SrcArray[256];
    uint8_t au8DestArray0[256];
    uint8_t au8DestArray1[256];
#else
    __attribute__((aligned(4))) uint8_t au8SrcArray[256];
    __attribute__((aligned(4))) uint8_t au8DestArray0[256];
    __attribute__((aligned(4))) uint8_t au8DestArray1[256];
#endif

uint32_t PDMA_TEST_LENGTH = 64;



PDMA_DSCT_T DMA_DESC[2]; /* Descriptor table */

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_NUC121.s.
 */
void PDMA_IRQHandler(void)
{
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);
    /* Select UART module clock source as HIRC_DIV2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;
}

void UART0_Init(void)
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
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i, u32Src, u32Dst0, u32Dst1, u32TimeOutCnt;
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
    printf("+-----------------------------------------------------------------------+ \n");
    printf("|    NUC121 PDMA Memory to Memory Driver Sample Code (Scatter-gather)   | \n");
    printf("+-----------------------------------------------------------------------+ \n");
    u32Src = (uint32_t)au8SrcArray;
    u32Dst0 = (uint32_t)au8DestArray0;
    u32Dst1 = (uint32_t)au8DestArray1;

    for (i = 0; i < 0x100; i++)
    {
        au8SrcArray[i] = (uint8_t) i;
        au8DestArray0[i] = 0;
        au8DestArray1[i] = 0;
    }

    /* This sample will transfer data by finished two descriptor table in sequence.(descriptor table 1 -> descriptor table 2) */
    /*----------------------------------------------------------------------------------
      PDMA transfer configuration:

        Channel = 4
        Operation mode = scatter-gather mode
        First scatter-gather descriptor table = DMA_DESC[0]
        Request source = PDMA_MEM(memory to memory)

        Transmission flow:
           ------------------------      -----------------------
          |                        |    |                       |
          |  DMA_DESC[0]           | -> |  DMA_DESC[1]          | -> transfer done
          |  (Descriptor table 1)  |    |  (Descriptor table 2) |
          |                        |    |                       |
           ------------------------      -----------------------

    ----------------------------------------------------------------------------------*/
    /* Open Channel 4 */
    PDMA_Open(1 << 4);
    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as memory to memory */
    PDMA_SetTransferMode(4, PDMA_MEM, 1, (uint32_t)&DMA_DESC[0]);
    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray                         au8DestArray0
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      Descriptor table 1 configuration:

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)
        transfer done and table empty interrupt = disable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray0
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].CTL =
        ((PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
        PDMA_WIDTH_32 |   /* Transfer width is 32 bits(one word) */ \
        PDMA_SAR_INC |    /* Source increment size is 32 bits(one word) */ \
        PDMA_DAR_INC |    /* Destination increment size is 32 bits(one word) */ \
        PDMA_REQ_BURST |  /* Transfer type is burst transfer type */ \
        PDMA_BURST_128 |  /* Burst size is 128. No effect in single transfer type */ \
        PDMA_DSCT_CTL_TBINTDIS_Msk |   /* Disable transfer done and table empty interrupt */ \
        PDMA_OP_SCATTER;  /* Operation mode is scatter-gather mode */
    /* Configure source address */
    DMA_DESC[0].SA = u32Src;
    /* Configure destination address */
    DMA_DESC[0].DA = u32Dst0;
    /* Configure next descriptor table address */
    DMA_DESC[0].NEXT = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* next descriptor table is table 2 */
    /*------------------------------------------------------------------------------------------------------

                         au8DestArray0                       au8DestArray1
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      Descriptor table 2 configuration:

        Operation mode = basic mode
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8DestArray0
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray1
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].CTL =
        ((PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
        PDMA_WIDTH_32 |   /* Transfer width is 32 bits(one word) */ \
        PDMA_SAR_INC |    /* Source increment size is 32 bits(one word) */ \
        PDMA_DAR_INC |    /* Destination increment size is 32 bits(one word) */ \
        PDMA_REQ_BURST |  /* Transfer type is burst transfer type */ \
        PDMA_BURST_128 |  /* Burst size is 128. No effect in single transfer type */ \
        PDMA_OP_BASIC;    /* Operation mode is basic mode */
    DMA_DESC[1].SA = u32Dst0;
    DMA_DESC[1].DA = u32Dst1;
    DMA_DESC[1].NEXT = 0; /* No next operation table. No effect in basic mode */
    /* Generate a software request to trigger transfer with PDMA channel 4 */
    PDMA_Trigger(4);
    /* Waiting for transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (PDMA_IS_CH_BUSY(4))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            break;
        }
    }

    printf("test done...\n");

    /* Compare data */
    for (i = 0; i < 0x100; i++)
    {
        if ((au8SrcArray[i] != au8DestArray0[i]) || (au8SrcArray[i] != au8DestArray1[i]))
        {
            printf(" - Compare data fail\n");
            break;
        }
    }

    /* Close Channel 4 */
    PDMA_Close();

    while (1);
}
