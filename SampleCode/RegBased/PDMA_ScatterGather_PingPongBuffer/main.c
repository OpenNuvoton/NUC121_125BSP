/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief Use PDMA to implement Ping-Pong buffer by scatter-gather mode(memory to memory).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t g_au32SrcArray0[1] = {0x55555555};
uint32_t g_au32SrcArray1[1] = {0xAAAAAAAA};
uint32_t g_au32DestArray[1];

uint32_t PDMA_TEST_COUNT = 50;
uint32_t volatile g_u32IsTestOver = 0;
uint32_t volatile g_u32TransferredCount = 0;
uint32_t g_u32DMAConfig = 0;
static uint32_t s_u32TableIndex = 0;

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

    uint32_t u32Intsts = PDMA_GET_INT_STATUS();

    if (u32Intsts & PDMA_INTSTS_TDIF_Msk)
    {

        /* Check channel transfer done status */
        if (PDMA_GET_TD_STS() == PDMA_TDSTS_TDIF4_Msk)
        {

            /* Reload PDMA Descriptor table configuration after transmission finished */
            DMA_DESC[s_u32TableIndex].CTL = g_u32DMAConfig;
            s_u32TableIndex ^= 1;

            /* When finished a descriptor table then g_u32TransferredCount increases 1 */
            g_u32TransferredCount++;

            /* Check if PDMA has finished PDMA_TEST_COUNT tasks */
            if (g_u32TransferredCount == PDMA_TEST_COUNT)
            {
                /* Set PDMA into idle state by Descriptor table */
                DMA_DESC[0].CTL &= ~PDMA_DSCT_CTL_OPMODE_Msk;
                DMA_DESC[1].CTL &= ~PDMA_DSCT_CTL_OPMODE_Msk;
                g_u32IsTestOver = 1;
            }

            /* Clear transfer done flag of channel 4 */
            PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF4_Msk);
        }
        else
        {
            printf("Unexpected channel transfer done !!\n");
        }

    }
    else if (u32Intsts & PDMA_INTSTS_TEIF_Msk)
    {
        /*
           The flag will set if scatter-gather table is empty while PDMA transferring.
           User should reserve enough time for scatter-gather table reloading in PDMA handler.
           PDMA controller will stop the transfer if the flag is set.
        */
        uint32_t u32EmptySts = PDMA_GET_EMPTY_STS();

        if (u32EmptySts & PDMA_SCATSTS_TEMPTYF4_Msk)
        {
            if (g_u32IsTestOver)
            {

                /*
                    Because the PDMA is still transferring while the transfer count reach 50,
                    PDMA assert the scatter-gather table empty event while getting the scatter-table that mode is IDLE.
                    User can stop PDMA transferring by scatter-gather table empty event, that is, setting scatter-table mode to IDLE.
                */

                /* Clear table empty flag of channel 4 */
                PDMA_CLR_EMPTY_FLAG(PDMA_SCATSTS_TEMPTYF4_Msk);


            }
            else
            {
                /* Not transfer over but PDMA fetch the IDLE scatter-gather table */
                PDMA_CLR_EMPTY_FLAG(PDMA_SCATSTS_TEMPTYF4_Msk);

                printf("Not transfer over but PDMA fetch the IDLE scatter-gather table !!\n");
            }

        }
        else
        {
            printf("Unexpected channel sets assert scatter-table empty event !!\n");
        }
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

    /* Enable HIRC clock (Internal HIRC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Update System Core Clock if core clock is from HIRC*/
    PllClock        = 0;                 // PLL
    SystemCoreClock = __HIRC / 1;        // HCLK
    CyclesPerUs     = __HIRC / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC_DIV2 and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* IP clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;


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
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register.

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------+ \n");
    printf("|    NUC121 PDMA Driver Ping-Pong Buffer Sample Code (Scatter-gather)   | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    /* This sample will transfer data by looped around two descriptor tables from two different source to the same destination buffer in sequence.
       And operation sequence will be table 1 -> table 2-> table 1 -> table 2 -> table 1 -> ... -> until PDMA configuration doesn't be reloaded. */

    /*--------------------------------------------------------------------------------------------------
      PDMA transfer configuration:

        Channel = 4
        Operation mode = scatter-gather mode
        First scatter-gather descriptor table = DMA_DESC[0]
        Request source = PDMA_MEM(memory to memory)

        Transmission flow:

                                            loop around
                                      PDMA_TEST_COUNT/2 times
           ------------------------                             -----------------------
          |                        | ------------------------> |                       |
          |  DMA_DESC[0]           |                           |  DMA_DESC[1]          |
          |  (Descriptor table 1)  |                           |  (Descriptor table 2) |
          |                        | <-----------------------  |                       |
           ------------------------                             -----------------------

        Note: The configuration of each table in SRAM need to be reloaded after transmission finished.
    --------------------------------------------------------------------------------------------------*/

    /* Open Channel 4 */
    PDMA->CHCTL |= BIT4;

    /* Set transfer mode as memory to memory */
    PDMA->REQSEL4 = (PDMA->REQSEL4 & ~PDMA_REQSEL4_REQSRC4_Msk) | (PDMA_MEM << PDMA_REQSEL4_REQSRC4_Pos);

    /* Enable Scatter Gather mode and set the descriptor table address in SRAM */
    /* Enable Scatter Gather mode */
    PDMA->DSCT[4].CTL = PDMA_OP_SCATTER;
    /* Assign the first scatter-gather descriptor table is table 1 */
    PDMA->DSCT[4].FIRST = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);


    /* Scatter-Gather descriptor table configuration in SRAM */
    g_u32DMAConfig = \
                     (16383 << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is 1                */ \
                     PDMA_WIDTH_32  |   /* Transfer width is 32 bits(one word)                  */ \
                     PDMA_SAR_FIX   |   /* Source increment size is fixed(no increment)         */ \
                     PDMA_DAR_FIX   |   /* Destination increment size is fixed(no increment)    */ \
                     PDMA_REQ_BURST |   /* Transfer type is burst transfer type                 */ \
                     PDMA_BURST_1   |   /* Burst size is 128. No effect in single transfer type */ \
                     PDMA_OP_SCATTER;   /* Operation mode is scatter-gather mode                */
    /*-----------------------------------------------------------------------------------------------------------------------------------------------------------
       Note: PDMA_REQ_BURST is only supported in memory-to-memory transfer mode.
             PDMA transfer type should be set as PDMA_REQ_SINGLE in memory-to-peripheral and peripheral-to-memory transfer mode,
             then above code will be modified as follows:
             g_u32DMAConfig = (0 << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX | PDMA_BURST_1 | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    -----------------------------------------------------------------------------------------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------------------------------------
      Descriptor table 1 configuration:

             g_au32SrcArray0               transfer 1 times    g_au32DestArray
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   32bits(one word)                                  32bits(one word)

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)
        transfer done and table empty interrupt = enable

        Transfer count = 1
        Transfer width = 32 bits(one word)
        Source address = g_au32SrcArray0
        Source address increment size = fixed address(no increment)
        Destination address = au8DestArray0
        Destination address increment size = fixed address(no increment)
        Transfer type = burst transfer

        Total transfer length = 1 * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].CTL = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[0].SA = (uint32_t)g_au32SrcArray0; /* Ping-Pong buffer 1 */
    /* Configure destination address */
    DMA_DESC[0].DA = (uint32_t)&g_au32DestArray[0];
    /* Configure next descriptor table address */
    DMA_DESC[0].NEXT = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* next operation table is table 2 */

    /*------------------------------------------------------------------------------------------------------
      Descriptor table 2 configuration:

             g_au32SrcArray1               transfer 1 times    g_au32DestArray
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   32bits(one word)                                  32bits(one word)

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[0](Descriptor table 1)
        transfer done and table empty interrupt = enable

        Transfer count = 1
        Transfer width = 32 bits(one word)
        Source address = g_au32SrcArray1
        Source address increment size = fixed address(no increment)
        Destination address = au8DestArray0
        Destination address increment size = fixed address(no increment)
        Transfer type = burst transfer

        Total transfer length = 1 * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].CTL = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[1].SA = (uint32_t)g_au32SrcArray1; /* Ping-Pong buffer 2 */
    /* Configure destination address */
    DMA_DESC[1].DA = (uint32_t)&g_au32DestArray[0];
    /* Configure next descriptor table address */
    DMA_DESC[1].NEXT = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA); /* next operation table is table 1 */


    /* Enable transfer done interrupt */
    PDMA->INTEN |= BIT4;
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;

    /* Start PDMA operation */
    PDMA->SWREQ = BIT4;

    while (1)
    {
        if (g_u32IsTestOver == 1)
        {
            printf("test done...\n");
            break;
        }
    }

    /* Close Channel 4 */
    PDMA->CHCTL = 0;

    while (1);
}
