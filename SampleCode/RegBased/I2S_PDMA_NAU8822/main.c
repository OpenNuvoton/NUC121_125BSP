/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    NUC121 I2S Driver Sample Code
 *           This is a I2S demo with PDMA function connected with NAU8822 codec.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/* use Line-in as source, undefine it if MIC is used */
//#define INPUT_IS_LIN

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
//#define BUFF_LEN      512
#define BUFF_LEN        256
#define BUFF_HALF_LEN   (BUFF_LEN/2)
#define I2S_TIMEOUT     (SystemCoreClock >> 2)

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

#define NAU8822_ADDR    0x1A                /* NAU8822 Device ID */

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
volatile uint8_t g_u8TxIdx = 0, g_u8RxIdx = 0;
uint32_t g_au32PcmRxBuff[2][BUFF_LEN] = {0};
uint32_t g_au32PcmTxBuff[2][BUFF_LEN] = {0};
uint32_t volatile g_u32BuffPos = 0;
DESC_TABLE_T g_asDescTable_TX[2], g_asDescTable_RX[2];

void NAU8822_Setup(void);

void Delay(uint32_t i32DelayCount)
{
    volatile uint32_t u32Count;

    for (u32Count = 0; u32Count < i32DelayCount ; u32Count++);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data)
{
    volatile int32_t i32TimeoutCnt = I2S_TIMEOUT;

    I2C0->CTL |= I2C_CTL_SI_Msk | I2C_CTL_STA_Msk;

    i32TimeoutCnt = I2S_TIMEOUT;

    while (!(I2C0->CTL & I2C_CTL_SI_Msk))
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    I2C0->DAT = NAU8822_ADDR << 1;
    I2C0->CTL = (I2C0->CTL & ~0x3c) | I2C_CTL_SI;

    i32TimeoutCnt = I2S_TIMEOUT;

    while (!(I2C0->CTL & I2C_CTL_SI_Msk))
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    I2C0->DAT = (uint8_t)((u8Addr << 1) | (u16Data >> 8));
    I2C0->CTL = (I2C0->CTL & ~0x3c) | I2C_CTL_SI;

    i32TimeoutCnt = I2S_TIMEOUT;

    while (!(I2C0->CTL & I2C_CTL_SI_Msk))
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    I2C0->DAT = (uint8_t)(u16Data & 0x00FF);
    I2C0->CTL = (I2C0->CTL & ~0x3c) | I2C_CTL_SI;

    i32TimeoutCnt = I2S_TIMEOUT;

    while (!(I2C0->CTL & I2C_CTL_SI_Msk))
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    I2C0->CTL |= I2C_CTL_SI_Msk | I2C_CTL_STO_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup(void)
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

#ifdef INPUT_IS_LIN //input source is LIN
    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1FF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1FF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x060);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x060);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#else   //input source is MIC
    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */
    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

    printf("[OK]\n");
}

void SYS_Init(void)
{
    volatile int32_t i32TimeoutCnt = I2S_TIMEOUT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
    {
        if (--i32TimeoutCnt <= 0)
        {
            break;
        }
    }

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /*------------------------------------------------------------------------------------------------------*/
    /* Enable Module Clock                                                                                  */
    /*------------------------------------------------------------------------------------------------------*/

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC_DIV2;

    /* select source from HIRC/2(24MHz) */
    CLK->CLKSEL2 &= ~CLK_CLKSEL2_SPI0SEL_Msk;
    CLK->CLKSEL2 |= CLK_CLKSEL2_SPI0SEL_PCLK0;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_SPI0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk;
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Update System Core Clock */
    PllClock        = 0;                 // PLL
    SystemCoreClock = __HIRC;            // HCLK
    CyclesPerUs     = __HIRC / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* PB.0 is UART0_RX */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB0MFP_Msk;
    SYS->GPB_MFPL |= (1 << SYS_GPB_MFPL_PB0MFP_Pos);
    /* PB.1 is UART0_TX */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB1MFP_Msk;
    SYS->GPB_MFPL |= (1 << SYS_GPB_MFPL_PB1MFP_Pos);

    /* Set I2C0 PC multi-function pins */
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC11MFP_Msk | SYS_GPC_MFPH_PC12MFP_Msk);
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC11MFP_I2C0_SDA | SYS_GPC_MFPH_PC12MFP_I2C0_SCL);

    /* set PC.4 as SPIO_I2S_MCLK */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC4MFP_Msk;
    SYS->GPC_MFPL |= (3 << SYS_GPC_MFPL_PC4MFP_Pos);

    /* PC.0 is I2S_LRCLK */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC0MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC0MFP_Pos);
    /* PC.1 is I2S_BCLK */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC1MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC1MFP_Pos);
    /* PC.2 is I2S_DI */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC2MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC2MFP_Pos);
    /* PC.3 is I2S_DO */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC3MFP_Msk;
    SYS->GPC_MFPL |= (1 << SYS_GPC_MFPL_PC3MFP_Pos);
}

void UART0_Init(void)
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

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    volatile int32_t  i32Channel;

    /* Tx description */
    g_asDescTable_TX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[0].SA = (uint32_t)&g_au32PcmTxBuff[0];
    g_asDescTable_TX[0].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[0].FIRST = (uint32_t)&g_asDescTable_TX[1] - (PDMA->SCATBA);

    g_asDescTable_TX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[1].SA = (uint32_t)&g_au32PcmTxBuff[1];
    g_asDescTable_TX[1].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[1].FIRST = (uint32_t)&g_asDescTable_TX[0] - (PDMA->SCATBA);   //link to first description

    /* Rx description */
    g_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[0].DA = (uint32_t)&g_au32PcmRxBuff[0];
    g_asDescTable_RX[0].FIRST = (uint32_t)&g_asDescTable_RX[1] - (PDMA->SCATBA);

    g_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[1].DA = (uint32_t)&g_au32PcmRxBuff[1];
    g_asDescTable_RX[1].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);   //link to first description

    /* Open PDMA channel 1 for SPI TX and channel 2 for SPI RX */
    for (i32Channel = 0; i32Channel < PDMA_CH_MAX; i32Channel++)
        PDMA->DSCT[i32Channel].CTL = 0;

    PDMA->CHCTL |= (1 << 1) | (1 << 2);

    /* Configure PDMA transfer mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 8)) | (PDMA_SPI0_TX << 8);
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 16)) | (PDMA_SPI0_RX << 16);
    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[1].FIRST = (uint32_t)&g_asDescTable_TX[0] - (PDMA->SCATBA);
    PDMA->DSCT[2].CTL = (PDMA->DSCT[2].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[2].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA->INTEN |= (1 << 1) | (1 << 2);

    NVIC_EnableIRQ(PDMA_IRQn);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t u8Id)
{
    g_asDescTable_TX[u8Id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_TX[u8Id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t u8Id)
{
    g_asDescTable_RX[u8Id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_RX[u8Id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Init I2C interface */
void I2C0_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C0 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->CTL |= I2C_CTL_I2CEN_Msk;

    /* Open I2C0 and set clock to 100k */
    u32BusClock = 100000;
    I2C0->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", (SystemCoreClock / (((I2C0->CLKDIV) + 1) << 2)));
}

static uint32_t I2S_GetSourceClockFreq(SPI_T *i2s)
{
    uint32_t u32Freq, u32HCLKFreq;

    if (i2s == SPI0)
    {
        if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_HXT)
            u32Freq = __HXT; /* Clock source is HXT */
        else if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PLL)
            u32Freq = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        else if ((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PCLK0)
        {
            /* Get system clock frequency */
            u32HCLKFreq = SystemCoreClock;

            /* Clock source is PCLK0 */
            if ((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk) == CLK_CLKSEL0_PCLK0SEL_HCLK_DIV2)
                u32Freq = (u32HCLKFreq / 2);
            else
                u32Freq = u32HCLKFreq;
        }
        else
            u32Freq = __HIRC; /* Clock source is HIRC */

        return u32Freq;
    }

    return FALSE;
}

uint32_t I2S_EnableMCLK(SPI_T *i2s, uint32_t u32BusClock)
{
    uint32_t u32Divider;
    uint32_t u32SrcClk;

    u32SrcClk = I2S_GetSourceClockFreq(i2s);

    if (u32BusClock == u32SrcClk)
        u32Divider = 0;
    else
    {
        u32Divider = (u32SrcClk / u32BusClock) >> 1;

        /* MCLKDIV is a 6-bit width configuration. The maximum value is 0x3F. */
        if (u32Divider > 0x3F)
            u32Divider = 0x3F;
    }

    /* Write u32Divider to MCLKDIV (SPI_I2SCLK[5:0]) */
    i2s->I2SCLK = (i2s->I2SCLK & ~SPI_I2SCLK_MCLKDIV_Msk) | (u32Divider << SPI_I2SCLK_MCLKDIV_Pos);

    /* Enable MCLK output */
    i2s->I2SCTL |= SPI_I2SCTL_MCLKEN_Msk;

    if (u32Divider == 0)
        return u32SrcClk; /* If MCLKDIV=0, master clock rate is equal to the source clock rate. */
    else
        return ((u32SrcClk >> 1) / u32Divider); /* If MCLKDIV>0, master clock rate = source clock rate / (MCLKDIV * 2) */
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   SPI Driver Sample Code with NAU8822                  |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with NAU8822.\n");

    /* Init I2C0 to access NAU8822 */
    I2C0_Init();

    /* Reset SPI IP */
    SYS->IPRST1 |= SYS_IPRST1_SPI0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_SPI0RST_Msk;

    /* Configure I2S controller */
#ifdef INPUT_IS_LIN
    SPI0->I2SCTL = I2S_MODE_SLAVE | I2S_DATABIT_16 | I2S_STEREO | I2S_FORMAT_I2S;
#else
    SPI0->I2SCTL = I2S_MODE_SLAVE | I2S_DATABIT_16 | I2S_MONO | I2S_FORMAT_I2S;
#endif
    /* Set TX and RX FIFO threshold to middle value */
    SPI0->FIFOCTL = I2S_FIFO_TX_LEVEL_WORD_2 | I2S_FIFO_RX_LEVEL_WORD_2;
    SPI0->I2SCLK &= ~SPI_I2SCLK_BCLKDIV_Msk;

    /* Enable TX function, RX function and I2S mode. */
    SPI0->I2SCTL |= (SPI_I2SCTL_RXEN_Msk | SPI_I2SCTL_TXEN_Msk | SPI_I2SCTL_I2SEN_Msk);

    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(SPI0, 12000000);

#ifndef INPUT_IS_LIN
    I2S_SET_MONO_RX_CHANNEL(SPI0, I2S_MONO_LEFT);       //NAU8822 will store data in left channel
#endif

    PDMA_Init();

    /* Enable RX PDMA and TX PDMA function */
    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);

    while (1);
}

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA->INTSTS;

    if (u32Status & 0x1)   /* abort */
    {
        if ((PDMA->ABTSTS) & 0x4)
            PDMA->ABTSTS |= PDMA_ABTSTS_ABTIF1_Msk;

        PDMA->ABTSTS |= PDMA_ABTSTS_ABTIF2_Msk;
    }
    else if (u32Status & 0x2)
    {
        if ((PDMA->TDSTS) & 0x4)            /* channel 2 done */
        {
            /* Copy RX data to TX buffer */
            memcpy(&g_au32PcmTxBuff[g_u8TxIdx ^ 1], &g_au32PcmRxBuff[g_u8RxIdx], BUFF_LEN * 4);
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetRxSGTable(g_u8RxIdx);
            g_u8RxIdx ^= 1;
        }

        if ((PDMA->TDSTS) & 0x2)            /* channel 1 done */
        {
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetTxSGTable(g_u8TxIdx);
            g_u8TxIdx ^= 1;
        }

        PDMA->TDSTS |= PDMA_TDSTS_TDIF1_Msk;
        PDMA->TDSTS |= PDMA_TDSTS_TDIF2_Msk;
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
