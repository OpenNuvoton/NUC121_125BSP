/****************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief A composite device sample code for USB audio class with HID data transfer function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

#define CRYSTAL_LESS    1
#define HIRC_AUTO_TRIM  (SYS_IRCTCTL_REFCKSEL_Msk | 0x2);   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT       (SYS_BASE+0x110)
#define TRIM_THRESHOLD  16      /* Each value is 0.125%, max 2% */

#if CRYSTAL_LESS
    static volatile uint32_t s_u32DefaultTrim, s_u32LastTrim;
#endif

void HID_UpdateKbData(void);

void SYS_Init(void)
{

    /* Set PF multi-function pins for X32_OUT(PF.0) and X32_IN(PF.1) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_XT_OUT;

    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_XT_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal HIRC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

#if (CRYSTAL_LESS)
    /* Switch HCLK clock source to Internal HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select module clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));
#else
    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_48MHZ);

    /* Select module clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(2));
#endif

    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PC.4 as SPI1_I2SMCLK function pins */
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC4MFP_SPI0_I2SMCLK;

    /*Set PC.11, PC.12 as I2C0_SDA and I2C0_SCL function pins*/
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC11MFP_I2C0_SDA | SYS_GPC_MFPH_PC12MFP_I2C0_SCL;

    /* Set SPI0_I2S interface: I2S1_LRCLK (PC.0), I2S1_DO (PC.3), I2S1_DI (PC.2), I2S1_BCLK (PC.1) */
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_SPI0_SS | SYS_GPC_MFPL_PC3MFP_SPI0_MOSI | SYS_GPC_MFPL_PC2MFP_SPI0_MISO | SYS_GPC_MFPL_PC1MFP_SPI0_CLK;

    /* Set PD.10 as CLKO pin */
    SYS->GPD_MFPH = SYS_GPD_MFPH_PD10MFP_CLKO;

    /* Enable CLKO (PD.10) for monitor HCLK. CLKO = HCLK/8 Hz */
    CLK_EnableCKO(CLK_CLKSEL2_CLKOSEL_HCLK, 2, 0);

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD;

}

void UART0_Init(void)
{
    /* Reset IP */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);

    /* Get I2C Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C_PORT));
}

void KEY_Init(void)
{
    /* Init key I/O */
    PC->MODE |= (0x3ul << 5 * 2);
    PC->MODE |= (0x3ul << 8 * 2);
    PD->MODE |= (0x3ul << 11 * 2) | (0x3ul << 8 * 2);
    PF->MODE |= (0x3ul << 2 * 2);
    PC5 = 1;   // Play/pause
    PD11 = 1;  // Next (Not support on M45xD/M45xC)
    PF2 = 1;   // Previous
    PD8 = 1;   // Volumn Up
    PC8 = 1;   // Volumn Down

    /* Enable Debounce and set debounce time */
    PC->DBEN = (1 << 5);
    PC->DBEN = (1 << 8);
    PD->DBEN = (1 << 11) | (1 << 8);
    PF->DBEN = (1 << 2);
    GPIO->DBCTL = 0x17; // ~12.8 ms

}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;

    /*
        This sample code is used to demo USB Audio Class + NAU8822 (WAU8822) + HID keyboard + HID media key + HID Transer.
        User can define PLAY_RATE in usbd_audio.h to support 48000Hz, 32000Hz, 16000Hz and 8000Hz.

        The audio is input from NAU8822 AUXIN.
        The audio is output by NAU8822 Headphone output.

        NAU8822 is connect with I2S (PC.0~PC.3) and controlled by I2C0 (PC.11, PC.12).
        NAU8822 clock source is also come from SPI_I2S (MCLK, PC.4).
        Headphone MUTE control by PC13.

        keyboard:
            PC5 ==> 'a'
            PC8 ==> 'b'

        Media Key:
            PC5  ==> Play/Pause
            PD11 ==> Next
            PF2  ==> Previous
            PD8  ==> Vol+
            PC8  ==> Vol-

        HID Transfer:
            A windows tool with source files are in path \WindowsTool of sample code.
            User can use it to transfer data between PC and USB device.

    */


    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

    /* Init HID key */
    KEY_Init();


    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Audio CODEC Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C to access NAU8822 */

    I2C_Init();

    /* User can change audio codec settings through UART and I2C interfaces. Set PA.11 to 0 to controll codec through I2C. */
    GPIO_SetMode(PA, BIT11, GPIO_MODE_QUASI);
    PA11 = 0;

    /* Initialize WAU8822 codec */
    WAU8822_Setup();

    I2S_Open(SPI0, I2S_MODE_SLAVE, PLAY_RATE, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(SPI0, 12000000);

    /* Fill dummy data to I2S TX for start I2S iteration */
    for (i = 0; i < 4; i++)
        I2S_WRITE_TX_FIFO(SPI0, 0);

    /* Start I2S play iteration */
    I2S_EnableInt(SPI0, I2S_FIFO_TXTH_INT_MASK | I2S_FIFO_RXTH_INT_MASK);

    USBD_Open(&gsInfo, UAC_ClassRequest, (SET_INTERFACE_REQ)UAC_SetInterface);
    /* Endpoint configuration */
    UAC_Init();
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(SPI0_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim value */
    s_u32DefaultTrim = M32(TRIM_INIT);
    s_u32LastTrim = s_u32DefaultTrim;

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
#endif

    /* I2S interrupt has higher frequency then USBD interrupt.
       Therefore, we need to set I2S with higher priority to avoid
       I2S interrupt pending too long time when USBD interrupt happen. */
    NVIC_SetPriority(USBD_IRQn, 3);
    NVIC_SetPriority(SPI0_IRQn, 2);


    /* start to IN data */
    g_u8KeyReady = 1;
    g_u8MediaKeyReady = 1;


    while (SYS->PDID)
    {
        uint8_t ch;
        uint32_t u32Reg, u32Data;
        extern int32_t kbhit(void);

#if CRYSTAL_LESS

        /* Start USB trim function if it is not enabled. */
        if ((SYS->IRCTCTL & SYS_IRCTCTL_FREQSEL_Msk) != 0x2)
        {
            /* Start USB trim only when USB signal arrived */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /* Enable USB clock trim function */
                SYS->IRCTCTL = HIRC_AUTO_TRIM;
            }
        }

        /* Disable USB Trim when any error found */
        if (SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk))
        {
            /* Last TRIM */
            M32(TRIM_INIT) = s_u32LastTrim;

            /* Disable USB clock trim function */
            SYS->IRCTCTL = 0;

            /* Clear trim error flags */
            SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

        }

        /* Check trim value whether it is over the threshold */
        if ((M32(TRIM_INIT) > (s_u32DefaultTrim + TRIM_THRESHOLD)) || (M32(TRIM_INIT) < (s_u32DefaultTrim - TRIM_THRESHOLD)))
        {
            /* Write updated value */
            M32(TRIM_INIT) = s_u32LastTrim;
        }
        else
        {
            /* Backup trim value */
            s_u32LastTrim =  M32(TRIM_INIT);
        }

#endif

        /* Adjust codec sampling rate to synch with USB. The adjustment range is +-0.005% */
        AdjFreq();

        /* Set audio volume according USB volume control settings */
        VolumnControl();

        /* User can change audio codec settings by I2C at run-time if necessary */
        if (!kbhit())
        {
            printf("\nEnter codec setting:\n");
            // Get Register number
            ch = getchar();
            u32Reg = ch - '0';
            ch = getchar();
            u32Reg = u32Reg * 10 + (ch - '0');
            printf("%u\n", u32Reg);

            // Get data
            ch = getchar();
            u32Data = (ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10;
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            printf("%03x\n", u32Data);
            I2C_WriteWAU8822(u32Reg,  u32Data);
        }

        /* HID Keyboard */
        HID_UpdateKbData();

#ifdef NONBLOCK_PRINTF
        putchar(0);
#endif

    }
}

void HID_UpdateKbData(void)
{
    int32_t i;
    uint8_t *buf;
    uint32_t key = 0xF;
    static uint32_t preKey;
    int32_t n;
    int32_t i32Mode;

    // Select the hardware key function
    i32Mode = 0;

    switch (i32Mode)
    {
        case 0:   // Keyboard
        {

            // PC5 => 'a'
            // PD11=> 'b'
            n = 8;

            if (g_u8KeyReady)
            {
                buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(HID_KEY_IN_EP));

                key = (!PC5) | (!PD11 << 1);


                if (key == 0)
                {
                    for (i = 0; i < n; i++)
                    {
                        buf[i] = 0;
                    }

                    if (key != preKey)
                    {
                        preKey = key;
                        g_u8KeyReady = 0;
                        /* Trigger to note key release */
                        USBD_SET_PAYLOAD_LEN(HID_KEY_IN_EP, n);
                    }
                }
                else
                {
                    preKey = key;
                    buf[2] = 0;

                    if (!PC5)
                    {
                        buf[2] = 0x04; /* Key A */
                        printf("PC5 button pressed\n");
                    }
                    else if (!PD11)
                    {
                        buf[2] = 0x05; /* key B */
                        printf("PD11 button pressed\n");
                    }

                    g_u8KeyReady = 0;
                    USBD_SET_PAYLOAD_LEN(HID_KEY_IN_EP, n);

                }
            }

            break;
        }

        case 1:   // MultiMedia key
        {
            n = 8;

            if (g_u8MediaKeyReady)
            {
                buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(HID_MEDIA_IN_EP));

                // PC5, play/pause
                // PD11, Next
                // PF2, Previous
                // PD8, Vol+
                // PC8, Vol-
                key = (!PC5) | (!PD11 << 1) | (!PF2 << 1) | (!PD8 << 1) | (!PC8 << 1);

                if (key == 0)
                {
                    for (i = 0; i < n; i++)
                    {
                        buf[i] = 0;
                    }

                    if (key != preKey)
                    {
                        preKey = key;
                        g_u8MediaKeyReady = 0;
                        /* Trigger to note key release */
                        USBD_SET_PAYLOAD_LEN(HID_MEDIA_IN_EP, n);
                    }
                }
                else
                {

                    // Don't repeat key when it is media key
                    if (preKey != key)
                    {
                        preKey = key;
                        buf[0] = 0;
                        buf[1] = 0;

                        if (!PC5)
                        {
                            buf[1] |= HID_CTRL_PAUSE;
                            printf("Pause\n");
                        }
                        else if (!PD11)
                        {
                            buf[1] |= HID_CTRL_NEXT;
                            printf("Next\n");
                        }
                        else if (!PF2)
                        {
                            buf[1] |= HID_CTRL_PREVIOUS;
                            printf("Previous\n");
                        }
                        else if (!PD8)
                        {
                            buf[0] |= HID_CTRL_VOLUME_INC;
                            printf("Vol+\n");
                        }
                        else if (!PC8)
                        {
                            buf[0] |= HID_CTRL_VOLUME_DEC;
                            printf("Vol-\n");
                        }

                        g_u8MediaKeyReady = 0;
                        USBD_SET_PAYLOAD_LEN(HID_MEDIA_IN_EP, n);
                    }

                }
            }

            break;
        }

        default:
            break;


    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

