/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit LIN frame including header and response in UART LIN mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/* CheckSum Method */
#define MODE_CLASSIC    2
#define MODE_ENHANCED   1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t g_i32Pointer;
uint8_t g_au8SendData[12] ;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTest(void);
void LIN_FunctionTestUsingLinCtlReg(void);
void LIN_MasterTest(uint32_t u32id, uint32_t u32ModeSel);
void LIN_MasterTestUsingLinCtlReg(uint32_t u32Id, uint32_t u32ModeSel);
void LIN_SendHeader(uint32_t u32id);
void LIN_SendHeaderUsingLinCtlReg(uint32_t u32Id, uint32_t u32HeaderSel);
void LIN_SendResponse(int32_t i32CheckSumOption, uint32_t *pu32TxBuf);
void LIN_SendResponseWithByteCnt(int32_t i32CheckSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt);
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel);
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt);


/*---------------------------------------------------------------------------------------------------------*/
/*  Sample Code Menu                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|                LIN Sample Program                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| LIN Master function test                            - [1] |\n");
    printf("| LIN Master function test using UART_LINCTL register - [2] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

void LIN_TestItem()
{
    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     LIN Master Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Master send header with ID = 0x30                     |\n");
    printf("| [2] Master send header and response with classic checksum |\n");
    printf("| [3] Master send header and response with enhanced checksum|\n");
    printf("|                                                           |\n");
    printf("| To measure UART0_TXD(PC.5) to check waveform ...          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LIN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTest()
{
    uint32_t u32Item;

    /* Set UART Configuration */
    UART_SetLine_Config(UART0, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART0 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART0 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART0 Tx pin to check it.
    */

    do
    {
        LIN_TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);

        switch (u32Item)
        {
            case '1':
                LIN_SendHeader(0x30);
                break;

            case '2':
                LIN_MasterTest(0x35, MODE_CLASSIC);
                break;

            case '3':
                LIN_MasterTest(0x12, MODE_ENHANCED);
                break;

            default:
                break;
        }
    } while (u32Item != 27);

    UART_Close(UART0);

    printf("\nLIN Sample Code End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*  LIN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTestUsingLinCtlReg(void)
{
    uint32_t u32Item;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLine_Config(UART0, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART0 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART0 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART0 Tx pin to check it.
    */

    do
    {
        LIN_TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);

        switch (u32Item)
        {
            case '1':
                LIN_SendHeaderUsingLinCtlReg(0x30, UART_LINCTL_HSEL_BREAK_SYNC_ID);
                break;

            case '2':
                LIN_MasterTestUsingLinCtlReg(0x35, MODE_CLASSIC);
                break;

            case '3':
                LIN_MasterTestUsingLinCtlReg(0x12, MODE_ENHANCED);
                break;

            default:
                break;
        }
    } while (u32Item != 27);

    /* Clear header select setting */
    UART0->LINCTL &= ~UART_LINCTL_HSEL_Msk;

    UART_Close(UART0);

    printf("\nLIN Sample Code End.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Master send header and response                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTest(uint32_t u32Id, uint32_t u32ModeSel)
{
    uint32_t au32testPattern[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};

    /* Send ID=0x35 Header and Response TestPatten */
    LIN_SendHeader(u32Id);
    LIN_SendResponse(u32ModeSel, &au32testPattern[0]);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Master send header and response                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTestUsingLinCtlReg(uint32_t u32Id, uint32_t u32ModeSel)
{
    uint8_t au8TestPattern[9] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x0}; // 8 data byte + 1 byte checksum
    uint32_t u32Idx;

    if (u32ModeSel == MODE_CLASSIC)
    {
        /* Send break+sync+ID */
        LIN_SendHeaderUsingLinCtlReg(u32Id, UART_LINCTL_HSEL_BREAK_SYNC_ID);
        /* Compute checksum without ID and fill checksum value to  au8TestPattern[8] */
        au8TestPattern[8] = ComputeChksumValue(&au8TestPattern[0], 8);
        UART_Write(UART0, &au8TestPattern[0], 9);
    }
    else if (u32ModeSel == MODE_ENHANCED)
    {
        /* Send break+sync+ID and fill ID value to g_u8SendData[0]*/
        LIN_SendHeaderUsingLinCtlReg(u32Id, UART_LINCTL_HSEL_BREAK_SYNC);

        /* Fill test pattern to g_u8SendData[1]~ g_u8SendData[8] */
        for (u32Idx = 0; u32Idx < 8; u32Idx++)
            g_au8SendData[g_i32Pointer++] = au8TestPattern[u32Idx];

        /* Compute checksum value with ID and fill checksum value to g_u8SendData[9] */
        g_au8SendData[g_i32Pointer++] = ComputeChksumValue(&g_au8SendData[0], 9) ;
        UART_Write(UART0, &g_au8SendData[1], 9);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute Parity Value                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t GetParityValue(uint32_t u32Id)
{
    uint32_t u32Res = 0, au32ID[6], au32PBit[2], u32Mask = 0;

    for (u32Mask = 0; u32Mask < 6; u32Mask++)
        au32ID[u32Mask] = (u32Id & (1 << u32Mask)) >> u32Mask;

    au32PBit[0] = (au32ID[0] + au32ID[1] + au32ID[2] + au32ID[4]) % 2;
    au32PBit[1] = (!((au32ID[1] + au32ID[3] + au32ID[4] + au32ID[5]) % 2));

    u32Res = u32Id + (au32PBit[0] << 6) + (au32PBit[1] << 7);
    return u32Res;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value , MODE_CLASSIC:(Not Include ID)    MODE_ENHANCED:(Include ID)                    */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel)
{
    uint32_t u32Idx, u32CheckSum = 0;

    for (u32Idx = u32ModeSel; u32Idx <= 9; u32Idx++)
    {
        u32CheckSum += pu8Buf[u32Idx];

        if (u32CheckSum >= 256)
            u32CheckSum -= 255;
    }

    return (255 - u32CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt)
{
    uint32_t u32Idx, u32CheckSum = 0;

    for (u32Idx = 0 ; u32Idx < u32ByteCnt; u32Idx++)
    {
        u32CheckSum += pu8Buf[u32Idx];

        if (u32CheckSum >= 256)
            u32CheckSum -= 255;
    }

    return (uint8_t)(255 - u32CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Header Field                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendHeader(uint32_t u32Id)
{
    g_i32Pointer = 0 ;

    /* Set LIN operation mode, Tx mode and break field length is 12 bits */
    UART_SelectLINMode(UART0, UART_ALTCTL_LINTXEN_Msk, 12);

    g_au8SendData[g_i32Pointer++] = 0x55 ;                   // SYNC Field
    g_au8SendData[g_i32Pointer++] = GetParityValue(u32Id);   // ID+Parity Field
    UART_Write(UART0, g_au8SendData, 2);
}

/*-------------------------------------------------------------------------------------------------------------------------------*/
/*  Send LIN Header Field                                                                                                        */
/*  u32HeaderSel =  UART_LINCTL_HSEL_BREAK/UART_LINCTL_HSEL_BREAK_SYNC/UART_LINCTL_HSEL_BREAK_SYNC_ID                            */
/*-------------------------------------------------------------------------------------------------------------------------------*/
void LIN_SendHeaderUsingLinCtlReg(uint32_t u32Id, uint32_t u32HeaderSel)
{
    g_i32Pointer = 0 ;

    /* Switch back to LIN Function */
    UART0->FUNCSEL = UART_FUNCSEL_LIN;

    /* Set LIN 1. PID as 0x30 [UART_LINCTL_PID(0x30)]
               2. Header select as includes "break field", "sync field" and "frame ID field".[UART_LINCTL_HSEL_BREAK_SYNC_ID]
               3. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               4. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
               5. ID Parity Enable. Hardware will calculate and fill P0/P1 automatically  [UART_LINCTL_IDPEN_Msk]
    */
    if (u32HeaderSel == UART_LINCTL_HSEL_BREAK_SYNC_ID)
    {
        UART0->LINCTL = UART_LINCTL_PID(u32Id) | UART_LINCTL_HSEL_BREAK_SYNC_ID |
                        UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12) | UART_LINCTL_IDPEN_Msk;
        /* LIN TX Send Header Enable */
        UART0->LINCTL |= UART_LINCTL_SENDH_Msk;

        /* Wait until break field, sync field and ID field transfer completed */
        while ((UART0->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);
    }

    /* Set LIN 1. Header select as includes "break field" and "sync field".[UART_LINCTL_HSEL_BREAK_SYNC]
               2. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               3. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
    */
    else if (u32HeaderSel == UART_LINCTL_HSEL_BREAK_SYNC)
    {
        UART0->LINCTL = UART_LINCTL_HSEL_BREAK_SYNC | UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12);
        /* LIN TX Send Header Enable */
        UART0->LINCTL |= UART_LINCTL_SENDH_Msk;

        /* Wait until break field and sync field transfer completed */
        while ((UART0->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);

        /* Send ID field, g_u8SendData[0] is ID+parity field*/
        g_au8SendData[g_i32Pointer++] = GetParityValue(u32Id);   // ID+Parity Field
        UART_Write(UART0, g_au8SendData, 1);
    }

    /* Set LIN 1. Header select as includes "break field".[UART_LINCTL_HSEL_BREAK]
               2. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               3. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
    */
    else if (u32HeaderSel == UART_LINCTL_HSEL_BREAK)
    {
        UART0->LINCTL = UART_LINCTL_HSEL_BREAK | UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12);
        /* LIN TX Send Header Enable */
        UART0->LINCTL |= UART_LINCTL_SENDH_Msk;

        /* Wait until break field transfer completed */
        while ((UART0->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);

        /* Send sync field and ID field*/
        g_au8SendData[g_i32Pointer++] = 0x55 ;                  // SYNC Field
        g_au8SendData[g_i32Pointer++] = GetParityValue(u32Id);   // ID+Parity Field
        UART_Write(UART0, g_au8SendData, 2);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Response Field                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponse(int32_t i32CheckSumOption, uint32_t *pu32TxBuf)
{
    int32_t i32Idx;

    for (i32Idx = 0; i32Idx < 8; i32Idx++)
        g_au8SendData[g_i32Pointer++] = pu32TxBuf[i32Idx] ;

    g_au8SendData[g_i32Pointer++] = GetCheckSumValue(g_au8SendData, i32CheckSumOption) ; //CheckSum Field

    UART_Write(UART0, g_au8SendData + 2, 9);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Response Field                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponseWithByteCnt(int32_t CheckSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt)
{
    uint32_t u32Idx;

    /* Prepare data */
    for (u32Idx = 0; u32Idx < u32ByteCnt; u32Idx++)
        g_au8SendData[g_i32Pointer++] = pu32TxBuf[u32Idx] ;

    /* Prepare check sum */
    if (CheckSumOption == MODE_CLASSIC)
        g_au8SendData[g_i32Pointer++] = GetCheckSumValue(&g_au8SendData[2], u32ByteCnt) ;  //CheckSum Field
    else if (CheckSumOption == MODE_ENHANCED)
        g_au8SendData[g_i32Pointer++] = GetCheckSumValue(&g_au8SendData[1], (u32ByteCnt + 1)) ; //CheckSum Field

    /* Send data and check sum */
    UART_Write(UART0, g_au8SendData + 2, 9);
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable USCI module clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PC multi-function pins for UART0 RXD(PC.4) and TXD(PC.5) */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_UART0_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_UART0_TXD;

    /* Set PB multi-function pins for USCI0_DAT0(PB.4) and USCI0_DAT1(PB.5) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_USCI0_DAT0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_USCI0_DAT1;

}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(USCI0_RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 9600);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    uint32_t u32Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for testing */
    UART0_Init();

    /* Init USCI0 for printf */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART sample LIN function */
    do
    {
        TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);

        switch (u32Item)
        {
            case '1':
                LIN_FunctionTest();
                break;

            case '2':
                LIN_FunctionTestUsingLinCtlReg();
                break;

            default:
                break;
        }
    } while (u32Item != 27);

    while (1);

}

