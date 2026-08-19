/****************************************************************************
 * @file     fmc.c
 * @version  V3.00
 * @brief    NUC121 series FMC driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup FMC_Driver FMC Driver
    @{
*/

/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
    @{
*/

int32_t  g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS; /*!< FMC global error code */

/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32Data  32-bit Data to program
 *
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *             If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
 *             If command fail, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_FAIL.
 *
 */
void FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t  i32Timeout;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = 0x1;
    __ISB();

    i32Timeout = FMC_TIMEOUT_WRITE;

    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;
    }
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *              If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
 *
 */
uint32_t FMC_Read(uint32_t u32Addr)
{
    int32_t  i32Timeout;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
    __ISB();

    i32Timeout = FMC_TIMEOUT_READ;

    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFFUL;
    }

    return FMC->ISPDAT;
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32Addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 512 bytes.
 *             If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
 *             If command fail, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_FAIL.
 *
 * @retval      0 Success
 * @retval     -1 Erase failed
 *
 */
int32_t FMC_Erase(uint32_t u32Addr)
{
    int32_t  i32Timeout = FMC_TIMEOUT_ERASE;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;

    if (u32Addr == FMC_SPROM_BASE)
    {
        FMC->ISPDAT = 0x0055AA03;
    }

    FMC->ISPTRG = 0x1;
    __ISB();

    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return -1;
    }

    /* Check ISPFF flag to know whether erase OK or fail. */
    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        // comment For FMC FPGA
        //FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_FAIL;

        return -1;
    }

    return 0;
}

/**
 * @brief       Read Unique ID
 *
 * @param[in]   u32Index  UID index. 0 = UID[31:0], 1 = UID[63:32], 2 = UID[95:64]
 *
 * @return      The 32-bit unique ID data of specified UID index.
 *
 * @details     To read out 96-bit Unique ID.
 *              If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
 *
 */
uint32_t FMC_ReadUID(uint32_t u32Index)
{
    int32_t  i32Timeout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = (u32Index << 2U);
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
    __ISB();

    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFFUL;
    }

    return FMC->ISPDAT;
}

/**
  * @brief    Read company ID
  *
  *
  * @return   The company ID (32-bit)
  *
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  *           If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
  *
  */
uint32_t FMC_ReadCID(void)
{
    int32_t  i32Timeout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    FMC->ISPADDR = 0x0;                          /* Must keep 0x0 when read CID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
    __ISB();                                     /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFFUL;
    }

    return FMC->ISPDAT;
}

/**
  * @brief    Read device ID
  *
  *
  * @return   The device ID (32-bit)
  *
  * @details  This function is used to read device ID.
  *           If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
  *
  */
uint32_t FMC_ReadDID(void)
{
    int32_t  i32Timeout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0;                           /* Must keep 0x0 when read DID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
    __ISB();                                    /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFFUL;
    }

    return FMC->ISPDAT;
}

/**
  * @brief    Read product ID
  *
  *
  * @return   The product ID (32-bit)
  *
  * @details  This function is used to read product ID.
  *           If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
  *
  */
uint32_t FMC_ReadPID(void)
{
    int32_t  i32Timeout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04;                        /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
    __ISB();                                    /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFFUL;
    }

    return FMC->ISPDAT;
}

/**
  * @brief      To read UCID
  *
  * @param[in]  u32Index    Index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  *
  * @return     The UCID of specified index
  *
  * @details    This function is used to read unique chip ID (UCID).
  *             If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
  *
  */
uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    int32_t  i32Timeout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;              /* Set ISP Command Code */
    FMC->ISPADDR = (0x04UL * u32Index) + 0x10UL;    /* The UCID is at offset 0x10 with word alignment. */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;             /* Trigger to start ISP procedure */
    __ISB();                                        /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFFUL;
    }

    return FMC->ISPDAT;
}

/**
 * @brief       Set vector mapping address
 *
 * @param[in]   u32PageAddr  The page address to remap to address 0x0. The address must be page alignment.
 *
 * @details     This function is used to set VECMAP to map specified page to vector page (0x0).
 *              If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
 * @note
 *              VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 */
void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    int32_t  i32Timeout = FMC_TIMEOUT_WRITE;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_VECMAP; /* Set ISP Command Code */
    FMC->ISPADDR = u32PageAddr;      /* The address of specified page which will be map to address 0x0. It must be page alignment. */
    FMC->ISPTRG = 0x1;               /* Trigger to start ISP procedure */
    __ISB();                         /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
    }
}

/**
 * @brief       Get current vector mapping address.
 *
 *
 * @return      The current vector mapping address.
 *
 * @details     To get VECMAP value which is the page address for remapping to vector page (0x0).
 *
 * @note
 *              VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 *
 */
uint32_t FMC_GetVECMAP(void)
{
    return (FMC->ISPSTS & FMC_ISPSTS_VECMAP_Msk);
}

/**
 * @brief       Get Flash Checksum
 *
 * @param[in]   u32Addr    Specific flash start address
 * @param[in]   i32Size    Specific a size of Flash area
 *
 * @return      A checksum value of a flash block.
 *
 * @details     To get VECMAP value which is the page address for remapping to vector page (0x0).
 *              If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
 *
 */
uint32_t FMC_GetCheckSum(uint32_t u32Addr, int32_t i32Size)
{
    int32_t  i32Timeout;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CHECKSUM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = i32Size;
    FMC->ISPTRG = 0x1;
    __ISB();

    i32Timeout = FMC_TIMEOUT_CHKSUM;

    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFFUL;
    }

    FMC->ISPCMD = FMC_ISPCMD_CHECKSUM;
    FMC->ISPTRG = 0x1;

    i32Timeout = FMC_TIMEOUT_CHKSUM;

    while ((i32Timeout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (i32Timeout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFFUL;
    }

    return FMC->ISPDAT;
}

/**
  * @brief      Set boot source from LDROM or APROM after next software reset
  *
  * @param[in]  i32BootSrc
  *                1: Boot from LDROM
  *                0: Boot from APROM
  *
  *
  * @details   This function is used to switch APROM boot or LDROM boot. User need to call
  *            FMC_SetBootSource to select boot source first, then use CPU reset or
  *            System Reset Request to reset system.
  *
  */
void FMC_SetBootSource(int32_t i32BootSrc)
{
    if (i32BootSrc)
    {
        FMC->ISPCTL |= FMC_ISPCTL_BS_Msk; /* Boot from LDROM */
    }
    else
    {
        FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;/* Boot from APROM */
    }
}

/**
  * @brief    Disable ISP Functions
  *
  *
  *
  * @details  This function will clear ISPEN bit of ISPCON to disable ISP function
  *
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief    Disable APROM update function
  *
  *
  *
  * @details  Disable APROM update function will forbid APROM programming when boot form APROM.
  *           APROM update is default to be disable.
  *
  */
void FMC_DisableAPUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk;
}


/**
  * @brief    Disable User Configuration update function
  *
  *
  *
  * @details  Disable User Configuration update function will forbid User Configuration programming.
  *           User Configuration update is default to be disable.
  */
void FMC_DisableConfigUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk;
}


/**
  * @brief    Disable LDROM update function
  *
  *

  * @details  Disable LDROM update function will forbid LDROM programming.
  *           LDROM update is default to be disable.
  */
void FMC_DisableLDUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;
}


/**
  * @brief    Disable SPROM update function
  *
  *

  * @details  Disable SPROM update function will forbid SPROM programming.
  *           SPROM update is default to be disable.
  */
void FMC_DisableSPUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_SPUEN_Msk;
}


/**
  * @brief    Enable APROM update function
  *
  *
  *
  * @details  Enable APROM to be able to program when boot from APROM.
  *
  */
void FMC_EnableAPUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_APUEN_Msk;
}


/**
  * @brief    Enable User Configuration update function
  *
  *
  *
  * @details  Enable User Configuration to be able to program.
  *
  */
void FMC_EnableConfigUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_CFGUEN_Msk;
}


/**
  * @brief    Enable LDROM update function
  *
  *
  *
  * @details  Enable LDROM to be able to program.
  *
  */
void FMC_EnableLDUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_LDUEN_Msk;
}


/**
  * @brief    Enable SPROM update function
  *
  *
  *
  * @details  Enable SPROM to be able to program.
  *
  */
void FMC_EnableSPUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_SPUEN_Msk;
}


/**
  * @brief    Get the current boot source
  *
  *
  * @retval   0 This chip is currently booting from APROM
  * @retval   1 This chip is currently booting from LDROM
  *
  * @note     This function only show the boot source.
  *           User need to read ISPSTA register to know if IAP mode supported or not in relative boot.
  */
int32_t FMC_GetBootSource(void)
{
    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
    {
        return 1;
    }

    return 0;
}


/**
  * @brief    Enable FMC ISP function
  *
  *
  *
  * @details  ISPEN bit of ISPCON must be set before we can use ISP commands.
  *           Therefore, To use all FMC function APIs, user needs to call FMC_Open() first to enable ISP functions.
  *
  * @note     ISP functions are write-protected. user also needs to unlock it by calling SYS_UnlockReg() before using all ISP functions.
  *
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief    Get the base address of Data Flash if enabled.
  *
  *
  * @return   The base address of Data Flash
  *
  * @details  This function is used to return the base address of Data Flash.
  *
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBA;
}


/**
  * @brief       Read the User Configuration words.
  *
  * @param[out]  u32Config  The word buffer to store the User Configuration data.
  * @param[in]   u32Count   The word count to be read.
  *
  * @retval       0 Success
  * @retval      -1 Failed
  *
  * @details     This function is used to read the settings of user configuration.
  *              if u32Count = 1, Only CONFIG0 will be returned to the buffer specified by u32Config.
  *              if u32Count = 2, Both CONFIG0 and CONFIG1 will be returned.
  */
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count)
{
    uint32_t i;

    for (i = 0; i < u32Count; i++)
    {
        u32Config[i] = FMC_Read(FMC_CONFIG_BASE + (i * 4UL));
    }

    if (g_FMC_i32ErrCode != 0)
    {
        return g_FMC_i32ErrCode;
    }

    return 0;
}


/**
  * @brief    Write User Configuration
  *
  * @param[in]  u32Config The word buffer to store the User Configuration data.
  * @param[in]  u32Count The word count to program to User Configuration.
  *
  * @retval     0 Success
  * @retval    -1 Failed
  *
  * @details  User must enable User Configuration update before writing it.
  *           User must erase User Configuration before writing it.
  *           User Configuration is also be page erase. User needs to backup necessary data
  *           before erase User Configuration.
  */
int32_t FMC_WriteConfig(const uint32_t *u32Config, uint32_t u32Count)
{
    uint32_t i;

    for (i = 0; i < u32Count; i++)
    {
        FMC_Write(FMC_CONFIG_BASE + (i * 4UL), u32Config[i]);

        if (FMC_Read(FMC_CONFIG_BASE + (i * 4UL)) != u32Config[i])
        {
            return -1;
        }
    }

    return 0;
}


/**
 * @brief      Enable Flash Access Frequency  Optimization Mode
 *
 * @param[in]  u32Mode   Optimize flash access cycle mode
 *             - \ref FMC_FTCTL_OPTIMIZE_24MHZ
 *             - \ref FMC_FTCTL_OPTIMIZE_50MHZ
 *
 *
 * @details    This function will set FOM bit fields of FTCTL register to set flash access frequency optimization mode.
 *
 * @note       The flash optimization mode (FOM) bits are write protect.
 *
 */
void FMC_EnableFreqOptimizeMode(uint32_t u32Mode)
{
    FMC->FTCTL &= ~FMC_FTCTL_FOM_Msk;
    FMC->FTCTL |= (u32Mode << FMC_FTCTL_FOM_Pos);
}

/** @} end of group FMC_EXPORTED_FUNCTIONS */

/** @} end of group FMC_Driver */

/** @} end of group Standard_Driver */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
