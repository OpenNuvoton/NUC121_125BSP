/**************************************************************************//**
 * @file     FMC.h
 * @version  V3.01
 * @brief    NUC121 series FMC driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __FMC_H__
#define __FMC_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/

/** @addtogroup FMC_EXPORTED_CONSTANTS FMC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00000000UL    /*!< APROM  Base Address         */
#define FMC_LDROM_BASE          0x00100000UL    /*!< LDROM  Base Address         */
#define FMC_SPROM_BASE          0x00200000UL    /*!< SPROM  Base Address         */
#define FMC_CONFIG_BASE         0x00300000UL    /*!< CONFIG Base Address         */

#define FMC_CONFIG0_ADDR        (FMC_CONFIG_BASE)       /*!< CONFIG 0 Address */
#define FMC_CONFIG1_ADDR        (FMC_CONFIG_BASE + 4)   /*!< CONFIG 1 Address */


#define FMC_FLASH_PAGE_SIZE     0x200           /*!< Flash Page Size (512 Bytes) */
#define FMC_LDROM_SIZE          0x1200          /*!< LDROM Size (4.5 kBytes)       */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCTL_BS_LDROM     0x2     /*!< ISPCTL setting to select to boot from LDROM */
#define FMC_ISPCTL_BS_APROM     0x0     /*!< ISPCTL setting to select to boot from APROM */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00     /*!< ISP Command: Read Flash               */
#define FMC_ISPCMD_PROGRAM      0x21     /*!< ISP Command: 32-bit Program Flash     */
#define FMC_ISPCMD_WRITE_8      0x61     /*!< ISP Command: 64-bit program Flash     */
#define FMC_ISPCMD_PAGE_ERASE   0x22     /*!< ISP Command: Page Erase Flash         */
#define FMC_ISPCMD_READ_CID     0x0B     /*!< ISP Command: Read Company ID          */
#define FMC_ISPCMD_READ_UID     0x04     /*!< ISP Command: Read Unique ID           */
#define FMC_ISPCMD_READ_DID     0x0C     /*!< ISP Command: Read Device ID           */
#define FMC_ISPCMD_VECMAP       0x2E     /*!< ISP Command: Set vector mapping       */
#define FMC_ISPCMD_CHECKSUM     0x0D     /*!< ISP Command: Read Checksum            */
#define FMC_ISPCMD_CAL_CHECKSUM 0x2D     /*!< ISP Command: Run Check Calculation    */

#define FMC_TIMEOUT_READ        ((SystemCoreClock/10)*2) /*!< Read command time-out 100 ms         \hideinitializer */
#define FMC_TIMEOUT_WRITE       ((SystemCoreClock/10)*2) /*!< Write command time-out 100 ms        \hideinitializer */
#define FMC_TIMEOUT_ERASE       ((SystemCoreClock/10)*4) /*!< Erase command time-out 200 ms        \hideinitializer */
#define FMC_TIMEOUT_CHKSUM      (SystemCoreClock*2)      /*!< Get checksum command time-out 2 s    \hideinitializer */
#define FMC_TIMEOUT_CHKALLONE   (SystemCoreClock*2)      /*!< Check-all-one command time-out 2 s   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Global variables                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    eFMC_ERRCODE_SUCCESS       = 0,
    eFMC_ERRCODE_CMD_TIMEOUT   = -1,
    eFMC_ERRCODE_INVALID_PARAM = -2,
    eFMC_ERRCODE_CMD_FAIL      = -3,
} E_FMC_ERRCODE;
extern int32_t  g_FMC_i32ErrCode; /*!< FMC global error code */

/*---------------------------------------------------------------------------------------------------------*/
/*  FTCTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_FTCTL_OPTIMIZE_24MHZ        0x01       /*!< Frequency Optimize Mode <= 24Mhz */
#define FMC_FTCTL_OPTIMIZE_50MHZ        0x02       /*!< Frequency Optimize Mode <= 50Mhz */

/** @} end of group FMC_EXPORTED_CONSTANTS */

/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief      Enable ISP Function
 *
 *
 *
 * @details    This function will set ISPEN bit of ISPCTL control register to enable ISP function.
 *
 * \hideinitializer
 */
#define FMC_ENABLE_ISP()          (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)  /*!< Enable ISP Function  */

/**
 * @brief      Disable ISP Function
 *
 *
 *
 * @details    This function will clear ISPEN bit of ISPCTL control register to disable ISP function.
 *
 * \hideinitializer
 */
#define FMC_DISABLE_ISP()         (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)  /*!< Disable ISP Function */

/**
 * @brief      Enable LDROM Update Function
 *
 *
 *
 * @details    This function will set LDUEN bit of ISPCTL control register to enable LDROM update function.
 *             User needs to set LDUEN bit before they can update LDROM.
 *
 * \hideinitializer
 */
#define FMC_ENABLE_LD_UPDATE()    (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)  /*!< Enable LDROM Update Function   */

/**
 * @brief      Disable LDROM Update Function
 *
 *
 *
 * @details    This function will set ISPEN bit of ISPCTL control register to disable LDROM update function.
 *
 * \hideinitializer
 */
#define FMC_DISABLE_LD_UPDATE()   (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)  /*!< Disable LDROM Update Function  */

/**
 * @brief      Enable User Configuration Update Function
 *
 *
 *
 * @details    This function will set CFGUEN bit of ISPCTL control register to enable User Configuration update function.
 *             User needs to set CFGUEN bit before they can update User Configuration area.
 *
 * \hideinitializer
 */
#define FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */

/**
 * @brief      Disable User Configuration Update Function
 *
 *
 *
 * @details    This function will clear CFGUEN bit of ISPCTL control register to disable User Configuration update function.
 *
 * \hideinitializer
 */
#define FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


/**
 * @brief      Enable APROM Update Function
 *
 *
 *
 * @details    This function will set APUEN bit of ISPCTL control register to enable APROM update function.
 *             User needs to set APUEN bit before they can update APROM in APROM boot mode.
 *
 * \hideinitializer
 */
#define FMC_ENABLE_AP_UPDATE()    (FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk)  /*!< Enable APROM Update Function   */

/**
 * @brief      Disable APROM Update Function
 *
 *
 *
 * @details    This function will clear APUEN bit of ISPCTL control register to disable APROM update function.
 *
 * \hideinitializer
 */
#define FMC_DISABLE_AP_UPDATE()   (FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk)  /*!< Disable APROM Update Function  */

/**
 * @brief      Get ISP fail flag
 *
 *
 * @retval     0 Previous ISP command execution result is successful
 * @retval     1 Previous ISP command execution result is fail
 *
 * @details    ISPFF flag of ISPCTL is used to indicate ISP command success or fail.
 *             This function will return the ISPFF flag to identify ISP command OK or fail.
 *
 * \hideinitializer
 */
#define FMC_GET_FAIL_FLAG()       ((FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) ? 1 : 0)


/**
 * @brief      Select booting from APROM
 *
 *
 *
 * @details    If MCU is working without IAP, user need to set BS bit of ISPCTL and reset CPU to execute the code of LDROM/APROM.
 *             This function is used to set BS bit of ISPCTL to boot to APROM.
 *
 * @note       To valid new BS bit setting, user also need to trigger CPU reset or System Reset Request after setting BS bit.
 *
 * \hideinitializer
 */
#define FMC_SET_APROM_BOOT()      (FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk)

/**
 * @brief      Select booting from APROM
 *
 *
 *
 * @details    If MCU is working without IAP, user need to set/clear BS bit of ISPCON and reset CPU to execute the code of APROM/LDROM.
 *             This function is used to clear BS bit of ISPCTL to boot to LDROM.
 *
 * @note       To valid new BS bit setting, user also need to trigger CPU reset or System Reset Request after clear BS bit.
 *
 * \hideinitializer
 */
#define FMC_SET_LDROM_BOOT()      (FMC->ISPCTL |= FMC_ISPCTL_BS_Msk)


/*---------------------------------------------------------------------------------------------------------*/
/* inline functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
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
static __INLINE void FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t  tout;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = 0x1;
    __ISB();

    tout = FMC_TIMEOUT_WRITE;

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
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
static __INLINE uint32_t FMC_Read(uint32_t u32Addr)
{
    int32_t  tout;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
    __ISB();

    tout = FMC_TIMEOUT_READ;

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
    }

    return FMC->ISPDAT;
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32Addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 2048 bytes.
 *             If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
 *             If command fail, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_FAIL.
 *
 * @retval      0 Success
 * @retval     -1 Erase failed
 *
 */
static __INLINE int32_t FMC_Erase(uint32_t u32Addr)
{
    int32_t  tout = FMC_TIMEOUT_ERASE;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;

    if (u32Addr == FMC_SPROM_BASE)
        FMC->ISPDAT = 0x0055AA03;

    FMC->ISPTRG = 0x1;
    __ISB();

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
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
 * @param[in]   u8Index  UID index. 0 = UID[31:0], 1 = UID[63:32], 2 = UID[95:64]
 *
 * @return      The 32-bit unique ID data of specified UID index.
 *
 * @details     To read out 96-bit Unique ID.
 *              If timeout, g_FMC_i32ErrCode will be set to eFMC_ERRCODE_CMD_TIMEOUT.
 *
 */
static __INLINE uint32_t FMC_ReadUID(uint8_t u8Index)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = (u8Index << 2);
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
    __ISB();

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
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
static __INLINE uint32_t FMC_ReadCID(void)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    FMC->ISPADDR = 0x0;                          /* Must keep 0x0 when read CID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
    __ISB();                                     /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
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
static __INLINE uint32_t FMC_ReadDID(void)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0;                           /* Must keep 0x0 when read DID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
    __ISB();                                    /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
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
static __INLINE uint32_t FMC_ReadPID(void)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04;                        /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
    __ISB();                                    /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
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
static __INLINE uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;          /* Set ISP Command Code */
    FMC->ISPADDR = (0x04 * u32Index) + 0x10;    /* The UCID is at offset 0x10 with word alignment. */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
    __ISB();                                    /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
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
static __INLINE void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    int32_t  tout = FMC_TIMEOUT_WRITE;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_VECMAP; /* Set ISP Command Code */
    FMC->ISPADDR = u32PageAddr;      /* The address of specified page which will be map to address 0x0. It must be page alignment. */
    FMC->ISPTRG = 0x1;               /* Trigger to start ISP procedure */
    __ISB();                         /* To make sure ISP/CPU be Synchronized */

    /* Waiting for ISP Done */
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
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
static __INLINE uint32_t FMC_GetVECMAP(void)
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
static __INLINE uint32_t FMC_GetCheckSum(uint32_t u32Addr, int32_t i32Size)
{
    int32_t  tout;

    g_FMC_i32ErrCode = eFMC_ERRCODE_SUCCESS;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CHECKSUM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = i32Size;
    FMC->ISPTRG = 0x1;
    __ISB();

    tout = FMC_TIMEOUT_CHKSUM;

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
    }

    FMC->ISPCMD = FMC_ISPCMD_CHECKSUM;
    FMC->ISPTRG = 0x1;

    tout = FMC_TIMEOUT_CHKSUM;

    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}

    if (tout <= 0)
    {
        g_FMC_i32ErrCode = eFMC_ERRCODE_CMD_TIMEOUT;
        return 0xFFFFFFFF;
    }

    return FMC->ISPDAT;
}

void FMC_Open(void);
void FMC_Close(void);
void FMC_EnableAPUpdate(void);
void FMC_DisableAPUpdate(void);
void FMC_EnableConfigUpdate(void);
void FMC_DisableConfigUpdate(void);
void FMC_EnableLDUpdate(void);
void FMC_DisableLDUpdate(void);
void FMC_EnableSPUpdate(void);
void FMC_DisableSPUpdate(void);
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
void FMC_SetBootSource(int32_t i32BootSrc);
int32_t FMC_GetBootSource(void);
uint32_t FMC_ReadDataFlashBaseAddr(void);
void FMC_EnableFreqOptimizeMode(uint32_t u32Mode);


/** @} end of group FMC_EXPORTED_FUNCTIONS */

/** @} end of group FMC_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif


#endif
