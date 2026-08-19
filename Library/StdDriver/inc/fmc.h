/******************************************************************************
 * @file     fmc.h
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

#define FMC_CONFIG0_ADDR        (FMC_CONFIG_BASE)           /*!< CONFIG 0 Address */
#define FMC_CONFIG1_ADDR        (FMC_CONFIG_BASE + 4UL)     /*!< CONFIG 1 Address */


#define FMC_FLASH_PAGE_SIZE     0x200UL         /*!< Flash Page Size (512 Bytes)    */
#define FMC_LDROM_SIZE          0x1200UL        /*!< LDROM Size (4.5 kBytes)        */
#define FMC_VECMAP_SIZE         0x200UL         /*!< VECMAP Size (512 bytes)        */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCTL_BS_LDROM     0x2UL     /*!< ISPCTL setting to select to boot from LDROM */
#define FMC_ISPCTL_BS_APROM     0x0UL     /*!< ISPCTL setting to select to boot from APROM */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00UL     /*!< ISP Command: Read Flash               */
#define FMC_ISPCMD_PROGRAM      0x21UL     /*!< ISP Command: 32-bit Program Flash     */
#define FMC_ISPCMD_WRITE_8      0x61UL     /*!< ISP Command: 64-bit program Flash     */
#define FMC_ISPCMD_PAGE_ERASE   0x22UL     /*!< ISP Command: Page Erase Flash         */
#define FMC_ISPCMD_READ_CID     0x0BUL     /*!< ISP Command: Read Company ID          */
#define FMC_ISPCMD_READ_UID     0x04UL     /*!< ISP Command: Read Unique ID           */
#define FMC_ISPCMD_READ_DID     0x0CUL     /*!< ISP Command: Read Device ID           */
#define FMC_ISPCMD_VECMAP       0x2EUL     /*!< ISP Command: Set vector mapping       */
#define FMC_ISPCMD_CHECKSUM     0x0DUL     /*!< ISP Command: Read Checksum            */
#define FMC_ISPCMD_CAL_CHECKSUM 0x2DUL     /*!< ISP Command: Run Check Calculation    */

#define FMC_TIMEOUT_READ        ((SystemCoreClock / 10UL) * 2UL)    /*!< Read command time-out 100 ms         \hideinitializer */
#define FMC_TIMEOUT_WRITE       ((SystemCoreClock / 10UL) * 2UL)    /*!< Write command time-out 100 ms        \hideinitializer */
#define FMC_TIMEOUT_ERASE       ((SystemCoreClock / 10UL) * 4UL)    /*!< Erase command time-out 200 ms        \hideinitializer */
#define FMC_TIMEOUT_CHKSUM      (SystemCoreClock * 2UL)             /*!< Get checksum command time-out 2 s    \hideinitializer */
#define FMC_TIMEOUT_CHKALLONE   (SystemCoreClock * 2UL)             /*!< Check-all-one command time-out 2 s   \hideinitializer */

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
#define FMC_FTCTL_OPTIMIZE_24MHZ        0x01UL       /*!< Frequency Optimize Mode <= 24Mhz */
#define FMC_FTCTL_OPTIMIZE_50MHZ        0x02UL       /*!< Frequency Optimize Mode <= 50Mhz */

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

void FMC_Open(void);
void FMC_Close(void);
void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
uint32_t FMC_Read(uint32_t u32Addr);
int32_t FMC_Erase(uint32_t u32Addr);
uint32_t FMC_ReadUID(uint32_t u32Index);
uint32_t FMC_ReadCID(void);
uint32_t FMC_ReadDID(void);
uint32_t FMC_ReadPID(void);
uint32_t FMC_ReadUCID(uint32_t u32Index);
void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
uint32_t FMC_GetVECMAP(void);
uint32_t FMC_GetCheckSum(uint32_t u32Addr, int32_t i32Size);
void FMC_EnableAPUpdate(void);
void FMC_DisableAPUpdate(void);
void FMC_EnableConfigUpdate(void);
void FMC_DisableConfigUpdate(void);
void FMC_EnableLDUpdate(void);
void FMC_DisableLDUpdate(void);
void FMC_EnableSPUpdate(void);
void FMC_DisableSPUpdate(void);
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
int32_t FMC_WriteConfig(const uint32_t *u32Config, uint32_t u32Count);
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
