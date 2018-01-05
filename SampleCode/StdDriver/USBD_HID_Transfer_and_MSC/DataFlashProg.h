/******************************************************************************//**
 * @file     DataFlashProg.h
 * @version  V3.00
 * @brief    Data flash programming driver header
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#define MASS_STORAGE_OFFSET       0x00002800  /* To avoid the code to write APROM */
#define DATA_FLASH_STORAGE_SIZE   (22*1024)  /* Configure the DATA FLASH storage size. To pass USB-IF MSC Test, it needs > 64KB */ 
                                             /*Windows 7 will take about 20KB to format. actually we can not get full 21 KB free space*/
#define FLASH_PAGE_SIZE           512
#define BUFFER_PAGE_SIZE          512

#endif  /* __DATA_FLASH_PROG_H__ */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
