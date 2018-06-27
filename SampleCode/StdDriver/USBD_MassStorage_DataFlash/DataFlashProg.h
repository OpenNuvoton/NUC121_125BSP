/****************************************************************************//**
 * @file     DataFlashProg.h
 * @version  V3.00
 * @brief    Data flash programming driver header
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#if defined(__GNUC__)

    #define MASS_STORAGE_OFFSET       0x00004000  /* To avoid the code to write APROM */
    #define DATA_FLASH_STORAGE_SIZE   (16*1024)  /* Configure the DATA FLASH storage size. To pass USB-IF MSC Test, it needs > 64KB */
    /* Windows 7 will take about 20 KB to do formatting. We cannot format the flash due to the free space is not enough. */
#else //for KEIL/IAR

    #define MASS_STORAGE_OFFSET       0x00002000 /* To avoid the code to write APROM */
    #define DATA_FLASH_STORAGE_SIZE   (24*1024)  /* Configure the DATA FLASH storage size. To pass USB-IF MSC Test, it needs > 64KB */
    /* Windows 7 will take about 20KB to do formatting. We only get free space, 4 KB, to use after the flash is formatted */
#endif

#define FLASH_PAGE_SIZE           512
#define BUFFER_PAGE_SIZE          512

/* HIRC trim setting:
 *    HIRC trim reference clock is from USB signal.
 *    HIRC trim operation is keep going if clock is inaccuracy.
 *    HIRC Trim retry count limitation is 512 loops.
 *    Trim value calculation is based on average difference in 4 clocks of reference clock.
 *    Enable HIRC auto trim function and trim HIRC to 48 MHz.
 */
#define DEFAULT_HIRC_TRIM_SETTING    ((0x1ul<<SYS_IRCTCTL_REFCKSEL_Pos)| \
                                      (0x0ul<<SYS_IRCTCTL_CESTOPEN_Pos)| \
                                      (0x3ul<<SYS_IRCTCTL_RETRYCNT_Pos)| \
                                      (0x0ul<<SYS_IRCTCTL_LOOPSEL_Pos) | \
                                      (0x2ul<<SYS_IRCTCTL_FREQSEL_Pos))


#endif  /* __DATA_FLASH_PROG_H__ */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
