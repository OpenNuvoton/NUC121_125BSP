/***************************************************************************//**
 * @file     fmc_user.c
 * @brief    NUC121 series FMC driver source file
 * @version  2.0.0
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "fmc_user.h"


int FMC_Proc(unsigned int u32Cmd, unsigned int addr_start, unsigned int addr_end, unsigned int *data)
{
    unsigned int u32Addr;
    unsigned int u32Reg;

    for (u32Addr = addr_start; u32Addr < addr_end; data++, u32Addr += 4)
    {
        FMC->ISPADDR = u32Addr;

        if ((u32Addr & (FMC_FLASH_PAGE_SIZE - 1)) == 0 && u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPTRG = 0x1;

            while (FMC->ISPTRG & 0x1) ;
        }

        FMC->ISPCMD = u32Cmd;

        if (u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *data;
        }

        FMC->ISPTRG = 0x1;
        //        __ISB();

        while (FMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

        u32Reg = FMC->ISPCTL;

        if (u32Reg & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL = u32Reg;
            return -1;
        }

        if (u32Cmd == FMC_ISPCMD_READ)
        {
            *data = FMC->ISPDAT;
        }

    }

    return 0;
}

void ReadData(unsigned int addr_start, unsigned int addr_end, unsigned int *data)    // Read data from flash
{
    FMC_Proc(FMC_ISPCMD_READ, addr_start, addr_end, data);
    return;
}

int WriteData(unsigned int addr_start, unsigned int addr_end, unsigned int *data)  // Write data into flash
{
    return FMC_Proc(FMC_ISPCMD_PROGRAM, addr_start, addr_end, data);
}
