;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.                                       */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.globl	loaderImage1Base
	.globl	loaderImage1Limit

	.align	4

	.text

loaderImage1Base:
    .incbin  "FMC_IAP_LDROM.bin"
loaderImage1Limit:
    .space   4

    .end
