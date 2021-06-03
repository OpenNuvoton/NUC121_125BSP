# NUC121/NUC125 Series CMSIS BSP

This BSP folder

## .\Document\


- CMSIS.html<br>
	Document of CMSIS version 5.1.1.

- NuMicro NUC121_125 Series CMSIS BSP Driver Reference.chm<br>
	This document shows the usage of drivers in NUC121/NUC125 BSP.

- NuMicro NUC121_125 Series CMSIS BSP Revision History.pdf<br>
	This document shows the revision history of NUC121/NUC125 BSP.

## .\Library\


- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V5.1.1 definitions by Arm® Corp.

- Device<br>
	CMSIS compliant device header file.

- StdDriver<br>
	All peripheral driver header and source files.

## .\Sample Code\


- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened.<p>
	The hard fault handler show some information including program counter, which is the address where the processor is executed when the hard fault occurs. The listing file (or map file) can show what function and instruction that is.<p>
	It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample code for Nuvoton NuMicro ISP Programming Tool.

- NuTiny-SDK-NUC121_125<br>
	Sample code for NUC121/NUC125 Tiny Board.

- RegBased<br>
	Sample codes which access control registers directly.

- Semihost<br>
	Show how to print and get character through IDE console window.

- StdDriver<br>
	Demonstrate the usage of NUC121/NUC125 series MCU peripheral driver APIs.

- Template<br>
	A project template for NUC121/NUC125 series MCU.


# License

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.<p>
NUC121/NUC125 BSP files are provided under the Apache-2.0 license.

