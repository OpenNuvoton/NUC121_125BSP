Windows Test Tool - HIDTransferTest
=====================================

A Windows command-line tool for testing HID Transfer on the Audio + HID composite device.

Location
--------
BSP/Tool/HIDTransferTest

Usage
-----
HIDTransferTest -pid 0xB00A -pagesize 512

Options
-------
  -pid 0xB00A      Specify the USB Product ID of the HID interface (Audio + HID composite)
  -pagesize 512   Set the page size to 512 bytes (matches NUC121 RRAM page size)

Description
-----------
This tool communicates with the NUC121 device running the USBD_Audio_HID_Transfer
firmware. It performs read/write/erase operations over the HID interface of the
Audio + HID composite device, using the specified PID to identify the correct
USB device and a page size of 512 bytes aligned to the NUC121 RRAM page size.
