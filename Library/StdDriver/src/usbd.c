/**************************************************************************//**
 * @file     usbd.c
 * @version  V3.00
 * @brief    NUC121 series USBD driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <string.h>
#include "NuMicro.h"

#if 0
    #define DBG_PRINTF      printf
#else
    #define DBG_PRINTF(...)
#endif

#define USBNULL                ((void*)0)                 ///< For USB Library Usage: USBNULL pointer

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup USBD_Driver USBD Driver
  @{
*/


/** @addtogroup USBD_EXPORTED_FUNCTIONS USBD Exported Functions
  @{
*/

/* Global variables for Control Pipe */
uint8_t g_USBD_au8SetupPacket[8];                /*!< Setup packet buffer */
volatile uint8_t g_USBD_u8RemoteWakeupEn = 0U;  /*!< Remote wake up function enable flag */

/**
 * @cond HIDDEN_SYMBOLS
 */
static volatile uint8_t *s_USBD_pu8CtrlInPointer        = USBNULL;
static volatile uint8_t *s_USBD_pu8CtrlOutPointer       = USBNULL;
static volatile uint32_t s_USBD_u32CtrlInSize           = 0UL;
static volatile uint32_t s_USBD_u32CtrlOutSize          = 0UL;
static volatile uint32_t s_USBD_u32CtrlOutSizeLimit     = 0UL;
static volatile uint32_t s_USBD_u32UsbAddr              = 0UL;
static volatile uint32_t s_USBD_u32CtrlMaxPktSize       = 8UL;
static volatile uint8_t  s_USBD_u8CtrlInZeroFlag        = 0U;
static volatile uint32_t s_USBD_u32CtrlOutToggle        = 0UL;
/**
 * @endcond
 */

static const S_USBD_INFO_T *g_USBD_sINFO;                  /*!< A pointer for USB information structure */

VENDOR_REQ g_usbd_pfnVendorRequest          = USBNULL;    /*!< USB Vendor Request Functional Pointer */
CLASS_REQ g_usbd_pfnClassRequest            = USBNULL;    /*!< USB Class Request Functional Pointer */
SET_INTERFACE_REQ g_usbd_pfnSetInterface    = USBNULL;    /*!< USB Set Interface Functional Pointer */
SET_CONFIG_CB g_usbd_pfnSetConfigCallback   = USBNULL;    /*!< USB Set configuration callback function pointer */
uint32_t g_u32EpStallLock                   = 0UL;        /*!< Bit map flag to lock specified EP when SET_FEATURE */

/**
  * @brief      This function makes USBD module to be ready to use
  *
  * @param[in]  param           The structure of USBD information.
  * @param[in]  pfnClassReq     USB Class request callback function.
  * @param[in]  pfnSetInterface USB Set Interface request callback function.
  *
  *
  * @details    This function will enable USB controller, USB PHY transceiver and pull-up resistor of USB_D+ pin. USB PHY will drive SE0 to bus.
  */
void USBD_Open(const S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface)
{
    g_USBD_sINFO = param;
    g_usbd_pfnClassRequest = pfnClassReq;
    g_usbd_pfnSetInterface = pfnSetInterface;

    /* get EP0 maximum packet size */
    s_USBD_u32CtrlMaxPktSize = g_USBD_sINFO->gu8DevDesc[7];

    /* Initial USB engine */
    USBD->ATTR = 0x7D0UL;
    /* Force SE0 */
    USBD_SET_SE0();
}

/**
  * @brief    This function makes USB host to recognize the device
  *
  *
  *
  * @details  Enable WAKEUP, FLDET, USB and BUS interrupts. Disable software-disconnect function after 100ms delay with SysTick timer.
  */
void USBD_Start(void)
{
    CLK_SysTickDelay(100000UL);
    /* Disable software-disconnect function */
    USBD_CLR_SE0();

    /* Clear USB-related interrupts before enable interrupt */
    USBD_CLR_INT_FLAG(USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INTSTS_WAKEUP);

    /* Enable USB-related interrupts. */
    USBD_ENABLE_INT(USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP_IDLE);
}

/**
  * @brief      Get the received SETUP packet
  *
  * @param[in]  buf A buffer pointer used to store 8-byte SETUP packet.
  *
  *
  * @details    Store SETUP packet to a user-specified buffer.
  *
  */
void USBD_GetSetupPacket(uint8_t *buf)
{
    USBD_MemCopy(buf, g_USBD_au8SetupPacket, 8UL);
}

/**
  * @brief    Process SETUP packet
  *
  *
  *
  * @details  Parse SETUP packet and perform the corresponding action.
  *
  */
void USBD_ProcessSetupPacket(void)
{
    s_USBD_u32CtrlOutToggle = 0UL;
    /* Get SETUP packet from USB buffer */
    USBD_MemCopy(g_USBD_au8SetupPacket, (uint8_t *)(USBD_BUF_BASE + USBD->STBUFSEG), 8UL);

    /* Check the request type */
    switch (g_USBD_au8SetupPacket[0] & 0x60UL)
    {
        case REQ_STANDARD:   // Standard
        {
            USBD_StandardRequest();
            break;
        }

        case REQ_CLASS:   // Class
        {
            if (g_usbd_pfnClassRequest != USBNULL)
            {
                g_usbd_pfnClassRequest();
            }

            break;
        }

        case REQ_VENDOR:   // Vendor
        {
            if (g_usbd_pfnVendorRequest != USBNULL)
            {
                g_usbd_pfnVendorRequest();
            }

            break;
        }

        default:   // reserved
        {
            /* Setup error, stall the device */
            USBD_SET_EP_STALL(EP0);
            USBD_SET_EP_STALL(EP1);
            break;
        }
    }
}

/**
  * @brief    Process GetDescriptor request
  *
  *
  *
  * @details  Parse GetDescriptor request and perform the corresponding action.
  *
  */
void USBD_GetDescriptor(void)
{
    uint32_t u32Len;

    s_USBD_u8CtrlInZeroFlag = (uint8_t)0UL;
    u32Len = g_USBD_au8SetupPacket[7];
    u32Len <<= 8UL;
    u32Len += g_USBD_au8SetupPacket[6];

    switch (g_USBD_au8SetupPacket[3])
    {
        // Get Device Descriptor
        case DESC_DEVICE:
        {
            u32Len = Minimum(u32Len, LEN_DEVICE);
            DBG_PRINTF("Get device desc, %d\n", u32Len);

            USBD_PrepareCtrlIn((uint8_t *)g_USBD_sINFO->gu8DevDesc, u32Len);

            break;
        }

        // Get Configuration Descriptor
        case DESC_CONFIG:
        {
            uint32_t u32TotalLen;

            u32TotalLen = g_USBD_sINFO->gu8ConfigDesc[3];
            u32TotalLen = g_USBD_sINFO->gu8ConfigDesc[2] + (u32TotalLen << 8);

            DBG_PRINTF("Get config desc len %d, acture len %d\n", u32Len, u32TotalLen);

            if (u32Len > u32TotalLen)
            {
                u32Len = u32TotalLen;

                if ((u32Len % s_USBD_u32CtrlMaxPktSize) == 0UL)
                {
                    s_USBD_u8CtrlInZeroFlag = (uint8_t)1UL;
                }
            }

            DBG_PRINTF("Minimum len %d\n", u32Len);

            USBD_PrepareCtrlIn((uint8_t *)g_USBD_sINFO->gu8ConfigDesc, u32Len);

            break;
        }

        // Get HID Descriptor
        case DESC_HID:
        {
            /* CV3.0 HID Class Descriptor Test,
               Need to indicate index of the HID Descriptor within gu8ConfigDescriptor, specifically HID Composite device. */
            uint32_t u32ConfigDescOffset;   // u32ConfigDescOffset is configuration descriptor offset (HID descriptor start index)
            u32Len = Minimum(u32Len, LEN_HID);
            DBG_PRINTF("Get HID desc, %d\n", u32Len);

            u32ConfigDescOffset = g_USBD_sINFO->gu32ConfigHidDescIdx[g_USBD_au8SetupPacket[4]];
            USBD_PrepareCtrlIn((uint8_t *)&g_USBD_sINFO->gu8ConfigDesc[u32ConfigDescOffset], u32Len);

            break;
        }

        // Get Report Descriptor
        case DESC_HID_RPT:
        {
            DBG_PRINTF("Get HID report, %d\n", u32Len);

            if (u32Len > g_USBD_sINFO->gu32HidReportSize[g_USBD_au8SetupPacket[4]])
            {
                u32Len = g_USBD_sINFO->gu32HidReportSize[g_USBD_au8SetupPacket[4]];

                if ((u32Len % s_USBD_u32CtrlMaxPktSize) == 0UL)
                {
                    s_USBD_u8CtrlInZeroFlag = (uint8_t)1UL;
                }
            }

            USBD_PrepareCtrlIn((uint8_t *)g_USBD_sINFO->gu8HidReportDesc[g_USBD_au8SetupPacket[4]], u32Len);
            break;
        }

        // Get String Descriptor
        case DESC_STRING:
        {
            // Get String Descriptor
            if (g_USBD_au8SetupPacket[2] < 4UL)
            {
                if (u32Len > g_USBD_sINFO->gu8StringDesc[g_USBD_au8SetupPacket[2]][0])
                {
                    u32Len = g_USBD_sINFO->gu8StringDesc[g_USBD_au8SetupPacket[2]][0];

                    if ((u32Len % s_USBD_u32CtrlMaxPktSize) == 0UL)
                    {
                        s_USBD_u8CtrlInZeroFlag = (uint8_t)1UL;
                    }
                }

                DBG_PRINTF("Get string desc %d\n", u32Len);

                USBD_PrepareCtrlIn((uint8_t *)g_USBD_sINFO->gu8StringDesc[g_USBD_au8SetupPacket[2]], u32Len);
            }
            else
            {
                // Not support. Reply STALL.
                USBD_SET_EP_STALL(EP0);
                USBD_SET_EP_STALL(EP1);

                DBG_PRINTF("Unsupported string desc (%d). Stall ctrl pipe.\n", g_USBD_au8SetupPacket[2]);
            }

            break;
        }

        case DESC_BOS:
        {
            if (g_USBD_sINFO->gu8BosDesc)
            {
                u32Len = Minimum(u32Len, LEN_BOS + LEN_DEVCAP);
                USBD_PrepareCtrlIn((uint8_t *)g_USBD_sINFO->gu8BosDesc, u32Len);
            }
            else
            {
                USBD_SET_EP_STALL(EP0);
                USBD_SET_EP_STALL(EP1);
            }

            break;
        }

        default:
        {
            // Not support. Reply STALL.
            USBD_SET_EP_STALL(EP0);
            USBD_SET_EP_STALL(EP1);

            DBG_PRINTF("Unsupported get desc type. stall ctrl pipe\n");

            break;
        }
    }
}

/**
  * @brief    Process standard request
  *
  *
  *
  * @details  Parse standard request and perform the corresponding action.
  *
  */
void USBD_StandardRequest(void)
{
    static volatile uint32_t s_USBD_u32UsbConfig = 0UL;
    static volatile uint32_t s_USBD_u32UsbAltInterface = 0UL;

    /* clear global variables for new request */
    s_USBD_pu8CtrlInPointer = USBNULL;
    s_USBD_u32CtrlInSize = 0UL;

    if (g_USBD_au8SetupPacket[0] & 0x80UL)   /* request data transfer direction */
    {
        // Device to host
        switch (g_USBD_au8SetupPacket[1])
        {
            case GET_CONFIGURATION:
            {
                // Return current configuration setting
                /* Data stage */
                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = s_USBD_u32UsbConfig;
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 1UL);
                /* Status stage */
                USBD_PrepareCtrlOut(USBNULL, 0UL);

                DBG_PRINTF("Get configuration\n");

                break;
            }

            case GET_DESCRIPTOR:
            {
                USBD_GetDescriptor();
                USBD_PrepareCtrlOut(USBNULL, 0UL); /* For status stage */
                break;
            }

            case GET_INTERFACE:
            {
                // Return current interface setting
                /* Data stage */
                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = s_USBD_u32UsbAltInterface;
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 1UL);
                /* Status stage */
                USBD_PrepareCtrlOut(USBNULL, 0UL);

                DBG_PRINTF("Get interface\n");

                break;
            }

            case GET_STATUS:
            {
                // Device
                if (g_USBD_au8SetupPacket[0] == 0x80UL)
                {
                    uint8_t u8Tmp;

                    u8Tmp = 0;

                    if (g_USBD_sINFO->gu8ConfigDesc[7] & 0x40UL)
                    {
                        u8Tmp |= (uint8_t)1UL; // Self-Powered/Bus-Powered.
                    }

                    if (g_USBD_sINFO->gu8ConfigDesc[7] & 0x20UL)
                    {
                        u8Tmp |= (uint8_t)(g_USBD_u8RemoteWakeupEn << 1UL); // Remote wake up
                    }

                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = u8Tmp;

                }
                // Interface
                else if (g_USBD_au8SetupPacket[0] == 0x81UL)
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = (uint8_t)0UL;
                }
                // Endpoint
                else if (g_USBD_au8SetupPacket[0] == 0x82UL)
                {
                    uint8_t ep = g_USBD_au8SetupPacket[4] & 0xFUL;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = (uint8_t)(USBD_GetStall(ep) ? 1UL : 0UL);
                }
                else
                {
                    /* Not Supported */
                }

                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1UL) = (uint8_t)0UL;
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 2UL);
                /* Status stage */
                USBD_PrepareCtrlOut(USBNULL, 0UL);

                DBG_PRINTF("Get status\n");

                break;
            }

            default:
            {
                /* Setup error, stall the device */
                USBD_SET_EP_STALL(EP0);
                USBD_SET_EP_STALL(EP1);

                DBG_PRINTF("Unknown request. stall ctrl pipe.\n");

                break;
            }
        }
    }
    else
    {
        // Host to device
        switch (g_USBD_au8SetupPacket[1])
        {
            case CLEAR_FEATURE:
            {
                if (g_USBD_au8SetupPacket[2] == FEATURE_ENDPOINT_HALT)
                {
                    uint32_t epNum;
                    uint32_t i;

                    /* EP number stall is not allow to be clear in MSC class "Error Recovery Test".
                       a flag: g_u32EpStallLock is added to support it */
                    epNum = g_USBD_au8SetupPacket[4] & 0xFUL;

                    for (i = 0UL; i < USBD_MAX_EP; i++)
                    {
                        if (((USBD->EP[i].CFG & 0xFUL) == epNum) && ((g_u32EpStallLock & (1UL << i)) == 0UL))
                        {
                            USBD->EP[i].CFGP &= ~USBD_CFGP_SSTALL_Msk;
                            USBD->EP[i].CFG &= ~USBD_CFG_DSQSYNC_Msk;
                            DBG_PRINTF("Clr stall ep%d %x\n", i, USBD->EP[i].CFGP);
                        }
                    }
                }
                else if (g_USBD_au8SetupPacket[2] == FEATURE_DEVICE_REMOTE_WAKEUP)
                {
                    g_USBD_u8RemoteWakeupEn = 0U;
                }
                else
                {
                    /* Not Supported */
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0UL);

                DBG_PRINTF("Clear feature op %d\n", g_USBD_au8SetupPacket[2]);

                break;
            }

            case SET_ADDRESS:
            {
                s_USBD_u32UsbAddr = g_USBD_au8SetupPacket[2];
                DBG_PRINTF("Set addr to %d\n", g_USBD_u32UsbAddr);

                // DATA IN for end of setup
                /* Status Stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0UL);

                break;
            }

            case SET_CONFIGURATION:
            {
                s_USBD_u32UsbConfig = g_USBD_au8SetupPacket[2];

                if (g_usbd_pfnSetConfigCallback != USBNULL)
                {
                    g_usbd_pfnSetConfigCallback();
                }

                // DATA IN for end of setup
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0UL);

                DBG_PRINTF("Set config to %d\n", g_USBD_u32UsbConfig);

                break;
            }

            case SET_FEATURE:
            {
                if (g_USBD_au8SetupPacket[2] == FEATURE_ENDPOINT_HALT)
                {
                    USBD_SetStall(g_USBD_au8SetupPacket[4] & 0xFUL);
                    DBG_PRINTF("Set feature. stall ep %d\n", g_USBD_au8SetupPacket[4] & 0xFUL);
                }
                else if (g_USBD_au8SetupPacket[2] == FEATURE_DEVICE_REMOTE_WAKEUP)
                {
                    g_USBD_u8RemoteWakeupEn = 1U;
                    DBG_PRINTF("Set feature. enable remote wakeup\n");
                }
                else
                {
                    /* Not Supported */
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0UL);

                break;
            }

            case SET_INTERFACE:
            {
                s_USBD_u32UsbAltInterface = g_USBD_au8SetupPacket[2];

                if (g_usbd_pfnSetInterface != USBNULL)
                {
                    g_usbd_pfnSetInterface();
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0UL);

                DBG_PRINTF("Set interface to %d\n", g_USBD_u32UsbAltInterface);

                break;
            }

            default:
            {
                /* Setup error, stall the device */
                USBD_SET_EP_STALL(EP0);
                USBD_SET_EP_STALL(EP1);

                DBG_PRINTF("Unsupported request. stall ctrl pipe.\n");

                break;
            }
        }
    }
}

/**
  * @brief      Prepare the first Control IN pipe
  *
  * @param[in]  pu8Buf  The pointer of data sent to USB host.
  * @param[in]  u32Size The IN transfer size.
  *
  *
  * @details    Prepare data for Control IN transfer.
  *
  */
void USBD_PrepareCtrlIn(const uint8_t *pu8Buf, uint32_t u32Size)
{
    uint32_t addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0);

    DBG_PRINTF("Prepare Ctrl In %d\n", u32Size);

    if (u32Size > s_USBD_u32CtrlMaxPktSize)
    {
        // Data size > MXPLD
        s_USBD_pu8CtrlInPointer = (uint8_t *)((uintptr_t)pu8Buf + s_USBD_u32CtrlMaxPktSize);
        s_USBD_u32CtrlInSize = u32Size - s_USBD_u32CtrlMaxPktSize;
        USBD_SET_DATA1(EP0);
        USBD_MemCopy((uint8_t *)addr, pu8Buf, s_USBD_u32CtrlMaxPktSize);
        USBD_SET_PAYLOAD_LEN(EP0, s_USBD_u32CtrlMaxPktSize);
    }
    else
    {
        // Data size <= MXPLD
        s_USBD_pu8CtrlInPointer = USBNULL;
        s_USBD_u32CtrlInSize = 0UL;
        USBD_SET_DATA1(EP0);
        USBD_MemCopy((uint8_t *)addr, pu8Buf, u32Size);
        USBD_SET_PAYLOAD_LEN(EP0, u32Size);
    }
}

/**
  * @brief    Repeat Control IN pipe
  *
  *
  *
  * @details  This function processes the remained data of Control IN transfer.
  *
  */
void USBD_CtrlIn(void)
{
    DBG_PRINTF("Ctrl In Ack. residue %d\n", g_USBD_u32CtrlInSize);

    if (s_USBD_u32CtrlInSize)
    {
        uint32_t addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0);

        // Process remained data
        if (s_USBD_u32CtrlInSize > s_USBD_u32CtrlMaxPktSize)
        {
            // Data size > MXPLD
            USBD_MemCopy((uint8_t *)addr, (uint8_t *)s_USBD_pu8CtrlInPointer, s_USBD_u32CtrlMaxPktSize);
            USBD_SET_PAYLOAD_LEN(EP0, s_USBD_u32CtrlMaxPktSize);
            s_USBD_pu8CtrlInPointer = (uint8_t *)((uintptr_t)s_USBD_pu8CtrlInPointer + s_USBD_u32CtrlMaxPktSize);
            s_USBD_u32CtrlInSize -= s_USBD_u32CtrlMaxPktSize;
        }
        else
        {
            // Data size <= MXPLD
            USBD_MemCopy((uint8_t *)addr, (uint8_t *)s_USBD_pu8CtrlInPointer, s_USBD_u32CtrlInSize);
            USBD_SET_PAYLOAD_LEN(EP0, s_USBD_u32CtrlInSize);
            s_USBD_pu8CtrlInPointer = USBNULL;
            s_USBD_u32CtrlInSize    = 0;
        }
    }
    else     // No more data for IN token
    {
        // In ACK for Set address
        if ((g_USBD_au8SetupPacket[0] == REQ_STANDARD) && (g_USBD_au8SetupPacket[1] == SET_ADDRESS))
        {
            if ((USBD_GET_ADDR() != s_USBD_u32UsbAddr) && (USBD_GET_ADDR() == 0UL))
            {
                USBD_SET_ADDR(s_USBD_u32UsbAddr);
            }
        }

        /* For the case of data size is integral times maximum packet size */
        if (s_USBD_u8CtrlInZeroFlag)
        {
            USBD_SET_PAYLOAD_LEN(EP0, 0UL);
            s_USBD_u8CtrlInZeroFlag = (uint8_t)0UL;
        }

        DBG_PRINTF("Ctrl In done.\n");

    }
}

/**
  * @brief      Prepare the first Control OUT pipe
  *
  * @param[in]  pu8Buf  The pointer of data received from USB host.
  * @param[in]  u32Size The OUT transfer size.
  *
  *
  * @details    This function is used to prepare the first Control OUT transfer.
  *
  */
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size)
{
    s_USBD_pu8CtrlOutPointer = pu8Buf;
    s_USBD_u32CtrlOutSize = 0UL;
    s_USBD_u32CtrlOutSizeLimit = u32Size;
    USBD_SET_PAYLOAD_LEN(EP1, s_USBD_u32CtrlMaxPktSize);
}

/**
  * @brief    Repeat Control OUT pipe
  *
  *
  *
  * @details  This function processes the successive Control OUT transfer.
  *
  */
void USBD_CtrlOut(void)
{
    DBG_PRINTF("Ctrl Out Ack %d\n", g_USBD_u32_CtrlOutSize);

    if (s_USBD_u32CtrlOutToggle != (USBD->EPSTS & USBD_EPSTS_EPSTS1_Msk))
    {
        s_USBD_u32CtrlOutToggle = USBD->EPSTS & USBD_EPSTS_EPSTS1_Msk; //keep H/W EP1 toggle status

        if (s_USBD_u32CtrlOutSize < s_USBD_u32CtrlOutSizeLimit)
        {
            uint32_t u32Size = USBD_GET_PAYLOAD_LEN(EP1);
            uint32_t addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP1);
            USBD_MemCopy((uint8_t *)s_USBD_pu8CtrlOutPointer, (uint8_t *)addr, u32Size);
            s_USBD_pu8CtrlOutPointer = (uint8_t *)((uintptr_t)s_USBD_pu8CtrlOutPointer + u32Size);
            s_USBD_u32CtrlOutSize += u32Size;

            if (s_USBD_u32CtrlOutSize < s_USBD_u32CtrlOutSizeLimit)
            {
                USBD_SET_PAYLOAD_LEN(EP1, s_USBD_u32CtrlMaxPktSize);
            }
        }
    }
    else
    {
        USBD_SET_PAYLOAD_LEN(EP1, s_USBD_u32CtrlMaxPktSize);
    }
}

/**
  * @brief    Reset software flags
  *
  *
  *
  * @details  This function resets all variables for protocol and resets USB device address to 0.
  *
  */
void USBD_SwReset(void)
{
    // Reset all variables for protocol
    s_USBD_pu8CtrlInPointer = USBNULL;
    s_USBD_u32CtrlInSize = 0UL;
    s_USBD_pu8CtrlOutPointer  = USBNULL;
    s_USBD_u32CtrlOutSize = 0UL;
    s_USBD_u32CtrlOutSizeLimit = 0UL;
    g_u32EpStallLock = 0UL;
    (void)memset(g_USBD_au8SetupPacket, 0, 8UL);

    /* Reset PID DATA0 */
    for (uint32_t i = 0UL; i < USBD_MAX_EP; i++)
    {
        USBD->EP[i].CFG &= ~USBD_CFG_DSQSYNC_Msk;
    }

    // Reset USB device address
    USBD_SET_ADDR(0UL);
}

/**
 * @brief       USBD Set Vendor Request
 *
 * @param[in]   pfnVendorReq    Vendor Request Callback Function
 *
 *
 * @details     This function is used to set USBD vendor request callback function
 */
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq)
{
    g_usbd_pfnVendorRequest = pfnVendorReq;
}

/**
 * @brief       The callback function which called when get SET CONFIGURATION request
 *
 * @param[in]   pfnSetConfigCallback    Callback function pointer for SET CONFIGURATION request
 *
 *
 * @details     This function is used to set the callback function which will be called at SET CONFIGURATION request.
 */
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback)
{
    g_usbd_pfnSetConfigCallback = pfnSetConfigCallback;
}


/**
 * @brief       EP stall lock function to avoid stall clear by USB SET FEATURE request.
 *
 * @param[in]   u32EpBitmap    Use bitmap to select which endpoints will be locked
 *
 *
 * @details     This function is used to lock relative endpoint to avoid stall clear by SET FEATURE requst.
 *              If ep stall locked, user needs to reset USB device or re-configure device to clear it.
 */
void USBD_LockEpStall(uint32_t u32EpBitmap)
{
    g_u32EpStallLock = u32EpBitmap;
}





/** @} end of group USBD_EXPORTED_FUNCTIONS */

/** @} end of group USBD_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}

#endif

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
