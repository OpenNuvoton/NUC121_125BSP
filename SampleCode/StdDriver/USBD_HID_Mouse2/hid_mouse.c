/******************************************************************************//**
 * @file     hid_mouse.c
 * @version  V3.00
 * @brief    USBD HID mouse sample file
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include "NuMicro.h"
#include "hid_mouse.h"
#include "usbd.h"


uint8_t volatile g_u8EP2Ready = 0;
uint8_t volatile g_u8Suspend = 0;


void PowerDown(void);

void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_VBDETIF_Msk)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_VBDETIF_Msk);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
            g_u8Suspend = 0;
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
            g_u8Suspend = 1;
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUSIF_Msk)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUSIF_Msk);

        if (u32State & USBD_ATTR_USBRST_Msk)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u8Suspend = 0;
        }

        if (u32State & USBD_ATTR_SUSPEND_Msk)
        {

            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();


        }

        if (u32State & USBD_ATTR_RESUME_Msk)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();

            g_u8Suspend = 0;
        }
    }

    if (u32IntSts & USBD_INTSTS_NEVWKIF_Msk)
    {
        /*Clear no-event wake up interrupt */
        USBD_CLR_INT_FLAG(USBD_INTSTS_NEVWKIF_Msk);
        USBD_ENABLE_USB();
        /*
           TODO: Implement the function that will be executed when device is woken by non-USB event.
        */
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USBIF_Msk)
    {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP_Msk)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP_Msk);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Interrupt IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }
    }
}

void EP2_Handler(void)  /* Interrupt IN handler */
{
    g_u8EP2Ready = 1;
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void HID_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* start to IN data */
    g_u8EP2Ready = 1;
}

void HID_ClassRequest(void)
{
    uint8_t au8Buf[8];

    USBD_GetSetupPacket(au8Buf);

    if (au8Buf[0] & 0x80)   /* request data transfer direction */
    {
        // Device to host
        switch (au8Buf[1])
        {
            case GET_REPORT:

            //             {
            //                 break;
            //             }
            case GET_IDLE:

            //             {
            //                 break;
            //             }
            case GET_PROTOCOL:

            //            {
            //                break;
            //            }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch (au8Buf[1])
        {
            case SET_REPORT:
            {
                if (au8Buf[3] == 3)
                {
                    /* Request Type = Feature */
                    USBD_SET_DATA1(EP1);
                    USBD_SET_PAYLOAD_LEN(EP1, 0);
                }

                break;
            }

            case SET_IDLE:
            {
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_PROTOCOL:

            //             {
            //                 break;
            //             }
            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}



void PowerDown()
{

    /* Wakeup Enable */
    USBD->INTEN |= USBD_INTEN_WKEN_Msk;

    PB4 = 1; // LED off to show we are in suspend.
    CLK_PowerDown();
    PB4 = 0; // LED on to show we are working.

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if (CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    /* Note HOST to resume USB tree if it is suspended and remote wakeup enabled */
    if (g_USBD_u8RemoteWakeupEn)
    {
        /* Enable PHY before sending Resume('K') state */
        USBD->ATTR |= USBD_ATTR_PHYEN_Msk;

        /* Keep remote wakeup for 1 ms */
        USBD->ATTR |= USBD_ATTR_RWAKEUP_Msk;
        CLK_SysTickDelay(1000); /* Delay 1ms */
        USBD->ATTR ^= USBD_ATTR_RWAKEUP_Msk;
    }

}

void HID_UpdateMouseData(void)
{
    uint32_t u32Reg;

    /*
       Key definition:
           PC0 Down
           PC1 right
           PC2 up
           PC3 right key
           PC4 left
           PC5 left key
    */

    u32Reg = PC->PIN & 0x3F;

    /* Enter power down when USB suspend */
    if (g_u8Suspend)
    {
        PowerDown();

        /* Waiting for key release */
        while ((PC->PIN & 0x3F) != 0x3F);
    }

    if (g_u8EP2Ready)
    {
        uint32_t u32MouseKey;
        uint8_t u8Buf[4] = {0};
        static int32_t i32x = 0, i32y = 0;
        static uint32_t u32MousePreKey = 0xFFFF;

        /* To control Y axis */
        if ((u32Reg & 1) == 0)
            i32y += 1;
        else if ((u32Reg & 4) == 0)
            i32y += -1;
        else
            i32y = 0;

        if (i32y > 48) i32y = 48;

        if (i32y < -48) i32y = -48;

        /* To control X axis */
        if ((u32Reg & 2) == 0)
            i32x += 1;
        else if ((u32Reg & 0x10) == 0)
            i32x += -1;
        else
            i32x = 0;

        if (i32x > 48) i32x = 48;

        if (i32x < -48) i32x = -48;

        /* Mouse key */
        u32MouseKey = 0;

        if ((u32Reg & 0x20) == 0)
            u32MouseKey |= 1; /* Left key */

        if ((u32Reg & 0x8) == 0)
            u32MouseKey |= 2; /* Right key */

        /* Update new report data */
        u8Buf[0] = u32MouseKey;
        u8Buf[1] = (uint8_t)i32x >> 2;
        u8Buf[2] = (uint8_t)i32y >> 2;
        u8Buf[3] = 0x00;

        if (i32x | i32y | (u32MousePreKey != u32MouseKey))
        {
            u32MousePreKey = u32MouseKey;
            g_u8EP2Ready = 0;
            /* Set transfer length and trigger IN transfer */
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), u8Buf, 4);
            USBD_SET_PAYLOAD_LEN(EP2, 4);
        }
    }
}
