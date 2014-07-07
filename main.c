//*****************************************************************************
//! @file       lcd_example.c
//! @brief      LCD access example for CC2538 on SmartRF06EB. The SmartRF06EB
//!             LCD display is a DOGM128W-6 128x64 dot matrix.
//!
//!             In this example, use the directional keys (LEFT, RIGHT) on
//!             SmartRF06EB to switch between two different display buffers.
//!             Use the UP key to invert the bits of the displayed buffer.
//!
//!             @Warning This example will not work if \c LCD_NO_DEFAULT_BUFFER
//!             is defined!
//!
//! Revised     $Date: 2013-04-11 19:50:45 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9709 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "bsp_key.h"
#include "lcd_dogm128_6.h"
#include "sys_ctrl.h"           // Access to driverlib SysCtrl fns
#include "interrupt.h"          // Access to driverlib interrupt fns

#include "bsp.h"
#include "string.h"
#include "usb_firmware_library_headers.h"
#include "usb_cdc.h"
#include "usb_in_buffer.h"
#include "usb_out_buffer.h"

/******************************************************************************
* DEFINES
*/


/******************************************************************************
* LOCAL VARIABLES AND FUNCTIONS
*/
USB_EPIN_RINGBUFFER_DATA usbCdcInBufferData;
USB_EPOUT_RINGBUFFER_DATA usbCdcOutBufferData;
static uint8_t pInBuffer[128];
static uint8_t pOutBuffer[128];
static uint8_t pAppBuffer[128];

/******************************************************************************
* FUNCTIONS
*/

void usbsuspHookEnteringSuspend(bool remoteWakeupAllowed) {
    if (remoteWakeupAllowed) {
    }
}


void usbsuspHookExitingSuspend(void) {
}

static void initUSB(void)
{
    //
    // Initialize buffers
    //
    memset(&usbCdcInBufferData, 0x00, sizeof(USB_EPIN_RINGBUFFER_DATA));
    usbCdcInBufferData.pBuffer = pInBuffer;
    usbCdcInBufferData.size = sizeof(pInBuffer);
    usbCdcInBufferData.endpointReg = USB_F4;
    usbCdcInBufferData.endpointIndex = 4;
    usbCdcInBufferData.endpointSize = 64;
    memset(&usbCdcOutBufferData, 0x00, sizeof(USB_EPOUT_RINGBUFFER_DATA));
    usbCdcOutBufferData.pBuffer = pOutBuffer;
    usbCdcOutBufferData.size = sizeof(pOutBuffer);
    usbCdcOutBufferData.endpointReg = USB_F4;
    usbCdcOutBufferData.endpointIndex = 4;
    //
    // Enable the USB interface
    //
    usbCdcInit(76800);
}
/**************************************************************************//**
* @brief    Main function of example.
******************************************************************************/

// https://lufa-lib.googlecode.com/svn/trunk/Demos/Device/LowLevel/VirtualSerial/LUFA%20VirtualSerial.inf
// https://github.com/contiki-os/contiki/tree/master/platform/cc2538dk

int main(void)
{
    //
    // Initialize board
    //
    bspInit(BSP_SYS_CLK_SPD);

    initUSB();

    //
    // Initialize SPI interface
    //
    bspSpiInit(BSP_SPI_CLK_SPD);

    //
    // Initialize keys driver
    //
    bspKeyInit(BSP_KEY_MODE_ISR);

    //
    // Initialize LCD
    //
    lcdInit();

    //
    // Clear local LCD buffers
    //
    lcdBufferClear(0);

    //
    // Write default buffer
    //
    lcdBufferPrintString(0, "USB DEMO", 1, eLcdPage0);
    lcdSendBuffer(0);

    //
    // Enable interrupts on needed keys
    //
    bspKeyIntEnable(BSP_KEY_LEFT|BSP_KEY_RIGHT|BSP_KEY_UP|BSP_KEY_DOWN);

    //
    // Enable global interrupts
    //
    IntMasterEnable();

    while(1)
    {
        //
        // Process USB events
        //
        usbCdcProcessEvents();

        //
        // Implement COM-port loopback
        //
        uint16_t count = usbibufGetMaxPushCount(&usbCdcInBufferData);
        uint16_t maxPopCount = usbobufGetMaxPopCount(&usbCdcOutBufferData);

        if (count > maxPopCount)
        {
            count = maxPopCount;
        }
        if (count)
        {
            usbobufPop(&usbCdcOutBufferData, pAppBuffer, count);
            usbibufPush(&usbCdcInBufferData, pAppBuffer, count);
        }
    }

    return 0;
}
