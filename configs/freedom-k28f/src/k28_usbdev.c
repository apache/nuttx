/************************************************************************************
 * configs/freedom-k28f/src/k28_usbdev.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "up_arch.h"
#include "kinetis.h"
#include "kinetis_usbotg.h"
#include "chip/kinetis_sim.h"
#include "freedom-k28f.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define khci_getreg(addr)      getreg8(addr)
#define khci_putreg(val,addr)  putreg8(val,addr)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: k28_usbdev_initialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the KwikStik-K40 board.
 *
 ************************************************************************************/

void k28_usbdev_initialize(void)
{
}

/************************************************************************************
 * Name:  kinetis_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB software
 *   connect and disconnect), then the board software must provide kinetis_pullup.
 *   See include/nuttx/usb/usbdev.h for additional description of this method.
 *   Alternatively, if no pull-up GPIO the following EXTERN can be redefined to be
 *   NULL.
 *
 ************************************************************************************/

int kinetis_usbpullup(FAR struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);
#if 0
  uint32_t regval;
#endif

  if (enable)
    {
      khci_putreg(USB_CONTROL_DPPULLUPNONOTG, KINETIS_USB0_CONTROL);
    }
  else
    {
      khci_putreg(0,KINETIS_USB0_CONTROL);
    }

#if 0
  regval = khci_getreg(KINETIS_USB0_OTGCTL);

  if (enable)
    {
      regval |= (1 << 2);
    }
  else
    {
      regval &= ~(1 << 2);
    }

  khci_putreg(regval,KINETIS_USB0_OTGCTL);
#endif

  return OK;
}

/************************************************************************************
 * Name:  kinetis_usbsuspend
 *
 * Description:
 *   Board logic must provide the kinetis_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

void kinetis_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
#warning "Missing logic"
}
