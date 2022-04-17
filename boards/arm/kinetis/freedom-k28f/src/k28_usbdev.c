/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/src/k28_usbdev.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "kinetis.h"
#include "kinetis_usbotg.h"
#include "hardware/kinetis_sim.h"
#include "freedom-k28f.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define khci_getreg(addr)      getreg8(addr)
#define khci_putreg(val,addr)  putreg8(val,addr)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k28_usbdev_initialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the KwikStik-K40 board.
 *
 ****************************************************************************/

void k28_usbdev_initialize(void)
{
}

/****************************************************************************
 * Name:  kinetis_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB
 *   software connect and disconnect), then the board software must provide
 *   kinetis_pullup. See include/nuttx/usb/usbdev.h for additional
 *   description of this method.
 *   Alternatively, if no pull-up GPIO the following EXTERN can be redefined
 *   to be NULL.
 *
 ****************************************************************************/

int kinetis_usbpullup(struct usbdev_s *dev, bool enable)
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
      khci_putreg(0, KINETIS_USB0_CONTROL);
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

  khci_putreg(regval, KINETIS_USB0_OTGCTL);
#endif

  return OK;
}

/****************************************************************************
 * Name:  kinetis_usbsuspend
 *
 * Description:
 *   Board logic must provide the kinetis_usbsuspend logic if the USBDEV
 *   driver is used.
 *   This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power,
 *   etc. while the USB is suspended.
 *
 ****************************************************************************/

void kinetis_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
#warning "Missing logic"
}
