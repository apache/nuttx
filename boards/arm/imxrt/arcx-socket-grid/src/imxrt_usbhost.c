/****************************************************************************
 * boards/arm/imxrt/arcx-socket-grid/src/imxrt_usbhost.c
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
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/ehci.h>

#include <imxrt_ehci.h>

#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_usbotg.h"
#include "hardware/imxrt_usbphy.h"
#include "imxrt_periphclks.h"
#include "arcx-socket-grid.h"
#include "arm_internal.h"

#include <arch/board/board.h>  /* Must always be included last */

#if defined(CONFIG_IMXRT_USBOTG) || defined(CONFIG_USBHOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

int imxrt_usbhost_initialize(int ctrid)
{
  struct usbhost_connection_s *ehciconn_usb1 = NULL;
  struct usbhost_connection_s *ehciconn_usb2 = NULL;
  uint32_t regval;
  static bool usb1_initialized = false;
  static bool usb2_initialized = false;
  int ret;

  /* Check if each USB was initialized already */

  if (ctrid == 0 && usb1_initialized)
    {
      return OK;
    }

  if (ctrid == 1 && usb2_initialized)
    {
      return OK;
    }

  imxrt_clockall_usboh3();

  /* Leave from reset */

  regval = getreg32(IMXRT_USBPHY_CTRL(ctrid));
  regval &= ~USBPHY_CTRL_SFTRST;
  putreg32(regval, IMXRT_USBPHY_CTRL(ctrid));

  /* Enable the clock */

  regval = getreg32(IMXRT_USBPHY_CTRL(ctrid));
  regval &= ~USBPHY_CTRL_CLKGATE;
  putreg32(regval, IMXRT_USBPHY_CTRL(ctrid));

  /* Power up the PHY */

  putreg32(0, IMXRT_USBPHY_PWD(ctrid));

  /* Enable PHY Negotiation */

  regval = getreg32(IMXRT_USBPHY_CTRL(ctrid));
  regval |= USBPHY_CTRL_ENAUTOCLR_PHY_PWD | USBPHY_CTRL_ENAUTOCLR_CLKGATE |
            USBPHY_CTRL_ENUTMILEVEL2 | USBPHY_CTRL_ENUTMILEVEL3;
  putreg32(regval, IMXRT_USBPHY_CTRL(ctrid));

  /* Setup pins, with power initially off */

  imxrt_config_gpio(GPIO_USBOTG1_ID);

  /* Enable USB HOST POWER */

  imxrt_config_gpio(USB_HOST_PWR);
  imxrt_gpio_write(USB_HOST_PWR, true);

  /* First, register all of the class drivers needed to support the drivers
   * that we care about
   */

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB host Mass Storage Class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the CDC/ACM serial class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Register the USB host HID keyboard class driver */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the KBD class\n");
    }
#endif

#ifdef CONFIG_IMXRT_USBOTG1
  /* Then get an instance of the USB EHCI interface. */

  ehciconn_usb1 = imxrt_ehci_initialize(ctrid);
  if (!ehciconn_usb1)
    {
      uerr("ERROR: imxrt_ehci_initialize failed\n");
      return -ENODEV;
    }

  /* Initialize waiter */

  ret = usbhost_waiter_initialize(ehciconn_usb1);
  if (ret < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  usb1_initialized = true;
#endif

#ifdef CONFIG_IMXRT_USBOTG2
  /* Then get an instance of the USB EHCI interface. */

  ehciconn_usb2 = imxrt_ehci_initialize(ctrid);
  if (!ehciconn_usb2)
    {
      uerr("ERROR: imxrt_ehci_initialize failed\n");
      return -ENODEV;
    }

  /* Initialize waiter */

  ret = usbhost_waiter_initialize(ehciconn_usb2);
  if (ret < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  usb2_initialized = true;
#endif

  return OK;
}

/****************************************************************************
 * Name: imxrt_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each platform that implements the OHCI or EHCI host
 *   interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.
 *            Since the IMXRT has only a downstream port, zero is
 *            the only possible value for this parameter.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define HCOR(n) ((volatile struct ehci_hcor_s *)IMXRT_USBOTG_HCOR_BASE(n))

void imxrt_usbhost_vbusdrive(int ctrid, int rhport, bool enable)
{
  uint32_t regval;

  uinfo("RHPort%d: enable=%d\n", rhport + 1, enable);

  /* The IMXRT has two root hub ports */

  if (rhport == 0 || rhport == 1)
    {
      /* Then enable or disable VBUS power */

      regval = HCOR(rhport)->portsc[ctrid];
      regval &= ~EHCI_PORTSC_PP;
      if (enable)
        {
          regval |= EHCI_PORTSC_PP;
        }

      HCOR(rhport)->portsc[ctrid] = regval;
    }
}

/****************************************************************************
 * Name: imxrt_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent condition
 *   is detected.
 *
 * Input Parameters:
 *   handler - New overcurrent interrupt handler
 *   arg     - The argument that will accompany the interrupt
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#if 0 /* Not ready yet */
int imxrt_setup_overcurrent(xcpt_t handler, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Configure the interrupt */

#warning Missing logic

  leave_critical_section(flags);
  return OK;
}
#endif /* 0 */

#endif /* CONFIG_IMXRT_USBOTG || CONFIG_USBHOST */
