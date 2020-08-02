/*****************************************************************************
 * boards/arm/kinetis/freedom-k28f/src/k28_usbhshost.c
 *
 *   Copyright (C) 2013, 2015-2017 Gregory Nutt. All rights reserved.
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

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

#include <kinetis_usbhshost.h>

#include "arm_arch.h"
#include "hardware/kinetis_k28pinmux.h"
#include "hardware/kinetis_mcg.h"
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_osc.h"
#include "hardware/kinetis_usbhs.h"
#include "freedom-k28f.h"

#include <arch/board/board.h>  /* Must always be included last */

#if defined(CONFIG_KINETIS_USBHS) && defined(CONFIG_USBHOST)

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define BOARD_USB_PHY_D_CAL     (0x0CU)
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#define BOARD_USB_PHY_TXCAL45DM (0x06U)

#ifndef CONFIG_USBHOST_DEFPRIO
#  define CONFIG_USBHOST_DEFPRIO 50
#endif

#ifndef CONFIG_USBHOST_STACKSIZE
#  ifdef CONFIG_USBHOST_HUB
#    define CONFIG_USBHOST_STACKSIZE 1536
#  else
#    define CONFIG_USBHOST_STACKSIZE 1024
#  endif
#endif

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* Retained device driver handle */

static struct usbhost_connection_s *g_ehciconn;

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: ehci_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to the EHCI root hub.
 *
 *****************************************************************************/

static int ehci_waiter(int argc, char *argv[])
{
  FAR struct usbhost_hubport_s *hport;

  uinfo("ehci_waiter:  Running\n");
  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_ehciconn, &hport));
      syslog(LOG_INFO, "ehci_waiter: %s\n",
             hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(g_ehciconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/*****************************************************************************
 * Name: ehci_hw_init
 *
 * Description:
 *   Initialize PHY and clocks for EHCI
 *
 *****************************************************************************/

static void ehci_hwinit(void)
{
  uint32_t regval;
  uint8_t  regval8;

  /* Enable Internal Reference Clock */

  regval8  = getreg8(KINETIS_MCG_C1);
  regval8 |= (MCG_C1_IRCLKEN);
  putreg8(regval8, KINETIS_MCG_C1);

  /* Enable External Reference Clock */

  regval8  = getreg8(KINETIS_OSC_CR);
  regval8 |= (OSC_CR_ERCLKEN);
  putreg8(regval8, KINETIS_OSC_CR);

  /* Enable PLL Regulator */

  regval  = getreg32(KINETIS_SIM_SOPT2);
  regval |= (SIM_SOPT2_USBREGEN);
  putreg32(regval, KINETIS_SIM_SOPT2);

  /* Gate USB clock */

  regval  = getreg32(KINETIS_SIM_SCGC3);
  regval |= (SIM_SCGC3_USBHSPHY);
  putreg32(regval, KINETIS_SIM_SCGC3);

  /* Release Softreset and ungate PHY Clock */

  regval  = getreg32(KINETIS_USBHSPHY_CTRL);
  regval &= ~(USBPHY_CTRLn_SFTRST);
  regval &= ~(USBPHY_CTRLn_CLKGATE);
  putreg32(regval, KINETIS_USBHSPHY_CTRL);

  /* Set PHY PLL Clock */

  regval  = getreg32(KINETIS_USBHSPHY_PLL_SIC);
  regval &= ~(USBPHY_PLL_SICn_PLL_BYPASS);
  regval &= ~(USBPHY_PLL_SICn_PLL_DIV_SEL_MASK);
  regval |= (USBPHY_PLL_SICn_PLL_POWER);
  regval |= (USBPHY_PLL_SICn_PLL_EN_USB_CLKS);
#  if (BOARD_EXTAL_FREQ == 24000000)
  regval |= USBPHY_PLL_SICn_PLL_DIV_SEL_24MHZ;
#  elif (BOARD_EXTAL_FREQ == 16000000)
  regval |= USBPHY_PLL_SICn_PLL_DIV_SEL_16MHZ;
#  elif (BOARD_EXTAL_FREQ == 12000000)
  regval |= USBPHY_PLL_SICn_PLL_DIV_SEL_12MHZ;
#  else
#    warning Not supported.
#  endif
  putreg32(regval, KINETIS_USBHSPHY_PLL_SIC);

  regval  = getreg32(KINETIS_USBHSPHY_TRIM_OVERRIDE_EN);
  regval |= (USBPHY_TRIM_OVERRIDE_ENn_TRIM_DIV_SEL_OVERRIDE);
  putreg32(regval, KINETIS_USBHSPHY_TRIM_OVERRIDE_EN);

  do
    {
      regval = getreg32(KINETIS_USBHSPHY_PLL_SIC);
    }
  while (!(regval & USBPHY_PLL_SICn_PLL_LOCK));

  /* Enable USBHS Clock and Regulator */

  regval  = getreg32(KINETIS_SIM_SCGC3);
  regval |= (SIM_SCGC3_USBHS);
  putreg32(regval, KINETIS_SIM_SCGC3);

  regval  = getreg32(KINETIS_SIM_USBPHYCTL);
  regval &= ~(SIM_USBPHYCTL_USB3VOUTTRG_MASK);
  regval |= SIM_USBPHYCTL_USB3VOUTTRG_3V310;
  regval |= SIM_USBPHYCTL_USBVREGSEL;
  putreg32(regval, KINETIS_SIM_USBPHYCTL);

  /* Disable Powerdown */

  putreg32(0, KINETIS_USBHSPHY_PWD);

  /* Misc */

  regval  = getreg32(KINETIS_USBHSPHY_CTRL);
  regval |= USBPHY_CTRLn_ENUTMILEVEL2;
  regval |= USBPHY_CTRLn_ENUTMILEVEL3;
  putreg32(regval, KINETIS_USBHSPHY_CTRL);
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: k28_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 *****************************************************************************/

int k28_usbhost_initialize(void)
{
  pid_t    pid;
  int      ret;

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

  /* Enable PHY and clocks */

  ehci_hwinit();

  /* Then get an instance of the USB EHCI interface. */

  g_ehciconn = kinetis_ehci_initialize(0);

  if (!g_ehciconn)
    {
      uerr("ERROR: kinetis_ehci_initialize failed\n");
      return -ENODEV;
    }

  /* Start a thread to handle device connection. */

  pid = kthread_create("EHCI Monitor", CONFIG_USBHOST_DEFPRIO,
                       CONFIG_USBHOST_STACKSIZE,
                       (main_t)ehci_waiter, (FAR char * const *)NULL);
  if (pid < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  return OK;
}

/*****************************************************************************
 * Name: kinetis_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each platform that implements the OHCI or EHCI host
 *   interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.
 *            Since the KINETIS has only a downstream port, zero is
 *            the only possible value for this parameter.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

#define HCOR ((volatile struct ehci_hcor_s *)KINETIS_USBHS_HCOR_BASE)

void kinetis_usbhost_vbusdrive(int rhport, bool enable)
{
  uint32_t regval;

  uinfo("RHPort%d: enable=%d\n", rhport + 1, enable);

  /* The KINETIS has only a single root hub port */

  if (rhport == 0)
    {
      /* Then enable or disable VBUS power */

      regval = HCOR->portsc[rhport];
      regval &= ~EHCI_PORTSC_PP;
      if (enable)
        {
          regval |= EHCI_PORTSC_PP;
        }

      HCOR->portsc[rhport] = regval;
    }
}

/****************************************************************************
 * Name: kinetis_setup_overcurrent
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
int kinetis_setup_overcurrent(xcpt_t handler, void *arg)
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

#endif /* CONFIG_KINETIS_USBOTG || CONFIG_USBHOST */
