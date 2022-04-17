/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/src/k28_usbhshost.c
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
#include <stdio.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/ehci.h>

#include <sys/mount.h>

#include <kinetis_usbhshost.h>

#include "arm_internal.h"
#include "hardware/kinetis_k28pinmux.h"
#include "hardware/kinetis_mcg.h"
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_osc.h"
#include "hardware/kinetis_usbhs.h"
#include "freedom-k28f.h"

#include <arch/board/board.h>  /* Must always be included last */

#if defined(CONFIG_KINETIS_USBHS) && defined(CONFIG_USBHOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ehci_waiter(int argc, char *argv[]);
static void ehci_hwinit(void);

#  ifdef HAVE_USB_AUTOMOUNTER
static void usb_msc_connect(void *arg);
static void unmount_retry_timeout(wdparm_t arg);
static void usb_msc_disconnect(void *arg);
#  endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Retained device driver handle */

static struct usbhost_connection_s *g_ehciconn;

#  ifdef HAVE_USB_AUTOMOUNTER
/* Unmount retry timer */

static struct wdog_s g_umount_tmr[CONFIG_FRDMK28F_USB_AUTOMOUNT_NUM_BLKDEV];
#  endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ehci_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to the EHCI root hub.
 *
 ****************************************************************************/

static int ehci_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

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

/****************************************************************************
 * Name: ehci_hw_init
 *
 * Description:
 *   Initialize PHY and clocks for EHCI
 *
 ****************************************************************************/

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
  regval &= ~(USBPHY_CTRLN_SFTRST);
  regval &= ~(USBPHY_CTRLN_CLKGATE);
  putreg32(regval, KINETIS_USBHSPHY_CTRL);

  /* Set PHY PLL Clock */

  regval  = getreg32(KINETIS_USBHSPHY_PLL_SIC);
  regval &= ~(USBPHY_PLL_SICN_PLL_BYPASS);
  regval &= ~(USBPHY_PLL_SICN_PLL_DIV_SEL_MASK);
  regval |= (USBPHY_PLL_SICN_PLL_POWER);
  regval |= (USBPHY_PLL_SICN_PLL_EN_USB_CLKS);
#  if (BOARD_EXTAL_FREQ == 24000000)
  regval |= USBPHY_PLL_SICN_PLL_DIV_SEL_24MHZ;
#  elif (BOARD_EXTAL_FREQ == 16000000)
  regval |= USBPHY_PLL_SICN_PLL_DIV_SEL_16MHZ;
#  elif (BOARD_EXTAL_FREQ == 12000000)
  regval |= USBPHY_PLL_SICN_PLL_DIV_SEL_12MHZ;
#  else
#    warning Not supported.
#  endif
  putreg32(regval, KINETIS_USBHSPHY_PLL_SIC);

  regval  = getreg32(KINETIS_USBHSPHY_TRIM_OVERRIDE_EN);
  regval |= (USBPHY_TRIM_OVERRIDE_ENN_TRIM_DIV_SEL_OVERRIDE);
  putreg32(regval, KINETIS_USBHSPHY_TRIM_OVERRIDE_EN);

  do
    {
      regval = getreg32(KINETIS_USBHSPHY_PLL_SIC);
    }
  while (!(regval & USBPHY_PLL_SICN_PLL_LOCK));

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
  regval |= USBPHY_CTRLN_ENUTMILEVEL2;
  regval |= USBPHY_CTRLN_ENUTMILEVEL3;
  putreg32(regval, KINETIS_USBHSPHY_CTRL);
}

#  ifdef HAVE_USB_AUTOMOUNTER
/****************************************************************************
 * Name: usb_msc_connect
 *
 * Description:
 *   Mount the USB mass storage device
 *
 ****************************************************************************/

static void usb_msc_connect(void *arg)
{
  int  index  = (int)arg;
  char sdchar = 'a' + index;
  int  ret;

  char blkdev[32];
  char mntpnt[32];

  DEBUGASSERT(index >= 0 &&
              index < CONFIG_FRDMK28F_USB_AUTOMOUNT_NUM_BLKDEV);

  wd_cancel(&g_umount_tmr[index]);

  /* Resetup the event. */

  usbhost_msc_notifier_setup(usb_msc_connect, WORK_USB_MSC_CONNECT,
      sdchar, arg);

  snprintf(blkdev, sizeof(blkdev), "%s%c",
      CONFIG_FRDMK28F_USB_AUTOMOUNT_BLKDEV, sdchar);
  snprintf(mntpnt, sizeof(mntpnt), "%s%c",
      CONFIG_FRDMK28F_USB_AUTOMOUNT_MOUNTPOINT, sdchar);

  /* Mount */

  ret = nx_mount((const char *)blkdev, (const char *)mntpnt,
      CONFIG_FRDMK28F_USB_AUTOMOUNT_FSTYPE, 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Mount failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: unmount_retry_timeout
 *
 * Description:
 *   A previous unmount failed because the volume was busy... busy meaning
 *   the volume could not be unmounted because there are open references
 *   the files or directories in the volume.  When this failure occurred,
 *   the unmount logic setup a delay and this function is called as a result
 *   of that delay timeout.
 *
 *   This function will attempt the unmount again.
 *
 * Input Parameters:
 *   Standard wdog timeout parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void unmount_retry_timeout(wdparm_t arg)
{
  int  index  = arg;
  char sdchar = 'a' + index;

  finfo("Timeout!\n");
  DEBUGASSERT(index >= 0 &&
              index < CONFIG_FRDMK28F_USB_AUTOMOUNT_NUM_BLKDEV);

  /* Resend the notification. */

  usbhost_msc_notifier_signal(WORK_USB_MSC_DISCONNECT, sdchar);
}

/****************************************************************************
 * Name: usb_msc_disconnect
 *
 * Description:
 *   Unmount the USB mass storage device
 *
 ****************************************************************************/

static void usb_msc_disconnect(void *arg)
{
  int  index  = (int)arg;
  char sdchar = 'a' + index;
  int  ret;

  char mntpnt[32];

  DEBUGASSERT(index >= 0 &&
              index < CONFIG_FRDMK28F_USB_AUTOMOUNT_NUM_BLKDEV);

  wd_cancel(&g_umount_tmr[index]);

  /* Resetup the event. */

  usbhost_msc_notifier_setup(usb_msc_disconnect, WORK_USB_MSC_DISCONNECT,
      sdchar, arg);

  snprintf(mntpnt, sizeof(mntpnt), "%s%c",
      CONFIG_FRDMK28F_USB_AUTOMOUNT_MOUNTPOINT, sdchar);

  /* Unmount */

  ret = nx_umount2((const char *)mntpnt, MNT_FORCE);
  if (ret < 0)
    {
      /* We expect the error to be EBUSY meaning that the volume could
       * not be unmounted because there are currently reference via open
       * files or directories.
       */

      if (ret == -EBUSY)
        {
          finfo("WARNING: Volume is busy, try again later\n");

          /* Start a timer to retry the umount2 after a delay */

          ret = wd_start(&g_umount_tmr[index],
                          MSEC2TICK(CONFIG_FRDMK28F_USB_AUTOMOUNT_UDELAY),
                          unmount_retry_timeout, index);
          if (ret < 0)
            {
              ferr("ERROR: wd_start failed: %d\n", ret);
            }
        }

      /* Other errors are fatal */

      else
        {
          ferr("ERROR: Unmount failed: %d\n", errcode);
        }
    }
}
#  endif /* HAVE_USB_AUTOMOUNTER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k28_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

int k28_usbhost_initialize(void)
{
  int      ret;
#  ifdef HAVE_USB_AUTOMOUNTER
  int      index;
#  endif

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

#  ifdef HAVE_USB_AUTOMOUNTER
  /* Initialize the notifier listener for automount */

  for (index = 0; index < CONFIG_FRDMK28F_USB_AUTOMOUNT_NUM_BLKDEV; index++)
    {
      char sdchar = 'a' + index;

      usbhost_msc_notifier_setup(usb_msc_connect,
          WORK_USB_MSC_CONNECT, sdchar, (void *)(intptr_t)index);
      usbhost_msc_notifier_setup(usb_msc_disconnect,
          WORK_USB_MSC_DISCONNECT, sdchar, (void *)(intptr_t)index);
    }
#  endif

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

  ret = kthread_create("EHCI Monitor", CONFIG_USBHOST_DEFPRIO,
                       CONFIG_USBHOST_STACKSIZE,
                       (main_t)ehci_waiter, (char * const *)NULL);
  if (ret < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
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
 ****************************************************************************/

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
