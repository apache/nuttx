/****************************************************************************
 * config/freedom-k64f/src/k64_bringup.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#ifdef CONFIG_KINETIS_SDHC
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#endif

#include "kinetis.h"
#include "freedom-k64f.h"

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_INITIALIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure encapsulates the global variable used in this file and
 * reduces the probability of name collistions.
 */

#ifdef NSH_HAVEMMCSD
struct k64_nsh_s
{
  FAR struct sdio_dev_s *sdhc; /* SDIO driver handle */
  bool inserted;               /* True: card is inserted */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef NSH_HAVEMMCSD
static struct k64_nsh_s g_nsh;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k64_mediachange
 ****************************************************************************/

#ifdef NSH_HAVEMMCSD
static void k64_mediachange(void)
{
  bool inserted;

  /* Get the current value of the card detect pin.  This pin is pulled up on
   * board.  So low means that a card is present.
   */

  inserted = !kinetis_gpioread(GPIO_SD_CARDDETECT);

  /* Has the pin changed state? */

  if (inserted != g_nsh.inserted)
    {
      /* Yes.. perform the appropriate action (this might need some debounce). */

      g_nsh.inserted = inserted;
      sdhc_mediachange(g_nsh.sdhc, inserted);

      /* If the card has been inserted, then check if it is write protected
       * as well.  The pin is pulled up, but apparently logic high means
       * write protected.
       */

      if (inserted)
        {
          sdhc_wrprotect(g_nsh.sdhc, kinetis_gpioread(GPIO_SD_WRPROTECT));
        }
    }
}
#endif

/****************************************************************************
 * Name: k64_cdinterrupt
 ****************************************************************************/

#ifdef NSH_HAVEMMCSD
static int k64_cdinterrupt(int irq, FAR void *context)
{
  /* All of the work is done by k64_mediachange() */

  k64_mediachange();
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: k64_bringup
 *
 * Description:
 *   Bring up board features
 *
 ************************************************************************************/

int k64_bringup(void)
{
  int ret;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = mount(NULL, PROCFS_MOUNTPOUNT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

#ifdef NSH_HAVEMMCSD
  /* Configure GPIO pins */

  /* Attached the card detect interrupt (but don't enable it yet) */

  kinetis_pinconfig(GPIO_SD_CARDDETECT);
  kinetis_pinirqattach(GPIO_SD_CARDDETECT, k64_cdinterrupt);

  /* Configure the write protect GPIO */

  //kinetis_pinconfig(GPIO_SD_WRPROTECT);

  /* Mount the SDHC-based MMC/SD block driver */
  /* First, get an instance of the SDHC interface */

  syslog(LOG_INFO, "Initializing SDHC slot %d\n", MMCSD_SLOTNO);

  g_nsh.sdhc = sdhc_initialize(MMCSD_SLOTNO);
  if (!g_nsh.sdhc)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDHC slot %d\n",
             MMCSD_SLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDHC interface to the MMC/SD driver */

  syslog(LOG_INFO, "Bind SDHC to the MMC/SD driver, minor=%d\n",
         CONFIG_NSH_MMCSDMINOR);

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_nsh.sdhc);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SDHC to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfully bound SDHC to the MMC/SD driver\n");

  /* Handle the initial card state */

  k64_mediachange();

  /* Enable CD interrupts to handle subsequent media changes */

  kinetis_pinirqenable(GPIO_SD_CARDDETECT);
#endif

  UNUSED(ret);
  return OK;
}

#endif /* CONFIG_LIB_BOARDCTL CONFIG_BOARD_INITIALIZE */
