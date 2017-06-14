/****************************************************************************
 * config/freedom-k66f/src/k66_bringup.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#include <sys/types.h>
#include <sys/mount.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/board.h>

#include <nuttx/spi/spi.h>
#include <nuttx/input/buttons.h>

#include "kinetis_spi.h"
#include "freedom-k66f.h"

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_INITIALIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k66_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int k66_bringup(void)
{
#ifdef HAVE_SPI
  FAR struct spi_dev_s *spi1;
#endif
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

#ifdef HAVE_MMCSD
  /* Initialize the SDHC driver */

  ret = k66_sdhc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k66_sdhc_initialize() failed: %d\n", ret);
    }

#  ifdef CONFIG_FRDMK66F_SDHC_MOUNT
  else
    {
      /* Mount the volume on HSMCI0 */

      ret = mount(CONFIG_FRDMK66F_SDHC_MOUNT_BLKDEV,
                  CONFIG_FRDMK66F_SDHC_MOUNT_MOUNTPOINT,
                  CONFIG_FRDMK66F_SDHC_MOUNT_FSTYPE,
                  0, NULL);

      if (ret < 0)
        {
          syslog(LOG_ERR,"ERROR: Failed to mount %s: %d\n",
                 CONFIG_FRDMK66F_SDHC_MOUNT_MOUNTPOINT, errno);
        }
    }

#  endif /* CONFIG_FRDMK66F_SDHC_MOUNT */
#endif /* HAVE_MMCSD */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = k66_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k66_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  k66_automount_initialize();
#endif

#ifdef HAVE_RTC_DRIVER
  /* Initialize the RTC */

  ret = k66_rtc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initalize the RTC driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_SPI

  /* Verify we can initialize SPI bus 1 */

  spi1 = kinetis_spibus_initialize(1);

  if (!spi1)
    {
      syslog(LOG_ERR, "ERROR:FAILED to initialize SPI port 1\n");
      return -ENODEV;
    }
#endif

  UNUSED(ret);
  return OK;
}
#endif /* CONFIG_LIB_BOARDCTL CONFIG_BOARD_INITIALIZE */
