/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/src/k28_bringup.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "freedom-k28f.h"

#ifdef CONFIG_PL2303
#  include <nuttx/usb/pl2303.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k28_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int k28_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = k28_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k28_pwm_setup() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_KINETIS_I2C0) || defined(CONFIG_KINETIS_I2C1)
  /* Initialize I2C buses */

  k28_i2cdev_initialize();
#endif

#ifdef HAVE_MMCSD
  /* Initialize the SDHC driver */

  ret = k28_sdhc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k28_sdhc_initialize() failed: %d\n", ret);
    }

#ifdef CONFIG_FRDMK28F_SDHC_MOUNT
  else
    {
      /* Mount the volume on HSMCI0 */

      ret = mount(CONFIG_FRDMK28F_SDHC_MOUNT_BLKDEV,
                  CONFIG_FRDMK28F_SDHC_MOUNT_MOUNTPOINT,
                  CONFIG_FRDMK28F_SDHC_MOUNT_FSTYPE,
                  0, NULL);

      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
                 CONFIG_FRDMK28F_SDHC_MOUNT_MOUNTPOINT, errno);
        }
    }

#endif /* CONFIG_FRDMK28F_SDHC_MOUNT */
#endif /* HAVE_MMCSD */

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  k28_automount_initialize();
#endif

#if defined(CONFIG_USBDEV) && defined(CONFIG_KINETIS_USBOTG)
  if (k28_usbdev_initialize)
    {
      k28_usbdev_initialize();
    }
#endif

#ifdef CONFIG_PL2303
  usbdev_serialinitialize(0);
#endif

  UNUSED(ret);
  return OK;
}
