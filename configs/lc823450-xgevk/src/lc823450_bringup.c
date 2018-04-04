/****************************************************************************
 * configs/lc823450-xgevk/src/lc823450_bringup.c
 *
 *   Copyright 2017,2018 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#include <stdbool.h>
#include <syslog.h>

#ifdef CONFIG_SMP
#  include <sched.h>
#endif

#include <nuttx/sched.h>

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_WATCHDOG
#  include "lc823450_wdt.h"
#endif

#ifdef CONFIG_DVFS
#  include "lc823450_dvfs2.h"
#endif

#include "lc823450-xgevk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int lc823450_bringup(void)
{
  int ret;

#ifdef CONFIG_WATCHDOG
  lc823450_wdt_initialize();
#endif

#ifdef CONFIG_FS_PROCFS

#ifdef CONFIG_DVFS
  (void)dvfs_procfs_register();
#endif

  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_FAT
  ret = mount("/dev/mtdblock0p10", "/mnt/sd0", "vfat", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount vfat at /mnt/sd0: %d\n", ret);
    }
#endif

#ifdef CONFIG_BMA250
  lc823450_bma250initialize("/dev/accel");
#endif

#ifdef CONFIG_AUDIO_WM8776
  lc823450_wm8776initialize(0);
#endif

#if defined(CONFIG_RNDIS) && defined(CONFIG_NSH_MACADDR)
  uint8_t mac[6];
  mac[0] = 0xaa; /* TODO */
  mac[1] = (CONFIG_NSH_MACADDR >> (8 * 4)) & 0xff;
  mac[2] = (CONFIG_NSH_MACADDR >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NSH_MACADDR >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NSH_MACADDR >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NSH_MACADDR >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#if defined(CONFIG_SMP) && defined (CONFIG_RNDIS)
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(1, &cpuset); /* assigned to CPU1 */

  /* NOTE: pid=4 is assumed to be lpwork */

  (void)nxsched_setaffinity(4, sizeof(cpu_set_t), &cpuset);
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
