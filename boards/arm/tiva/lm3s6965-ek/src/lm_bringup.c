/****************************************************************************
 * boards/arm/tiva/lm3s6965-ek/src/lm_bringup.c
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

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include <debug.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "lm3s6965-ek.h"
#include "tiva_ssi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define NSH_HAVEMMCSD  1

/* Can't support MMC/SD features if mountpoints or MMC/SPI are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_MMCSD_SPI)
#  undef NSH_HAVEMMCSD
#endif

/* PORT and SLOT number depend on the board configuration */

#ifdef CONFIG_NSH_ARCHINIT
#  if !defined(CONFIG_NSH_MMCSDSPIPORTNO) || CONFIG_NSH_MMCSDSPIPORTNO != 0
#    error "The LM3S6965 Eval Kit MMC/SD is on SSI0"
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 0
#  endif
#  if !defined(CONFIG_NSH_MMCSDSLOTNO) || CONFIG_NSH_MMCSDSLOTNO != 0
#    error "The LM3S6965 Eval Kit MMC/SD is on SSI0 slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif
#else
#  undef  CONFIG_NSH_MMCSDSPIPORTNO
#  define CONFIG_NSH_MMCSDSPIPORTNO 0
#  undef  CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#  undef  CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int lm_bringup(void)
{
  int ret = 0;

#ifdef NSH_HAVEMMCSD
  struct spi_dev_s *spi;

  /* Get the SPI port */

  mcinfo("Initializing SPI port %d\n", CONFIG_NSH_MMCSDSPIPORTNO);

  spi = tiva_ssibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!spi)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  mcinfo("Successfully initialized SPI port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  mcinfo("Binding SPI port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                                CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO, ret);
      return ret;
    }

  mcinfo("Successfully bound SPI port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);
#endif

#ifdef CONFIG_FS_BINFS
  /* Mount the binfs file system */

  ret = nx_mount(NULL, "/bin", "binfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount binfs at /bin: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, LM_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             LM_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

  return ret;
}
