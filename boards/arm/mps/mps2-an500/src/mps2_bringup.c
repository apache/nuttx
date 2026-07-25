/****************************************************************************
 * boards/arm/mps/mps2-an500/src/mps2_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <syslog.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_RAMMTD
#  include <nuttx/drivers/drivers.h>
#  include <nuttx/kmalloc.h>
#  include <nuttx/mtd/mtd.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RAMMTD
#  define MPS2_RAMMTD_SIZE (256 * 1024)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mps2_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

static int mps2_bringup(void)
{
  int ret = 0;

#ifdef CONFIG_FS_PROCFS

  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }

#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmp file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at /tmp: %d\n", ret);
    }
#endif

#ifdef CONFIG_RAMMTD
  /* A RAM backed MTD device standing in for memory mapped NOR.  rammtd
   * answers BIOC_XIPBASE with the base of its buffer, so xipfs layered on it
   * can hand out real, directly executable pointers -- which is what lets a
   * module be executed in place rather than copied.  This gives the xipfs
   * test suite a target with no real flash.
   */

    {
      FAR uint8_t *ramstart = kmm_malloc(MPS2_RAMMTD_SIZE);
      FAR struct mtd_dev_s *mtd;

      if (ramstart == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to allocate RAM MTD\n");
        }
      else if ((mtd = rammtd_initialize(ramstart, MPS2_RAMMTD_SIZE)) == NULL)
        {
          syslog(LOG_ERR, "ERROR: rammtd_initialize failed\n");
          kmm_free(ramstart);
        }
      else
        {
          mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

          ret = register_mtddriver("/dev/rammtd", mtd, 0755, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: register_mtddriver failed: %d\n", ret);
            }

#ifdef CONFIG_FS_XIPFS
          else
            {
              ret = nx_mount("/dev/rammtd", "/mnt/xipfs", "xipfs", 0,
                             "autoformat");
              if (ret < 0)
                {
                  syslog(LOG_ERR,
                         "ERROR: Failed to mount xipfs at /mnt/xipfs: %d\n",
                         ret);
                }
            }
#endif
        }
    }
#endif

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_intitialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board initialization */

  mps2_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
