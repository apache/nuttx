/****************************************************************************
 * boards/arm/samv7/pic32czca70-curiosity/src/sam_sdcard.c
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

#include <sys/mount.h>

#include <arch/board/board.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include "sam_board.h"

#ifdef HAVE_HSMCI
#  include "board_hsmci.h"
#endif

#ifdef HAVE_AUTOMOUNTER
#  include "sam_automount.h"
#endif

#ifdef CONFIG_SAMV7_HSMCI0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_HSMCI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdcard_initialize
 *
 * Description:
 *  Initialize SD Card (HSMCI driver) and performs mount if selected.
 *
 ****************************************************************************/

int sam_sdcard_initialize(void)
{
  int ret;

  /* Initialize the HSMCI0 driver */

  ret = sam_hsmci_initialize(
      HSMCI0_SLOTNO, HSMCI0_MINOR, GPIO_HSMCI0_CD, 0, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
             HSMCI0_SLOTNO, HSMCI0_MINOR, ret);
      return ret;
    }

#ifdef CONFIG_SAMV7_HSMCI0_MOUNT
  else
    {
      if (sam_cardinserted(HSMCI0_SLOTNO))
        {
          usleep(1000 * 1000);

          /* Mount the volume on HSMCI0 */

          ret = nx_mount(CONFIG_SAMV7_HSMCI0_MOUNT_BLKDEV,
                         CONFIG_SAMV7_HSMCI0_MOUNT_MOUNTPOINT,
                         CONFIG_SAMV7_HSMCI0_MOUNT_FSTYPE,
                         0, NULL);

          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
                     CONFIG_SAMV7_HSMCI0_MOUNT_MOUNTPOINT, ret);
            }
        }
    }
#endif /* CONFIG_SAMV7_HSMCI0_MOUNT */

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  sam_automount_initialize();
#endif

  return ret;
}

#endif /* HAVE_HSMCI */
#endif /* CONFIG_SAMV7_HSMCI0 */

