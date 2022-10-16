/****************************************************************************
 * boards/z80/ez80/z20x/src/sd_main.c
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

#include <sys/mount.h>
#include <unistd.h>
#include <fcntl.h>
#include <syslog.h>
#include <hex2bin.h>
#include <assert.h>
#include <errno.h>

#include <arch/irq.h>

#include "z20x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MMCSD_BLOCKDEV   "/dev/mmcsd0"
#define MMCSD_MOUNTPT    "/mnt/sdcard"
#define MMCSD_HEXFILE    "/mnt/sdcard/nuttx.hex"

#define SRAM_ENTRY       ((sram_entry_t)PROGSTART)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef CODE void (*sram_entry_t)(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sd_main
 *
 * Description:
 *   sd_main is a tiny program that runs in FLASH.  sd_main will
 *   configure SRAM and load an Intel HEX file into SRAM,
 *   and either start that program or wait for you to break in with the
 *   debugger.
 *
 ****************************************************************************/

int sd_main(int argc, char *argv[])
{
  int fd;
  int ret;

  /* SRAM was already initialized at boot time, so we are ready to load the
   * Intel HEX stream into SRAM.
   */

#ifndef CONFIG_BOARD_LATE_INITIALIZE
  /* Perform board-level initialization.  This should include registering
   * the MMC/SD block driver at /dev/mmcsd0.
   */

  DEBUGVERIFY(ez80_bringup());
#endif

  syslog(LOG_INFO, "Loading %s\n", MMCSD_HEXFILE);

  /* Mount the MMC/SD block drivers at /mnt/sdcard */

  ret = mount(MMCSD_BLOCKDEV, MMCSD_MOUNTPT, "vfat", 0, NULL);
  if (ret < 0)
    {
      int errcode = errno;
      syslog(LOG_ERR, "ERROR: Failed to mount filesystem at %s: %d\n",
             MMCSD_MOUNTPT, errcode);
      goto halt;
    }

  /* Open the file /mnt/sdcard/nuttx.hex */

  fd = open(MMCSD_HEXFILE, O_RDONLY);
  if (fd < 0)
    {
      int errcode = errno;
      syslog(LOG_ERR, "ERROR: Failed to mount filesystem at %s: %d\n",
             MMCSD_MOUNTPT, errcode);
      goto halt_with_mount;
    }

  /* Load the HEX image into memory */

  ret = hex2mem(fd, (uint32_t)PROGSTART, (uint32_t)PROGEND, 0);
  if (ret < 0)
    {
      /* We failed to load the HEX image */

      syslog(LOG_ERR, "ERROR: Intel HEX file load failed: %d\n", ret);
      goto halt_with_hexfile;
    }

  close(fd);
  umount(MMCSD_MOUNTPT);

  /* The reset vector should now be present at the beginning of SRAM.
   * This assumes that the image was loaded at 0x040000 and that the reset
   * vector is the first thing in memory.
   */

  syslog(LOG_INFO, "Starting at %p\n", SRAM_ENTRY);

  /* Interrupts must be disabled through the following. */

  up_irq_save();

  /* Then jump into SRAM via the reset vector at 0x040000 */

  SRAM_ENTRY();
  goto halt;

halt_with_hexfile:
  close(fd);
halt_with_mount:
  umount(MMCSD_MOUNTPT);
halt:
  for (; ; );
  return 0; /* Will not get here */
}
