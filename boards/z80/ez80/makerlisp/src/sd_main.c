/****************************************************************************
 * boards/z80/ez80/makerlisp/src/sd_main.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <fcntl.h>
#include <syslog.h>
#include <hex2bin.h>
#include <assert.h>
#include <errno.h>

#include <arch/irq.h>

#include "makerlisp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MMCSD_BLOCKDEV   "/dev/mmcsd0"
#define MMCSD_MOUNTPT    "/mnt/sdcard"
#define MMCSD_HEXFILE    "/mnt/sdcard/nuttx.hex"

#define SRAM_START       0x040000
#define SRAM_SIZE        0x100000
#define SRAM_END         (SRAM_START + SRAM_SIZE)

#define SRAM_RESET       SRAM_START
#define SRAM_ENTRY       ((sram_entry_t)SRAM_START)

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

int sd_main(int argc, char *argv)
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

  ret = hex2mem(fd, (uint32_t)SRAM_START, (uint32_t)SRAM_END, 0);
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
