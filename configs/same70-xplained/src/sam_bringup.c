/****************************************************************************
 * config/same70-xplained/src/sam_bringup.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SYSTEM_USBMONITOR
#  include <apps/usbmonitor.h>
#endif

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ramdisk.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/binfmt/elf.h>

#include "same70-xplained.h"

#ifdef HAVE_PROGMEM_CHARDEV
#  include <nuttx/mtd/mtd.h>
#  include "sam_progmem.h"
#endif

#ifdef HAVE_ROMFS
#  include <arch/board/boot_romfsimg.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS(n) \
  (((n)+CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_SECTSIZE-1) / \
   CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_SECTSIZE)

/* Debug ********************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
#  define SYSLOG lldbg
#else
#  define SYSLOG dbg
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void)
{
#ifdef HAVE_PROGMEM_CHARDEV
  FAR struct mtd_dev_s *mtd;
  char blockdev[18];
  char chardev[12];
#endif
  int ret;

#ifdef HAVE_MACADDR
  /* Read the Ethernet MAC address from the AT24 FLASH and configure the
   * Ethernet driver with that address.
    */

  ret = sam_emac0_setmac();
  if (ret < 0)
    {
      SYSLOG("ERROR: sam_emac0_setmac() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_MTDCONFIG
  /* Create an AT24xx-based MTD configuration device for storage device
   * configuration information.
   */

  ret = sam_at24config();
  if (ret < 0)
    {
      SYSLOG("ERROR: sam_at24config() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_HSMCI
  /* Initialize the HSMCI0 driver */

  ret = sam_hsmci_initialize(HSMCI0_SLOTNO, HSMCI0_MINOR);
  if (ret < 0)
    {
      SYSLOG("ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
             HSMCI0_SLOTNO, HSMCI0_MINOR, ret);
    }

#ifdef CONFIG_SAME70XPLAINED_HSMCI0_MOUNT
  else
    {
      /* REVISIT:  A delay seems to be required here or the mount will fail. */
      /* Mount the volume on HSMCI0 */

      ret = mount(CONFIG_SAME70XPLAINED_HSMCI0_MOUNT_BLKDEV,
                  CONFIG_SAME70XPLAINED_HSMCI0_MOUNT_MOUNTPOINT,
                  CONFIG_SAME70XPLAINED_HSMCI0_MOUNT_FSTYPE,
                  0, NULL);

      if (ret < 0)
        {
          SYSLOG("ERROR: Failed to mount %s: %d\n",
                 CONFIG_SAME70XPLAINED_HSMCI0_MOUNT_MOUNTPOINT, errno);
        }
    }

#endif /* CONFIG_SAME70XPLAINED_HSMCI0_MOUNT */
#endif /* HAVE_HSMCI */

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  sam_automount_initialize();
#endif

#ifdef HAVE_ROMFS
  /* Create a ROM disk for the /etc filesystem */

  ret = romdisk_register(CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_MINOR, romfs_img,
                         NSECTORS(romfs_img_len),
                         CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_SECTSIZE);
  if (ret < 0)
    {
      SYSLOG("ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = mount(CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_DEVNAME,
                  CONFIG_SAME70XPLAINED_ROMFS_MOUNT_MOUNTPOINT,
                  "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          SYSLOG("ERROR: mount(%s,%s,romfs) failed: %d\n",
                 CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_DEVNAME,
                 CONFIG_SAME70XPLAINED_ROMFS_MOUNT_MOUNTPOINT, errno);
        }
    }
#endif

#ifdef HAVE_PROGMEM_CHARDEV
  /* Initialize the SAME70 FLASH programming memory library */

  sam_progmem_initialize();

  /* Create an instance of the SAME70 FLASH program memory device driver */

  mtd = progmem_initialize();
  if (!mtd)
    {
      SYSLOG("ERROR: progmem_initialize failed\n");
    }

  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(PROGMEM_MTD_MINOR, mtd);
  if (ret < 0)
    {
      SYSLOG("ERROR: Failed to initialize the FTL layer: %d\n", ret);
      return ret;
    }

  /* Use the minor number to create device paths */

  snprintf(blockdev, 18, "/dev/mtdblock%d", PROGMEM_MTD_MINOR);
  snprintf(chardev, 12, "/dev/mtd%d", PROGMEM_MTD_MINOR);

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      SYSLOG("ERROR: bchdev_register %s failed: %d\n", chardev, ret);
      return ret;
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  sam_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = sam_usbhost_initialize();
  if (ret != OK)
    {
      SYSLOG("ERROR: Failed to initialize USB host: %d\n", ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start(0, NULL);
  if (ret != OK)
    {
      SYSLOG("ERROR: Failed to start the USB monitor: %d\n", ret);
    }
#endif

#ifdef HAVE_ELF
  /* Initialize the ELF binary loader */

  SYSLOG("Initializing the ELF binary loader\n");
  ret = elf_initialize();
  if (ret < 0)
    {
      SYSLOG("ERROR: Initialization of the ELF loader failed: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
