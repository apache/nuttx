/****************************************************************************
 * config/sama5d4-ek/src/sam_bringup.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <nuttx/fs/ramdisk.h>
#include <nuttx/binfmt/elf.h>
#include <nuttx/i2c/i2c_master.h>

#include "sam_twi.h"
#include "sama5d4-ek.h"

#ifdef HAVE_ROMFS
#  include <arch/board/boot_romfsimg.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS(n) \
  (((n)+CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE-1) / \
   CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE)

/* Debug ********************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
#  define SYSLOG lldbg
#else
#  define SYSLOG dbg
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void sam_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = sam_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      dbg("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          dbg("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          sam_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: sam_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void sam_i2ctool(void)
{
#ifdef CONFIG_SAMA5_TWI0
  sam_i2c_register(0);
#endif
#ifdef CONFIG_SAMA5_TWI1
  sam_i2c_register(1);
#endif
#ifdef CONFIG_SAMA5_TWI2
  sam_i2c_register(2);
#endif
#ifdef CONFIG_SAMA5_TWI3
  sam_i2c_register(3);
#endif
}
#else
#  define sam_i2ctool()
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
#if defined(HAVE_NAND) || defined(HAVE_AT25) || defined(HAVE_HSMCI)  || \
    defined(HAVE_USBHOST) || defined(HAVE_USBMONITOR) || defined(HAVE_WM8904) || \
    defined(HAVE_AUTOMOUNTER) || defined(HAVE_ELF) || defined(HAVE_ROMFS) || \
    defined(CONFIG_FS_PROCFS)
  int ret;
#endif

  /* Register I2C drivers on behalf of the I2C tool */

  sam_i2ctool();

#ifdef HAVE_NAND
  /* Initialize the NAND driver */

  ret = sam_nand_automount(NAND_MINOR);
  if (ret < 0)
    {
      SYSLOG("ERROR: sam_nand_automount failed: %d\n", ret);
    }
#endif

#ifdef HAVE_AT25
  /* Initialize the AT25 driver */

  ret = sam_at25_automount(AT25_MINOR);
  if (ret < 0)
    {
      SYSLOG("ERROR: sam_at25_automount failed: %d\n", ret);
    }
#endif

#ifdef HAVE_HSMCI
#ifdef CONFIG_SAMA5_HSMCI0
  /* Initialize the HSMCI0 driver */

  ret = sam_hsmci_initialize(HSMCI0_SLOTNO, HSMCI0_MINOR);
  if (ret < 0)
    {
      SYSLOG("ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
             HSMCI0_SLOTNO, HSMCI0_MINOR, ret);
    }

#ifdef CONFIG_SAMA5D4EK_HSMCI0_MOUNT
  else
    {
      /* REVISIT:  A delay seems to be required here or the mount will fail. */
      /* Mount the volume on HSMCI0 */

      ret = mount(CONFIG_SAMA5D4EK_HSMCI0_MOUNT_BLKDEV,
                  CONFIG_SAMA5D4EK_HSMCI0_MOUNT_MOUNTPOINT,
                  CONFIG_SAMA5D4EK_HSMCI0_MOUNT_FSTYPE,
                  0, NULL);

      if (ret < 0)
        {
          SYSLOG("ERROR: Failed to mount %s: %d\n",
                 CONFIG_SAMA5D4EK_HSMCI0_MOUNT_MOUNTPOINT, errno);
        }
    }
#endif
#endif

#ifdef CONFIG_SAMA5_HSMCI1
  /* Initialize the HSMCI1 driver */

  ret = sam_hsmci_initialize(HSMCI1_SLOTNO, HSMCI1_MINOR);
  if (ret < 0)
    {
      SYSLOG("ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
             HSMCI1_SLOTNO, HSMCI1_MINOR, ret);
    }

#ifdef CONFIG_SAMA5D4EK_HSMCI1_MOUNT
  else
    {
      /* REVISIT:  A delay seems to be required here or the mount will fail. */
      /* Mount the volume on HSMCI1 */

      ret = mount(CONFIG_SAMA5D4EK_HSMCI1_MOUNT_BLKDEV,
                  CONFIG_SAMA5D4EK_HSMCI1_MOUNT_MOUNTPOINT,
                  CONFIG_SAMA5D4EK_HSMCI1_MOUNT_FSTYPE,
                  0, NULL);

      if (ret < 0)
        {
          SYSLOG("ERROR: Failed to mount %s: %d\n",
                 CONFIG_SAMA5D4EK_HSMCI1_MOUNT_MOUNTPOINT, errno);
        }
    }
#endif
#endif
#endif

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  sam_automount_initialize();
#endif

#ifdef HAVE_ROMFS
  /* Create a ROM disk for the /etc filesystem */

  ret = romdisk_register(CONFIG_SAMA5D4EK_ROMFS_ROMDISK_MINOR, romfs_img,
                         NSECTORS(romfs_img_len),
                         CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE);
  if (ret < 0)
    {
      SYSLOG("ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = mount(CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME,
                  CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT,
                  "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          SYSLOG("ERROR: mount(%s,%s,romfs) failed: %d\n",
                 CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME,
                 CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT, errno);
        }
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

#ifdef HAVE_WM8904
  /* Configure WM8904 audio */

  ret = sam_wm8904_initialize(0);
  if (ret != OK)
    {
      SYSLOG("ERROR: Failed to initialize WM8904 audio: %d\n", ret);
    }
#endif

#ifdef HAVE_AUDIO_NULL
  /* Configure the NULL audio device */

  ret = sam_audio_null_initialize(0);
  if (ret != OK)
    {
      SYSLOG("ERROR: Failed to initialize the NULL audio device: %d\n", ret);
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

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, SAMA5_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      SYSLOG("ERROR: Failed to mount procfs at %s: %d\n",
             SAMA5_PROCFS_MOUNTPOINT, ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  return OK;
}
