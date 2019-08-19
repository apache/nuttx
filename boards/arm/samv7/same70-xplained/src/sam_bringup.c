/****************************************************************************
 * boards/arm/samv7/same70-xplained/src/sam_bringup.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include <nuttx/drivers/drivers.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/i2c/i2c_master.h>

#include "sam_twihs.h"
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
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
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
#ifdef CONFIG_SAMV7_TWIHS0
  sam_i2c_register(0);
#endif
#ifdef CONFIG_SAMV7_TWIHS1
  sam_i2c_register(1);
#endif
#ifdef CONFIG_SAMV7_TWIHS2
  sam_i2c_register(2);
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
#ifdef HAVE_PROGMEM_CHARDEV
  FAR struct mtd_dev_s *mtd;
  char blockdev[18];
  char chardev[12];
#endif
  int ret;

  /* Register I2C drivers on behalf of the I2C tool */

  sam_i2ctool();

#ifdef HAVE_MACADDR
  /* Read the Ethernet MAC address from the AT24 FLASH and configure the
   * Ethernet driver with that address.
   */

  ret = sam_emac0_setmac();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_emac0_setmac() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, SAME70_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             SAME70_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef HAVE_MTDCONFIG
  /* Create an AT24xx-based MTD configuration device for storage device
   * configuration information.
   */

  ret = sam_at24config();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_at24config() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_HSMCI
  /* Initialize the HSMCI0 driver */

  ret = sam_hsmci_initialize(HSMCI0_SLOTNO, HSMCI0_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
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
          syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
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
      syslog(LOG_ERR, "ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = mount(CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_DEVNAME,
                  CONFIG_SAME70XPLAINED_ROMFS_MOUNT_MOUNTPOINT,
                  "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: mount(%s,%s,romfs) failed: %d\n",
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
      syslog(LOG_ERR, "ERROR: progmem_initialize failed\n");
    }

  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(PROGMEM_MTD_MINOR, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n", ret);
      return ret;
    }

  /* Use the minor number to create device paths */

  snprintf(blockdev, 18, "/dev/mtdblock%d", PROGMEM_MTD_MINOR);
  snprintf(chardev, 12, "/dev/mtd%d", PROGMEM_MTD_MINOR);

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n", chardev, ret);
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
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start the USB monitor: %d\n", ret);
    }
#endif

#ifdef CONFIG_SAMV7_MCAN
  /* Initialize CAN and register the CAN driver. */

  ret = sam_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_can_setup failed: %d\n", ret);
    }
#endif

#ifdef HAVE_MRF24J40
  /* Configure MRF24J40 wireless */

  ret = sam_mrf24j40_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_mrf24j40_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_XBEE
  /* Configure XBee  */

  ret = sam_xbee_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_xbee_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_SAMV7_DAC0) || defined(CONFIG_SAMV7_DAC1)
  ret = sam_dacdev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Initialization of the DAC module failed: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
