/****************************************************************************
 * config/samv71-xult/src/sam_bringup.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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
#include <nuttx/video/fb.h>

#include "sam_twihs.h"
#include "samv71-xult.h"

#if defined(HAVE_S25FL1) || defined(HAVE_PROGMEM_CHARDEV)
#  include <nuttx/mtd/mtd.h>
#endif

#ifdef HAVE_S25FL1
#  include <nuttx/spi/qspi.h>
#  include "sam_qspi.h"
#endif

#ifdef HAVE_LED_DRIVER
#  include <nuttx/leds/userled.h>
#endif

#ifdef HAVE_PROGMEM_CHARDEV
#  include "sam_progmem.h"
#endif

#if defined(HAVE_RTC_DSXXXX) || defined(HAVE_RTC_PCF85263)
#  include <nuttx/clock.h>
#  include <nuttx/i2c/i2c_master.h>
#ifdef HAVE_RTC_DSXXXX
#  include <nuttx/timers/ds3231.h>
#else
#  include <nuttx/timers/pcf85263.h>
#endif
#  include "sam_twihs.h"
#endif

#ifdef HAVE_ROMFS
#  include <arch/board/boot_romfsimg.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS(n) \
  (((n)+CONFIG_SAMV71XULT_ROMFS_ROMDISK_SECTSIZE-1) / \
   CONFIG_SAMV71XULT_ROMFS_ROMDISK_SECTSIZE)

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
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
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
#ifdef HAVE_S25FL1
  FAR struct qspi_dev_s *qspi;
#endif
#if defined(HAVE_S25FL1) || defined(HAVE_PROGMEM_CHARDEV)
  FAR struct mtd_dev_s *mtd;
#endif
#if defined(HAVE_RTC_DSXXXX) || defined(HAVE_RTC_PCF85263)
  FAR struct i2c_master_s *i2c;
#endif
#if defined(HAVE_S25FL1_CHARDEV) || defined(HAVE_PROGMEM_CHARDEV)
  char blockdev[18];
  char chardev[12];
#endif
  int ret;

  /* Register I2C drivers on behalf of the I2C tool */

  sam_i2ctool();

#ifdef HAVE_LED_DRIVER
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n",
             ret);
    }
#endif

#if defined(HAVE_RTC_PCF85263)
  /* Get an instance of the TWIHS0 I2C interface */

  i2c = sam_i2cbus_initialize(PCF85263_TWI_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: sam_i2cbus_initialize(%d) failed\n",
             PCF85263_TWI_BUS);
    }
  else
    {
      /* Use the I2C interface to initialize the PCF2863 timer */

      ret = pcf85263_rtc_initialize(i2c);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: pcf85263_rtc_initialize() failed: %d\n",
                 ret);
        }
      else
        {
          /* Synchronize the system time to the RTC time */

          clock_synchronize();
        }
    }

#elif defined(HAVE_RTC_DSXXXX)
  /* Get an instance of the TWIHS0 I2C interface */

  i2c = sam_i2cbus_initialize(DSXXXX_TWI_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: sam_i2cbus_initialize(%d) failed\n",
             DSXXXX_TWI_BUS);
    }
  else
    {
      /* Use the I2C interface to initialize the DSXXXX timer */

      ret = dsxxxx_rtc_initialize(i2c);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: dsxxxx_rtc_initialize() failed: %d\n",
                 ret);
        }
      else
        {
          /* Synchronize the system time to the RTC time */

          clock_synchronize();
        }
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

  ret = mount(NULL, SAMV71_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             SAMV71_PROCFS_MOUNTPOINT, ret);
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

#ifdef CONFIG_SAMV71XULT_HSMCI0_MOUNT
  else
    {
      /* REVISIT:  A delay seems to be required here or the mount will fail. */
      /* Mount the volume on HSMCI0 */

      ret = mount(CONFIG_SAMV71XULT_HSMCI0_MOUNT_BLKDEV,
                  CONFIG_SAMV71XULT_HSMCI0_MOUNT_MOUNTPOINT,
                  CONFIG_SAMV71XULT_HSMCI0_MOUNT_FSTYPE,
                  0, NULL);

      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
                 CONFIG_SAMV71XULT_HSMCI0_MOUNT_MOUNTPOINT, errno);
        }
    }

#endif /* CONFIG_SAMV71XULT_HSMCI0_MOUNT */
#endif /* HAVE_HSMCI */

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  sam_automount_initialize();
#endif

#ifdef HAVE_ROMFS
  /* Create a ROM disk for the /etc filesystem */

  ret = romdisk_register(CONFIG_SAMV71XULT_ROMFS_ROMDISK_MINOR, romfs_img,
                         NSECTORS(romfs_img_len),
                         CONFIG_SAMV71XULT_ROMFS_ROMDISK_SECTSIZE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = mount(CONFIG_SAMV71XULT_ROMFS_ROMDISK_DEVNAME,
                  CONFIG_SAMV71XULT_ROMFS_MOUNT_MOUNTPOINT,
                  "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: mount(%s,%s,romfs) failed: %d\n",
                 CONFIG_SAMV71XULT_ROMFS_ROMDISK_DEVNAME,
                 CONFIG_SAMV71XULT_ROMFS_MOUNT_MOUNTPOINT, errno);
        }
    }
#endif

#ifdef HAVE_S25FL1
  /* Create an instance of the SAMV71 QSPI device driver */

  qspi = sam_qspi_initialize(0);
  if (!qspi)
    {
      syslog(LOG_ERR, "ERROR: sam_qspi_initialize failed\n");
    }
  else
    {
      /* Use the QSPI device instance to initialize the
       * S25FL1 device.
       */

      mtd = s25fl1_initialize(qspi, true);
      if (!mtd)
        {
          syslog(LOG_ERR, "ERROR: s25fl1_initialize failed\n");
        }

#ifdef HAVE_S25FL1_SMARTFS
      /* Configure the device with no partition support */

      ret = smart_initialize(S25FL1_SMART_MINOR, mtd, NULL);
      if (ret != OK)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize SmartFS: %d\n", ret);
        }

#elif defined(HAVE_S25FL1_NXFFS)
      /* Initialize to provide NXFFS on the S25FL1 MTD interface */

      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
         syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n", ret);
        }

      /* Mount the file system at /mnt/s25fl1 */

      ret = mount(NULL, "/mnt/s25fl1", "nxffs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the NXFFS volume: %d\n",
                 errno);
          return ret;
        }

#else /* if  defined(HAVE_S25FL1_CHARDEV) */
      /* Use the FTL layer to wrap the MTD driver as a block driver */

      ret = ftl_initialize(S25FL1_MTD_MINOR, mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n",
                 ret);
          return ret;
        }

      /* Use the minor number to create device paths */

      snprintf(blockdev, 18, "/dev/mtdblock%d", S25FL1_MTD_MINOR);
      snprintf(chardev, 12, "/dev/mtd%d", S25FL1_MTD_MINOR);

      /* Now create a character device on the block device */

      ret = bchdev_register(blockdev, chardev, false);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n",
                 chardev, ret);
          return ret;
        }
#endif
    }
#endif

#ifdef HAVE_PROGMEM_CHARDEV
  /* Initialize the SAMV71 FLASH programming memory library */

  sam_progmem_initialize();

  /* Create an instance of the SAMV71 FLASH program memory device driver */

  mtd = progmem_initialize();
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize failed\n");
    }

  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(PROGMEM_MTD_MINOR, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n",
             ret);
      return ret;
    }

  /* Use the minor number to create device paths */

  snprintf(blockdev, 18, "/dev/mtdblock%d", PROGMEM_MTD_MINOR);
  snprintf(chardev, 12, "/dev/mtd%d", PROGMEM_MTD_MINOR);

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n",
             chardev, ret);
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

#ifdef HAVE_MAXTOUCH
  /* Initialize the touchscreen */

  ret = sam_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_tsc_setup failed: %d\n", ret);
    }
#endif

#ifdef HAVE_WM8904
  /* Configure WM8904 audio */

  ret = sam_wm8904_initialize(0);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize WM8904 audio: %d\n",
             ret);
    }
#endif

#ifdef HAVE_AUDIO_NULL
  /* Configure the NULL audio device */

  ret = sam_audio_null_initialize(0);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize the NULL audio device: %d\n",
             ret);
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

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the LCD framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
