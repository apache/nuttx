/****************************************************************************
 * boards/arm/samv7/same70-xplained/src/sam_bringup.c
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
#include <sys/param.h>

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
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/i2c/i2c_master.h>

#include "sam_twihs.h"
#include "same70-xplained.h"

#ifdef HAVE_HSMCI
#  include "board_hsmci.h"
#endif /* HAVE_HSMCI */

#ifdef HAVE_AUTOMOUNTER
#  include "sam_automount.h"
#endif /* HAVE_AUTOMOUNTER */

#ifdef HAVE_ROMFS
#  include <arch/board/boot_romfsimg.h>
#endif

#ifdef HAVE_PROGMEM_CHARDEV
#  include "board_progmem.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS(n) \
  (((n)+CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_SECTSIZE-1) / \
   CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_SECTSIZE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_SAMV7_PROGMEM_OTA_PARTITION)
static struct mtd_partition_s g_mtd_partition_table[] =
{
  {
    .offset  = CONFIG_SAMV7_OTA_PRIMARY_SLOT_OFFSET,
    .size    = CONFIG_SAMV7_OTA_SLOT_SIZE,
    .devpath = CONFIG_SAMV7_OTA_PRIMARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_SAMV7_OTA_SECONDARY_SLOT_OFFSET,
    .size    = CONFIG_SAMV7_OTA_SLOT_SIZE,
    .devpath = CONFIG_SAMV7_OTA_SECONDARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_SAMV7_OTA_SCRATCH_OFFSET,
    .size    = CONFIG_SAMV7_OTA_SCRATCH_SIZE,
    .devpath = CONFIG_SAMV7_OTA_SCRATCH_DEVPATH
  }
};

static const size_t g_mtd_partition_table_size =
    nitems(g_mtd_partition_table);
#else
#  define g_mtd_partition_table         NULL
#  define g_mtd_partition_table_size    0
#endif /* CONFIG_SAMV7_PROGMEM_OTA_PARTITION */

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
  struct i2c_master_s *i2c;
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

  ret = nx_mount(NULL, SAME70_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
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

  ret = sam_hsmci_initialize(HSMCI0_SLOTNO, HSMCI0_MINOR, GPIO_HSMCI0_CD,
                             IRQ_HSMCI0_CD, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
             HSMCI0_SLOTNO, HSMCI0_MINOR, ret);
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
#endif /* HAVE_HSMCI */

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  sam_automount_initialize();
#endif

#ifdef HAVE_ROMFS
  /* Create a ROM disk for the /etc filesystem */

  ret = romdisk_register(CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_MINOR,
                         romfs_img, NSECTORS(romfs_img_len),
                         CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_SECTSIZE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = nx_mount(CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_DEVNAME,
                     CONFIG_SAME70XPLAINED_ROMFS_MOUNT_MOUNTPOINT,
                     "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
                 CONFIG_SAME70XPLAINED_ROMFS_ROMDISK_DEVNAME,
                 CONFIG_SAME70XPLAINED_ROMFS_MOUNT_MOUNTPOINT, ret);
        }
    }
#endif

#ifdef HAVE_PROGMEM_CHARDEV
  /* Initialize the SAME70 FLASH programming memory library */

  ret = board_progmem_init(PROGMEM_MTD_MINOR, g_mtd_partition_table,
                           g_mtd_partition_table_size);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize progmem: %d\n", ret);
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

#ifdef CONFIG_SAMV7_PWM
  /* Initialize PWM and register the driver. */

  ret = sam_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_afec_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SAMV7_AFEC
  /* Initialize AFEC and register the ADC driver. */

  ret = sam_afec_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_afec_initialize failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_SAMV7_DAC0) || defined(CONFIG_SAMV7_DAC1)
  ret = sam_dacdev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Initialization of the DAC module failed: %d\n",
             ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
