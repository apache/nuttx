/****************************************************************************
 * boards/arm/sama5/jupiter-nano/src/sam_bringup.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdlib.h>
#include <syslog.h>
#include <debug.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "jupiter-nano.h"

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_NET_CDCECM
#  include <nuttx/usb/cdcecm.h>
#  include <net/if.h>
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_MMCSD
#  include <nuttx/mmcsd.h>
#  include "sam_sdmmc.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS(n) \
  (((n)+CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE-1) / \
   CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE)

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
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
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
 * Name: nsh_sdmmc_initialize
 *
 * Description:
 *   Initialize SDMMC drivers
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SDMMC

static int nsh_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdmmc0;
  struct sdio_dev_s *sdmmc1;
  int ret = 0;

  /* Get an instance of the SDIO interface */

#ifdef CONFIG_SAMA5_SDMMC0
  sdmmc0 = sam_sdmmc_sdio_initialize(SDMMC0_SLOTNO);
  if (!sdmmc0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(SDMMC0_MINOR, sdmmc0);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver (slot 0): "
                 "%d\n",
                 ret);
        }
    }

#ifdef CONFIG_SAMA5D27_SDMMC0_MOUNT
  /* Mount the volume on SDMMC0 */

  ret = nx_mount(CONFIG_SAMA5D27_SDMMC0_MOUNT_BLKDEV,
                 CONFIG_SAMA5D27_SDMMC0_MOUNT_MOUNTPOINT,
                 CONFIG_SAMA5D27_SDMMC0_MOUNT_FSTYPE,
                 0, NULL);

  if (ret < 0)
    {
      _err("ERROR: Failed to mount %s: %d\n",
           CONFIG_SAMA5D27_SDMMC0_MOUNT_MOUNTPOINT, ret);
    }
#endif
#endif

#ifdef CONFIG_SAMA5_SDMMC1
  sdmmc1 = sam_sdmmc_sdio_initialize(SDMMC1_SLOTNO);
  if (!sdmmc1)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(SDMMC1_MINOR, sdmmc1);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver (slot 0): "
                 "%d\n",
                 ret);
        }
    }

#ifdef CONFIG_SAMA5D27_SDMMC1_MOUNT
  /* Mount the volume on SDMMC1 */

  ret = nx_mount(CONFIG_SAMA5D27_SDMMC1_MOUNT_BLKDEV,
                 CONFIG_SAMA5D27_SDMMC1_MOUNT_MOUNTPOINT,
                 CONFIG_SAMA5D27_SDMMC1_MOUNT_FSTYPE,
                 0, NULL);

  if (ret < 0)
    {
      _err("ERROR: Failed to mount %s: %d\n",
           CONFIG_SAMA5D27_SDMMC1_MOUNT_MOUNTPOINT, ret);
    }
#endif
#endif

  return OK;
}
#else
#  define nsh_sdmmc_initialize() (OK)
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

#ifdef HAVE_SDMMC
#ifdef CONFIG_SAMA5_SDMMC
  /* Initialize SDMCC-based MMC/SD card support */

  nsh_sdmmc_initialize();
#endif
#endif

#ifdef HAVE_HSMCI
#ifdef CONFIG_SAMA5_HSMCI0
  /* Initialize the HSMCI0 driver */

  ret = sam_hsmci_initialize(HSMCI0_SLOTNO, HSMCI0_MINOR);
  if (ret < 0)
    {
      _err("ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
           HSMCI0_SLOTNO, HSMCI0_MINOR, ret);
    }

#ifdef CONFIG_SAMA5D4EK_HSMCI0_MOUNT
  else
    {
      /* REVISIT:
       *  A delay seems to be required here or the mount will fail.
       */

      /* Mount the volume on HSMCI0 */

      ret = nx_mount(CONFIG_SAMA5D4EK_HSMCI0_MOUNT_BLKDEV,
                     CONFIG_SAMA5D4EK_HSMCI0_MOUNT_MOUNTPOINT,
                     CONFIG_SAMA5D4EK_HSMCI0_MOUNT_FSTYPE,
                     0, NULL);

      if (ret < 0)
        {
          _err("ERROR: Failed to mount %s: %d\n",
               CONFIG_SAMA5D4EK_HSMCI0_MOUNT_MOUNTPOINT, ret);
        }
    }
#endif
#endif

#ifdef CONFIG_SAMA5_HSMCI1
  /* Initialize the HSMCI1 driver */

  ret = sam_hsmci_initialize(HSMCI1_SLOTNO, HSMCI1_MINOR);
  if (ret < 0)
    {
      _err("ERROR: sam_hsmci_initialize(%d,%d) failed: %d\n",
           HSMCI1_SLOTNO, HSMCI1_MINOR, ret);
    }

#ifdef CONFIG_SAMA5D4EK_HSMCI1_MOUNT
  else
    {
      /* REVISIT: A delay seems required here or the mount will fail. */

      /* Mount the volume on HSMCI1 */

      ret = nx_mount(CONFIG_SAMA5D4EK_HSMCI1_MOUNT_BLKDEV,
                     CONFIG_SAMA5D4EK_HSMCI1_MOUNT_MOUNTPOINT,
                     CONFIG_SAMA5D4EK_HSMCI1_MOUNT_FSTYPE,
                     0, NULL);

      if (ret < 0)
        {
          _err("ERROR: Failed to mount %s: %d\n",
               CONFIG_SAMA5D4EK_HSMCI1_MOUNT_MOUNTPOINT, ret);
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
      _err("ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = nx_mount(CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME,
                     CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT,
                     "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          _err("ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
               CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME,
               CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT, ret);
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
        _err("ERROR: Failed to initialize USB host: %d\n", ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
        _err("ERROR: Failed to start the USB monitor: %d\n", ret);
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

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = sam_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = sam_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef HAVE_WM8904
  /* Configure WM8904 audio */

  ret = sam_wm8904_initialize(0);
  if (ret != OK)
    {
      _err("ERROR: Failed to initialize WM8904 audio: %d\n", ret);
    }
#endif

#ifdef HAVE_AUDIO_NULL
  /* Configure the NULL audio device */

  ret = sam_audio_null_initialize(0);
  if (ret != OK)
    {
      _err("ERROR: Failed to initialize the NULL audio device: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, SAMA5_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount procfs at %s: %d\n",
           SAMA5_PROCFS_MOUNTPOINT, ret);
    }
#endif

#if defined(CONFIG_RNDIS)
  /* Set up a MAC address for the RNDIS device. */

  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#ifdef CONFIG_NET_CDCECM
    ret = cdcecm_initialize(0, NULL);
  if (ret < 0)
    {
      _err("ERROR: cdcecm_initialize() failed: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
