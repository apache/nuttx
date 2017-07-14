/****************************************************************************
 * config/spark/src/stm32_composite.c
 *
 *   Copyright (C) 2012-2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David_s5 <david_s5@nscdg.com>
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/board.h>

#ifdef CONFIG_MTD_SST25
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <sys/mount.h>
#endif

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_USBDEV
#  include "stm32_usbdev.h"
#endif

#include "stm32.h"
#include "spark.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Can't support the SST25 device if it SPI2 or SST25 support is not enabled */

#define HAVE_SST25  1
#if !defined(CONFIG_STM32_SPI2) || !defined(CONFIG_MTD_SST25)
#  undef HAVE_SST25
#endif

/* Can't support SST25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_SST25
#endif

#ifndef CONFIG_SPARK_FLASH_MOUNT_POINT
#  define CONFIG_SPARK_FLASH_MOUNT_POINT "/mnt/p%d"
#endif

/* Use minor device number 0 is not is provided */

#ifndef CONFIG_SPARK_FLASH_MINOR
#  define CONFIG_SPARK_FLASH_MINOR 0
#endif

/* Can't support both FAT and NXFFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_NXFFS)
#  warning "Can't support both FAT and NXFFS -- using FAT"
#endif

#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1

/* Can't support USB device is USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_composite_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

#ifndef CONFIG_NSH_BUILTIN_APPS
static int stm32_composite_initialize(void)
{
#ifdef HAVE_SST25
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  int ret;

  /* Configure SPI-based devices */

  /* Get the SPI port */

  finfo("Initializing SPI port %d\n", CONFIG_SPARK_FLASH_SPI);

  spi = stm32_spibus_initialize(CONFIG_SPARK_FLASH_SPI);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI port %d\n",
           CONFIG_SPARK_FLASH_SPI);
      return -ENODEV;
    }

  finfo("Successfully initialized SPI port %d\n", CONFIG_SPARK_FLASH_SPI);

  /* Now bind the SPI interface to the SST25 SPI FLASH driver */

  finfo("Bind SPI to the SPI flash driver\n");
  mtd = sst25_initialize(spi);
  if (!mtd)
    {
      ferr("ERROR: Failed to bind SPI port %d to the SPI FLASH driver\n",
           CONFIG_SPARK_FLASH_SPI);
    }
  else
    {
      finfo("Successfully bound SPI port %d to the SPI FLASH driver\n",
            CONFIG_SPARK_FLASH_SPI);
    }

#ifndef CONFIG_SPARK_FLASH_PART

  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(CONFIG_SPARK_FLASH_MINOR, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initialize the FTL layer\n");
      return ret;
    }

#ifdef CONFIG_SPARK_MOUNT_FLASH
  char  partname[16];
  char  mntpoint[16];

  /* mount -t vfat /dev/mtdblock0 /mnt/p0 */

  snprintf(partname, sizeof(partname), "/dev/mtdblock%d",
           CONFIG_SPARK_FLASH_MINOR);
  snprintf(mntpoint, sizeof(mntpoint)-1, CONFIG_SPARK_FLASH_MOUNT_POINT,
           CONFIG_SPARK_FLASH_MINOR);

  /* Mount the file system at /mnt/pn  */

  ret = mount(partname, mntpoint, "vfat", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the FAT volume: %d\n", errno);
      return ret;
    }

#endif
#else
    {
      int partno;
      int partsize;
      int partoffset;
      const char *partstring = CONFIG_SPARK_FLASH_PART_LIST;
      const char *ptr;
      FAR struct mtd_dev_s *mtd_part;
      char  partname[16];
      char  mntpoint[16];

      /* Now create a partition on the FLASH device */

      partno = CONFIG_SPARK_FLASH_MINOR;
      ptr = partstring;
      partoffset = 0;
      while (*ptr != '\0')
        {
          /* Get the partition size */

          partsize = atoi(ptr);
          mtd_part = mtd_partition(mtd, partoffset, (partsize >> 2) * 16);
          partoffset += (partsize >> 2) * 16;

          /* Use the FTL layer to wrap the MTD driver as a block driver */

          ret = ftl_initialize(partno, mtd_part);
          if (ret < 0)
            {
              ferr("ERROR: Initialize the FTL layer\n");
              return ret;
            }

          snprintf(partname,sizeof(partname), "/dev/mtdblock%d", partno);
          snprintf(mntpoint,sizeof(mntpoint)-1, CONFIG_SPARK_FLASH_MOUNT_POINT,
                   partno);

          /* Mount the file system at /mnt/pn  */

          ret = mount(partname, mntpoint, "vfat", 0, NULL);
          if (ret < 0)
            {
              ferr("ERROR: Failed to mount the FAT volume: %d\n", errno);
              return ret;
            }

          /* Update the pointer to point to the next size in the list */

          while ((*ptr >= '0') && (*ptr <= '9'))
            {
              ptr++;
            }

          if (*ptr == ',')
            {
              ptr++;
            }

          /* Increment the part number */

          partno++;
        }
    }
#endif /* CONFIG_SPARK_FLASH_PART */

#endif /* HAVE_SST25 */

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      ferr("ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: board_cdcclassobject
 *
 * Description:
 *   If the CDC serial class driver is part of composite device, then
 *   board-specific logic must provide board_cdcclassobject().  In the
 *   simplest case, board_cdcclassobject() is simply a wrapper around
 *   cdcacm_classobject() that provides the correct device minor number.
 *
 * Input Parameters:
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

static int board_cdcclassobject(int minor,
                                FAR struct usbdev_description_s *devdesc,
                                FAR struct usbdevclass_driver_s **classdev)
{
  return cdcacm_classobject(minor, devdesc, classdev);
}

/****************************************************************************
 * Name: board_cdcuninitialize
 *
 * Description:
 *   Un-initialize the USB serial class driver.  This is just an application-
 *   specific wrapper around cdcadm_unitialize() that is called form the
 *   composite device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instance previously given to the composite
 *     driver by board_cdcclassobject().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void board_cdcuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  cdcacm_uninitialize(classdev);
}

/************************************************************************************
 * Name: board_mscclassobject
 *
 * Description:
 *   If the mass storage class driver is part of composite device, then
 *   its instantiation and configuration is a multi-step, board-specific,
 *   process (See comments for usbmsc_configure below).  In this case,
 *   board-specific logic must provide board_mscclassobject().
 *
 *   board_mscclassobject() is called from the composite driver.  It must
 *   encapsulate the instantiation and configuration of the mass storage
 *   class and the return the mass storage device's class driver instance
 *   to the composite dirver.
 *
 * Input Parameters:
 *   classdev - The location to return the mass storage class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ************************************************************************************/

static int board_mscclassobject(int minor, FAR struct usbdev_description_s *devdesc,
                                FAR struct usbdevclass_driver_s **classdev)
{
  FAR void *handle;
  int ret;

  ret = usbmsc_configure(1, &handle);
  if (ret >= 0)
    {
      ret = usbmsc_classobject(handle, devdesc, classdev);
    }

  return ret;
}

 /************************************************************************************
 * Name: board_mscuninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This is just an application-
 *   specific wrapper aboutn usbmsc_unitialize() that is called form the composite
 *   device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instrance previously give to the composite
 *     driver by board_mscclassobject().
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void board_mscuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  usbmsc_uninitialize(classdev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_composite_initialize
 *
 * Description:
 *   Perform architecture specific initialization of a composite USB device.
 *
 ****************************************************************************/

int board_composite_initialize(int port)
{
#ifdef CONFIG_NSH_BUILTIN_APPS
  return OK;
#else
  return stm32_composite_initialize();
#endif
}

/****************************************************************************
 * Name:  board_composite_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port using
 *   the specified configuration.  The interpretation of the configid is
 *   board specific.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *   configid - The USB composite configuration
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

FAR void *board_composite_connect(int port, int configid)
{
  /* Here we are composing the configuration of the usb composite device.
   *
   * The standard is to use one CDC/ACM and one USB mass storage device.
   */

  struct composite_devdesc_s dev[2];
  int ifnobase = 0;
  int strbase  = COMPOSITE_NSTRIDS;

  /* Configure the CDC/ACM device */

  /* Ask the cdcacm driver to fill in the constants we didn't
   * know here.
   */

  cdcacm_get_composite_devdesc(&dev[0]);

  /* Overwrite and correct some values... */
  /* The callback functions for the CDC/ACM class */

  dev[0].classobject  = board_cdcclassobject;
  dev[0].uninitialize = board_cdcuninitialize;

  /* Interfaces */

  dev[0].devdesc.ifnobase = ifnobase;             /* Offset to Interface-IDs */
  dev[0].minor = CONFIG_SYSTEM_COMPOSITE_TTYUSB;  /* The minor interface number */

  /* Strings */

  dev[0].devdesc.strbase = strbase;               /* Offset to String Numbers */

  /* Endpoints */

  dev[0].devdesc.epno[CDCACM_EP_INTIN_IDX]   = 1;
  dev[0].devdesc.epno[CDCACM_EP_BULKIN_IDX]  = 2;
  dev[0].devdesc.epno[CDCACM_EP_BULKOUT_IDX] = 3;

  /* Count up the base numbers */

  ifnobase += dev[0].devdesc.ninterfaces;
  strbase  += dev[0].devdesc.nstrings;

  /* Configure the mass storage device device */
  /* Ask the usbmsc driver to fill in the constants we didn't
   * know here.
   */

  usbmsc_get_composite_devdesc(&dev[1]);

  /* Overwrite and correct some values... */
  /* The callback functions for the USBMSC class */

  dev[1].classobject  = board_mscclassobject;
  dev[1].uninitialize = board_mscuninitialize;

  /* Interfaces */

  dev[1].devdesc.ifnobase = ifnobase;               /* Offset to Interface-IDs */
  dev[1].minor = CONFIG_SYSTEM_COMPOSITE_DEVMINOR1; /* The minor interface number */

  /* Strings */

  dev[1].devdesc.strbase = strbase;   /* Offset to String Numbers */

  /* Endpoints */

  dev[1].devdesc.epno[USBMSC_EP_BULKIN_IDX]  = 5;
  dev[1].devdesc.epno[USBMSC_EP_BULKOUT_IDX] = 4;

  /* Count up the base numbers */

  ifnobase += dev[1].devdesc.ninterfaces;
  strbase  += dev[1].devdesc.nstrings;

  return composite_initialize(2, dev);
}
