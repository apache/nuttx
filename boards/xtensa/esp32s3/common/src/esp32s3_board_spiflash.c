/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_board_spiflash.c
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

#include "inttypes.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>
#ifdef CONFIG_BCH
#include <nuttx/drivers/drivers.h>
#endif

#include "esp32s3_spiflash.h"
#include "esp32s3_spiflash_mtd.h"
#include "esp32s3-devkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_WIFI_MTD_ENCRYPT
#  define WIFI_ENCRYPT true
#else
#  define WIFI_ENCRYPT false
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setup_smartfs
 *
 * Description:
 *   Provide a block driver wrapper around MTD partition and mount a
 *   SMART FS over it.
 *
 * Parameters:
 *   smartn - Number used to register the mtd partition: /dev/smartx, where
 *            x = smartn.
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined (CONFIG_ESP32S3_SPIFLASH_SMARTFS)
static int setup_smartfs(int smartn, struct mtd_dev_s *mtd,
                         const char *mnt_pt)
{
  int ret = OK;
  char path[22];

  ret = smart_initialize(smartn, mtd, NULL);
  if (ret < 0)
    {
      finfo("smart_initialize failed, Trying to erase first...\n");
      ret = mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);
      if (ret < 0)
        {
          ferr("ERROR: ioctl(BULKERASE) failed: %d\n", ret);
          return ret;
        }

      finfo("Erase successful, initializing it again.\n");
      ret = smart_initialize(smartn, mtd, NULL);
      if (ret < 0)
        {
          ferr("ERROR: smart_initialize failed: %d\n", ret);
          return ret;
        }
    }

  if (mnt_pt != NULL)
    {
      snprintf(path, sizeof(path), "/dev/smart%d", smartn);

      ret = nx_mount(path, mnt_pt, "smartfs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the FS volume: %d\n", ret);
          return ret;
        }
    }

  return ret;
}

#endif

/****************************************************************************
 * Name: setup_littlefs
 *
 * Description:
 *   Register a mtd driver and mount a Little FS over it.
 *
 * Parameters:
 *   path   - Path name used to register the mtd driver.
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *   priv   - Privileges
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined (CONFIG_ESP32S3_SPIFLASH_LITTLEFS)
static int setup_littlefs(const char *path, struct mtd_dev_s *mtd,
                          const char *mnt_pt, int priv)
{
  int ret = OK;

  ret = register_mtddriver(path, mtd, priv, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register MTD: %d\n", ret);
      return ERROR;
    }

  if (mnt_pt != NULL)
    {
      ret = nx_mount(path, mnt_pt, "littlefs", 0, NULL);
      if (ret < 0)
        {
          ret = nx_mount(path, mnt_pt, "littlefs", 0, "forceformat");
          if (ret < 0)
            {
              ferr("ERROR: Failed to mount the FS volume: %d\n", ret);
              return ret;
            }
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: setup_spiffs
 *
 * Description:
 *   Register a mtd driver and mount a SPIFFS over it.
 *
 * Parameters:
 *   path   - Path name used to register the mtd driver.
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *   priv   - Privileges
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined (CONFIG_ESP32S3_SPIFLASH_SPIFFS)
static int setup_spiffs(const char *path, struct mtd_dev_s *mtd,
                        const char *mnt_pt, int priv)
{
  int ret = OK;

  ret = register_mtddriver(path, mtd, priv, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register MTD: %d\n", ret);
      return ERROR;
    }

  if (mnt_pt != NULL)
    {
      ret = nx_mount(path, mnt_pt, "spiffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the FS volume: %d\n", ret);
          return ret;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: setup_nxffs
 *
 * Description:
 *   Register a mtd driver and mount a SPIFFS over it.
 *
 * Parameters:
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined (CONFIG_ESP32S3_SPIFLASH_NXFFS)
static int setup_nxffs(struct mtd_dev_s *mtd, const char *mnt_pt)
{
  int ret = OK;

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS init failed: %d\n", ret);
      return ret;
    }

  if (mnt_pt != NULL)
    {
      ret = nx_mount(NULL, mnt_pt, "nxffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the FS volume: %d\n", ret);
          return ret;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: init_wifi_partition
 *
 * Description:
 *   Initialize partition that is dedicated to Wi-Fi.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_WIFI_SAVE_PARAM
static int init_wifi_partition(void)
{
  int ret = OK;
  struct mtd_dev_s *mtd;

  mtd = esp32s3_spiflash_alloc_mtdpart(CONFIG_ESP32S3_WIFI_MTD_OFFSET,
                                       CONFIG_ESP32S3_WIFI_MTD_SIZE,
                                       WIFI_ENCRYPT);
  if (!mtd)
    {
      ferr("Failed to alloc MTD partition of SPI Flash\n");
      return -ENOMEM;
    }

#if defined (CONFIG_ESP32S3_SPIFLASH_SMARTFS)

  ret = setup_smartfs(1, mtd, CONFIG_ESP32S3_WIFI_FS_MOUNTPT);
  if (ret < 0)
    {
      ferr("Failed to setup smartfs\n");
      return ret;
    }

#elif defined(CONFIG_ESP32S3_SPIFLASH_LITTLEFS)

  const char *path = "/dev/mtdblock1";
  ret = setup_littlefs(path, mtd, CONFIG_ESP32S3_WIFI_FS_MOUNTPT, 0777);
  if (ret < 0)
    {
      ferr("Failed to setup littlefs\n");
      return ret;
    }

#elif defined(CONFIG_ESP32S3_SPIFLASH_SPIFFS)

  const char *path = "/dev/mtdblock1";
  ret = setup_spiffs(path, mtd, CONFIG_ESP32S3_WIFI_FS_MOUNTPT, 0777);
  if (ret < 0)
    {
      ferr("Failed to setup spiffs\n");
      return ret;
    }

#else

    ferr("No supported FS selected. Wi-Fi partition "
         "should be mounted before Wi-Fi initialization\n");

#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: init_storage_partition
 *
 * Description:
 *   Initialize partition that is dedicated to general use.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int init_storage_partition(void)
{
  int ret = OK;
  struct mtd_dev_s *mtd;

  mtd = esp32s3_spiflash_alloc_mtdpart(CONFIG_ESP32S3_STORAGE_MTD_OFFSET,
                                       CONFIG_ESP32S3_STORAGE_MTD_SIZE,
                                       false);
  if (!mtd)
    {
      ferr("ERROR: Failed to alloc MTD partition of SPI Flash\n");
      return ERROR;
    }

#if defined (CONFIG_ESP32S3_SPIFLASH_SMARTFS)

  ret = setup_smartfs(0, mtd, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup smartfs\n");
      return ret;
    }

#elif defined (CONFIG_ESP32S3_SPIFLASH_NXFFS)

  ret = setup_nxffs(mtd, "/mnt");
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup nxffs\n");
      return ret;
    }

#elif defined (CONFIG_ESP32S3_SPIFLASH_LITTLEFS)

  const char *path = "/dev/esp32s3flash";
  ret = setup_littlefs(path, mtd, NULL, 0755);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup littlefs\n");
      return ret;
    }

#elif defined (CONFIG_ESP32S3_SPIFLASH_SPIFFS)

  const char *path = "/dev/esp32s3flash";
  ret = setup_spiffs(path, mtd, NULL, 0755);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup spiffs\n");
      return ret;
    }

#else

  ret = register_mtddriver("/dev/esp32s3flash", mtd, 0755, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register MTD: %d\n", ret);
      return ret;
    }

#endif

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spiflash_init
 *
 * Description:
 *   Initialize the SPIFLASH and register the MTD device.
 *
 ****************************************************************************/

int board_spiflash_init(void)
{
  int ret = OK;

  ret = esp32s3_spiflash_init();
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32S3_WIFI_SAVE_PARAM
  ret = init_wifi_partition();
  if (ret < 0)
    {
      return ret;
    }
#endif

  ret = init_storage_partition();
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

