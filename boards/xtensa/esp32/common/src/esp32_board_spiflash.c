/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_spiflash.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <sys/param.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/partition.h>
#ifdef CONFIG_ESP32_SPIFLASH_NXFFS
#include <nuttx/fs/nxffs.h>
#endif

#include "esp32_spiflash.h"
#include "esp32_board_spiflash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32_OTA_PARTITION_ENCRYPT
#  define OTA_ENCRYPT true
#else
#  define OTA_ENCRYPT false
#endif

#ifdef CONFIG_ESP32_STORAGE_MTD_ENCRYPT
#  define STORAGE_ENCRYPT true
#else
#  define STORAGE_ENCRYPT false
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32_HAVE_OTA_PARTITION
static int init_ota_partitions(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_HAVE_OTA_PARTITION
static const struct partition_s g_ota_partition_table[] =
{
  {
    .name       = CONFIG_ESP32_OTA_PRIMARY_SLOT_DEVPATH,
    .index      = 0,
    .firstblock = CONFIG_ESP32_OTA_PRIMARY_SLOT_OFFSET,
    .blocksize  = CONFIG_ESP32_OTA_SLOT_SIZE,
  },
  {
    .name       = CONFIG_ESP32_OTA_SECONDARY_SLOT_DEVPATH,
    .index      = 1,
    .firstblock = CONFIG_ESP32_OTA_SECONDARY_SLOT_OFFSET,
    .blocksize  = CONFIG_ESP32_OTA_SLOT_SIZE,
  },
  {
    .name       = CONFIG_ESP32_OTA_SCRATCH_DEVPATH,
    .index      = 2,
    .firstblock = CONFIG_ESP32_OTA_SCRATCH_OFFSET,
    .blocksize  = CONFIG_ESP32_OTA_SCRATCH_SIZE,
  }
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESP32_HAVE_OTA_PARTITION
static int init_ota_partitions(void)
{
  struct mtd_dev_s *mtd;
  int ret = OK;

  for (int i = 0; i < nitems(g_ota_partition_table); ++i)
    {
      const struct partition_s *part = &g_ota_partition_table[i];
      mtd = esp32_spiflash_alloc_mtdpart(part->firstblock, part->blocksize,
                                         OTA_ENCRYPT);

      ret = register_mtddriver(part->name, mtd, 0755, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "register_mtddriver %s failed: %d\n",
                 part->name, ret);
          return ret;
        }
    }

  return ret;
}
#endif

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

#ifdef CONFIG_ESP32_SPIFLASH_SMARTFS
static int setup_smartfs(int smartn, struct mtd_dev_s *mtd,
                         const char *mnt_pt)
{
  int ret = OK;
  char path[22];

  ret = smart_initialize(smartn, mtd, NULL);
  if (ret < 0)
    {
      syslog(LOG_INFO, "smart_initialize failed, "
             "Trying to erase first...\n");
      ret = mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: ioctl(BULKERASE) failed: %d\n", ret);
          return ret;
        }

      syslog(LOG_INFO, "Erase successful, initializing it again.\n");
      ret = smart_initialize(smartn, mtd, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: smart_initialize failed: %d\n", ret);
          return ret;
        }
    }

  if (mnt_pt != NULL)
    {
      snprintf(path, sizeof(path), "/dev/smart%d", smartn);

      ret = nx_mount(path, mnt_pt, "smartfs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the FS volume: %d\n", ret);
          if (ret == -ENODEV)
            {
              syslog(LOG_WARNING, "Smartfs seems unformatted. "
                     "Did you run 'mksmartfs /dev/smart%d'?\n", smartn);
            }

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

#ifdef CONFIG_ESP32_SPIFLASH_LITTLEFS
static int setup_littlefs(const char *path, struct mtd_dev_s *mtd,
                          const char *mnt_pt, int priv)
{
  int ret = OK;

  ret = register_mtddriver(path, mtd, priv, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD: %d\n", ret);
      return -ENOMEM;
    }

  if (mnt_pt != NULL)
    {
      ret = nx_mount(path, mnt_pt, "littlefs", 0, NULL);
      if (ret < 0)
        {
          ret = nx_mount(path, mnt_pt, "littlefs", 0, "forceformat");
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to mount the FS volume: %d\n",
                     ret);
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

#ifdef CONFIG_ESP32_SPIFLASH_SPIFFS
static int setup_spiffs(const char *path, struct mtd_dev_s *mtd,
                        const char *mnt_pt, int priv)
{
  int ret = OK;

  ret = register_mtddriver(path, mtd, priv, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD: %d\n", ret);
      return -ENOMEM;
    }

  if (mnt_pt != NULL)
    {
      ret = nx_mount(path, mnt_pt, "spiffs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the FS volume: %d\n", ret);
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

#ifdef CONFIG_ESP32_SPIFLASH_NXFFS
static int setup_nxffs(struct mtd_dev_s *mtd, const char *mnt_pt)
{
  int ret = OK;

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: NXFFS init failed: %d\n", ret);
      return ret;
    }

  if (mnt_pt != NULL)
    {
      ret = nx_mount(NULL, mnt_pt, "nxffs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the FS volume: %d\n", ret);
          return ret;
        }
    }

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

  mtd = esp32_spiflash_alloc_mtdpart(CONFIG_ESP32_STORAGE_MTD_OFFSET,
                                     CONFIG_ESP32_STORAGE_MTD_SIZE,
                                     STORAGE_ENCRYPT);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to alloc MTD partition of SPI Flash\n");
      return -ENOMEM;
    }

#ifdef CONFIG_ESP32_SPIFLASH_SMARTFS

  ret = setup_smartfs(0, mtd, "/data");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup smartfs\n");
      return ret;
    }

#elif defined(CONFIG_ESP32_SPIFLASH_NXFFS)

  ret = setup_nxffs(mtd, "/data");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup nxffs\n");
      return ret;
    }

#elif defined(CONFIG_ESP32_SPIFLASH_LITTLEFS)

  const char *path = "/dev/esp32flash";
  ret = setup_littlefs(path, mtd, "/data", 0755);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup littlefs\n");
      return ret;
    }

#elif defined(CONFIG_ESP32_SPIFLASH_SPIFFS)

  const char *path = "/dev/esp32flash";
  ret = setup_spiffs(path, mtd, "/data", 0755);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup spiffs\n");
      return ret;
    }

#elif defined(CONFIG_MTD_NVBLK)

  ret = nvblk_initialize("/dev/mtdblock0", mtd,
                         CONFIG_MTD_NVBLK_DEFAULT_LBS,
                         CONFIG_MTD_NVBLK_DEFAULT_IOBS,
                         CONFIG_MTD_NVBLK_DEFAULT_SPEB);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup nvblk\n");
      return ret;
    }

#else

  ret = register_mtddriver("/dev/mtdblock0", mtd, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD mtdblock0: %d\n", ret);
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
 *   Initialize the SPI Flash and register the MTD.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int board_spiflash_init(void)
{
  int ret = OK;

  ret = esp32_spiflash_init();
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32_HAVE_OTA_PARTITION
  ret = init_ota_partitions();
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
