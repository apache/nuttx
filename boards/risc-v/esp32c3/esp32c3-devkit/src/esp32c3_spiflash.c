/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/src/esp32c3_spiflash.c
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

#include "esp32c3_spiflash.h"
#include "esp32c3-devkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x)                (sizeof((x)) / sizeof((x)[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_HAVE_OTA_PARTITION

struct ota_partition_s
{
  uint32_t    offset;          /* Partition offset from the beginning of MTD */
  uint32_t    size;            /* Partition size in bytes */
  const char *devpath;         /* Partition device path */
};

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_HAVE_OTA_PARTITION
static int init_ota_partitions(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_HAVE_OTA_PARTITION
static const struct ota_partition_s g_ota_partition_table[] =
{
  {
    .offset  = CONFIG_ESP32C3_OTA_PRIMARY_SLOT_OFFSET,
    .size    = CONFIG_ESP32C3_OTA_SLOT_SIZE,
    .devpath = CONFIG_ESP32C3_OTA_PRIMARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_ESP32C3_OTA_SECONDARY_SLOT_OFFSET,
    .size    = CONFIG_ESP32C3_OTA_SLOT_SIZE,
    .devpath = CONFIG_ESP32C3_OTA_SECONDARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_ESP32C3_OTA_SCRATCH_OFFSET,
    .size    = CONFIG_ESP32C3_OTA_SCRATCH_SIZE,
    .devpath = CONFIG_ESP32C3_OTA_SCRATCH_DEVPATH
  }
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_ota_partitions
 *
 * Description:
 *   Initialize partitions that are dedicated to firmware OTA update.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_HAVE_OTA_PARTITION
static int init_ota_partitions(void)
{
  FAR struct mtd_dev_s *mtd;
#ifdef CONFIG_BCH
  char blockdev[18];
#endif
  int ret = OK;

  for (int i = 0; i < ARRAYSIZE(g_ota_partition_table); ++i)
    {
      const struct ota_partition_s *part = &g_ota_partition_table[i];
      mtd = esp32c3_spiflash_alloc_mtdpart(part->offset, part->size);

      ret = ftl_initialize(i, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#ifdef CONFIG_BCH
      snprintf(blockdev, 18, "/dev/mtdblock%d", i);

      ret = bchdev_register(blockdev, part->devpath, false);
      if (ret < 0)
        {
          ferr("ERROR: bchdev_register %s failed: %d\n", part->devpath, ret);
          return ret;
        }
#endif
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

#if defined (CONFIG_ESP32C3_SPIFLASH_SMARTFS)
static int setup_smartfs(int smartn, FAR struct mtd_dev_s *mtd,
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

#if defined (CONFIG_ESP32C3_SPIFLASH_LITTLEFS)
static int setup_littlefs(const char *path, FAR struct mtd_dev_s *mtd,
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

#if defined  (CONFIG_ESP32C3_SPIFLASH_SPIFFS)
static int setup_spiffs(const char *path, FAR struct mtd_dev_s *mtd,
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

#if defined (CONFIG_ESP32C3_SPIFLASH_NXFFS)
static int setup_nxffs(FAR struct mtd_dev_s *mtd, const char *mnt_pt)
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

#if defined (CONFIG_ESP32C3_WIFI_SAVE_PARAM)
static int init_wifi_partition(void)
{
  int ret = OK;
  FAR struct mtd_dev_s *mtd;

  mtd = esp32c3_spiflash_alloc_mtdpart(CONFIG_ESP32C3_WIFI_MTD_OFFSET,
                                       CONFIG_ESP32C3_WIFI_MTD_SIZE);
  if (!mtd)
    {
      ferr("ERROR: Failed to alloc MTD partition of SPI Flash\n");
      return ERROR;
    }

#if defined (CONFIG_ESP32C3_SPIFLASH_SMARTFS)

  ret = setup_smartfs(1, mtd, CONFIG_ESP32C3_WIFI_FS_MOUNTPT);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup smartfs\n");
      return ret;
    }

#elif defined (CONFIG_ESP32C3_SPIFLASH_LITTLEFS)

  const char *path = "/dev/mtdblock1";
  ret = setup_littlefs(path, mtd, CONFIG_ESP32C3_WIFI_FS_MOUNTPT, 0777);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup littlefs\n");
      return ret;
    }

#elif defined (CONFIG_ESP32C3_SPIFLASH_SPIFFS)

  const char *path = "/dev/mtdblock1";
  ret = setup_spiffs(path, mtd, CONFIG_ESP32C3_WIFI_FS_MOUNTPT, 0777);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup spiffs\n");
      return ret;
    }

#else

    ferr("ERROR: No supported FS selected. Wi-Fi partition "
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
  FAR struct mtd_dev_s *mtd;

  mtd = esp32c3_spiflash_alloc_mtdpart(CONFIG_ESP32C3_MTD_OFFSET,
                                       CONFIG_ESP32C3_MTD_SIZE);
  if (!mtd)
    {
      ferr("ERROR: Failed to alloc MTD partition of SPI Flash\n");
      return ERROR;
    }

#if defined (CONFIG_ESP32C3_SPIFLASH_SMARTFS)

  ret = setup_smartfs(0, mtd, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup smartfs\n");
      return ret;
    }

#elif defined (CONFIG_ESP32C3_SPIFLASH_NXFFS)

  ret = setup_nxffs(mtd, "/mnt");
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup nxffs\n");
      return ret;
    }

#elif defined (CONFIG_ESP32C3_SPIFLASH_LITTLEFS)

  const char *path = "/dev/esp32c3flash";
  ret = setup_littlefs(path, mtd, NULL, 0755);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup littlefs\n");
      return ret;
    }

#elif defined (CONFIG_ESP32C3_SPIFLASH_SPIFFS)

  const char *path = "/dev/esp32c3flash";
  ret = setup_spiffs(path, mtd, NULL, 0755);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup spiffs\n");
      return ret;
    }

#else

  ret = register_mtddriver("/dev/esp32c3flash", mtd, 0755, NULL);
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
 * Name: esp32c3_spiflash_init
 *
 * Description:
 *   Initialize the SPIFLASH and register the MTD device.
 ****************************************************************************/

int esp32c3_spiflash_init(void)
{
  int ret = OK;

#ifdef CONFIG_ESP32C3_HAVE_OTA_PARTITION
  ret = init_ota_partitions();
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32C3_WIFI_SAVE_PARAM
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

/****************************************************************************
 * Name: esp32c3_spiflash_encrypt_test
 *
 * Description:
 *   Test ESP32-C3 SPI Flash driver read/write with encryption.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_SPIFLASH_ENCRYPTION_TEST

void esp32c3_spiflash_encrypt_test(void)
{
  int i;
  int ret;
  uint8_t *wbuf;
  uint8_t *rbuf;
  struct mtd_geometry_s geo;
  uint32_t erase_block;
  uint32_t erase_nblocks;
  uint32_t rw_block;
  uint32_t rw_nblocks;
  struct mtd_dev_s *mtd = esp32c3_spiflash_mtd();
  struct mtd_dev_s *enc_mtd = esp32c3_spiflash_encrypt_mtd();
  const uint32_t address = CONFIG_ESP32C3_SPIFLASH_TEST_ADDRESS;
  const uint32_t size = 4096;

  ret = MTD_IOCTL(enc_mtd, MTDIOC_GEOMETRY,
                  (unsigned long)(uintptr_t)&geo);
  if (ret < 0)
    {
      ferr("ERROR: Failed to get GEO errno =%d\n", ret);
      DEBUGASSERT(0);
    }

  wbuf = kmm_malloc(size);
  if (!wbuf)
    {
      ferr("ERROR: Failed to alloc %" PRIu32 " heap\n", size);
      DEBUGASSERT(0);
    }

  rbuf = kmm_malloc(size);
  if (!rbuf)
    {
      ferr("ERROR: Failed to alloc %" PRIu32 " heap\n", size);
      DEBUGASSERT(0);
    }

  for (i = 0; i < size; i++)
    {
      wbuf[i] = (uint8_t)random();
    }

  erase_block = address / geo.erasesize;
  erase_nblocks = size / geo.erasesize;

  rw_block = address / geo.blocksize;
  rw_nblocks = size / geo.blocksize;

  ret = MTD_ERASE(enc_mtd, erase_block, erase_nblocks);
  if (ret != erase_nblocks)
    {
      ferr("ERROR: Failed to erase block errno=%d\n", ret);
      DEBUGASSERT(0);
    }

  ret = MTD_BWRITE(enc_mtd, rw_block, rw_nblocks, wbuf);
  if (ret != rw_nblocks)
    {
      ferr("ERROR: Failed to encrypt write errno=%d\n", ret);
      DEBUGASSERT(0);
    }

  memset(rbuf, 0, size);
  ret = MTD_BREAD(enc_mtd, rw_block, rw_nblocks, rbuf);
  if (ret != rw_nblocks)
    {
      ferr("ERROR: Failed to decrypt read errno=%d\n", ret);
      DEBUGASSERT(0);
    }

  if (memcmp(wbuf, rbuf, size))
    {
      ferr("ASSERT: Encrypted and decrypted data is not same\n");
      DEBUGASSERT(0);
    }

  memset(rbuf, 0, size);
  ret = MTD_BREAD(mtd, rw_block, rw_nblocks, rbuf);
  if (ret != rw_nblocks)
    {
      ferr("ERROR: Failed to read errno=%d\n", ret);
      DEBUGASSERT(0);
    }

  if (!memcmp(wbuf, rbuf, size))
    {
      ferr("ASSERT: Encrypted and normal data is same\n");
      DEBUGASSERT(0);
    }

  for (i = 0; i < size; i++)
    {
      wbuf[i] = (uint8_t)random();
    }

  ret = MTD_ERASE(enc_mtd, erase_block, erase_nblocks);
  if (ret != erase_nblocks)
    {
      ferr("ERROR: Failed to erase errno=%d\n", ret);
      DEBUGASSERT(0);
    }

  ret = MTD_BWRITE(mtd, rw_block, rw_nblocks, wbuf);
  if (ret != rw_nblocks)
    {
      ferr("ERROR: Failed to write errno=%d\n", ret);
      DEBUGASSERT(0);
    }

  memset(rbuf, 0, size);
  ret = MTD_BREAD(enc_mtd, rw_block, rw_nblocks, rbuf);
  if (ret != rw_nblocks)
    {
      ferr("ERROR: Failed to decrypt read errno=%d\n", ret);
      DEBUGASSERT(0);
    }

  if (!memcmp(wbuf, rbuf, size))
    {
      ferr("ASSERT: Normal and decrypted data is same\n");
      DEBUGASSERT(0);
    }

  memset(rbuf, 0, size);
  ret = MTD_BREAD(mtd, rw_block, rw_nblocks, rbuf);
  if (ret != rw_nblocks)
    {
      ferr("ERROR: Failed to read errno=%d\n", ret);
      DEBUGASSERT(0);
    }

  if (memcmp(wbuf, rbuf, size))
    {
      ferr("ASSERT: Normal and normal data is not same\n");
      DEBUGASSERT(0);
    }

  kmm_free(wbuf);
  kmm_free(rbuf);

  finfo("INFO: SPI Flash encryption test success\n");
}

#endif /* CONFIG_ESP32C3_SPIFLASH_ENCRYPTION_TEST */
