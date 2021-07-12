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

#include "esp32c3_spiflash.h"
#include "esp32c3-devkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_MTD_PATH      "/dev/esp32c3flash"

#define ESP32C3_FS_MOUNT_PT   CONFIG_ESP32C3_SPIFLASH_FS_MOUNT_PT

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
  FAR struct mtd_dev_s *mtd;
  int ret = ERROR;

  mtd = esp32c3_spiflash_alloc_mtdpart();

#if defined (CONFIG_ESP32C3_SPIFLASH_SMARTFS)
  ret = smart_initialize(0, mtd, NULL);
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
      ret = smart_initialize(0, mtd, NULL);
      if (ret < 0)
        {
          ferr("ERROR: smart_initialize failed: %d\n", ret);
          return ret;
        }
    }

#elif defined (CONFIG_ESP32C3_SPIFLASH_NXFFS)
  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS init failed: %d\n", ret);
      return ret;
    }

#elif defined (CONFIG_ESP32C3_SPIFLASH_LITTLEFS)
  ret = register_mtddriver(ESP32C3_MTD_PATH, mtd, 0755, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Register MTD failed: %d\n", ret);
      return ret;
    }

  ret = mount(ESP32C3_MTD_PATH, ESP32C3_FS_MOUNT_PT,
              "littlefs", 0, NULL);
  if (ret < 0)
    {
      ret = mount(ESP32C3_MTD_PATH, ESP32C3_FS_MOUNT_PT,
                  "littlefs", 0, "forceformat");
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the FS volume: %d\n",
                 errno);
          return ret;
        }
    }
#else
  ret = register_mtddriver("/dev/esp32c3flash", mtd, 0755, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Register MTD failed: %d\n", ret);
      return ret;
    }
#endif

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
