/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_spiflash.c
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

#include <stdint.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/semaphore.h>
#include <nuttx/mtd/mtd.h>

#include "esp32c3_spiflash.h"
#include "rom/esp32c3_spiflash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_FLASH_BLK_SIZE          256
#define SPI_FLASH_ERASE_SIZE        4096
#define SPI_FLASH_SIZE              (4 * 1024 * 1024)

#define ESP32C3_MTD_OFFSET          CONFIG_ESP32C3_MTD_OFFSET
#define ESP32C3_MTD_SIZE            CONFIG_ESP32C3_MTD_SIZE

#define MTD2PRIV(_dev)              ((FAR struct esp32c3_spiflash_s *)_dev)
#define MTD_SIZE(_priv)             ((_priv)->chip->chip_size)
#define MTD_BLKSIZE(_priv)          ((_priv)->chip->page_size)
#define MTD_ERASESIZE(_priv)        ((_priv)->chip->sector_size)
#define MTD_BLK2SIZE(_priv, _b)     (MTD_BLKSIZE(_priv) * (_b))
#define MTD_SIZE2BLK(_priv, _s)     ((_s) / MTD_BLKSIZE(_priv))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ESP32-C3 SPI Flash device private data  */

struct esp32c3_spiflash_s
{
  struct mtd_dev_s mtd;

  /* SPI Flash data */

  esp32c3_spiflash_chip_t *chip;
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int esp32c3_erase(struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks);
static ssize_t esp32c3_read(struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, uint8_t *buffer);
static ssize_t esp32c3_read_decrypt(struct mtd_dev_s *dev,
                                    off_t offset,
                                    size_t nbytes,
                                    uint8_t *buffer);
static ssize_t esp32c3_bread(struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, uint8_t *buffer);
static ssize_t esp32c3_bread_decrypt(struct mtd_dev_s *dev,
                                     off_t startblock,
                                     size_t nblocks,
                                     uint8_t *buffer);
static ssize_t esp32c3_write(struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, const uint8_t *buffer);
static ssize_t esp32c3_bwrite(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, const uint8_t *buffer);
static ssize_t esp32c3_bwrite_encrypt(struct mtd_dev_s *dev,
                                      off_t startblock,
                                      size_t nblocks,
                                      const uint8_t *buffer);
static int esp32c3_ioctl(struct mtd_dev_s *dev, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32c3_spiflash_s g_esp32c3_spiflash =
{
  .mtd =
          {
            .erase  = esp32c3_erase,
            .bread  = esp32c3_bread,
            .bwrite = esp32c3_bwrite,
            .read   = esp32c3_read,
            .ioctl  = esp32c3_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
            .write  = esp32c3_write,
#endif
            .name   = "esp32c3_spiflash"
          },
  .chip = &g_rom_flashchip,
};

static struct esp32c3_spiflash_s g_esp32c3_spiflash_encrypt =
{
  .mtd =
          {
            .erase  = esp32c3_erase,
            .bread  = esp32c3_bread_decrypt,
            .bwrite = esp32c3_bwrite_encrypt,
            .read   = esp32c3_read_decrypt,
            .ioctl  = esp32c3_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
            .write  = NULL,
#endif
            .name   = "esp32c3_spiflash_encrypt"
          }
};

/* Ensure exclusive access to the driver */

static sem_t g_exclsem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_erase
 *
 * Description:
 *   Erase SPI Flash designated sectors.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - Number of blocks
 *
 * Returned Value:
 *   Erased blocks if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32c3_erase(struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks)
{
  ssize_t ret;
  uint32_t offset = startblock * SPI_FLASH_ERASE_SIZE;
  uint32_t nbytes = nblocks * SPI_FLASH_ERASE_SIZE;

  if ((offset > SPI_FLASH_SIZE) || ((offset + nbytes) > SPI_FLASH_SIZE))
    {
      return -EINVAL;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("(%p, %d, %d)\n", dev, startblock, nblocks);
#endif

  ret = nxsem_wait(&g_exclsem);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("spi_flash_erase_range(%p, 0x%x, %d)\n", dev, offset, nbytes);
#endif

  ret = spi_flash_erase_range(offset, nbytes);

  nxsem_post(&g_exclsem);

  if (ret == OK)
    {
      ret = nblocks;
    }
  else
    {
#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
      finfo("Failed to erase the flash range!\n");
#endif
      ret = -1;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32c3_read
 *
 * Description:
 *   Read data from SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Read data bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_read(struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, uint8_t *buffer)
{
  ssize_t ret;

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s(%p, 0x%x, %d, %p)\n", __func__, dev, offset, nbytes, buffer);
#endif

  /* Acquire the semaphore. */

  ret = nxsem_wait(&g_exclsem);
  if (ret < 0)
    {
      goto error_with_buffer;
    }

  ret = spi_flash_read(offset, buffer, nbytes);

  nxsem_post(&g_exclsem);

  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

error_with_buffer:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_bread
 *
 * Description:
 *   Read data from designated blocks.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Read block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_bread(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, uint8_t *buffer)
{
  ssize_t ret;
  uint32_t addr = startblock * SPI_FLASH_BLK_SIZE;
  uint32_t size = nblocks * SPI_FLASH_BLK_SIZE;

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s(%p, 0x%x, %d, %p)\n", __func__, dev, startblock, nblocks,
        buffer);
#endif

  ret = nxsem_wait(&g_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = spi_flash_read(addr, buffer, size);

  nxsem_post(&g_exclsem);

  if (ret == OK)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32c3_read_decrypt
 *
 * Description:
 *   Read encrypted data and decrypt automatically from SPI Flash
 *   at designated address.
 *
 * Input Parameters:
 *   dev    - MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Read data bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_read_decrypt(struct mtd_dev_s *dev,
                                  off_t offset,
                                  size_t nbytes,
                                  uint8_t *buffer)
{
  ssize_t ret;

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s(%p, 0x%x, %d, %p)\n", __func__, dev, offset, nbytes, buffer);
#endif

  /* Acquire the semaphore. */

  ret = nxsem_wait(&g_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = spi_flash_read_encrypted(offset, buffer, nbytes);

  nxsem_post(&g_exclsem);

  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32c3_bread_decrypt
 *
 * Description:
 *   Read encrypted data and decrypt automatically from designated blocks.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Read block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_bread_decrypt(struct mtd_dev_s *dev,
                                   off_t startblock,
                                   size_t nblocks,
                                   uint8_t *buffer)
{
  ssize_t ret;
  uint32_t addr = startblock * SPI_FLASH_BLK_SIZE;
  uint32_t size = nblocks * SPI_FLASH_BLK_SIZE;

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s(%p, 0x%x, %d, %p)\n", __func__, dev, startblock, nblocks,
        buffer);
#endif

  ret = nxsem_wait(&g_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = spi_flash_read_encrypted(addr, buffer, size);

  nxsem_post(&g_exclsem);

  if (ret == OK)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32c3_write
 *
 * Description:
 *   write data to SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Writen bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_write(struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, const uint8_t *buffer)
{
  int ret;

  ASSERT(buffer);

  if ((offset > SPI_FLASH_SIZE) || ((offset + nbytes) > SPI_FLASH_SIZE))
    {
      return -EINVAL;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s(%p, 0x%x, %d, %p)\n", __func__, dev, offset, nbytes, buffer);
#endif

  /* Acquire the semaphore. */

  ret = nxsem_wait(&g_exclsem);
  if (ret < 0)
    {
      goto error_with_buffer;
    }

  ret = spi_flash_write(offset, buffer, nbytes);

  nxsem_post(&g_exclsem);

  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

error_with_buffer:

  return (ssize_t)ret;
}

/****************************************************************************
 * Name: esp32c3_bwrite
 *
 * Description:
 *   Write data to designated blocks.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start MTD block number,
 *                it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Writen block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_bwrite(struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, const uint8_t *buffer)
{
  ssize_t ret;
  uint32_t addr = startblock * SPI_FLASH_BLK_SIZE;
  uint32_t size = nblocks * SPI_FLASH_BLK_SIZE;

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s(%p, 0x%x, %d, %p)\n", __func__, dev, startblock,
        nblocks, buffer);
#endif

  ret = nxsem_wait(&g_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = spi_flash_write(addr, buffer, size);

  nxsem_post(&g_exclsem);

  if (ret == OK)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32c3_bwrite_encrypt
 *
 * Description:
 *   Write data to designated blocks by SPI Flash hardware encryption.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start MTD block number,
 *                it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Writen block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_bwrite_encrypt(struct mtd_dev_s *dev,
                                      off_t startblock,
                                      size_t nblocks,
                                      const uint8_t *buffer)
{
  ssize_t ret;
  uint32_t addr = startblock * SPI_FLASH_BLK_SIZE;
  uint32_t size = nblocks * SPI_FLASH_BLK_SIZE;

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s(%p, 0x%x, %d, %p)\n", __func__, dev, startblock,
        nblocks, buffer);
#endif

  ret = nxsem_wait(&g_exclsem);
  if (ret < 0)
    {
      goto error_with_buffer;
    }

  ret = spi_flash_write_encrypted(addr, buffer, size);

  nxsem_post(&g_exclsem);

  if (ret == OK)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32C3_SPIFLASH_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

error_with_buffer:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_ioctl
 *
 * Description:
 *   Set/Get option to/from ESP32-C3 SPI Flash MTD device data.
 *
 * Input Parameters:
 *   dev - ESP32-C3 MTD device data
 *   cmd - operation command
 *   arg - operation argument
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32c3_ioctl(struct mtd_dev_s *dev, int cmd,
                         unsigned long arg)
{
  int ret = OK;
  struct mtd_geometry_s *geo;

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          geo = (struct mtd_geometry_s *)arg;
          if (geo)
            {
              geo->blocksize    = SPI_FLASH_BLK_SIZE;
              geo->erasesize    = SPI_FLASH_ERASE_SIZE;
              geo->neraseblocks = SPI_FLASH_SIZE / SPI_FLASH_ERASE_SIZE;
              ret               = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32 \
                    " neraseblocks: %" PRId32 "\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_spiflash_alloc_mtdpart
 *
 * Description:
 *   Allocate SPI Flash MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   SPI Flash MTD data pointer if success or NULL if fail.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *esp32c3_spiflash_alloc_mtdpart(void)
{
  struct esp32c3_spiflash_s *priv = &g_esp32c3_spiflash;
  esp32c3_spiflash_chip_t *chip = priv->chip;
  FAR struct mtd_dev_s *mtd_part;
  uint32_t blocks;
  uint32_t startblock;
  uint32_t size;

  ASSERT((ESP32C3_MTD_OFFSET + ESP32C3_MTD_SIZE) <= chip->chip_size);
  ASSERT((ESP32C3_MTD_OFFSET % chip->sector_size) == 0);
  ASSERT((ESP32C3_MTD_SIZE % chip->sector_size) == 0);

  finfo("ESP32 SPI Flash information:\n");
  finfo("\tID = 0x%" PRIx32 "\n", chip->device_id);
  finfo("\tStatus mask = 0x%" PRIx32 "\n", chip->status_mask);
  finfo("\tChip size = %" PRId32 " KB\n", chip->chip_size / 1024);
  finfo("\tPage size = %" PRId32 " B\n", chip->page_size);
  finfo("\tSector size = %" PRId32 " KB\n", chip->sector_size / 1024);
  finfo("\tBlock size = %" PRId32 " KB\n", chip->block_size / 1024);

#if ESP32C3_MTD_SIZE == 0
  size = chip->chip_size - ESP32C3_MTD_OFFSET;
#else
  size = ESP32C3_MTD_SIZE;
#endif

  finfo("\tMTD offset = 0x%x\n", ESP32C3_MTD_OFFSET);
  finfo("\tMTD size = 0x%" PRIx32 "\n", size);

  startblock = MTD_SIZE2BLK(priv, ESP32C3_MTD_OFFSET);
  blocks = MTD_SIZE2BLK(priv, size);

  mtd_part = mtd_partition(&priv->mtd, startblock, blocks);
  if (!mtd_part)
    {
      ferr("ERROR: Failed to create MTD partition\n");
      return NULL;
    }

  return mtd_part;
}

/****************************************************************************
 * Name: esp32c3_spiflash_mtd
 *
 * Description:
 *   Get SPI Flash MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32-C3 SPI Flash MTD pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32c3_spiflash_mtd(void)
{
  struct esp32c3_spiflash_s *priv = &g_esp32c3_spiflash;

  return &priv->mtd;
}

/****************************************************************************
 * Name: esp32c3_spiflash_encrypt_mtd
 *
 * Description:
 *   Get SPI Flash encryption MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   SPI Flash encryption MTD pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32c3_spiflash_encrypt_mtd(void)
{
  struct esp32c3_spiflash_s *priv = &g_esp32c3_spiflash_encrypt;

  return &priv->mtd;
}
