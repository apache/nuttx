/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_partition.c
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
#include <string.h>
#include <debug.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "esp32c3_spiflash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Partition table max size */

#define PARTITION_MAX_SIZE    (0xc00)

/* Partition table header magic value */

#define PARTITION_MAGIC       (0x50aa)

/* Partition table member label length */

#define PARTITION_LABEL_LEN   (16)

/* OTA data offset in OTA partition */

#define OTA_DATA_OFFSET       (4096)

/* OTA data number */

#define OTA_DATA_NUM          (2)

/* Partition offset in SPI Flash */

#define PARTITION_TABLE_OFFSET CONFIG_ESP32C3_PARTITION_TABLE_OFFSET

/* Partition MTD device mount point */

#define PARTITION_MOUNT_POINT CONFIG_ESP32C3_PARTITION_MOUNTPT

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* OTA image operation code */

enum ota_img_ctrl_e
{
  OTA_IMG_GET_BOOT        = 0xe1,
  OTA_IMG_SET_BOOT        = 0xe2
};

/* OTA image state */

enum ota_img_state_e
{
  /**
   *  Monitor the first boot. In bootloader of esp-idf this state is changed
   *  to ESP_OTA_IMG_PENDING_VERIFY if this bootloader enable app rollback.
   *
   *  So this driver doesn't use this state currently.
   */

  OTA_IMG_NEW             = 0x0,

  /**
   * First boot for this app was. If while the second boot this state is then
   * it will be changed to ABORTED if this bootloader enable app rollback.
   *
   * So this driver doesn't use this state currently.
   */

  OTA_IMG_PENDING_VERIFY  = 0x1,

  /* App was confirmed as workable. App can boot and work without limits. */

  OTA_IMG_VALID           = 0x2,

  /* App was confirmed as non-workable. This app will not selected to boot. */

  OTA_IMG_INVALID         = 0x3,

  /**
   * App could not confirm the workable or non-workable. In bootloader
   * IMG_PENDING_VERIFY state will be changed to IMG_ABORTED. This app will
   * not selected to boot at all if this bootloader enable app rollback.
   *
   * So this driver doesn't use this state currently.
   */

  OTA_IMG_ABORTED         = 0x4,

  /**
   * Undefined. App can boot and work without limits in esp-idf.
   *
   * This state is not used.
   */

  OTA_IMG_UNDEFINED       = 0xffffffff,
};

/* OTA image boot sequency */

enum ota_img_bootseq_e
{
  OTA_IMG_BOOT_FACTORY    = 0,
  OTA_IMG_BOOT_OTA_0      = 1,
  OTA_IMG_BOOT_OTA_1      = 2,
  OTA_IMG_BOOT_SEQ_MAX
};

/* Partition information data */

struct partition_info_priv_s
{
  uint16_t magic;                       /* Partition magic */
  uint8_t  type;                        /* Partition type */
  uint8_t  subtype;                     /* Partition sub-type */

  uint32_t offset;                      /* Offset in SPI Flash */
  uint32_t size;                        /* Size by byte */

  uint8_t  label[PARTITION_LABEL_LEN];  /* Partition label */

  uint32_t flags;                       /* Partition flags */
};

/* Partition device data */

struct mtd_dev_priv_s
{
  struct mtd_dev_s mtd;                 /* MTD data */

  uint8_t  type;                        /* Partition type */
  uint8_t  subtype;                     /* Partition sub-type */
  uint32_t flags;                       /* Partition flags */

  struct mtd_dev_s  *ll_mtd;            /* Low-level MTD data */
  struct mtd_dev_s  *part_mtd;          /* Partition MTD data */
};

/* OTA data entry */

struct ota_data_entry_s
{
  uint32_t ota_seq;                     /* Boot sequence */
  uint8_t  seq_label[20];               /* Boot sequence label */
  uint32_t ota_state;                   /* Boot entry state */
  uint32_t crc;                         /* Boot ota_seq CRC32 */
};

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

extern uint32_t crc32_le(uint32_t crc, uint8_t const *buf, uint32_t len);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ota_is_valid
 *
 * Description:
 *   Check if OTA data is valid
 *
 * Input Parameters:
 *   ota_data - OTA data
 *
 * Returned Value:
 *   true if checking success or false if fail
 *
 ****************************************************************************/

static bool ota_is_valid(struct ota_data_entry_s *ota_data)
{
  if ((ota_data->ota_seq >= OTA_IMG_BOOT_SEQ_MAX) ||
      (ota_data->ota_state != OTA_IMG_VALID) ||
      (ota_data->crc != crc32_le(UINT32_MAX, (uint8_t *)ota_data, 4)))
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: ota_get_bootseq
 *
 * Description:
 *   Get boot sequence
 *
 * Input Parameters:
 *   dev - Partition private MTD data
 *   num - boot sequence buffer
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int ota_get_bootseq(struct mtd_dev_priv_s *dev, int *num)
{
  int i;
  int ret;
  struct ota_data_entry_s ota_data;
  int size = sizeof(struct ota_data_entry_s);

  /* Each OTA data locates in independent sector */

  for (i = 0; i < OTA_DATA_NUM; i++)
    {
      ret = MTD_READ(dev->part_mtd, i * OTA_DATA_OFFSET,
                     size, (uint8_t *)&ota_data);
      if (ret != size)
        {
          ferr("ERROR: Failed to read OTA%d data error=%d\n", i, ret);
          return -EIO;
        }

      if (ota_is_valid(&ota_data))
        {
          *num = i + OTA_IMG_BOOT_OTA_0;
          break;
        }
    }

  if (i >= 2)
    {
      *num = OTA_IMG_BOOT_FACTORY;
    }

  return 0;
}

/****************************************************************************
 * Name: ota_set_bootseq
 *
 * Description:
 *   Set boot sequence
 *
 * Input Parameters:
 *   dev - Partition private MTD data
 *   num - boot sequence buffer
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int ota_set_bootseq(struct mtd_dev_priv_s *dev, int num)
{
  int ret;
  int id;
  int old_id;
  struct ota_data_entry_s ota_data;
  int size = sizeof(struct ota_data_entry_s);

  finfo("INFO: num=%d\n", num);

  switch (num)
    {
      case OTA_IMG_BOOT_FACTORY:

        /* Erase all OTA data to force use factory app */

        ret = MTD_ERASE(dev->part_mtd, 0, OTA_DATA_NUM);
        if (ret != OTA_DATA_NUM)
          {
            ferr("ERROR: Failed to erase OTA data error=%d\n", ret);
            return -EIO;
          }

        break;
      case OTA_IMG_BOOT_OTA_0:
      case OTA_IMG_BOOT_OTA_1:
        {
          id = num - 1;
          old_id = num == OTA_IMG_BOOT_OTA_0 ? OTA_IMG_BOOT_OTA_1 - 1:
                                               OTA_IMG_BOOT_OTA_0 - 1;

          ret = MTD_ERASE(dev->part_mtd, id, 1);
          if (ret != 1)
            {
              ferr("ERROR: Failed to erase OTA%d data error=%d\n", id, ret);
              return -EIO;
            }

          ota_data.ota_state = OTA_IMG_VALID;
          ota_data.ota_seq = num;
          ota_data.crc = crc32_le(UINT32_MAX, (uint8_t *)&ota_data, 4);
          ret = MTD_WRITE(dev->part_mtd, id * OTA_DATA_OFFSET,
                          size, (uint8_t *)&ota_data);
          if (ret != size)
            {
              ferr("ERROR: Failed to write OTA%d data error=%d\n",
                   id, ret);
              return -1;
            }

          /* Erase old OTA data to force new OTA bin */

          ret = MTD_ERASE(dev->part_mtd, old_id, 1);
          if (ret != 1)
            {
              ferr("ERROR: Failed to erase OTA%d data error=%d\n",
                   old_id, ret);
              return -EIO;
            }
        }

        break;
      default:
        ferr("ERROR: num=%d is error\n", num);
        return -EINVAL;
    }

    return 0;
}

/****************************************************************************
 * Name: esp32c3_part_erase
 *
 * Description:
 *   Erase SPI Flash designated sectors.
 *
 * Input Parameters:
 *   dev        - ESP32-C3 MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32c3_part_erase(struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks)
{
  struct mtd_dev_priv_s *mtd_priv = (struct mtd_dev_priv_s *)dev;

  return MTD_ERASE(mtd_priv->ll_mtd, startblock, nblocks);
}

/****************************************************************************
 * Name: esp32c3_part_read
 *
 * Description:
 *   Read data from SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - ESP32-C3 MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Read data bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_part_read(struct mtd_dev_s *dev, off_t offset,
                               size_t nbytes, uint8_t *buffer)
{
  struct mtd_dev_priv_s *mtd_priv = (struct mtd_dev_priv_s *)dev;

  return MTD_READ(mtd_priv->ll_mtd, offset, nbytes, buffer);
}

/****************************************************************************
 * Name: esp32c3_part_bread
 *
 * Description:
 *   Read data from designated blocks.
 *
 * Input Parameters:
 *   dev        - ESP32-C3 MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Read block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_part_bread(struct mtd_dev_s *dev,
                                  off_t startblock, size_t nblocks,
                                  uint8_t *buffer)
{
  struct mtd_dev_priv_s *mtd_priv = (struct mtd_dev_priv_s *)dev;

  return MTD_BREAD(mtd_priv->ll_mtd, startblock, nblocks, buffer);
}

/****************************************************************************
 * Name: esp32c3_part_write
 *
 * Description:
 *   write data to SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - ESP32-C3 MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Written bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_part_write(struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, const uint8_t *buffer)
{
  struct mtd_dev_priv_s *mtd_priv = (struct mtd_dev_priv_s *)dev;

  return MTD_WRITE(mtd_priv->ll_mtd, offset, nbytes, buffer);
}

/****************************************************************************
 * Name: esp32c3_part_bwrite
 *
 * Description:
 *   Write data to designated blocks.
 *
 * Input Parameters:
 *   dev        - ESP32-C3 MTD device data
 *   startblock - start MTD block number,
 *                it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Written block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32c3_part_bwrite(struct mtd_dev_s *dev,
                                   off_t startblock, size_t nblocks,
                                   const uint8_t *buffer)
{
  struct mtd_dev_priv_s *mtd_priv = (struct mtd_dev_priv_s *)dev;

  return MTD_BWRITE(mtd_priv->ll_mtd, startblock, nblocks, buffer);
}

/****************************************************************************
 * Name: esp32c3_part_ioctl
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

static int esp32c3_part_ioctl(struct mtd_dev_s *dev, int cmd,
                              unsigned long arg)
{
  int ret;
  struct mtd_dev_priv_s *mtd_priv = (struct mtd_dev_priv_s *)dev;

  finfo("INFO: cmd=%d(0x%x) arg=0x%" PRIx32 "\n", cmd, cmd, arg);

  switch (_IOC_NR(cmd))
    {
      case OTA_IMG_GET_BOOT:
        {
          int *num = (int *)arg;

          ret = ota_get_bootseq(mtd_priv, num);
          if (ret < 0)
            {
              ferr("ERROR: Failed to get boot img\n");
            }
        }

        break;
      case OTA_IMG_SET_BOOT:
        {
          ret = ota_set_bootseq(mtd_priv, arg);
          if (ret < 0)
            {
              ferr("ERROR: Failed to set boot img\n");
            }
        }

        break;
      default:
        {
          ret = MTD_IOCTL(mtd_priv->ll_mtd, cmd, arg);
        }

        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_partition_init
 *
 *   Initialize ESP32-C3 partition. Read partition information, and use
 *   these data for creating MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32c3_partition_init(void)
{
  int i;
  struct partition_info_priv_s *info;
  uint8_t *pbuf;
  struct mtd_dev_s *mtd;
  struct mtd_dev_s *mtd_part;
  struct mtd_geometry_s geo;
  struct mtd_dev_priv_s *mtd_priv;
  int ret = 0;
  const int num = PARTITION_MAX_SIZE / sizeof(struct partition_info_priv_s);
  const char path_base[] = PARTITION_MOUNT_POINT;
  char label[PARTITION_LABEL_LEN + 1];
  char path[PARTITION_LABEL_LEN + sizeof(path_base)];

  pbuf = kmm_malloc(PARTITION_MAX_SIZE);
  if (pbuf == NULL)
    {
      ferr("ERROR: Failed to allocate %d byte\n", PARTITION_MAX_SIZE);
      ret = -ENOMEM;
      goto errout_with_malloc;
    }

  mtd = esp32c3_spiflash_mtd();
  if (mtd == NULL)
    {
      ferr("ERROR: Failed to get SPI flash MTD\n");
      ret = -EIO;
      goto errout_with_mtd;
    }

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      ferr("ERROR: Failed to get info from MTD\n");
      ret = -EIO;
      goto errout_with_mtd;
    }

  ret = MTD_READ(mtd, PARTITION_TABLE_OFFSET, PARTITION_MAX_SIZE, pbuf);
  if (ret != PARTITION_MAX_SIZE)
    {
      ferr("ERROR: Failed to get read data from MTD\n");
      ret = -EIO;
      goto errout_with_mtd;
    }

  info = (struct partition_info_priv_s *)pbuf;

  for (i = 0; i < num; i++)
    {
      if (info->magic != PARTITION_MAGIC)
        {
          break;
        }

      strncpy(label, (char *)info->label, PARTITION_LABEL_LEN);
      label[PARTITION_LABEL_LEN] = '\0';
      sprintf(path, "%s%s", path_base, label);

      finfo("INFO: [label]:   %s\n", label);
      finfo("INFO: [type]:    %d\n", info->type);
      finfo("INFO: [subtype]: %d\n", info->subtype);
      finfo("INFO: [offset]:  0x%08" PRIx32 "\n", info->offset);
      finfo("INFO: [size]:    0x%08" PRIx32 "\n", info->size);
      finfo("INFO: [flags]:   0x%08" PRIx32 "\n", info->flags);
      finfo("INFO: [mount]:   %s\n", path);

      mtd_priv = kmm_malloc(sizeof(struct mtd_dev_priv_s));
      if (!mtd_priv)
        {
          ferr("ERROR: Failed to allocate %d byte\n",
               sizeof(struct mtd_dev_priv_s));
          ret = -1;
          goto errout_with_mtd;
        }

      mtd_priv->ll_mtd = mtd;
      mtd_priv->mtd.bread  = esp32c3_part_bread;
      mtd_priv->mtd.bwrite = esp32c3_part_bwrite;
      mtd_priv->mtd.erase  = esp32c3_part_erase;
      mtd_priv->mtd.ioctl  = esp32c3_part_ioctl;
      mtd_priv->mtd.read   = esp32c3_part_read;
      mtd_priv->mtd.write  = esp32c3_part_write;
      mtd_priv->mtd.name   = label;

      mtd_part = mtd_partition(&mtd_priv->mtd,
                               info->offset / geo.blocksize,
                               info->size / geo.blocksize);
      if (!mtd_part)
        {
          ferr("ERROR: Failed to create MTD partition\n");
          kmm_free(mtd_priv);
          ret = -1;
          goto errout_with_mtd;
        }

      mtd_priv->part_mtd = mtd_part;

      ret = register_mtddriver(path, mtd_part, 0777, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to regitser MTD @ %s\n", path);
          kmm_free(mtd_priv);
          ret = -1;
          goto errout_with_mtd;
        }

      info++;
    }

  ret = 0;

errout_with_mtd:
  kmm_free(pbuf);

errout_with_malloc:
  return ret;
}
