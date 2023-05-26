/****************************************************************************
 * arch/xtensa/src/esp32/esp32_partition.c
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
#include <sys/param.h>
#include <debug.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "esp32_spiflash.h"
#include "esp32_partition.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Partition table max size */

#define PARTITION_MAX_SIZE    (0xc00)

/* Partition max number */

#define PARTITION_MAX_NUM     (PARTITION_MAX_SIZE / \
                               sizeof(struct partition_info_priv))

/* Partition table header magic value */

#define PARTITION_MAGIC       (0x50aa)

/* Partition table member label length */

#define PARTITION_LABEL_LEN   (16)

/* OTA data offset in OTA partition */

#define OTA_DATA_OFFSET       (4096)

/* OTA data number */

#define OTA_DATA_NUM          (2)

/* Partition offset in SPI Flash */

#define PARTITION_TABLE_OFFSET CONFIG_ESP32_PARTITION_TABLE_OFFSET

/* Partition MTD device mount point */

#define PARTITION_MOUNT_POINT CONFIG_ESP32_PARTITION_MOUNTPT

/* Partition mount pointer max length */

#define PARTITION_MOUNTPTR_LEN_MAX        (PARTITION_LABEL_LEN + \
                                           sizeof(g_path_base))

/* Partition encrypted flag */

#define PARTITION_FLAG_ENCRYPTED          (1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* OTA image operation code */

enum ota_img_ctrl
{
  OTA_IMG_GET_BOOT        = 0xe1,
  OTA_IMG_SET_BOOT        = 0xe2,
  OTA_IMG_SET_ENCRYPTED   = 0xe3,
  OTA_IMG_GET_ENCRYPTED   = 0xe4,
  OTA_IMG_GET_TYPE        = 0xe5,
  OTA_IMG_GET_SUBTYPE     = 0xe6
};

/* OTA image state */

enum ota_img_state
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

enum ota_img_bootseq
{
  OTA_IMG_BOOT_FACTORY    = 0,
  OTA_IMG_BOOT_OTA_0      = 1,
  OTA_IMG_BOOT_OTA_1      = 2,
  OTA_IMG_BOOT_SEQ_MAX
};

/* Partition information data */

struct partition_info_priv
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

struct mtd_dev_priv
{
  struct mtd_dev_s mtd;                 /* MTD data */

  uint8_t  type;                        /* Partition type */
  uint8_t  subtype;                     /* Partition sub-type */
  uint32_t flags;                       /* Partition flags */
  uint32_t offset;                      /* Partition offset in SPI Flash */
  uint32_t size;                        /* Partition size in SPI Flash */

  struct mtd_dev_s *mtd_ll;             /* Low-level MTD data */

  struct mtd_dev_s *mtd_part;           /* MTD partition device */

  struct mtd_geometry_s   geo;          /* Partition geometry information */
};

/* OTA data entry */

struct ota_data_entry
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

const char g_path_base[] = PARTITION_MOUNT_POINT;

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

static bool ota_is_valid(struct ota_data_entry *ota_data)
{
  if ((ota_data->ota_seq == UINT32_MAX) ||
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
 *   Get boot app ID
 *
 * Input Parameters:
 *   dev    - Partition private MTD data
 *   seqptr - boot app data sequence buffer
 *
 * Returned Value:
 *   Booting APP ID(>= 0) if success or a negative value if fail.
 *
 ****************************************************************************/

static int ota_get_bootseq(struct mtd_dev_priv *dev, uint32_t *seqptr)
{
  int i;
  int ret;
  uint32_t seq = 0;
  struct ota_data_entry ota_data;
  int size = sizeof(struct ota_data_entry);

  /* Each OTA data locates in independent sector */

  for (i = 0; i < OTA_DATA_NUM; i++)
    {
      ret = MTD_READ(dev->mtd_part, i * dev->geo.erasesize,
                     size, (uint8_t *)&ota_data);
      if (ret != size)
        {
          ferr("ERROR: Failed to read OTA%d data error=%d\n", i, ret);
          return -EIO;
        }

      if (ota_is_valid(&ota_data))
        {
          seq = MAX(seq, ota_data.ota_seq);
        }
    }

  finfo("seq=%u\n", seq);

  if (seq > 0)
    {
      ret = (seq - 1) % OTA_DATA_NUM + OTA_IMG_BOOT_OTA_0;
    }
  else
    {
      ret = OTA_IMG_BOOT_FACTORY;
    }

  if (seqptr)
    {
      *seqptr = seq;
    }

  return ret;
}

/****************************************************************************
 * Name: ota_set_bootseq
 *
 * Description:
 *   Set boot sequence
 *
 * Input Parameters:
 *   dev - Partition private MTD data
 *   num - boot OTA sequence number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int ota_set_bootseq(struct mtd_dev_priv *dev, int num)
{
  int ret;
  int size;
  uint8_t *buffer;
  uint32_t sec;
  uint32_t blk;
  uint32_t blkcnt;
  uint32_t next_seq;
  struct ota_data_entry ota_data;

  finfo("INFO: num=%d\n", num);

  switch (num)
    {
      case OTA_IMG_BOOT_FACTORY:

        /* Erase all OTA data to force use factory app */

        ret = MTD_ERASE(dev->mtd_part, 0, OTA_DATA_NUM);
        if (ret != OTA_DATA_NUM)
          {
            ferr("ERROR: Failed to erase OTA data error=%d\n", ret);
            return -1;
          }

        break;
      case OTA_IMG_BOOT_OTA_0:
      case OTA_IMG_BOOT_OTA_1:
        ret = ota_get_bootseq(dev, &next_seq);
        if (ret < 0)
          {
            ferr("ERROR: Failed to get boot sequence error=%d\n", ret);
            return ret;
          }
        else if (ret == OTA_IMG_BOOT_FACTORY)
          {
            next_seq = 1;
          }
        else
          {
            next_seq++;
          }

        sec = num - OTA_IMG_BOOT_OTA_0;

        ret = MTD_ERASE(dev->mtd_part, sec, 1);
        if (ret != 1)
          {
            ferr("ERROR: Failed to erase OTA%d data error=%d\n", sec, ret);
            return -EIO;
          }

        ota_data.ota_state = OTA_IMG_VALID;
        ota_data.ota_seq   = next_seq;
        ota_data.crc       = crc32_le(UINT32_MAX, (uint8_t *)&ota_data, 4);

        if (dev->flags & PARTITION_FLAG_ENCRYPTED)
          {
            blkcnt = sizeof(struct ota_data_entry) / dev->geo.blocksize;
            size   = sizeof(struct ota_data_entry) % dev->geo.blocksize;
            if (size)
              {
                blkcnt++;
              }

            size = blkcnt * dev->geo.blocksize;
            buffer = kmm_malloc(size);
            if (!buffer)
              {
                ferr("ERROR:Failed to allocate %d bytes\n", size);
                return -ENOMEM;
              }

            memcpy(buffer, &ota_data, sizeof(struct ota_data_entry));

            blk = sec * dev->geo.erasesize / dev->geo.blocksize;
            ret = MTD_BWRITE(dev->mtd_part, blk, blkcnt, buffer);
            kmm_free(buffer);
            if (ret != blkcnt)
              {
                ferr("ERROR: Failed to write OTA%d data error=%d\n",
                      sec, ret);
                return -EIO;
              }
          }
        else
          {
            ret = MTD_WRITE(dev->mtd_part, sec * dev->geo.erasesize,
                            sizeof(struct ota_data_entry),
                            (uint8_t *)&ota_data);
            if (ret != sizeof(struct ota_data_entry))
              {
                ferr("ERROR: Failed to write OTA%d data error=%d\n",
                      sec, ret);
                return -1;
              }
          }

        break;
      default:
        ferr("ERROR: num=%d is error\n", num);
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_part_erase
 *
 * Description:
 *   Erase SPI Flash designated sectors.
 *
 * Input Parameters:
 *   dev        - ESP32 MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32_part_erase(struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks)
{
  struct mtd_dev_priv *mtd_priv = (struct mtd_dev_priv *)dev;

  return MTD_ERASE(mtd_priv->mtd_ll, startblock, nblocks);
}

/****************************************************************************
 * Name: esp32_part_read
 *
 * Description:
 *   Read data from SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - ESP32 MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Read data bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_part_read(struct mtd_dev_s *dev, off_t offset,
                               size_t nbytes, uint8_t *buffer)
{
  struct mtd_dev_priv *mtd_priv = (struct mtd_dev_priv *)dev;

  return MTD_READ(mtd_priv->mtd_ll, offset, nbytes, buffer);
}

/****************************************************************************
 * Name: esp32_part_bread
 *
 * Description:
 *   Read data from designated blocks.
 *
 * Input Parameters:
 *   dev        - ESP32 MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Read block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_part_bread(struct mtd_dev_s *dev, off_t startblock,
                                size_t nblocks, uint8_t *buffer)
{
  struct mtd_dev_priv *mtd_priv = (struct mtd_dev_priv *)dev;

  return MTD_BREAD(mtd_priv->mtd_ll, startblock, nblocks, buffer);
}

/****************************************************************************
 * Name: esp32_part_write
 *
 * Description:
 *   write data to SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - ESP32 MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Written bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_part_write(struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, const uint8_t *buffer)
{
  struct mtd_dev_priv *mtd_priv = (struct mtd_dev_priv *)dev;

  return MTD_WRITE(mtd_priv->mtd_ll, offset, nbytes, buffer);
}

/****************************************************************************
 * Name: esp32_part_bwrite
 *
 * Description:
 *   Write data to designated blocks.
 *
 * Input Parameters:
 *   dev        - ESP32 MTD device data
 *   startblock - start MTD block number,
 *                it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Written block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_part_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                 size_t nblocks, const uint8_t *buffer)
{
  struct mtd_dev_priv *mtd_priv = (struct mtd_dev_priv *)dev;

  return MTD_BWRITE(mtd_priv->mtd_ll, startblock, nblocks, buffer);
}

/****************************************************************************
 * Name: esp32_part_ioctl
 *
 * Description:
 *   Set/Get option to/from ESP32 SPI Flash MTD device data.
 *
 * Input Parameters:
 *   dev - ESP32 MTD device data
 *   cmd - operation command
 *   arg - operation argument
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32_part_ioctl(struct mtd_dev_s *dev, int cmd,
                            unsigned long arg)
{
  int ret = OK;
  struct mtd_dev_priv *mtd_priv = (struct mtd_dev_priv *)dev;

  finfo("INFO: cmd=%d(%x) arg=%lx\n", cmd, cmd, arg);

  switch (_IOC_NR(cmd))
    {
      case OTA_IMG_GET_BOOT:
        {
          ret = ota_get_bootseq(mtd_priv, NULL);
          if (ret < 0)
            {
              ferr("ERROR: Failed to get boot img\n");
            }
          else
            {
              *(int *)arg = ret;
            }
        }

        break;
      case OTA_IMG_SET_BOOT:
        {
          ret = ota_set_bootseq(mtd_priv, arg);
          if (ret)
            {
              ferr("ERROR: Failed to set boot img\n");
            }
        }

        break;
      case OTA_IMG_GET_ENCRYPTED:
        if (mtd_priv->flags & PARTITION_FLAG_ENCRYPTED)
          {
            *(int *)arg = 1;
          }
        else
          {
            *(int *)arg = 0;
          }

        break;
      case OTA_IMG_SET_ENCRYPTED:
        if (arg)
          {
            mtd_priv->flags |= PARTITION_FLAG_ENCRYPTED;
          }
        else
          {
            mtd_priv->flags &= ~PARTITION_FLAG_ENCRYPTED;
          }

        break;
      case OTA_IMG_GET_TYPE:
        *(int *)arg = mtd_priv->type;
        break;
      case OTA_IMG_GET_SUBTYPE:
        *(int *)arg = mtd_priv->subtype;
        break;
      default:
        {
          ret = MTD_IOCTL(mtd_priv->mtd_ll, cmd, arg);
        }

        break;
    }

  return ret;
}

/****************************************************************************
 * Name: partition_create_dev
 *
 * Description:
 *   Create Partition device by given data.
 *
 * Input Parameters:
 *   info        - ESP-IDF partition data information
 *   encrypt     - True: Enable SPI Flash encryption; False: Not encryption
 *   mtd         - MTD device pointer
 *   mtd_encrypt - Encryption MTD device pointer
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int partition_create_dev(const struct partition_info_priv *info,
                                bool encrypt,
                                struct mtd_dev_s *mtd,
                                struct mtd_dev_s *mtd_encrypt)
{
  int ret;
  uint32_t flags;
  struct mtd_dev_s *mtd_ll;
  struct mtd_dev_priv *mtd_priv;
  struct mtd_geometry_s geo;
  char path[PARTITION_MOUNTPTR_LEN_MAX];

  if (info->magic != PARTITION_MAGIC)
    {
      return -EINVAL;
    }

  snprintf(path, PARTITION_MOUNTPTR_LEN_MAX, "%s/%s",
           g_path_base, info->label);

  /**
   * If SPI Flash encryption is enable, "APP", "OTA data" and "NVS keys" are
   * force to set as encryption partition.
   */

  flags = info->flags;
  if (encrypt)
    {
      if ((info->type == PARTITION_TYPE_DATA &&
           info->subtype == PARTITION_SUBTYPE_DATA_OTA) ||
          (info->type == PARTITION_TYPE_DATA &&
           info->subtype == PARTITION_SUBTYPE_DATA_NVS_KEYS))
        {
          flags |= PARTITION_FLAG_ENCRYPTED;
        }
    }

  finfo("INFO: [label]:   %s\n", info->label);
  finfo("INFO: [type]:    %d\n", info->type);
  finfo("INFO: [subtype]: %d\n", info->subtype);
  finfo("INFO: [offset]:  0x%08x\n", info->offset);
  finfo("INFO: [size]:    0x%08x\n", info->size);
  finfo("INFO: [flags]:   0x%08x\n", info->flags);
  finfo("INFO: [mount]:   %s\n", path);
  if (flags & PARTITION_FLAG_ENCRYPTED)
    {
      mtd_ll = mtd_encrypt;
      finfo("INFO: [encrypted]\n\n");
    }
  else
    {
      mtd_ll = mtd;
      finfo("INFO: [no-encrypted]\n\n");
    }

  ret = MTD_IOCTL(mtd_ll, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      ferr("ERROR: Failed to get GEOMETRY from mtd_ll\n");
      return ret;
    }

  mtd_priv = kmm_malloc(sizeof(struct mtd_dev_priv));
  if (!mtd_priv)
    {
      ferr("ERROR: Failed to allocate %d byte\n",
           sizeof(struct mtd_dev_priv));
      return -ENOMEM;
    }

  mtd_priv->offset  = info->offset;
  mtd_priv->size    = info->size;
  mtd_priv->type    = info->type;
  mtd_priv->subtype = info->subtype;
  mtd_priv->flags   = flags;
  mtd_priv->mtd_ll  = mtd_ll;
  memcpy(&mtd_priv->geo, &geo, sizeof(geo));

  mtd_priv->mtd.bread  = esp32_part_bread;
  mtd_priv->mtd.bwrite = esp32_part_bwrite;
  mtd_priv->mtd.erase  = esp32_part_erase;
  mtd_priv->mtd.ioctl  = esp32_part_ioctl;
  mtd_priv->mtd.read   = esp32_part_read;
  mtd_priv->mtd.write  = esp32_part_write;
  mtd_priv->mtd.name   = mtd_priv->mtd_ll->name;
  mtd_priv->mtd_part   = mtd_partition(&mtd_priv->mtd,
                                       info->offset / geo.blocksize,
                                       info->size / geo.blocksize);
  if (!mtd_priv->mtd_part)
    {
      ferr("ERROR: Failed to create MTD partition\n");
      kmm_free(mtd_priv);
      return -ENOSPC;
    }

  ret = register_mtddriver(path, mtd_priv->mtd_part, 0777, mtd_priv);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register MTD @ %s\n", path);
      kmm_free(mtd_priv);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: partition_get_offset
 *
 * Description:
 *   Get offset in SPI flash of the partition label
 *
 * Input Parameters:
 *   label - Partition label
 *   size  - Data number
 *
 * Returned Value:
 *   Get partition offset(>= 0) if success or a negative value if fail.
 *
 ****************************************************************************/

static int partition_get_offset(const char *label, size_t size)
{
  int i;
  int ret;
  uint8_t *pbuf;
  int partion_offset;
  const struct partition_info_priv *info;
  DEBUGASSERT(label != NULL);
  struct mtd_dev_s *mtd = esp32_spiflash_get_mtd();
  if (!mtd)
    {
      ferr("ERROR: Failed to get SPI flash MTD\n");
      return -ENOSYS;
    }

  pbuf = kmm_malloc(PARTITION_MAX_SIZE);
  if (!pbuf)
    {
      ferr("ERROR: Failed to allocate %d byte\n", PARTITION_MAX_SIZE);
      return -ENOMEM;
    }

  ret = MTD_READ(mtd, PARTITION_TABLE_OFFSET,
                 PARTITION_MAX_SIZE, pbuf);
  if (ret != PARTITION_MAX_SIZE)
    {
      ferr("ERROR: Failed to get read data from MTD\n");
      kmm_free(pbuf);
      return -EIO;
    }

  info = (struct partition_info_priv *)pbuf;
  for (i = 0; i < PARTITION_MAX_NUM; i++)
    {
      if (memcmp(info[i].label, label, size) == 0)
        {
          partion_offset = info[i].offset;
          break;
        }
    }

  kmm_free(pbuf);
  if (i == PARTITION_MAX_NUM)
    {
      ferr("ERROR: No %s  partition is created\n", label);
      return -EPERM;
    }

  finfo("Get Partition offset: 0x%x\n", partion_offset);
  return partion_offset;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_partition_init
 *
 * Description:
 *   Initialize ESP32 partition. Read partition information of esp-idf,
 *   and create MTD by these data
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32_partition_init(void)
{
  int i;
  int ret;
  uint8_t *pbuf;
  bool encrypt;
  struct mtd_dev_s *mtd;
  struct mtd_dev_s *mtd_encrypt;
  const struct partition_info_priv *info;

  mtd = esp32_spiflash_get_mtd();
  if (!mtd)
    {
      ferr("ERROR: Failed to get SPI flash MTD\n");
      return -ENOSYS;
    }

  mtd_encrypt = esp32_spiflash_encrypt_get_mtd();
  if (!mtd_encrypt)
    {
      ferr("ERROR: Failed to get SPI flash encrypted MTD\n");
      return -ENOSYS;
    }

  pbuf = kmm_malloc(PARTITION_MAX_SIZE);
  if (!pbuf)
    {
      ferr("ERROR: Failed to allocate %d byte\n", PARTITION_MAX_SIZE);
      return -ENOMEM;
    }

  /**
   * Even without SPI Flash encryption, we can also use encrypted
   * MTD to read no-encrypted data.
   */

  ret = MTD_READ(mtd_encrypt, PARTITION_TABLE_OFFSET,
                 PARTITION_MAX_SIZE, pbuf);
  if (ret != PARTITION_MAX_SIZE)
    {
      ferr("ERROR: Failed to get read data from MTD\n");
      kmm_free(pbuf);
      return -EIO;
    }

  info = (struct partition_info_priv *)pbuf;
  encrypt = esp32_flash_encryption_enabled();

  for (i = 0; i < PARTITION_MAX_NUM; i++)
    {
      ret = partition_create_dev(&info[i], encrypt, mtd, mtd_encrypt);
      if (ret != OK)
        {
          break;
        }
    }

  kmm_free(pbuf);

  if (i == 0)
    {
      ferr("ERROR: No partition is created\n");
      return -EPERM;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_partition_read
 *
 * Description:
 *   Read data from SPI Flash at designated address.
 *
 * Input Parameters:
 *   label  - Partition label
 *   offset - Offset in SPI Flash
 *   buf    - Data buffer pointer
 *   size   - Data number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32_partition_read(const char *label, size_t offset, void *buf,
                         size_t size)
{
  int ret;
  int partion_offset;
  DEBUGASSERT(label != NULL && buf != NULL);
  struct mtd_dev_s *mtd = esp32_spiflash_get_mtd();
  if (!mtd)
    {
      ferr("ERROR: Failed to get SPI flash MTD\n");
      return -ENOSYS;
    }

  partion_offset = partition_get_offset(label, sizeof(label));
  if (partion_offset < 0)
    {
      ferr("ERROR: Failed to get partition: %s offset\n", label);
      return partion_offset;
    }

  ret = MTD_READ(mtd, partion_offset + offset,
                 size, (uint8_t *)buf);
  if (ret != size)
    {
      ferr("ERROR: Failed to get read data from MTD\n");
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_partition_write
 *
 * Description:
 *   Write data to SPI Flash at designated address.
 *
 * Input Parameters:
 *   label  - Partition label
 *   offset - Offset in SPI Flash
 *   buf    - Data buffer pointer
 *   size   - Data number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32_partition_write(const char *label, size_t offset, void *buf,
                          size_t size)
{
  int ret;
  int partion_offset;
  DEBUGASSERT(label != NULL && buf != NULL);
  struct mtd_dev_s *mtd = esp32_spiflash_get_mtd();
  if (!mtd)
    {
      ferr("ERROR: Failed to get SPI flash MTD\n");
      return -ENOSYS;
    }

  partion_offset = partition_get_offset(label, sizeof(label));
  if (partion_offset < 0)
    {
      ferr("ERROR: Failed to get partition: %s offset\n", label);
      return partion_offset;
    }

  ret = MTD_WRITE(mtd, partion_offset + offset,
                  size, (uint8_t *)buf);
  if (ret != size)
    {
      ferr("ERROR: Failed to get read data from MTD\n");
      return -EIO;
    }

  return OK;
}
