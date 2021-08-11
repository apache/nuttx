/****************************************************************************
 * boards/arm/imxrt/imxrt1064-evk/src/imxrt_flexspi_nor.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "imxrt_flexspi.h"
#include "imxrt1064-evk.h"
#include "hardware/imxrt_pinmux.h"

#ifdef CONFIG_IMXRT_FLEXSPI

#define NOR_PAGE_SIZE   0x0100U
#define NOR_SECTOR_SIZE 0x1000U

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

enum
{
  /* SPI instructions */

  READ_ID,
  READ_STATUS_REG,
  WRITE_STATUS_REG,
  WRITE_ENABLE,
  ERASE_SECTOR,
  ERASE_CHIP,

  /* Quad SPI instructions */

  READ_FAST_QUAD_OUTPUT,
  PAGE_PROGRAM_QUAD_INPUT,
  ENTER_QPI,
};

static const uint32_t g_flexspi_nor_lut[][4] =
{
  [READ_ID] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x9f,
        FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_1PAD, 0x04),
  },

  [READ_STATUS_REG] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x05,
        FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_1PAD, 0x04),
  },

  [WRITE_STATUS_REG] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x01,
        FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_1PAD, 0x04),
  },

  [WRITE_ENABLE] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x06,
        FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
  },

  [ERASE_SECTOR] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x20,
        FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
  },

  [ERASE_CHIP] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0xc7,
        FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
  },

  [READ_FAST_QUAD_OUTPUT] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x6b,
        FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_DUMMY_SDR, FLEXSPI_4PAD, 0x08,
        FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_4PAD, 0x04),
  },

  [PAGE_PROGRAM_QUAD_INPUT] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x32,
        FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_4PAD, 0x04,
        FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
  },

  [ENTER_QPI] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x35,
        FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
  },
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* FlexSPI NOR device private data */

struct imxrt_flexspi_nor_dev_s
{
  struct mtd_dev_s mtd;
  struct flexspi_dev_s *flexspi;   /* Saved FlexSPI interface instance */
  uint8_t *ahb_base;
  enum flexspi_port_e port;
  struct flexspi_device_config_s *config;
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int imxrt_flexspi_nor_erase(struct mtd_dev_s *dev,
                                   off_t startblock,
                                   size_t nblocks);
static ssize_t imxrt_flexspi_nor_read(struct mtd_dev_s *dev,
                                      off_t offset,
                                      size_t nbytes,
                                      uint8_t *buffer);
static ssize_t imxrt_flexspi_nor_bread(struct mtd_dev_s *dev,
                                       off_t startblock,
                                       size_t nblocks,
                                       uint8_t *buffer);
static ssize_t imxrt_flexspi_nor_bwrite(struct mtd_dev_s *dev,
                                        off_t startblock,
                                        size_t nblocks,
                                        const uint8_t *buffer);
static int imxrt_flexspi_nor_ioctl(struct mtd_dev_s *dev,
                                   int cmd,
                                   unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct flexspi_device_config_s g_flexspi_device_config =
{
  .flexspi_root_clk = 120000000,
  .flash_size = 8192,
  .cs_interval_unit = FLEXSPI_CS_INTERVAL_UNIT1_SCK_CYCLE,
  .cs_interval = 0,
  .cs_hold_time = 3,
  .cs_setup_time = 3,
  .data_valid_time = 0,
  .columnspace = 0,
  .enable_word_address = 0,
  .awr_seq_index = 0,
  .awr_seq_number = 0,
  .ard_seq_index = READ_FAST_QUAD_OUTPUT,
  .ard_seq_number = 1,
  .ahb_write_wait_unit = FLEXSPI_AHB_WRITE_WAIT_UNIT2_AHB_CYCLE,
  .ahb_write_wait_interval = 0
};

static struct imxrt_flexspi_nor_dev_s g_flexspi_nor =
{
  .mtd =
          {
            .erase  = imxrt_flexspi_nor_erase,
            .bread  = imxrt_flexspi_nor_bread,
            .bwrite = imxrt_flexspi_nor_bwrite,
            .read   = imxrt_flexspi_nor_read,
            .ioctl  = imxrt_flexspi_nor_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
            .write  = NULL,
#endif
            .name   = "imxrt_flexspi_nor"
          },
  .flexspi = (void *)0,
  .ahb_base = (uint8_t *) 0x60000000,
  .port = FLEXSPI_PORT_A1,
  .config = &g_flexspi_device_config
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imxrt_flexspi_nor_get_vendor_id(
                const struct imxrt_flexspi_nor_dev_s *dev,
    uint8_t *vendor_id)
{
  uint32_t buffer = 0;
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = 0,
    .port = dev->port,
    .cmd_type = FLEXSPI_READ,
    .seq_number = 1,
    .seq_index = READ_ID,
    .data = &buffer,
    .data_size = 1,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  *vendor_id = buffer;

  return 0;
}

static int imxrt_flexspi_nor_read_status(
                const struct imxrt_flexspi_nor_dev_s *dev,
                uint32_t *status)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = 0,
    .port = dev->port,
    .cmd_type = FLEXSPI_READ,
    .seq_number = 1,
    .seq_index = READ_STATUS_REG,
    .data = status,
    .data_size = 1,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imxrt_flexspi_nor_write_status(
                const struct imxrt_flexspi_nor_dev_s *dev,
    uint32_t *status)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = 0,
    .port = dev->port,
    .cmd_type = FLEXSPI_WRITE,
    .seq_number = 1,
    .seq_index = WRITE_STATUS_REG,
    .data = status,
    .data_size = 1,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imxrt_flexspi_nor_write_enable(
                const struct imxrt_flexspi_nor_dev_s *dev)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = 0,
    .port = dev->port,
    .cmd_type = FLEXSPI_COMMAND,
    .seq_number = 1,
    .seq_index = WRITE_ENABLE,
    .data = NULL,
    .data_size = 0,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imxrt_flexspi_nor_erase_sector(
                const struct imxrt_flexspi_nor_dev_s *dev,
  off_t offset)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = offset,
    .port = dev->port,
    .cmd_type = FLEXSPI_COMMAND,
    .seq_number = 1,
    .seq_index = ERASE_SECTOR,
    .data = NULL,
    .data_size = 0,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imxrt_flexspi_nor_erase_chip(
                const struct imxrt_flexspi_nor_dev_s *dev)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = 0,
    .port = dev->port,
    .cmd_type = FLEXSPI_COMMAND,
    .seq_number = 1,
    .seq_index = ERASE_CHIP,
    .data = NULL,
    .data_size = 0,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imxrt_flexspi_nor_page_program(
                      const struct imxrt_flexspi_nor_dev_s *dev,
                      off_t offset,
                      const void *buffer,
                      size_t len)
{
  int stat;

  struct flexspi_transfer_s transfer =
  {
    .device_address = offset,
    .port = dev->port,
    .cmd_type = FLEXSPI_WRITE,
    .seq_number = 1,
    .seq_index = PAGE_PROGRAM_QUAD_INPUT,
    .data = (uint32_t *) buffer,
    .data_size = len,
  };

  stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);
  if (stat != 0)
    {
      return -EIO;
    }

  return 0;
}

static int imxrt_flexspi_nor_wait_bus_busy(
                const struct imxrt_flexspi_nor_dev_s *dev)
{
  uint32_t status = 0;
  int ret;

  do
    {
      ret = imxrt_flexspi_nor_read_status(dev, &status);
      if (ret)
        {
          return ret;
        }
    }
  while (status & 1);

  return 0;
}

static int imxrt_flexspi_nor_enable_quad_mode(
                const struct imxrt_flexspi_nor_dev_s *dev)
{
  uint32_t status = 0x40;

  imxrt_flexspi_nor_write_status(dev, &status);
  imxrt_flexspi_nor_wait_bus_busy(dev);
  FLEXSPI_SOFTWARE_RESET(dev->flexspi);

  return 0;
}

static ssize_t imxrt_flexspi_nor_read(struct mtd_dev_s *dev,
                                      off_t offset,
                                      size_t nbytes,
                                      uint8_t *buffer)
{
  struct imxrt_flexspi_nor_dev_s *priv =
                  (struct imxrt_flexspi_nor_dev_s *)dev;
  uint8_t *src;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  if (priv->port >= FLEXSPI_PORT_COUNT)
    {
      return -EIO;
    }

  src = priv->ahb_base + offset;

  memcpy(buffer, src, nbytes);

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

static ssize_t imxrt_flexspi_nor_bread(struct mtd_dev_s *dev,
                                       off_t startblock,
                                       size_t nblocks,
                                       uint8_t *buffer)
{
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = imxrt_flexspi_nor_read(dev, startblock * NOR_PAGE_SIZE,
                                  nblocks * NOR_PAGE_SIZE, buffer);
  if (nbytes > 0)
    {
      nbytes /= NOR_PAGE_SIZE;
    }

  return nbytes;
}

static ssize_t imxrt_flexspi_nor_bwrite(struct mtd_dev_s *dev,
                                        off_t startblock,
                                        size_t nblocks,
                                        const uint8_t *buffer)
{
  struct imxrt_flexspi_nor_dev_s *priv =
                  (struct imxrt_flexspi_nor_dev_s *)dev;
  size_t len = nblocks * NOR_PAGE_SIZE;
  off_t offset = startblock * NOR_PAGE_SIZE;
  uint8_t *src = (uint8_t *) buffer;
  uint8_t *dst = priv->ahb_base + startblock * NOR_PAGE_SIZE;
  int i;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (len)
    {
      i = MIN(NOR_PAGE_SIZE, len);
      imxrt_flexspi_nor_write_enable(priv);
      imxrt_flexspi_nor_page_program(priv, offset, src, i);
      imxrt_flexspi_nor_wait_bus_busy(priv);
      FLEXSPI_SOFTWARE_RESET(priv->flexspi);
      offset += i;
      len -= i;
    }

#ifdef CONFIG_ARMV7M_DCACHE
  up_invalidate_dcache((uintptr_t)dst,
                       (uintptr_t)dst + nblocks * NOR_PAGE_SIZE);
#endif

  return nblocks;
}

static int imxrt_flexspi_nor_erase(struct mtd_dev_s *dev,
                                   off_t startblock,
                                   size_t nblocks)
{
  struct imxrt_flexspi_nor_dev_s *priv =
                  (struct imxrt_flexspi_nor_dev_s *)dev;
  size_t blocksleft = nblocks;
  uint8_t *dst = priv->ahb_base + startblock * NOR_SECTOR_SIZE;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      imxrt_flexspi_nor_write_enable(priv);
      imxrt_flexspi_nor_erase_sector(priv, startblock * NOR_SECTOR_SIZE);
      imxrt_flexspi_nor_wait_bus_busy(priv);
      FLEXSPI_SOFTWARE_RESET(priv->flexspi);
      startblock++;
    }

#ifdef CONFIG_ARMV7M_DCACHE
  up_invalidate_dcache((uintptr_t)dst,
                       (uintptr_t)dst + nblocks * NOR_SECTOR_SIZE);
#endif

  return (int)nblocks;
}

static int imxrt_flexspi_nor_ioctl(struct mtd_dev_s *dev,
                                   int cmd,
                                   unsigned long arg)
{
  struct imxrt_flexspi_nor_dev_s *priv =
                  (struct imxrt_flexspi_nor_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo =
            (struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the
               * client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

              geo->blocksize    = (NOR_PAGE_SIZE);
              geo->erasesize    = (NOR_SECTOR_SIZE);
              geo->neraseblocks = 2048; /* 8MB only */

              ret               = OK;

              finfo("blocksize: %lu erasesize: %lu neraseblocks: %lu\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          struct partition_info_s *info =
            (struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = 32768; /* 8MB only */
              info->sectorsize  = NOR_PAGE_SIZE;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          imxrt_flexspi_nor_write_enable(priv);
          imxrt_flexspi_nor_erase_chip(priv);
          imxrt_flexspi_nor_wait_bus_busy(priv);
          FLEXSPI_SOFTWARE_RESET(priv->flexspi);
        }
        break;

      case MTDIOC_PROTECT:

        /* TODO */

        break;

      case MTDIOC_UNPROTECT:

        /* TODO */

        break;

      default:
        ret = -ENOTTY; /* Bad/unsupported command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_flexspi_nor_initialize
 *
 * Description:
 *  This function is called by board-bringup logic to configure the
 *  flash device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int imxrt_flexspi_nor_initialize(void)
{
  uint8_t vendor_id;
#ifdef CONFIG_FS_LITTLEFS
  struct mtd_dev_s *mtd_dev = &g_flexspi_nor.mtd;
  int ret = -1;
#endif
  /* Configure multiplexed pins as connected on the board */

  imxrt_config_gpio(GPIO_FLEXSPI_DQS);
  imxrt_config_gpio(GPIO_FLEXSPI_CS);
  imxrt_config_gpio(GPIO_FLEXSPI_IO0);
  imxrt_config_gpio(GPIO_FLEXSPI_IO1);
  imxrt_config_gpio(GPIO_FLEXSPI_IO2);
  imxrt_config_gpio(GPIO_FLEXSPI_IO3);
  imxrt_config_gpio(GPIO_FLEXSPI_SCK);

  g_flexspi_nor.flexspi = imxrt_flexspi_initialize(0);
  if (!g_flexspi_nor.flexspi)
    {
      return -1;
    }

  FLEXSPI_SET_DEVICE_CONFIG(g_flexspi_nor.flexspi,
                            g_flexspi_nor.config,
                            g_flexspi_nor.port);
  FLEXSPI_UPDATE_LUT(g_flexspi_nor.flexspi,
                     0,
                     (const uint32_t *)g_flexspi_nor_lut,
                     sizeof(g_flexspi_nor_lut) / 4);
  FLEXSPI_SOFTWARE_RESET(g_flexspi_nor.flexspi);

  if (imxrt_flexspi_nor_get_vendor_id(&g_flexspi_nor, &vendor_id))
    {
      return -EIO;
    }

  if (imxrt_flexspi_nor_enable_quad_mode(&g_flexspi_nor))
    {
      return -EIO;
    }

#ifdef CONFIG_FS_LITTLEFS
  /* Register the MTD driver so that it can be accessed from the
   * VFS.
   */

  ret = register_mtddriver("/dev/nor", mtd_dev, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
             ret);
    }

  /* mtd_dev->ioctl(mtd_dev, MTDIOC_BULKERASE, 0); */

  /* Mount the LittleFS file system */

  ret = nx_mount("/dev/nor", "/mnt/lfs", "littlefs", 0,
                 "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
             ret);
    }
#endif

  return 0;
}

#endif /* CONFIG_IMXRT_FLEXSPI */
