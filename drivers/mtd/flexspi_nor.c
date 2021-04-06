/****************************************************************************
 * drivers/mtd/flexspi_nor.c
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

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/flexspi.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Configuration ************************************************************/

#define NOR_PAGE_SIZE   0x0100U
#define NOR_SECTOR_SIZE 0x1000U

/* This type represents the state of the MTD device. The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct flexspi_nor_dev_s.
 */

struct flexspi_nor_dev_s
{
  struct mtd_dev_s mtd;
  FAR struct flexspi_dev_s *flexspi;   /* Saved FlexSPI interface instance */
  uint8_t *ahb_base;
  enum flexspi_port_e port;
  struct flexspi_device_config_s config;
};

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

static const uint32_t flexspi_nor_lut[][4] =
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

static int flexspi_nor_get_vendor_id(const struct flexspi_nor_dev_s *dev,
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

static int flexspi_nor_read_status(const struct flexspi_nor_dev_s *dev,
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

static int flexspi_nor_write_status(const struct flexspi_nor_dev_s *dev,
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

static int flexspi_nor_write_enable(const struct flexspi_nor_dev_s *dev)
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

static int flexspi_nor_erase_sector(const struct flexspi_nor_dev_s *dev,
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

static int flexspi_nor_erase_chip(const struct flexspi_nor_dev_s *dev)
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

static int flexspi_nor_page_program(const struct flexspi_nor_dev_s *dev,
    off_t offset, const void *buffer, size_t len)
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

static int flexspi_nor_wait_bus_busy(const struct flexspi_nor_dev_s *dev)
{
  uint32_t status = 0;
  int ret;

  do
    {
      ret = flexspi_nor_read_status(dev, &status);
      if (ret)
        {
          return ret;
        }
    }
  while (status & 1);

  return 0;
}

static int flexspi_nor_enable_quad_mode(const struct flexspi_nor_dev_s *dev)
{
  uint32_t status = 0x40;

  flexspi_nor_write_status(dev, &status);
  flexspi_nor_wait_bus_busy(dev);
  FLEXSPI_SOFTWARE_RESET(dev->flexspi);

  return 0;
}

static ssize_t flexspi_nor_read(FAR struct mtd_dev_s *dev,
                              off_t offset,
                              size_t nbytes,
                              FAR uint8_t *buffer)
{
  FAR struct flexspi_nor_dev_s *priv = (FAR struct flexspi_nor_dev_s *)dev;
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

static ssize_t flexspi_nor_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR uint8_t *buffer)
{
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = flexspi_nor_read(dev, startblock * NOR_PAGE_SIZE,
                       nblocks * NOR_PAGE_SIZE, buffer);
  if (nbytes > 0)
    {
      nbytes /= NOR_PAGE_SIZE;
    }

  return nbytes;
}

static ssize_t flexspi_nor_bwrite(FAR struct mtd_dev_s *dev,
                                  off_t startblock, size_t nblocks,
                                  FAR const uint8_t *buffer)
{
  FAR struct flexspi_nor_dev_s *priv = (FAR struct flexspi_nor_dev_s *)dev;
  size_t len = nblocks * NOR_PAGE_SIZE;
  off_t offset = startblock * NOR_PAGE_SIZE;
  uint8_t *src = (uint8_t *) buffer;
  uint8_t *dst = priv->ahb_base + startblock * NOR_PAGE_SIZE;
  int i;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (len)
    {
      i = MIN(NOR_PAGE_SIZE, len);
      flexspi_nor_write_enable(priv);
      flexspi_nor_page_program(priv, offset, src, i);
      flexspi_nor_wait_bus_busy(priv);
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

static int flexspi_nor_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks)
{
  FAR struct flexspi_nor_dev_s *priv = (FAR struct flexspi_nor_dev_s *)dev;
  size_t blocksleft = nblocks;
  uint8_t *dst = priv->ahb_base + startblock * NOR_SECTOR_SIZE;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      flexspi_nor_write_enable(priv);
      flexspi_nor_erase_sector(priv, startblock * NOR_SECTOR_SIZE);
      flexspi_nor_wait_bus_busy(priv);
      FLEXSPI_SOFTWARE_RESET(priv->flexspi);
      startblock++;
    }

#ifdef CONFIG_ARMV7M_DCACHE
  up_invalidate_dcache((uintptr_t)dst,
                       (uintptr_t)dst + nblocks * NOR_SECTOR_SIZE);
#endif

  return (int)nblocks;
}

static int flexspi_nor_ioctl(FAR struct mtd_dev_s *dev,
                           int cmd,
                           unsigned long arg)
{
  FAR struct flexspi_nor_dev_s *priv = (FAR struct flexspi_nor_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
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

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          flexspi_nor_write_enable(priv);
          flexspi_nor_erase_chip(priv);
          flexspi_nor_wait_bus_busy(priv);
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

static int flexspi_nor_init(const struct flexspi_nor_dev_s *dev)
{
  uint8_t vendor_id;
  struct flexspi_device_config_s device_config;

  if (dev->port >= FLEXSPI_PORT_COUNT)
    {
      return -EINVAL;
    }

  device_config.flexspi_root_clk = 120000000;
  device_config.flash_size = 8192;
  device_config.cs_interval_unit = FLEXSPI_CS_INTERVAL_UNIT1_SCK_CYCLE;
  device_config.cs_interval = 0;
  device_config.cs_hold_time = 3;
  device_config.cs_setup_time = 3;
  device_config.data_valid_time = 0;
  device_config.columnspace = 0;
  device_config.enable_word_address = 0;
  device_config.awr_seq_index = 0;
  device_config.awr_seq_number = 0;
  device_config.ard_seq_index = READ_FAST_QUAD_OUTPUT;
  device_config.ard_seq_number = 1;
  device_config.ahb_write_wait_unit = FLEXSPI_AHB_WRITE_WAIT_UNIT2_AHB_CYCLE;
  device_config.ahb_write_wait_interval = 0;

  FLEXSPI_SET_DEVICE_CONFIG(dev->flexspi,
                          (struct flexspi_device_config_s *) &device_config,
                            dev->port);
  FLEXSPI_UPDATE_LUT(dev->flexspi, 0, (const uint32_t *)flexspi_nor_lut,
                     sizeof(flexspi_nor_lut) / 4);
  FLEXSPI_SOFTWARE_RESET(dev->flexspi);

  if (flexspi_nor_get_vendor_id(dev, &vendor_id))
    {
      return -EIO;
    }

  if (flexspi_nor_enable_quad_mode(dev))
    {
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flexspi_nor_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for the FlexSPI-based NOR
 *   FLASH part.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *flexspi_nor_initialize(struct flexspi_dev_s *flexspi,
                                             bool unprotect)
{
  FAR struct flexspi_nor_dev_s *priv;
  int ret;

  finfo("flexspi: %p\n", flexspi);
  DEBUGASSERT(flexspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per FlexSPI
   * device (only because of the FLEXSPI_DEV_FLASH(0) definition) and so
   * would have to be extended to handle multiple FLASH parts on the same
   * FlexSPI bus.
   */

  priv = (FAR struct flexspi_nor_dev_s *)
         kmm_zalloc(sizeof(struct flexspi_nor_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = flexspi_nor_erase;
      priv->mtd.bread  = flexspi_nor_bread;
      priv->mtd.bwrite = flexspi_nor_bwrite;
      priv->mtd.read   = flexspi_nor_read;
      priv->mtd.ioctl  = flexspi_nor_ioctl;
      priv->mtd.name   = "flexspi_nor";
      priv->flexspi    = flexspi;
      priv->ahb_base = (uint8_t *) 0x60000000;
      priv->port = FLEXSPI_PORT_A1;

      ret = flexspi_nor_init(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR Unrecognized FlexSPI NOR device\n");
          goto errout_with_priv;
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;

errout_with_priv:
  kmm_free(priv);
  return NULL;
}
