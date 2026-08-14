/****************************************************************************
 * drivers/mtd/mx25uw25645g.c
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

#include <errno.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/qspi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI commands *************************************************************/

#define MX25UW_RDID             0x9f
#define MX25UW_RDSR             0x05
#define MX25UW_WREN             0x06
#define MX25UW_RDCR2            0x71
#define MX25UW_WRCR2            0x72
#define MX25UW_RSTEN            0x66
#define MX25UW_RST              0x99

/* Octal commands ***********************************************************/

#define MX25UW_OCTA_READ        0xec13
#define MX25UW_OCTA_PP          0x12ed
#define MX25UW_OCTA_SE          0x21de
#define MX25UW_OCTA_CE          0x609f
#define MX25UW_OCTA_RDSR        0x05fa
#define MX25UW_OCTA_WREN        0x06f9
#define MX25UW_OCTA_RDCR2       0x718e
#define MX25UW_OCTA_RSTEN       0x6699
#define MX25UW_OCTA_RST         0x9966

/* Registers ****************************************************************/

#define MX25UW_SR_WIP           (1 << 0)
#define MX25UW_SR_WEL           (1 << 1)

#define MX25UW_CR2_MODE         0x00000000
#define MX25UW_CR2_DUMMY        0x00000300
#define MX25UW_CR2_SOPI         0x01
#define MX25UW_CR2_DUMMY_6      0x07

/* Geometry *****************************************************************/

#define MX25UW_PAGE_SHIFT       8
#define MX25UW_PAGE_SIZE        (1 << MX25UW_PAGE_SHIFT)
#define MX25UW_SECTOR_SHIFT     12
#define MX25UW_SECTOR_SIZE      (1 << MX25UW_SECTOR_SHIFT)
#define MX25UW_NSECTORS         8192
#define MX25UW_FLASH_SIZE       (MX25UW_SECTOR_SIZE * MX25UW_NSECTORS)
#define MX25UW_ERASED_STATE     0xff

/* Timing *******************************************************************/

#define MX25UW_RESET_TIME_MS    100
#define MX25UW_PAGE_TIME_MS     1000
#define MX25UW_SUBSECTOR_TIME_MS 400
#define MX25UW_BULK_TIME_MS     460000
#define MX25UW_REGISTER_TIME_MS 40
#define MX25UW_REG_DUMMY        4
#define MX25UW_READ_DUMMY       6

/* Identification ***********************************************************/

#define MX25UW_MANUFACTURER     0xc2
#define MX25UW_MEMORY_TYPE      0x81
#define MX25UW_MEMORY_DENSITY   0x39

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mx25uw_dev_s
{
  struct mtd_dev_s mtd;
  FAR struct qspi_dev_s *qspi;
  bool sopi;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mx25uw_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks);
static ssize_t mx25uw_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR uint8_t *buffer);
static ssize_t mx25uw_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buffer);
static ssize_t mx25uw_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t mx25uw_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buffer);
#endif
static int mx25uw_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                        unsigned long arg);

static int mx25uw_lock(FAR struct mx25uw_dev_s *priv);
static void mx25uw_unlock(FAR struct mx25uw_dev_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx25uw_command
 ****************************************************************************/

static int mx25uw_command(FAR struct mx25uw_dev_s *priv,
                          uint32_t instruction, bool octal, bool dtr)
{
  struct qspi_cmdinfo_s cmd;

  cmd.flags   = (octal ? QSPICMD_IOCTAL : 0) |
                (dtr ? QSPICMD_DTR : 0);
  cmd.addrlen = 0;
  cmd.cmd     = instruction;
  cmd.buflen  = 0;
  cmd.addr    = 0;
  cmd.buffer  = NULL;

  return QSPI_COMMAND(priv->qspi, &cmd);
}

/****************************************************************************
 * Name: mx25uw_read_status
 ****************************************************************************/

static int mx25uw_read_status(FAR struct mx25uw_dev_s *priv,
                              FAR uint8_t *status)
{
  struct qspi_meminfo_s mem;

  mem.flags   = priv->sopi ? QSPIMEM_IOCTAL | QSPIMEM_OCTALIO : 0;
  mem.addrlen = priv->sopi ? 4 : 0;
  mem.dummies = priv->sopi ? MX25UW_REG_DUMMY : 0;
  mem.cmd     = priv->sopi ? MX25UW_OCTA_RDSR : MX25UW_RDSR;
  mem.buflen  = 1;
  mem.addr    = 0;
  mem.buffer  = status;

  return QSPI_MEMORY(priv->qspi, &mem);
}

/****************************************************************************
 * Name: mx25uw_wait_ready
 ****************************************************************************/

static int mx25uw_wait_ready(FAR struct mx25uw_dev_s *priv,
                             unsigned int timeout)
{
  uint8_t status;
  int retry;
  int ret;

  for (retry = 0; retry < timeout; retry++)
    {
      ret = mx25uw_read_status(priv, &status);
      if (ret < 0)
        {
          return ret;
        }

      if ((status & MX25UW_SR_WIP) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: mx25uw_write_enable
 ****************************************************************************/

static int mx25uw_write_enable(FAR struct mx25uw_dev_s *priv)
{
  uint8_t status;
  int ret;

  ret = mx25uw_wait_ready(priv, MX25UW_PAGE_TIME_MS);
  if (ret < 0)
    {
      return ret;
    }

  ret = mx25uw_command(priv,
                       priv->sopi ? MX25UW_OCTA_WREN : MX25UW_WREN,
                       priv->sopi, false);
  if (ret < 0)
    {
      return ret;
    }

  ret = mx25uw_read_status(priv, &status);
  if (ret < 0)
    {
      return ret;
    }

  return (status & MX25UW_SR_WEL) != 0 ? OK : -EACCES;
}

/****************************************************************************
 * Name: mx25uw_reset
 ****************************************************************************/

static int mx25uw_reset(FAR struct mx25uw_dev_s *priv)
{
  int ret;

  priv->sopi = false;
  ret = mx25uw_command(priv, MX25UW_RSTEN, false, false);
  if (ret == OK)
    {
      ret = mx25uw_command(priv, MX25UW_RST, false, false);
    }

  if (ret == OK)
    {
      ret = mx25uw_command(priv, MX25UW_OCTA_RSTEN, true, false);
    }

  if (ret == OK)
    {
      ret = mx25uw_command(priv, MX25UW_OCTA_RST, true, false);
    }

  if (ret == OK)
    {
      ret = mx25uw_command(priv, MX25UW_OCTA_RSTEN, true, true);
    }

  if (ret == OK)
    {
      ret = mx25uw_command(priv, MX25UW_OCTA_RST, true, true);
    }

  if (ret == OK)
    {
      up_mdelay(MX25UW_RESET_TIME_MS);
    }

  return ret;
}

/****************************************************************************
 * Name: mx25uw_read_id
 ****************************************************************************/

static int mx25uw_read_id(FAR struct mx25uw_dev_s *priv)
{
  uint8_t id[3];
  struct qspi_cmdinfo_s cmd;
  int ret;

  cmd.flags   = QSPICMD_READDATA;
  cmd.addrlen = 0;
  cmd.cmd     = MX25UW_RDID;
  cmd.buflen  = sizeof(id);
  cmd.addr    = 0;
  cmd.buffer  = id;

  ret = QSPI_COMMAND(priv->qspi, &cmd);
  if (ret < 0)
    {
      return ret;
    }

  if (id[0] != MX25UW_MANUFACTURER || id[1] != MX25UW_MEMORY_TYPE ||
      id[2] != MX25UW_MEMORY_DENSITY)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: mx25uw_write_cr2
 ****************************************************************************/

static int mx25uw_write_cr2(FAR struct mx25uw_dev_s *priv,
                            uint32_t address, uint8_t value)
{
  struct qspi_cmdinfo_s cmd;
  int ret;

  cmd.flags   = QSPICMD_ADDRESS | QSPICMD_WRITEDATA;
  cmd.addrlen = 4;
  cmd.cmd     = MX25UW_WRCR2;
  cmd.buflen  = 1;
  cmd.addr    = address;
  cmd.buffer  = &value;

  ret = mx25uw_write_enable(priv);
  return ret < 0 ? ret : QSPI_COMMAND(priv->qspi, &cmd);
}

/****************************************************************************
 * Name: mx25uw_read_cr2
 ****************************************************************************/

static int mx25uw_read_cr2(FAR struct mx25uw_dev_s *priv,
                           uint32_t address, FAR uint8_t *value)
{
  struct qspi_meminfo_s mem;

  mem.flags   = priv->sopi ? QSPIMEM_IOCTAL | QSPIMEM_OCTALIO : 0;
  mem.addrlen = 4;
  mem.dummies = priv->sopi ? MX25UW_REG_DUMMY : 0;
  mem.cmd     = priv->sopi ? MX25UW_OCTA_RDCR2 : MX25UW_RDCR2;
  mem.buflen  = 1;
  mem.addr    = address;
  mem.buffer  = value;

  return QSPI_MEMORY(priv->qspi, &mem);
}

/****************************************************************************
 * Name: mx25uw_enter_sopi
 ****************************************************************************/

static int mx25uw_enter_sopi(FAR struct mx25uw_dev_s *priv)
{
  uint8_t value;
  int ret;

  ret = mx25uw_write_cr2(priv, MX25UW_CR2_DUMMY,
                         MX25UW_CR2_DUMMY_6);
  if (ret < 0)
    {
      return ret;
    }

  ret = mx25uw_wait_ready(priv, MX25UW_REGISTER_TIME_MS);
  if (ret < 0)
    {
      return ret;
    }

  ret = mx25uw_write_cr2(priv, MX25UW_CR2_MODE, MX25UW_CR2_SOPI);
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(MX25UW_REGISTER_TIME_MS);
  priv->sopi = true;

  ret = mx25uw_wait_ready(priv, MX25UW_REGISTER_TIME_MS);
  if (ret == OK)
    {
      ret = mx25uw_read_cr2(priv, MX25UW_CR2_MODE, &value);
    }

  if (ret == OK && value != MX25UW_CR2_SOPI)
    {
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: mx25uw_lock
 ****************************************************************************/

static int mx25uw_lock(FAR struct mx25uw_dev_s *priv)
{
  int ret;

  ret = QSPI_LOCK(priv->qspi, true);
  if (ret >= 0)
    {
      QSPI_SETMODE(priv->qspi, CONFIG_MX25UW25645G_QSPIMODE);
      QSPI_SETBITS(priv->qspi, 8);
      QSPI_SETFREQUENCY(priv->qspi,
                        CONFIG_MX25UW25645G_QSPI_FREQUENCY);
    }

  return ret;
}

/****************************************************************************
 * Name: mx25uw_unlock
 ****************************************************************************/

static void mx25uw_unlock(FAR struct mx25uw_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, false);
}

/****************************************************************************
 * Name: mx25uw_read_data
 ****************************************************************************/

static int mx25uw_read_data(FAR struct mx25uw_dev_s *priv,
                            uint32_t address, FAR uint8_t *buffer,
                            size_t buflen)
{
  struct qspi_meminfo_s mem;

  mem.flags   = QSPIMEM_IOCTAL | QSPIMEM_OCTALIO;
  mem.addrlen = 4;
  mem.dummies = MX25UW_READ_DUMMY;
  mem.cmd     = MX25UW_OCTA_READ;
  mem.buflen  = buflen;
  mem.addr    = address;
  mem.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &mem);
}

/****************************************************************************
 * Name: mx25uw_page_program
 ****************************************************************************/

static int mx25uw_page_program(FAR struct mx25uw_dev_s *priv,
                               uint32_t address,
                               FAR const uint8_t *buffer, size_t buflen)
{
  struct qspi_meminfo_s mem;
  int ret;

  mem.flags   = QSPIMEM_WRITE | QSPIMEM_IOCTAL | QSPIMEM_OCTALIO;
  mem.addrlen = 4;
  mem.dummies = 0;
  mem.cmd     = MX25UW_OCTA_PP;
  mem.buflen  = buflen;
  mem.addr    = address;
  mem.buffer  = (FAR void *)buffer;

  ret = mx25uw_write_enable(priv);
  if (ret == OK)
    {
      ret = QSPI_MEMORY(priv->qspi, &mem);
    }

  return ret < 0 ? ret : mx25uw_wait_ready(priv, MX25UW_PAGE_TIME_MS);
}

/****************************************************************************
 * Name: mx25uw_sector_erase
 ****************************************************************************/

static int mx25uw_sector_erase(FAR struct mx25uw_dev_s *priv,
                               uint32_t address)
{
  struct qspi_cmdinfo_s cmd;
  int ret;

  cmd.flags   = QSPICMD_ADDRESS | QSPICMD_IOCTAL | QSPICMD_OCTALIO;
  cmd.addrlen = 4;
  cmd.cmd     = MX25UW_OCTA_SE;
  cmd.buflen  = 0;
  cmd.addr    = address;
  cmd.buffer  = NULL;

  ret = mx25uw_write_enable(priv);
  if (ret == OK)
    {
      ret = QSPI_COMMAND(priv->qspi, &cmd);
    }

  return ret < 0 ? ret :
         mx25uw_wait_ready(priv, MX25UW_SUBSECTOR_TIME_MS);
}

/****************************************************************************
 * Name: mx25uw_chip_erase
 ****************************************************************************/

static int mx25uw_chip_erase(FAR struct mx25uw_dev_s *priv)
{
  int ret;

  ret = mx25uw_write_enable(priv);
  if (ret == OK)
    {
      ret = mx25uw_command(priv, MX25UW_OCTA_CE, true, false);
    }

  return ret < 0 ? ret : mx25uw_wait_ready(priv, MX25UW_BULK_TIME_MS);
}

/****************************************************************************
 * Name: mx25uw_erase
 ****************************************************************************/

static int mx25uw_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks)
{
  FAR struct mx25uw_dev_s *priv = (FAR struct mx25uw_dev_s *)dev;
  size_t erased;
  int ret;

  if (startblock < 0 || startblock >= MX25UW_NSECTORS ||
      nblocks > MX25UW_NSECTORS - startblock)
    {
      return -EINVAL;
    }

  ret = mx25uw_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  for (erased = 0; ret == OK && erased < nblocks; erased++)
    {
      ret = mx25uw_sector_erase(priv,
                                (startblock + erased) <<
                                MX25UW_SECTOR_SHIFT);
    }

  mx25uw_unlock(priv);
  return ret < 0 ? ret : (int)erased;
}

/****************************************************************************
 * Name: mx25uw_bread
 ****************************************************************************/

static ssize_t mx25uw_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR uint8_t *buffer)
{
  ssize_t ret;

  ret = mx25uw_read(dev, startblock << MX25UW_PAGE_SHIFT,
                    nblocks << MX25UW_PAGE_SHIFT, buffer);
  return ret < 0 ? ret : (ssize_t)nblocks;
}

/****************************************************************************
 * Name: mx25uw_bwrite
 ****************************************************************************/

static ssize_t mx25uw_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct mx25uw_dev_s *priv = (FAR struct mx25uw_dev_s *)dev;
  size_t written;
  int ret;

  if (startblock < 0 || startblock >=
      (MX25UW_FLASH_SIZE >> MX25UW_PAGE_SHIFT) ||
      nblocks > (MX25UW_FLASH_SIZE >> MX25UW_PAGE_SHIFT) - startblock)
    {
      return -EINVAL;
    }

  ret = mx25uw_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  for (written = 0; ret == OK && written < nblocks; written++)
    {
      ret = mx25uw_page_program(priv,
                                (startblock + written) <<
                                MX25UW_PAGE_SHIFT,
                                buffer + (written << MX25UW_PAGE_SHIFT),
                                MX25UW_PAGE_SIZE);
    }

  mx25uw_unlock(priv);
  return ret < 0 ? ret : (ssize_t)written;
}

/****************************************************************************
 * Name: mx25uw_read
 ****************************************************************************/

static ssize_t mx25uw_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct mx25uw_dev_s *priv = (FAR struct mx25uw_dev_s *)dev;
  int ret;

  if (offset < 0 || offset >= MX25UW_FLASH_SIZE ||
      nbytes > MX25UW_FLASH_SIZE - offset)
    {
      return -EINVAL;
    }

  ret = mx25uw_lock(priv);
  if (ret == OK)
    {
      ret = mx25uw_read_data(priv, offset, buffer, nbytes);
      mx25uw_unlock(priv);
    }

  return ret < 0 ? ret : (ssize_t)nbytes;
}

#ifdef CONFIG_MTD_BYTE_WRITE
/****************************************************************************
 * Name: mx25uw_write
 ****************************************************************************/

static ssize_t mx25uw_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct mx25uw_dev_s *priv = (FAR struct mx25uw_dev_s *)dev;
  size_t pagesize;
  size_t written;
  int ret;

  if (offset < 0 || offset >= MX25UW_FLASH_SIZE ||
      nbytes > MX25UW_FLASH_SIZE - offset)
    {
      return -EINVAL;
    }

  ret = mx25uw_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  for (written = 0; ret == OK && written < nbytes; written += pagesize)
    {
      pagesize = MX25UW_PAGE_SIZE - ((offset + written) &
                                     (MX25UW_PAGE_SIZE - 1));
      if (pagesize > nbytes - written)
        {
          pagesize = nbytes - written;
        }

      ret = mx25uw_page_program(priv, offset + written,
                                buffer + written, pagesize);
    }

  mx25uw_unlock(priv);
  return ret < 0 ? ret : (ssize_t)written;
}
#endif

/****************************************************************************
 * Name: mx25uw_ioctl
 ****************************************************************************/

static int mx25uw_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct mx25uw_dev_s *priv = (FAR struct mx25uw_dev_s *)dev;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)(uintptr_t)arg;

          if (geo == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              geo->blocksize    = MX25UW_PAGE_SIZE;
              geo->erasesize    = MX25UW_SECTOR_SIZE;
              geo->neraseblocks = MX25UW_NSECTORS;
              strlcpy(geo->model, "MX25UW25645G", sizeof(geo->model));
              ret = OK;
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)(uintptr_t)arg;

          if (info == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              info->numsectors  = MX25UW_FLASH_SIZE / MX25UW_PAGE_SIZE;
              info->sectorsize  = MX25UW_PAGE_SIZE;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        ret = mx25uw_lock(priv);
        if (ret == OK)
          {
            ret = mx25uw_chip_erase(priv);
            mx25uw_unlock(priv);
          }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *state = (FAR uint8_t *)(uintptr_t)arg;

          if (state == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              *state = MX25UW_ERASED_STATE;
              ret = OK;
            }
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx25uw25645g_initialize
 *
 * Description:
 *   Bind an MX25UW25645G flash memory to an XSPI interface.
 *
 * Input Parameters:
 *   qspi - QSPI interface
 *
 * Returned Value:
 *   An MTD device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *
mx25uw25645g_initialize(FAR struct qspi_dev_s *qspi)
{
  FAR struct mx25uw_dev_s *priv;
  int ret;

  if (qspi == NULL)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->mtd.erase  = mx25uw_erase;
  priv->mtd.bread  = mx25uw_bread;
  priv->mtd.bwrite = mx25uw_bwrite;
  priv->mtd.read   = mx25uw_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = mx25uw_write;
#endif
  priv->mtd.ioctl  = mx25uw_ioctl;
  priv->mtd.name   = "mx25uw25645g";
  priv->qspi       = qspi;

  ret = mx25uw_lock(priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  ret = mx25uw_reset(priv);
  if (ret == OK)
    {
      ret = mx25uw_read_id(priv);
    }

  if (ret == OK)
    {
      ret = mx25uw_enter_sopi(priv);
    }

  mx25uw_unlock(priv);

  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  return &priv->mtd;
}
