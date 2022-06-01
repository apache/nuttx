/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_flash_mtd.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/irq.h>
#include <nuttx/mm/mm.h>
#include <nuttx/kmalloc.h>

#include "hardware/tlsr82_gpio.h"
#include "tlsr82_flash.h"
#include "tlsr82_flash_mtd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIN(a, b)                     (((a) > (b)) ? (b) : (a))

/* Flash size deifnitions */

#define TLSR82_SECTOR_SHIFT           12
#define TLSR82_SECTOR_SIZE            (1 << TLSR82_SECTOR_SHIFT)
#define TLSR82_PAGE_SHIFT             8
#define TLSR82_PAGE_SIZE              (1 << TLSR82_PAGE_SHIFT)

/* Flash layout definitions */

#define TLSR82_START_ADDR             (0)
#define TLSR82_TOTAL_SIZE             (512 * 1024)

#define TLSR82_BANKA_ADDR             TLSR82_START_ADDR
#define TLSR82_BANKA_SIZE             (240 * 1024)

#define TLSR82_USR_ADDR               (TLSR82_BANKA_ADDR + TLSR82_BANKA_SIZE)
#define TLSR82_USR_SIZE               (12 * 1024)

#define TLSR82_KEY_ADDR               (TLSR82_USR_ADDR + TLSR82_USR_SIZE)
#define TLSR82_KEY_SIZE               (4 * 1024)

#define TLSR82_BANKB_ADDR             (TLSR82_KEY_ADDR + TLSR82_KEY_SIZE)
#define TLSR82_BANKB_SIZE             (240 * 1024)

#define TLSR82_SYS_ADDR               (TLSR82_BANKB_ADDR + TLSR82_BANKB_SIZE)
#define TLSR82_SYS_SIZE               (16 * 1024)

/* Flash write buffer size definitions */

#ifdef CONFIG_TLSR82_FLASH_WRITE_BUFFER
#  define FLASH_WRITE_BUF_SIZE        CONFIG_TLSR82_FLASH_WRITE_BUFFER_SIZE
#endif

/* Flash test definitions */

#ifdef CONFIG_TLSR82_FLASH_TEST

/* Flash sector erase trace macro definition */

#define FLASH_ERASE_TRACE_START()
#define FLASH_ERASE_TRACE_END()

/* Flash page read/write trace macro definition */

#define FLASH_READ_TRACE_START()
#define FLASH_READ_TRACE_END()

#define FLASH_WRITE_TRACE_START()
#define FLASH_WRITE_TRACE_END()

/* Flash protect/unprotect trace marco definition */

#define FLASH_PROT_TRACE_START()
#define FLASH_PROT_TRACE_END()

#define FLASH_UNPROT_TRACE_START()
#define FLASH_UNPROT_TRACE_END()

#define FLASH_BUF_LIST16(n) \
  n    , n + 1, n +  2, n +  3, n +  4, n +  5, n +  6, n +  7, \
  n + 8, n + 9, n + 10, n + 11, n + 12, n + 13, n + 14, n + 15,

#define FLASH_BUF_LIST128(n) \
  FLASH_BUF_LIST16(n) \
  FLASH_BUF_LIST16(n + 0x10) \
  FLASH_BUF_LIST16(n + 0x20) \
  FLASH_BUF_LIST16(n + 0x30) \
  FLASH_BUF_LIST16(n + 0x40) \
  FLASH_BUF_LIST16(n + 0x50) \
  FLASH_BUF_LIST16(n + 0x60) \
  FLASH_BUF_LIST16(n + 0x70) \

#endif
/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.
 * The struct mtd_dev_s must appear at the beginning of the definition
 * so that you can freely cast between pointers to struct mtd_dev_s and
 * struct tlsr82_flash_dev_s.
 */

struct tlsr82_flash_dev_s
{
  struct mtd_dev_s      mtd;          /* MTD interface */
  uint32_t              baseaddr;     /* mtd flash start address */
  uint32_t              size;         /* avaliable size for MTD */
  uint16_t              nsectors;     /* Number of erase sectors */
  uint16_t              sectorsize;   /* Size of one sector */
  uint16_t              pagesize;     /* Size of one page */
};

struct tlsr82_partition_s
{
  char *path;
  off_t firstblock;
  off_t nblocks;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Flash mtd partition table */

#ifdef CONFIG_MTD_PARTITION
static const struct tlsr82_partition_s tlsr82_part_table[] =
{
  {
    .path       = "dev/banka",
    .firstblock = TLSR82_BANKA_ADDR / TLSR82_PAGE_SIZE,
    .nblocks    = TLSR82_BANKA_SIZE / TLSR82_PAGE_SIZE,
  },
  {
    .path       = "dev/usr",
    .firstblock = TLSR82_USR_ADDR / TLSR82_PAGE_SIZE,
    .nblocks    = TLSR82_USR_SIZE / TLSR82_PAGE_SIZE,
  },
  {
    .path       = "dev/key",
    .firstblock = TLSR82_KEY_ADDR / TLSR82_PAGE_SIZE,
    .nblocks    = TLSR82_KEY_SIZE / TLSR82_PAGE_SIZE,
  },
  {
    .path       = "dev/bankb",
    .firstblock = TLSR82_BANKB_ADDR / TLSR82_PAGE_SIZE,
    .nblocks    = TLSR82_BANKB_SIZE / TLSR82_PAGE_SIZE,
  },
  {
    .path       = "dev/sys",
    .firstblock = TLSR82_SYS_ADDR / TLSR82_PAGE_SIZE,
    .nblocks    = TLSR82_SYS_SIZE / TLSR82_PAGE_SIZE,
  }
};
#endif

/* Flash manufacture ID and unique ID */

static uint32_t g_flash_mid;
static uint8_t g_flash_uid[16];

/* Flash byte write buffer when enable the flash byte write buffer */

#ifdef CONFIG_TLSR82_FLASH_WRITE_BUFFER
static uint8_t flash_write_buffer[FLASH_WRITE_BUF_SIZE];
#endif

/* Flash test buffer when enable the flash test */

#ifdef CONFIG_TLSR82_FLASH_TEST

static uint8_t flash_buffer[TLSR82_PAGE_SIZE] =
{
  FLASH_BUF_LIST128(0)
  FLASH_BUF_LIST128(1)
};

static uint8_t flash_read_buffer[TLSR82_PAGE_SIZE];
static char print_buf[128];
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Flash driver methods */

static int tlsr82_flash_chip_erase(struct tlsr82_flash_dev_s *priv);

#ifdef CONFIG_TLSR82_FLASH_TEST
static void tlsr82_flash_print(const char *msg, const uint8_t *buf,
                               size_t len);

static int tlsr82_flash_test(struct tlsr82_flash_dev_s *priv);
#endif

/* MTD driver methods */

static int     tlsr82_flash_erase (struct mtd_dev_s *dev, off_t startblock,
                                   size_t nblocks);

static ssize_t tlsr82_flash_bread (struct mtd_dev_s *dev, off_t startblock,
                                   size_t nblocks, uint8_t *buf);

static ssize_t tlsr82_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                   size_t nblocks, const uint8_t *buf);

static ssize_t tlsr82_flash_read  (struct mtd_dev_s *dev, off_t offset,
                                   size_t nbytes, uint8_t *buffer);

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tlsr82_flash_write (struct mtd_dev_s *dev, off_t offset,
                                   size_t nbytes, const uint8_t *buffer);
#endif

static int     tlsr82_flash_ioctl (struct mtd_dev_s *dev, int cmd,
                                   unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_flash_readid
 ****************************************************************************/

static int tlsr82_flash_chip_erase(struct tlsr82_flash_dev_s *priv)
{
  int i;
  uint32_t addr;

  addr = priv->baseaddr;
  for (i = 0; i < priv->nsectors; i++)
    {
      finfo("Erase sector: %d, address: 0x%08lx\n", i, addr);
      tlsr82_flash_erase_sector(addr);
      addr += priv->sectorsize;
    }

  return OK;
}

/****************************************************************************
 * Name: tlsr82_flash_print
 ****************************************************************************/

#ifdef CONFIG_TLSR82_FLASH_TEST
static void tlsr82_flash_print(const char *msg, const uint8_t *buf,
                               size_t len)
{
  int i = 0;
  int off = 0;

  ferr("%s\n", msg);
  while (len--)
    {
      if (i % 16 == 0)
        {
          off += sprintf(&print_buf[off], "0x%08x:", i);
        }

      off += sprintf(&print_buf[off], "0x%02x ", buf[i]);
      i++;

      if (i % 16 == 0)
        {
          ferr("%s\n", print_buf);
          off = 0;
        }
    }

  if (i % 16 != 0)
    {
      ferr("%s\n", print_buf);
    }
}
#endif

/****************************************************************************
 * Name: tlsr82_flash_test
 ****************************************************************************/

#ifdef CONFIG_TLSR82_FLASH_TEST
static int tlsr82_flash_test(struct tlsr82_flash_dev_s *priv)
{
  int ret      = OK;
  int npages   = 0;
  int i        = 0;
  int j        = 0;

  npages = priv->nsectors * (TLSR82_SECTOR_SIZE / TLSR82_PAGE_SIZE);

  ferr("======== Flash test start ========\n");

  /* 1. print the manufacture id and unique id */

  ret = 0;
  ferr("Flash information print:\n");
  ferr("    Flash MID: 0x%08lx\n", g_flash_mid);
  ret += sprintf(&print_buf[ret], "    Flash UID: ");
  for (i = 1; i < 16; i++)
    {
      ret += sprintf(&print_buf[ret], "0x%x ", g_flash_uid[i]);
    }

  ferr("%s\n", print_buf);

  ferr("    Flash Start Address: 0x%08lx\n", priv->baseaddr);
  ferr("    Flash Size  : 0x%08lx\n", priv->size);
  ferr("    Flash Sector: %d\n", priv->nsectors);
  ferr("    Flash Page  : %d\n", npages);

  /* 2. erase chip and check all the erased sector, all the bit in erased
   *    sector should be 1.
   */

  ferr("Flash chip erase test start:\n");

  FLASH_ERASE_TRACE_START();
  tlsr82_flash_chip_erase(priv);
  FLASH_ERASE_TRACE_END();

  for (i = 0; i < npages; i++)
    {
      FLASH_READ_TRACE_START();
      ret = tlsr82_flash_bread(&priv->mtd, i, 1, flash_read_buffer);
      FLASH_READ_TRACE_END();
      if (ret != 1)
        {
          ferr("    Flash block read failed, ret=%d\n", ret);
          goto errout;
        }

      for (j = 0; j < TLSR82_PAGE_SIZE; j++)
        {
          if (flash_read_buffer[j] != 0xff)
            {
              ferr("    Flash erased data is not 0xff, page_i=%d, byte_j=%d,"
                   "data=0x%x\n", i, j, flash_read_buffer[j]);
              goto errout;
            }
        }
    }

  ferr("Flach chip erase test Success.\n");

  /* 3. Write all the flash and check result */

  ferr("Flash page write/read test start:\n");
  tlsr82_flash_print("Write buffer data:", flash_buffer,
                 TLSR82_PAGE_SIZE);
  for (i = 0; i < npages; i++)
    {
      FLASH_WRITE_TRACE_START();
      ret = tlsr82_flash_bwrite(&priv->mtd, i, 1, flash_buffer);
      FLASH_WRITE_TRACE_END();
      if (ret != 1)
        {
          ferr("    Flash block write failed, ret=%d\n", ret);
          goto errout;
        }
    }

  for (i = 0; i < npages; i++)
    {
      memset(flash_read_buffer, 0, TLSR82_PAGE_SIZE);
      ret = tlsr82_flash_bread(&priv->mtd, i, 1, flash_read_buffer);
      if (ret != 1)
        {
          ferr("    Flash block read failed, ret=%d\n", ret);
          goto errout;
        }

      if (memcmp(flash_read_buffer, flash_buffer, TLSR82_PAGE_SIZE) != 0)
        {
          ferr("    Flash write compre is not equal, page_i=%d\n", i);
          tlsr82_flash_print("Write buffer data:", flash_buffer,
                             TLSR82_PAGE_SIZE);
          tlsr82_flash_print("Read buffer data:", flash_read_buffer,
                             TLSR82_PAGE_SIZE);
          goto errout;
        }
    }

  ferr("Flash page write/read test Success.\n");

#ifdef CONFIG_MTD_BYTE_WRITE

  /* 4. Erase chip again for byte write/read test */

  uint32_t offset = 0;
  int k;

  ferr("Erase chip for byte write test\n");
  tlsr82_flash_chip_erase(priv);
  ferr("Erase chip finished\n");

  ferr("Flash byte read/write test start\n");
  do
    {
      /* i = the write number, j = the flash_buffer index */

      i = 40;
      if (i + offset > priv->size)
        {
          i = priv->size - offset;
        }

      j = offset & 128;

      FLASH_WRITE_TRACE_START();
      ret = tlsr82_flash_write(&priv->mtd, offset, i, &flash_buffer[j]);
      FLASH_WRITE_TRACE_END();

      if (ret != i)
        {
          ferr("    Flash byte write failed, offset: 0x%08lx, ret=%d\n",
               offset, ret);
          goto errout;
        }

      memset(flash_read_buffer, 0, i);

      FLASH_READ_TRACE_START();
      ret = tlsr82_flash_read(&priv->mtd, offset, i, flash_read_buffer);
      FLASH_READ_TRACE_END();

      if (ret != i)
        {
          ferr("    Flash byte read failed, offset: 0x%08lx, ret=%d\n",
               offset, ret);
          goto errout;
        }

      for (k = 0; k < i; k++)
        {
          if (flash_buffer[j + k] != flash_read_buffer[k])
            {
              ferr("    Flash byte write/read compare failed\n");
              ferr("    Flash address: 0x%08lx\n", priv->baseaddr + offset);
              tlsr82_flash_print("Write Data:", &flash_buffer[j], i);
              tlsr82_flash_print("Read Data:", flash_read_buffer, i);
              goto errout;
            }
        }

      offset += i;
    }
  while (offset < priv->size);

  ferr("Flash byte read/write test Success\n");

#endif

#ifdef CONFIG_TLSR82_FLASH_PROTECT
  /* 5. Flash protect/unprotect test:
   *    1) erase the chip;
   *    2) write data into chip;
   *    3) protect flash;
   *    4) erase the chip with flash protected;
   *    5) read the chip, the data should be same as the data written in 2);
   */

  uint8_t status;
  uint32_t addr;

  /* 1) erase the chip */

  ferr("Erase chip for protect/unprotect test\n");
  tlsr82_flash_chip_erase(priv);
  ferr("Erase chip finished\n");

  /* 2) write data into chip */

  tlsr82_flash_print("Write buffer data:", flash_buffer, TLSR82_PAGE_SIZE);
  for (i = 0; i < npages; i++)
    {
      ret = tlsr82_flash_bwrite(&priv->mtd, i, 1, flash_buffer);
      if (ret != 1)
        {
          ferr("    Flash block write failed, ret=%d\n", ret);
          goto errout;
        }
    }

  /* 3) protect the flash, read the flash status register */

  FLASH_PROT_TRACE_START();
  tlsr82_flash_protect();
  FLASH_PROT_TRACE_END();

  status = tlsr82_flash_read_status(5);
  ferr("Current flash status: 0x%x\n", status);

  /* 4) erase the chip with flash protected */

  addr = priv->baseaddr;
  for (i = 0; i < priv->nsectors; i++)
    {
      tlsr82_flash_erase_sector(addr);
      addr += priv->sectorsize;
    }

  FLASH_UNPROT_TRACE_START();
  tlsr82_flash_unprotect();
  FLASH_UNPROT_TRACE_END();

  /* 5) read the chip, the data should be same as the data written in 2) */

  for (i = 0; i < npages; i++)
    {
      memset(flash_read_buffer, 0, TLSR82_PAGE_SIZE);
      ret = tlsr82_flash_bread(&priv->mtd, i, 1, flash_read_buffer);
      if (ret != 1)
        {
          ferr("    Flash block read failed, ret=%d\n", ret);
          goto errout;
        }

      if (memcmp(flash_read_buffer, flash_buffer, TLSR82_PAGE_SIZE) != 0)
        {
          ferr("    Flash write compre is not equal, page_i=%d\n", i);
          tlsr82_flash_print("Write buffer data:", flash_buffer,
                             TLSR82_PAGE_SIZE);
          tlsr82_flash_print("Read buffer data:", flash_read_buffer,
                             TLSR82_PAGE_SIZE);
          goto errout;
        }
    }
#endif

  ferr("======== Flash test Success ========\n");

  return OK;

errout:
  ferr("Flash test Failed.\n");
  return ret;
}
#endif

/****************************************************************************
 * Name: tlsr82_flash_erase
 ****************************************************************************/

static int tlsr82_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks)
{
  struct tlsr82_flash_dev_s *priv = (struct tlsr82_flash_dev_s *)dev;
  size_t i;
  uint32_t addr;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  addr = priv->baseaddr + startblock * priv->sectorsize;
  for (i = 0; i < nblocks; i++)
    {
      tlsr82_flash_erase_sector(addr);
      addr += priv->sectorsize;
    }

  return (int)nblocks;
}

/****************************************************************************
 * Name: tlsr82_flash_bread
 ****************************************************************************/

static ssize_t tlsr82_flash_bread(struct mtd_dev_s *dev, off_t startblock,
                                  size_t nblocks, uint8_t *buffer)
{
  struct tlsr82_flash_dev_s *priv = (struct tlsr82_flash_dev_s *)dev;
  uint32_t addr;
  size_t i;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  addr = priv->baseaddr + startblock * priv->pagesize;

  for (i = 0; i < nblocks; i++)
    {
      tlsr82_flash_read_data(addr, buffer, priv->pagesize);
      addr   += priv->pagesize;
      buffer += priv->pagesize;
    }

  return nblocks;
}

/****************************************************************************
 * Name: tlsr82_flash_bwrite
 ****************************************************************************/

static ssize_t tlsr82_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                   size_t nblocks, const uint8_t *buffer)
{
  struct tlsr82_flash_dev_s *priv = (struct tlsr82_flash_dev_s *)dev;
  uint32_t addr;
  size_t i;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  addr = priv->baseaddr + startblock * priv->pagesize;

  for (i = 0; i < nblocks; i++)
    {
      tlsr82_flash_write_data(addr, buffer, priv->pagesize);
      addr   += priv->pagesize;
      buffer += priv->pagesize;
    }

  return nblocks;
}

/****************************************************************************
 * Name: tlsr82_flash_read
 ****************************************************************************/

static ssize_t tlsr82_flash_read(struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, uint8_t *buffer)
{
  struct tlsr82_flash_dev_s *priv = (struct tlsr82_flash_dev_s *)dev;
  uint32_t addr;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  addr = priv->baseaddr + offset;

  tlsr82_flash_read_data(addr, buffer, nbytes);

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: tlsr82_flash_write
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tlsr82_flash_write (struct mtd_dev_s *dev, off_t offset,
                                   size_t nbytes, const uint8_t *buffer)
{
  struct tlsr82_flash_dev_s *priv = (struct tlsr82_flash_dev_s *)dev;
  uint32_t addr;
  size_t allow_len = 0;
  size_t write_len = 0;
  size_t len = nbytes;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  addr      = priv->baseaddr + offset;

  do
    {
      /* The allow length is the max allow write lentgh that not cross page */

      allow_len = TLSR82_PAGE_SIZE - (addr & (TLSR82_PAGE_SIZE - 1));
      write_len = MIN(nbytes, allow_len);

#ifdef CONFIG_TLSR82_FLASH_WRITE_BUFFER

      /* The write length can not be larger than FLASH_WRITE_BUF_SIZE */

      write_len = MIN(FLASH_WRITE_BUF_SIZE, write_len);

      /* Copy the data to the buffer and write, the temporary buffer is used
       * to make sure the argument buffer is not in flash. Telink flash is
       * not allowed read during flash writing.
       */

      memcpy(flash_write_buffer, buffer, write_len);
      tlsr82_flash_write_data(addr, flash_write_buffer, write_len);
#else
      tlsr82_flash_write_data(addr, buffer, write_len);
#endif

      addr   += write_len;
      buffer += write_len;
      nbytes -= write_len;
    }
  while (nbytes > 0);

  return (ssize_t)len;
}
#endif

/****************************************************************************
 * Name: tlsr82_flash_ioctl
 ****************************************************************************/

static int tlsr82_flash_ioctl(struct mtd_dev_s *dev, int cmd,
                              unsigned long arg)
{
  struct tlsr82_flash_dev_s *priv = (struct tlsr82_flash_dev_s *)dev;
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
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but
               * the client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

              geo->blocksize    = TLSR82_PAGE_SIZE;
              geo->erasesize    = TLSR82_SECTOR_SIZE;
              geo->neraseblocks = priv->nsectors;
              ret               = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32
                    " neraseblocks: %" PRId32 "\n",
              geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          tlsr82_flash_chip_erase(priv);
        }
        break;

      case MTDIOC_PROTECT:
        {
          /* Ignore arg, direct protect full chip */

          tlsr82_flash_protect();
        }
        break;

      case MTDIOC_UNPROTECT:
        {
          /* Ignore arg, direct unprotect full chip */

          tlsr82_flash_unprotect();
        }
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
 * Name: tlsr82_flash_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for the internal FLASH.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 * Parameter:
 *   offset - offset from 0 of internal flash
 *   size   - avaiable size for NVM
 *
 ****************************************************************************/

struct mtd_dev_s *tlsr82_flash_initialize(uint32_t offset, uint32_t size)
{
  int ret;
  struct tlsr82_flash_dev_s *priv;

  /* If the mtd device address exceeded the flash address space, return
   * NULL.
   */

  if ((offset + size) > (CONFIG_TLSR82_FLASH_SIZE_KB * 1024))
    {
      goto errout;
    }

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same QuadSPI
   * bus.
   */

  priv = (struct tlsr82_flash_dev_s *)
          kmm_zalloc(sizeof(struct tlsr82_flash_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = tlsr82_flash_erase;
      priv->mtd.bread  = tlsr82_flash_bread;
      priv->mtd.bwrite = tlsr82_flash_bwrite;
      priv->mtd.read   = tlsr82_flash_read;
#ifdef CONFIG_MTD_BYTE_WRITE
      priv->mtd.write  = tlsr82_flash_write;
#endif
      priv->mtd.ioctl  = tlsr82_flash_ioctl;
      priv->mtd.name   = "tlsr82_flash_mtd";

      priv->baseaddr    = offset;
      priv->size        = size;
      priv->sectorsize  = TLSR82_SECTOR_SIZE;
      priv->pagesize    = TLSR82_PAGE_SIZE;
      priv->nsectors    = size >> TLSR82_SECTOR_SHIFT;

      /* Identify the FLASH chip */

      ret = tlsr82_flash_miduid_check(&g_flash_mid, &g_flash_uid[0]);
      if (ret < 0)
        {
          ferr("Flash mid or uid error\n");
          goto errout_with_priv;
        }
    }
  else
    {
      ferr("Flash device malloc failed, size=%lu\n",
           sizeof(struct tlsr82_flash_dev_s));
      goto errout;
    }

  /* Calibrate the flash */

  tlsr82_flash_calibrate(g_flash_mid);

#ifdef CONFIG_TLSR82_FLASH_TEST
  ret = tlsr82_flash_test(priv);
  if (ret < 0)
    {
      ferr("Flash test failed, ret=%d\n", ret);
      goto errout_with_priv;
    }
#endif

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (struct mtd_dev_s *)priv;

errout_with_priv:
  kmm_free(priv);
  return NULL;

errout:
  return NULL;
}

/****************************************************************************
 * Name: tlsr82_partition_init
 *
 * Description:
 *   Register partition MTD device on the parent MTD device.
 *
 * Parameter:
 *   path - the parent mtd device path.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_PARTITION
int tlsr82_partition_init(const char *path)
{
  int ret = OK;
  int num;
  int i;

  num = sizeof(tlsr82_part_table) / sizeof(struct tlsr82_partition_s);

  for (i = 0; i < num; i++)
    {
      ret = register_mtdpartition(tlsr82_part_table[i].path, 0, path,
                                  tlsr82_part_table[i].firstblock,
                                  tlsr82_part_table[i].nblocks);
      if (ret < 0)
        {
          ferr("Register mtd partition failed, path=%s, ret=%d\n",
               tlsr82_part_table[i].path, ret);
          break;
        }
    }

  return ret;
}
#endif
