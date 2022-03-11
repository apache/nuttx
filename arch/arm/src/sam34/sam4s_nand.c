/****************************************************************************
 * arch/arm/src/sam34/sam4s_nand.c
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
#include <nuttx/mtd/nand_config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_raw.h>
#include <nuttx/mtd/nand_model.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>
#include "hardware/sam4s_pinmap.h"
#include "arm_internal.h"
#include "sam4s_nand.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nand flash chip status codes */

#define STATUS_ERROR         (1 << 0)
#define STATUS_READY         (1 << 6)

/* Number of tries for erasing or writing block */

#define NAND_ERASE_NRETRIES  2
#define NAND_WRITE_NRETRIES  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level HSMC Helpers */

#ifdef CONFIG_SAM34_NAND_DUMP
#  define nand_dump(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define nand_dump(m,b,s)
#endif

/* Raw Data Transfer Helpers */

static int      nand_write(struct sam_nandcs_s *priv, uint32_t rowaddr,
                      uint32_t coladdr, uint8_t *buffer, uint16_t buflen,
                      uint16_t offset);
static int      nand_read(struct sam_nandcs_s *priv, uint32_t rowaddr,
                     uint32_t coladdr, uint8_t *buffer, uint16_t buflen,
                     uint16_t offset);

/* MTD driver methods */

static int      nand_eraseblock(struct nand_raw_s *raw, off_t block);
static int      nand_rawread(struct nand_raw_s *raw, off_t block,
                  unsigned int page, void *data, void *spare);
static int      nand_rawwrite(struct nand_raw_s *raw, off_t block,
                  unsigned int page, const void *data, const void *spare);

/* Initialization */

static void     nand_reset(struct sam_nandcs_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These pre-allocated structures hold the state of the MTD driver for NAND
 * on CS0..3 as configured.
 */

#ifdef CONFIG_SAM34_NCS0_NAND
static struct sam_nandcs_s g_cs0nand;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void  nand_reset(struct sam_nandcs_s *priv)
{
  WRITE_COMMAND8(&priv->raw, COMMAND_RESET);

  /* The device will be busy for a maximum of 1ms. */

  up_mdelay(1);
}

/****************************************************************************
 * Name: nand_wait_ready
 *
 * Description:
 *   Waiting for the completion of a page program, erase and random read
 *   completion.
 *
 * Input Parameters:
 *   priv  Pointer to a sam_nandcs_s instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int nand_wait_ready(struct sam_nandcs_s *priv)
{
  uint32_t timeout;
  uint8_t status;
  up_udelay(10);

  /* The ready/busy (R/nB) signal of the NAND Flash  */

  while (!sam_gpioread(priv->rb));
  WRITE_COMMAND8(&priv->raw, COMMAND_STATUS);

  /* Issue command */

  timeout = 0;
  while (timeout < MAX_READ_STATUS_COUNT)
    {
      /* Read status byte */

      status = READ_DATA8(&priv->raw);

      /* Check status. If status bit 6 = 1 device is ready */

      if ((status & STATUS_READY) == STATUS_READY)
        {
          /* If status bit 0 = 0 the last operation was successful */

          if ((status & STATUS_ERROR) == 0)
            {
              return OK;
            }
          else
            {
              return -EIO;
            }
        }

      timeout++;
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: nand_eraseblock
 *
 * Description:
 *   Erases the specified block of the device.
 *
 * Input Parameters:
 *   raw    - Lower-half, raw NAND FLASH interface
 *   block  - Number of the physical block to erase.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static inline int nand_tryeraseblock(struct sam_nandcs_s *priv, off_t block)
{
  uint32_t rowaddr;
  int ret = OK;

  /* Calculate address used for erase */

  rowaddr = block * nandmodel_pagesperblock(&priv->raw.model);

  WRITE_COMMAND8(&priv->raw, COMMAND_ERASE_1);
  WRITE_ADDRESS8(&priv->raw, rowaddr);          /* 3rd cycle row addr */
  WRITE_ADDRESS8(&priv->raw, rowaddr >> 8);     /* 4th cycle row addr */
  WRITE_ADDRESS8(&priv->raw, rowaddr >> 16);    /* 5st cycle row addr */
  WRITE_COMMAND8(&priv->raw, COMMAND_ERASE_2);

  ret = nand_wait_ready(priv);
  if (ret < 0)
    {
      ferr("ERROR: Block %jd Could not erase: %d\n", (intmax_t)block, ret);
    }

  return ret;
}

static int nand_eraseblock(struct nand_raw_s *raw, off_t block)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  int retries = NAND_ERASE_NRETRIES;
  int ret = OK;

  DEBUGASSERT(priv);

  finfo("block=%d\n", (int)block);

  if (ret < 0)
    {
      return ret;
    }

  /* Try up to NAND_ERASE_NRETRIES times to erase the FLASH */

  while (retries > 0)
    {
      ret = nand_tryeraseblock(priv, block);
      if (ret == OK)
        {
          return OK;
        }

      retries--;
    }

  ferr("ERROR: Block %d Failed to erase after %d tries\n",
       (int)block, NAND_ERASE_NRETRIES);

  return -EAGAIN;
}

/****************************************************************************
 * Name: nand_write
 *
 * Description:
 *   Write data to NAND using the NAND data address.
 *
 * Input Parameters:
 *   priv     - Lower-half, private NAND FLASH device state
 *   buffer   - Buffer that provides the data for the write
 *   offset   - Data offset in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_write(struct sam_nandcs_s *priv, uint32_t rowaddr,
                      uint32_t coladdr, uint8_t *buffer,
                      uint16_t buflen, uint16_t offset)
{
  uintptr_t dest;
  int ret = OK;

  nand_dump("NAND Write", buffer, buflen);

  dest =  priv->raw.dataaddr + offset;

  /* Apply the offset to the destination address */

  WRITE_COMMAND8(&priv->raw, COMMAND_WRITE_1);
  WRITE_ADDRESS8(&priv->raw, coladdr);         /* 1st cycle column addr */
  WRITE_ADDRESS8(&priv->raw, coladdr >> 8);    /* 2nt cycle column addr */
  WRITE_ADDRESS8(&priv->raw, rowaddr);         /* 3rd cycle row addr */
  WRITE_ADDRESS8(&priv->raw, rowaddr >> 8);    /* 4th cycle row addr */
  WRITE_ADDRESS8(&priv->raw, rowaddr >> 16);   /* 5st cycle row addr */

  volatile uint8_t *dest8  = (volatile uint8_t *)dest;
  for (; buflen > 0; buflen--)
    {
      *dest8 = *buffer++;
    }

  WRITE_COMMAND8(&priv->raw, COMMAND_WRITE_2);

  ret = nand_wait_ready(priv);

  return ret;
}

static int nand_read(struct sam_nandcs_s *priv, uint32_t rowaddr,
                     uint32_t coladdr, uint8_t *buffer,
                     uint16_t buflen, uint16_t offset)
{
  volatile uint8_t *src8  = (volatile uint8_t *)priv->raw.dataaddr + offset;
  uint8_t *dest8 = (uint8_t *)buffer;
  int remaining;
  int ret = OK;

  WRITE_COMMAND8(&priv->raw, COMMAND_READ_1);
  WRITE_ADDRESS8(&priv->raw, coladdr);           /* 1st cycle column addr */
  WRITE_ADDRESS8(&priv->raw, coladdr >> 8);      /* 2nt cycle column addr */
  WRITE_ADDRESS8(&priv->raw, rowaddr);           /* 3rd cycle row addr */
  WRITE_ADDRESS8(&priv->raw, rowaddr >> 8);      /* 4th cycle row addr */
  WRITE_ADDRESS8(&priv->raw, rowaddr >> 16);     /* 5st cycle row addr */
  WRITE_COMMAND8(&priv->raw, COMMAND_READ_2);
  up_udelay(10);
  while (!sam_gpioread(priv->rb));

  remaining = buflen;
  for (; remaining > 0; remaining--)
    {
      *dest8++ = *src8;
    }

  nand_dump("NAND Read", buffer, buflen);
  return ret;
}

/****************************************************************************
 * Name: nand_rawread
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  This is a raw read of the flash contents.
 *
 * Input Parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_rawread(struct nand_raw_s *raw, off_t block,
                        unsigned int page, void *data, void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  uint16_t pagesize;
  uint16_t sparesize;
  off_t rowaddr;
  off_t coladdr;
  int ret = OK;

  DEBUGASSERT(priv && (data || spare));

  /* Get page and spare sizes */

  pagesize  = nandmodel_getpagesize(&priv->raw.model);
  sparesize = nandmodel_getsparesize(&priv->raw.model);

  /* Calculate actual address of the page */

  rowaddr = block * nandmodel_pagesperblock(&priv->raw.model) + page;
  coladdr = data ? 0 : pagesize;
  fwarn("block=%jd page=%d rowaddr=%jd coladdr %jd data=%p spare=%p\n",
        (intmax_t)block, page, (intmax_t)rowaddr, (intmax_t)coladdr,
        data, spare);
  coladdr = (coladdr >> 8) & 0x4 ? coladdr & 0x83f : coladdr;
  if (data)
    {
      ret = nand_read(priv, rowaddr, coladdr, (uint8_t *)data, pagesize, 0);
      if (ret < 0)
        {
          ferr("ERROR: nand_nfcsram_read for data region failed: %d\n", ret);
          return ret;
        }
    }

  if (spare)
    {
      uint16_t offset = data ? pagesize : 0;
      ret = nand_read(priv, rowaddr, coladdr, (uint8_t *)spare, sparesize,
                                                               offset);
      if (ret < 0)
        {
          ferr("ERROR: nand_nfcsram_read for spare region failed: %d\n",
               ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nand_rawwrite
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   This is a raw write of the flash contents.
 *
 * Input Parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writing
 *   spare - Buffer containing the spare data to be written.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_rawwrite(struct nand_raw_s *raw, off_t block,
                         unsigned int page, const void *data,
                         const void *spare)
{
  struct sam_nandcs_s *priv = (struct sam_nandcs_s *)raw;
  uint16_t pagesize;
  uint16_t sparesize;
  off_t rowaddr;
  int ret = OK;

  DEBUGASSERT(raw);
  finfo("block=%d page=%d data=%p spare=%p\n",
        (int)block, page, data, spare);

  /* Get page and spare sizes */

  pagesize  = nandmodel_getpagesize(&priv->raw.model);
  sparesize = nandmodel_getsparesize(&priv->raw.model);

  rowaddr = block * nandmodel_pagesperblock(&priv->raw.model) + page;

  if (data)
    {
      ret = nand_write(priv, rowaddr, 0, (uint8_t *)data, pagesize, 0);
      if (ret < 0)
        {
          ferr("ERROR: Failed writing data area: %d\n", ret);
        }
    }

  if (spare)
    {
      ret = nand_write(priv, rowaddr, 0, (uint8_t *)spare, sparesize,
                                                           pagesize);
      if (ret < 0)
        {
          ferr("ERROR: Failed writing data spare: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_nand_initialize
 *
 * Description:
 *   Create and initialize an raw NAND device instance.  This driver
 *   implements the RAW NAND interface:  No software ECC or sparing is
 *   performed here.  Those necessary NAND features are provided by common,
 *   higher level NAND MTD layers found in drivers/mtd.
 *
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned Value:
 *   On success a non-NULL pointer to an MTD device structure is returned;
 *   NULL is returned on a failure.
 *
 ****************************************************************************/

struct mtd_dev_s *sam_nand_initialize(int cs)
{
  struct sam_nandcs_s *priv;
  struct mtd_dev_s *mtd;
  uintptr_t cmdaddr;
  uintptr_t addraddr;
  uintptr_t dataaddr;
  uint8_t ecctype;
  int ret;

  finfo("CS%d\n", cs);

  if (SAM_SMCCS_BASE(cs) == SAM_SMC_CS0_BASE)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs0nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_NCS0_NAND_CMDADDR;
      addraddr = BOARD_NCS0_NAND_ADDRADDR;
      dataaddr = BOARD_NCS0_NAND_DATAADDR;

      /* Pass on the configured ECC type */

      ecctype = SAM34_NCS0_ECCTYPE;
    }
  else
    {
      ferr("ERROR: CS%d unsupported or invalid\n", cs);
      return NULL;
    }

  /* Initialize the device structure */

  memset(priv, 0, sizeof(struct sam_nandcs_s));
  priv->raw.cmdaddr    = cmdaddr;
  priv->raw.addraddr   = addraddr;
  priv->raw.dataaddr   = dataaddr;
  priv->raw.ecctype    = ecctype;
  priv->raw.eraseblock = nand_eraseblock;
  priv->raw.rawread    = nand_rawread;
  priv->raw.rawwrite   = nand_rawwrite;

  priv->cs             = cs;
  priv->rb             = GPIO_SMC_RB;

  /* Initialize the NAND hardware for this CS */

  /**
   * Note: The initialization is shown for the reference purpose only, and
   * for other MCUs, refer to the Package and Pinout chapter of the
   * respective data sheet.
   *
   * The I/O pin initialization for the 8-bit NAND is connected to the NCS0:
   *
   * To initialize the 8-bit D0-D7 data bus, configure the Port C, PC0 to
   * PC7 in Peripheral-A mode
   *
   * To initialize the NANDOE,configure the Port C,PC9 in Peripheral-A mode
   *
   * To initialize the NANDWE,configure the Port C,PC10 in Peripheral-A mode
   *
   * To initialize the NANDCLE,configure the Port C,PC17 in Peripheral-A mode
   *
   * To initialize the NANDALE,configure the Port C,PC16 in Peripheral-A mode
   *
   * To initialize the R/nB, configure any PIO as an input pin with pull-up
   * enabled
   *
   * To initialize the nCE, configure any PIO as an output pin (refer to Tips
   * and Tricks for the supported nCE connection types)
   **/

  ret = board_nandflash_config(cs);
  if (ret < 0)
    {
      ferr("ERROR: board_nandflash_config failed for CS%d: %d\n",
           cs, ret);
      return NULL;
    }

  /* Reset the NAND FLASH part */

  nand_reset(priv);

  /* Probe the NAND part.  On success, an MTD interface that wraps
   * our raw NAND interface is returned.
   **/

  mtd = nand_initialize(&priv->raw);
  if (!mtd)
    {
      ferr("ERROR: CS%d nand_initialize failed\n", cs);
      return NULL;
    }

  /* Return the MTD wrapper interface as the MTD device */

  return mtd;
}
