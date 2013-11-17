/****************************************************************************
 * arch/arm/src/sama5/sam_nand.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatibile license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2011, 2012, Atmel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_model.h>

#include <arch/board/board.h>

#include "sam_nand.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the raw NAND MTD device.  The struct
 * mtd_dev_s must appear at the beginning of the definition so that you can
 * freely cast between pointers to struct mtd_dev_s and struct nand_raw_s.
 */

struct nand_raw_s
{
  struct mtd_dev_s mtd;      /* Externally visible part of the driver */
  uint8_t cs;                /* Chip select number (0..3) */
  struct nand_model_s model; /* The NAND model */
  uintptr_t cmdaddr;         /* NAND command address base */
  uintptr_t addraddr;        /* NAND address address base */
  uintptr_t dataaddr;        /* NAND data address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int     nand_erase(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks);
static ssize_t nand_bread(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, uint8_t *buf);
static ssize_t nand_bwrite(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, const uint8_t *buf);
static int     nand_ioctl(struct mtd_dev_s *dev, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* These pre-allocated structures hold the state of the MTD driver for NAND
 * on CS0..3 as configured.
 */

#ifdef CONFIG_SAMA5_EBICS0_NAND
static struct nand_raw_s g_cs0nand;
#endif
#ifdef CONFIG_SAMA5_EBICS1_NAND
static struct nand_raw_s g_cs1nand;
#endif
#ifdef CONFIG_SAMA5_EBICS2_NAND
static struct nand_raw_s g_cs2nand;
#endif
#ifdef CONFIG_SAMA5_EBICS3_NAND
static struct nand_raw_s g_cs3nand;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int nand_erase(struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
  struct nand_raw_s *priv = (struct nand_raw_s *)dev;

  /* The interface definition assumes that all erase blocks are the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */
#warning Missing logic

  /* Erase the specified blocks and return status (OK or a negated errno) */

  return OK;
}

/****************************************************************************
 * Name: nand_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t nand_bread(struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, uint8_t *buf)
{
  struct nand_raw_s *priv = (struct nand_raw_s *)dev;

  /* The interface definition assumes that all read/write blocks are the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */

  /* Read the specified blocks into the provided user buffer and return status
   * (The positive, number of blocks actually read or a negated errno).
   */
#warning Missing logic

  return 0;
}

/****************************************************************************
 * Name: nand_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t nand_bwrite(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, const uint8_t *buf)
{
  struct nand_raw_s *priv = (struct nand_raw_s *)dev;

  /* The interface definition assumes that all read/write blocks are the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */

  /* Write the specified blocks from the provided user buffer and return status
   * (The positive, number of blocks actually written or a negated errno)
   */
#warning Missing logic

  return 0;
}

/****************************************************************************
 * Name: nand_ioctl
 ****************************************************************************/

static int nand_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  struct nand_raw_s *priv = (struct nand_raw_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information needed to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = 512;  /* Size of one read/write block */
              geo->erasesize    = 4096; /* Size of one erase block */
              geo->neraseblocks = 1024; /* Number of erase blocks */
              ret               = OK;
          }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = OK;
        }
        break;

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
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
 *   Create and initialize an NAND MTD device instance.  MTD devices are
 *   not registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 *   This MTD devices implements a RAW NAND interface:  No ECC or sparing is
 *   performed here.  Those necessary NAND features are provided by common,
 *   higher level MTD layers found in drivers/mtd.
 *
 * Input parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned value.
 *   On success a non-NULL pointer to an MTD device structure is returned;
 *   NULL is returned on a failure.
 *
 ****************************************************************************/

struct mtd_dev_s *sam_nand_initialize(int cs)
{
  struct nand_raw_s *priv;
  struct mtd_s *mtd;
  uintptr_t cmdaddr;
  uintptr_t addraddr;
  uintptr_t dataaddr;
  int ret;

  fvdbg("CS%d\n", cs);

  /* Select the device structure */

#ifdef CONFIG_SAMA5_EBICS0_NAND
  if (cs == HSMC_CS0)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs0nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_EBICS0_NAND_CMDADDR;
      addraddr = BOARD_EBICS0_NAND_ADDRADDR;
      dataaddr = BOARD_EBICS0_NAND_DATAADDR;
  else
#endif
#ifdef CONFIG_SAMA5_EBICS1_NAND
  if (cs == HSMC_CS1)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs1nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_EBICS1_NAND_CMDADDR;
      addraddr = BOARD_EBICS1_NAND_ADDRADDR;
      dataaddr = BOARD_EBICS1_NAND_DATAADDR;
    }
  else
#endif
#ifdef CONFIG_SAMA5_EBICS2_NAND
  if (cs == HSMC_CS2)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs2nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_EBICS2_NAND_CMDADDR;
      addraddr = BOARD_EBICS2_NAND_ADDRADDR;
      dataaddr = BOARD_EBICS2_NAND_DATAADDR;
    }
  else
#endif
#ifdef CONFIG_SAMA5_EBICS3_NAND
  if (cs == HSMC_CS3)
    {
      /* Refer to the pre-allocated NAND device structure */

      priv = &g_cs3nand;

      /* Set up the NAND addresses.  These must be provided in the board.h
       * header file.
       */

      cmdaddr  = BOARD_EBICS3_NAND_CMDADDR;
      addraddr = BOARD_EBICS3_NAND_ADDRADDR;
      dataaddr = BOARD_EBICS3_NAND_DATAADDR;
    }
  else
#endif
    {
      fdbg("ERROR: CS%d unsupported or invalid\n", cs);
      return NULL;
    }

  /* Initialize the device structure */

  memset(priv, 0, sizeof(struct nand_raw_s));
  priv->mtd.erase  = nand_erase;
  priv->mtd.bread  = nand_bread;
  priv->mtd.bwrite = nand_bwrite;
  priv->mtd.ioctl  = nand_ioctl;
  priv->cs         = cs;
  priv->cmdaddr    = cmdaddr;
  priv->addraddr   = addraddr;
  priv->dataaddr   = dataaddr;

  /* Initialize the NAND hardware */
  /* Perform board-specific SMC intialization for this CS */

  ret = board_nandflash_config(cs);
  if (ret < 0)
    {
      fdbg("ERROR: board_nandflash_config failed for CS%d: %d\n",
           cs, ret);
      return NULL;
    }

  /* Probe the NAND part.  On success, an MTD interface that wraps
   * our raw NAND interface is returned.
   */

  mtd = nand_initialize(&priv->mtd, cmdaddr, addraddr, dataaddr, &priv->model);
  if (!mtd)
    {
      fdbg("ERROR: CS%d nand_initialize failed: %d at (%p, %p, %p)\n",
           cs, (FAR void *)cmdaddr, (FAR void *)addraddr, (FAR void *)dataaddr);
      return NULL;
    }

#warning Missing logic

  /* Return the MTD wrapper interface as the MTD device */

  return mtd;
}
