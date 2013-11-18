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
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_raw.h>
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
 * nand_raw_s must appear at the beginning of the definition so that you can
 * freely cast between pointers to struct nand_raw_s and struct sam_rawnand_s.
 */

struct sam_rawnand_s
{
  struct nand_raw_s raw;     /* Externally visible part of the driver */
  uint8_t cs;                /* Chip select number (0..3) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* NAND Helpers */

static int nand_readpage_noecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, void *data, void *spare);
#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int nand_readpage_hwecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, void *data, void *spare);
static int nand_readpage_pmecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, void *data, void *spare);
#endif
static int nand_writepage_noecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare);
#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int nand_writepage_hwecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare);
static int nand_writepage_pmecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare);
#endif

/* MTD driver methods */

static int nand_eraseblock(struct nand_raw_s *raw, off_t block);
static int nand_readpage(struct nand_raw_s *raw, off_t block,
             unsigned int page, void *data, void *spare);
static int nand_writepage(struct nand_raw_s *raw, off_t block,
             unsigned int page, const void *data, const void *spare);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* These pre-allocated structures hold the state of the MTD driver for NAND
 * on CS0..3 as configured.
 */

#ifdef CONFIG_SAMA5_EBICS0_NAND
static struct sam_rawnand_s g_cs0nand;
#endif
#ifdef CONFIG_SAMA5_EBICS1_NAND
static struct sam_rawnand_s g_cs1nand;
#endif
#ifdef CONFIG_SAMA5_EBICS2_NAND
static struct sam_rawnand_s g_cs2nand;
#endif
#ifdef CONFIG_SAMA5_EBICS3_NAND
static struct sam_rawnand_s g_cs3nand;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_readpage_noecc
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  The raw NAND contents are returned with no ECC
 *   corrections.
 *
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_readpage_noecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, void *data, void *spare)
{
#warning Missing logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: nand_readpage_hwecc
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  HSIAO ECC is used
 *
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int nand_readpage_hwecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, void *data, void *spare)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_MTD_NAND_BLOCKCHECK */

/****************************************************************************
 * Name: nand_readpage_pmecc
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.  PMECC is used
 *
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int nand_readpage_pmecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, void *data, void *spare)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_MTD_NAND_BLOCKCHECK */

/****************************************************************************
 * Name: nand_writepage_noecc
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   No ECC calculations are performed.
 *
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   spare - Buffer conatining the spare data to be written.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_writepage_noecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare)
{
#warning Missing logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: nand_writepage_noecc
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   HSIAO ECC calculations are performed.
 *
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   spare - Buffer conatining the spare data to be written.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int nand_writepage_hwecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_MTD_NAND_BLOCKCHECK */

/****************************************************************************
 * Name: nand_writepage_noecc
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *   PMECC calculations are performed.
 *
 * Input parameters:
 *   priv  - Lower-half, private NAND FLASH device state
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   spare - Buffer conatining the spare data to be written.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int nand_writepage_pmecc(struct sam_rawnand_s *priv, off_t block,
             unsigned int page, const void *data, const void *spare)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_MTD_NAND_BLOCKCHECK */

/****************************************************************************
 * Name: nand_eraseblock
 *
 * Description:
 *   Erases the specified block of the device.
 *
 * Input parameters:
 *   raw    - Lower-half, raw NAND FLASH interface
 *   block  - Number of the physical block to erase.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_eraseblock(struct nand_raw_s *raw, off_t block)
{
  struct sam_rawnand_s *priv = (struct sam_rawnand_s *)raw;
  DEBUGASSERT(raw);
#warning Missing logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: nand_readpage
 *
 * Description:
 *   Reads the data and/or the spare areas of a page of a NAND FLASH into the
 *   provided buffers.
 *
 * Input parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_readpage(struct nand_raw_s *raw, off_t block,
                         unsigned int page, void *data, void *spare)
{
  struct sam_rawnand_s *priv = (struct sam_rawnand_s *)raw;
  DEBUGASSERT(raw);

#ifndef CONFIG_MTD_NAND_BLOCKCHECK
  return nand_readpage_noecc(priv, block, page, data, spare);
#else
  DEBUGASSERT(raw->ecctype != NANDECC_SWECC);
  switch (raw->ecctype)
    {
    case NANDECC_NONE:
      return nand_readpage_noecc(priv, block, page, data, spare);
    case NANDECC_HWECC:
      return nand_readpage_hwecc(priv, block, page, data, spare);
    case NANDECC_PMECC:
      return nand_readpage_pmecc(priv, block, page, data, spare);

    case NANDECC_SWECC:
    default:
      return -EINVAL;
    }
#endif
}

/****************************************************************************
 * Name: nand_writepage
 *
 * Description:
 *   Writes the data and/or the spare area of a page on a NAND FLASH chip.
 *
 * Input parameters:
 *   raw   - Lower-half, raw NAND FLASH interface
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   spare - Buffer conatining the spare data to be written.
 *
 * Returned value.
 *   OK is returned in succes; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_writepage(struct nand_raw_s *raw, off_t block,
                          unsigned int page, const void *data,
                          const void *spare)
{
  struct sam_rawnand_s *priv = (struct sam_rawnand_s *)raw;
  DEBUGASSERT(raw);

#ifndef CONFIG_MTD_NAND_BLOCKCHECK
  return nand_writepage_noecc(priv, block, page, data, spare);
#else
  DEBUGASSERT(raw->ecctype != NANDECC_SWECC);
  switch (raw->ecctype)
    {
    case NANDECC_NONE:
      return nand_writepage_noecc(priv, block, page, data, spare);
    case NANDECC_HWECC:
      return nand_writepage_hwecc(priv, block, page, data, spare);
    case NANDECC_PMECC:
      return nand_writepage_pmecc(priv, block, page, data, spare);

    case NANDECC_SWECC:
    default:
      return -EINVAL;
    }
#endif
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
  struct sam_rawnand_s *priv;
  struct mtd_dev_s *mtd;
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

  memset(priv, 0, sizeof(struct sam_rawnand_s));
  priv->raw.cmdaddr    = cmdaddr;
  priv->raw.addraddr   = addraddr;
  priv->raw.dataaddr   = dataaddr;
  priv->raw.eraseblock = nand_eraseblock;
  priv->raw.readpage   = nand_readpage;
  priv->raw.writepage  = nand_writepage;
  priv->cs             = cs;

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

  mtd = nand_initialize(&priv->raw);
  if (!mtd)
    {
      fdbg("ERROR: CS%d nand_initialize failed %d\n", cs);
      return NULL;
    }

#warning Missing logic

  /* Return the MTD wrapper interface as the MTD device */

  return mtd;
}
