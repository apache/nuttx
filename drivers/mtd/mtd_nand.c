/****************************************************************************
 * drivers/mtd/mtd_nand.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was based largely on Atmel sample code with modifications for
 * better integration with NuttX.  The Atmel sample code has a BSD
 * compatibile license that requires this copyright notice:
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
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/onfi.h>
#include <nuttx/mtd/nand_raw.h>
#include <nuttx/mtd/nand_scheme.h>
#include <nuttx/mtd/nand_model.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Success Values returned by the nand_checkblock function */

#define BADBLOCK        255
#define GOODBLOCK       254

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Sparing logic */

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int     nand_checkblock(FAR struct nand_dev_s *nand, off_t block);
static int     nand_devscan(FAR struct nand_dev_s *nand);
#else
#  define      nand_checkblock(n,b) (GOODBLOCK)
#  define      nand_devscan(n)
#endif

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_checkblock
 *
 * Description:
 *   Read and check for a bad block.
 *
 * Input Parameters:
 *   nand  - Pointer to a struct nand_dev_s instance.
 *   block - Number of block to check.
 *
 * Returned Value:
 *   Returns BADBLOCK if the given block of a nandflash device is bad;
 *   returns GOODBLOCK if the block is good; or returns negated errno
 *   value on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int nand_checkblock(FAR struct nand_dev_s *nand, off_t block)
{
  uint8_t spare[CONFIG_MTD_NAND_MAXPAGESPARESIZE];
  const struct nand_raw_s *raw;
  const struct nand_model_s *model;
  const struct nand_scheme_s *scheme;
  uint8_t marker;
  int ret;

  DEBUGASSERT(nand && nand->raw);

  /* Retrieve model scheme */

  raw    = nand->raw;
  model  = &raw->model;
  scheme = nandmodel_getscheme(model);

  /* Read spare area of first page of block */

  ret = NAND_READPAGE(raw, block, 0, 0, spare);
  if (ret < 0)
    {
      fdbg("ERROR: Cannot read page #0 of block #%d\n", block);
      return ret;
    }

  nandscheme_readbadblockmarker(scheme, spare, &marker);
  if (marker != 0xff)
    {
      return BADBLOCK;
    }

  /* Read spare area of second page of block */

  ret = NAND_READPAGE(raw, block, 1, 0, spare);
  if (ret < 0)
    {
      fdbg("ERROR: Cannot read page #1 of block #%d\n", block);
      return ret;
    }

  nandscheme_readbadblockmarker(scheme, spare, &marker);
  if (marker != 0xFF)
    {
      return BADBLOCK;
    }

  return GOODBLOCK;
}
#endif /* CONFIG_MTD_NAND_BLOCKCHECK */

/****************************************************************************
 * Name: nand_devscan
 *
 * Description:
 *   Scans the device to retrieve or create block status information.
 *
 * Input Parameters:
 *   nand - Pointer to a struct nand_dev_s instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int nand_devscan(FAR struct nand_dev_s *nand)
{
  FAR const struct nand_raw_s *raw;
  FAR const struct nand_model_s *model;
  off_t numBlocks;
  off_t block;
  int ret;

  DEBUGASSERT(nand && nand->raw);

  /* Retrieve model information */

  raw       = nand->raw;
  model     = &raw->model;

  numBlocks = nandmodel_getdevblocksize(model);

  /* Initialize block statuses */

  fvdbg("Retrieving bad block information ...\n");

  /* Retrieve block status from their first page spare area */

  for (block = 0; block < numBlocks; block++)
    {
      /* Read spare of first page */

      ret = nand_checkblock(nand, block);
      if (ret != GOODBLOCK)
        {
          if (ret == BADBLOCK)
            {
              fvdbg("Block %u is bad\n", (unsigned int)block);
            }
          else
            {
              fdbg("ERROR: Cannot retrieve info from block %u: %d\n",
                   (unsigned int)block, ret);
            }
        }
    }

  return OK;
}
#endif /* CONFIG_MTD_NAND_BLOCKCHECK */

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
  struct nand_raw_s *nand = (struct nand_raw_s *)dev;

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
  struct nand_raw_s *nand = (struct nand_raw_s *)dev;

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
  struct nand_raw_s *nand = (struct nand_raw_s *)dev;

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
  struct nand_raw_s *nand = (struct nand_raw_s *)dev;
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
 * Name: nand_initialize
 *
 * Description:
 *   Probe and initialize NAND.
 *
 * Input parameters:
 *   raw      - Lower-half, raw NAND FLASH interface
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *   model    - A pointer to the model data (probably in the raw MTD
 *              driver instance.
 *
 * Returned value.
 *   A non-NULL MTD driver intstance is returned on success.  NULL is
 *   returned on any failaure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *nand_initialize(FAR struct nand_raw_s *raw)
{
  FAR struct nand_dev_s *nand;
  struct onfi_pgparam_s onfi;
  int ret;

  fvdbg("cmdaddr=%p addraddr=%p dataaddr=%p\n",
        (FAR void *)raw->cmdaddr, (FAR void *)raw->addraddr,
        (FAR void *)raw->dataaddr);

  /* Check if there is NAND connected on the EBI */

  if (!onfi_ebidetect(raw->cmdaddr, raw->addraddr, raw->dataaddr))
    {
      fdbg("ERROR: No NAND device detected at: %p %p %p\n",
           (FAR void *)raw->cmdaddr, (FAR void *)raw->addraddr,
           (FAR void *)raw->dataaddr);
      return NULL;
    }

  /* Read the ONFI page parameters from the NAND device */

  ret = onfi_read(raw->cmdaddr, raw->addraddr, raw->dataaddr, &onfi);
  if (ret < 0)
    {
      uint32_t chipid;

      fvdbg("Failed to get ONFI page parameters: %d\n", ret);

      /* If the ONFI model is not supported, determine the NAND
       * model from a lookup of known FLASH parts.
       */

      chipid = nand_chipid(raw);
      if (nandmodel_find(g_nandmodels, NAND_NMODELS, chipid,
                         &raw->model))
       {
          fdbg("ERROR: Could not determine NAND model\n");
          return NULL;
        }
    }
  else
    {
      FAR struct nand_model_s *model = &raw->model;
      uint64_t size;

      fvdbg("Found ONFI compliant NAND FLASH\n");

      /* Construct the NAND model structure */

      model->devid     = onfi.manufacturer;
      model->options   = onfi.buswidth ? NANDMODEL_DATAWIDTH16 : NANDMODEL_DATAWIDTH8;
      model->pagesize  = onfi.pagesize;
      model->sparesize = onfi.sparesize;

      size             = (uint64_t)onfi.pagesperblock *
                         (uint64_t)onfi.blocksperlun *
                         (uint64_t)onfi.pagesize;
      DEBUGASSERT(size < (uint64_t)(1 << 21));

      model->devsize   = (uint16_t)(size >> 20);

      size             = (uint64_t)onfi.pagesperblock *
                         (uint64_t)onfi.pagesize;
      DEBUGASSERT(size < (uint64_t)(1 << 11));

      model->blocksize = (uint16_t)(size >> 10);

      switch (onfi.pagesize)
        {
          case 256:
            model->scheme = &g_nand_sparescheme256;
            break;

          case 512:
            model->scheme = &g_nand_sparescheme512;
            break;

          case 2048:
            model->scheme = &g_nand_sparescheme2048;
            break;

          case 4096:
            model->scheme = &g_nand_sparescheme4096;
            break;
        }

      /* Disable any internal, embedded ECC function */

      (void)onfi_embeddedecc(&onfi, cmdaddr, addraddr, dataaddr, false);
    }

  /* Allocate an NAND MTD device structure */

  nand = (FAR struct nand_dev_s *)kzalloc(sizeof(struct nand_dev_s));
  if (!nand)
    {
      fdbg("ERROR: Failed to allocate the NAND MTD device structure\n");
      return NULL;
    }

  /* Initialize the NAND MTD device structure */

  nand->mtd.erase  = nand_erase;
  nand->mtd.bread  = nand_bread;
  nand->mtd.bwrite = nand_bwrite;
  nand->mtd.ioctl  = nand_ioctl;
  nand->raw        = raw;

  /* Scan the device for bad blocks */

  ret = nand_devscan(nand);
  if (ret < 0)
    {
      fdbg("ERROR: nandspare_intialize failed\n", ret);
      kfree(nand);
      return NULL;
    }

  /* Return the implementation-specific state structure as the MTD device */

  return &nand->mtd;
}
