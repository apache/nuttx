/****************************************************************************
 * drivers/mtd/mtd_nand.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was based largely on Atmel sample code with modifications for
 * better integration with NuttX.  The Atmel sample code has a BSD
 * compatible license that requires this copyright notice:
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
#include <nuttx/mtd/nand_ecc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Success Values returned by the nand_checkblock function */

#define GOODBLOCK            254
#define BADBLOCK             255

/* Bad block marker */

#define NAND_BLOCKSTATUS_BAD 0xba

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* NAND locking */

static int      nand_lock(FAR struct nand_dev_s *nand);
#define         nand_unlock(n) nxsem_post(&(n)->exclsem)

/* Bad block checking */

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
static int     nand_checkblock(FAR struct nand_dev_s *nand, off_t block);
static int     nand_devscan(FAR struct nand_dev_s *nand);
#else
#  define      nand_checkblock(n,b) (GOODBLOCK)
#  define      nand_devscan(n) (0)
#endif

/* Misc. NAND helpers */

static uint32_t nand_chipid(struct nand_raw_s *raw);
static int      nand_eraseblock(FAR struct nand_dev_s *nand,
                  off_t block, bool scrub);
static int      nand_readpage(FAR struct nand_dev_s *nand, off_t block,
                  unsigned int page, FAR uint8_t *data);
static int      nand_writepage(FAR struct nand_dev_s *nand, off_t block,
                  unsigned int page, FAR const void *data);

/* MTD driver methods */

static int     nand_erase(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks);
static ssize_t nand_bread(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, uint8_t *buffer);
static ssize_t nand_bwrite(struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, const uint8_t *buffer);
static int     nand_ioctl(struct mtd_dev_s *dev, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_lock
 *
 * Description:
 *   Get exclusive access to the nand.
 *
 * Input Parameters:
 *   nand  - Pointer to a struct nand_dev_s instance.
 *   block - Number of block to check.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_lock(FAR struct nand_dev_s *nand)
{
  return nxsem_wait(&nand->exclsem);
}

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
  FAR const struct nand_scheme_s *scheme;
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  uint8_t marker;
  int ret;

  DEBUGASSERT(nand && nand->raw);

  /* Retrieve the model and scheme */

  raw    = nand->raw;
  model  = &raw->model;
  scheme = nandmodel_getscheme(model);

  /* Read spare area of first page of block */

  ret = NAND_RAWREAD(raw, block, 0, 0, spare);
  if (ret < 0)
    {
      ferr("ERROR: Failed to read page 0 of block %d\n", block);
      return ret;
    }

  nandscheme_readbadblockmarker(scheme, spare, &marker);
  if (marker != 0xff)
    {
      finfo("Page 0 block %d marker=%02x\n", block, marker);
      return BADBLOCK;
    }

  /* Read spare area of second page of block */

  ret = NAND_RAWREAD(raw, block, 1, 0, spare);
  if (ret < 0)
    {
      ferr("ERROR: Failed to read page 1 of block %d\n", block);
      return ret;
    }

  nandscheme_readbadblockmarker(scheme, spare, &marker);
  if (marker != 0xff)
    {
      finfo("Page 1 block %d marker=%02x\n", block, marker);
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
 *   Currently, this function does nothing but scan the NAND and eat up time.
 *   This is a goot thing to do if you are debugging NAND, but otherwise,
 *   just a waste of time.  This logic could, however, be integrated into
 *   some bad block checking logic at sometime in the future.
 *
 * Input Parameters:
 *   nand - Pointer to a struct nand_dev_s instance.
 *
 * Returned Value:
 *   OK (always)
 *
 ****************************************************************************/

#if defined(CONFIG_MTD_NAND_BLOCKCHECK) && defined(CONFIG_DEBUG_INFO) && \
    defined(CONFIG_DEBUG_FS)
static int nand_devscan(FAR struct nand_dev_s *nand)
{
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  off_t nblocks;
  off_t block;
#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_FS)
  off_t good;
  unsigned int ngood;
#endif
  int ret;

  DEBUGASSERT(nand && nand->raw);

  /* Retrieve model information */

  raw     = nand->raw;
  model   = &raw->model;

  nblocks = nandmodel_getdevblocks(model);

  /* Initialize block statuses */

  finfo("Retrieving bad block information. nblocks=%d\n", nblocks);

  /* Retrieve block status from their first page spare area */

#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_FS)
  ngood = 0;
#endif

  for (block = 0; block < nblocks; block++)
    {
      /* Read spare of first page */

      ret = nand_checkblock(nand, block);
      if (ret != GOODBLOCK)
        {
#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_FS)
          if (ngood > 0)
            {
              finfo("Good blocks: %u - %u\n", good, good + ngood);
              ngood = 0;
            }
#endif
          if (ret == BADBLOCK)
            {
              finfo("Block %u is bad\n", (unsigned int)block);
            }
          else
            {
              ferr("ERROR: Cannot retrieve info from block %u: %d\n",
                   (unsigned int)block, ret);
            }
        }
#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_FS)
      else
        {
           if (ngood == 0)
             {
               good = block;
             }

           ngood++;
        }
#endif
    }

#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_FS)
  if (ngood > 0)
    {
      finfo("Good blocks: %u - %u\n", good, good + ngood);
    }
#endif

  return OK;
}
#endif /* CONFIG_MTD_NAND_BLOCKCHECK &&  CONFIG_DEBUG_INFO && CONFIG_DEBUG_FS */

/****************************************************************************
 * Name: nand_chipid
 *
 * Description:
 *   Reads and returns the identifiers of a NAND FLASH chip
 *
 * Input Parameters:
 *   raw - Pointer to a struct nand_raw_s instance.
 *
 * Returned Value:
 *   id1|(id2<<8)|(id3<<16)|(id4<<24)
 *
 ****************************************************************************/

static uint32_t nand_chipid(struct nand_raw_s *raw)
{
  uint8_t id[5];

  DEBUGASSERT(raw);

  WRITE_COMMAND8(raw, COMMAND_READID);
  WRITE_ADDRESS8(raw, 0);

  id[0] = READ_DATA8(raw);
  id[1] = READ_DATA8(raw);
  id[2] = READ_DATA8(raw);
  id[3] = READ_DATA8(raw);
  id[4] = READ_DATA8(raw);

  finfo("Chip ID: %02x %02x %02x %02x %02x\n",
        id[0], id[1], id[2], id[3], id[4]);

  return  (uint32_t)id[0]        |
         ((uint32_t)id[1] << 8)  |
         ((uint32_t)id[2] << 16) |
         ((uint32_t)id[3] << 24);
}

/****************************************************************************
 * Name: nand_eraseblock
 *
 * Description:
 *   Erase one block.
 *
 * Input Parameters:
 *   nand   Pointer to a struct nand_dev_s instance
 *   block  Number of block to erase
 *   scrub  True: Erase bad blocks
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nand_eraseblock(FAR struct nand_dev_s *nand, off_t block,
                           bool scrub)
{
  uint8_t spare[CONFIG_MTD_NAND_MAXPAGESPARESIZE];
  FAR const struct nand_scheme_s *scheme;
  FAR struct nand_model_s *model;
  FAR struct nand_raw_s *raw;
  int ret;

  /* finfo("Block %d\n", block); */
  DEBUGASSERT(nand && nand->raw);

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
  if (scrub)
    {
      /* Check block status */

      if (nand_checkblock(nand, block) != GOODBLOCK)
        {
          finfo("Block is BAD\n");
          return -EAGAIN;
        }
    }
#endif

  /* Erase block */

  ret = NAND_ERASEBLOCK(nand->raw, block);
  if (ret < 0)
    {
      int tmp;

      ferr("ERROR: Cannot erase block %ld\n", (long)block);

      /* Retrieve the model and scheme */

      raw    = nand->raw;
      model  = &raw->model;
      scheme = nandmodel_getscheme(model);

      /* Try to mark the block as BAD */

      memset(spare, 0xff, CONFIG_MTD_NAND_MAXPAGESPARESIZE);
      nandscheme_writebadblockmarker(scheme, spare, NAND_BLOCKSTATUS_BAD);
      tmp = NAND_WRITEPAGE(nand->raw, block, 0, 0, spare);
      if (tmp < 0)
        {
          ferr("ERROR: Failed bo marke block %ld as BAD\n", (long)block);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nandecc_readpage
 *
 * Description:
 *   Reads the data area (only) of a page of a NAND FLASH into the
 *   provided buffer.
 *
 * Input Parameters:
 *   nand  - Upper-half, NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_readpage(FAR struct nand_dev_s *nand, off_t block,
                         unsigned int page, FAR uint8_t *data)
{
  finfo("block=%d page=%d data=%p\n", (int)block, page, data);

#ifdef CONFIG_MTD_NAND_BLOCKCHECK
  /* Check that the block is not BAD if data is requested */

  if (nand_checkblock(nand, block) != GOODBLOCK)
    {
      ferr("ERROR: Block is BAD\n");
      return -EAGAIN;
   }
#endif

#ifdef CONFIG_MTD_NAND_SWECC
  /* nandecc_readpage will handle the software ECC case */

  DEBUGASSERT(nand && nand->raw);
  if (nand->raw->ecctype == NANDECC_SWECC)
    {
      /* Read data with software ECC verification */

      return nandecc_readpage(nand, block, page, data, NULL);
    }

  /* The lower half will handle the No ECC and all hardware assisted
   * ECC calculations.
   */

  else
#endif
    {
      return NAND_READPAGE(nand->raw, block, page, data, NULL);
    }
}

/****************************************************************************
 * Name: nand_writepage
 *
 * Description:
 *   Writes the data area (only) of a page into NAND FLASH from the
 *   provided buffer.
 *
 * Input Parameters:
 *   nand  - Upper-half, NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

static int nand_writepage(FAR struct nand_dev_s *nand, off_t block,
                          unsigned int page, FAR const void *data)
{
#ifdef CONFIG_MTD_NAND_BLOCKCHECK
  /* Check that the block is good */

  if (nand_checkblock(nand, block) != GOODBLOCK)
    {
      ferr("ERROR: Block is BAD\n");
      return -EAGAIN;
    }
#endif

#ifdef CONFIG_MTD_NAND_SWECC
  /* nandecc_writepage will handle the software ECC case */

  DEBUGASSERT(nand && nand->raw);
  if (nand->raw->ecctype == NANDECC_SWECC)
    {
      /* Write data with software ECC calculation */

      return nandecc_writepage(nand, block, page, data, NULL);
    }

  /* The lower half will handle the No ECC and all hardware assisted
   * ECC calculations.
   */

  else
#endif
    {
      return NAND_WRITEPAGE(nand->raw, block, page, data, NULL);
    }
}

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
  FAR struct nand_dev_s *nand = (FAR struct nand_dev_s *)dev;
  size_t blocksleft = nblocks;
  int ret;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the NAND until we complete the erase */

  nand_lock(nand);
  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      ret = nand_eraseblock(nand, startblock, false);
      if (ret < 0)
        {
          ferr("ERROR: nand_eraseblock failed on block %ld: %d\n",
               (long)startblock, ret);
          nand_unlock(nand);
          return ret;
        }

      startblock++;
    }

  nand_unlock(nand);
  return (int)nblocks;
}

/****************************************************************************
 * Name: nand_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t nand_bread(struct mtd_dev_s *dev, off_t startpage,
                          size_t npages, FAR uint8_t *buffer)
{
  FAR struct nand_dev_s *nand = (FAR struct nand_dev_s *)dev;
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  unsigned int pagesperblock;
  unsigned int page;
  uint16_t pagesize;
  size_t remaining;
  off_t maxblock;
  off_t block;
  int ret;

  finfo("startpage: %ld npages: %d\n", (long)startpage, (int)npages);
  DEBUGASSERT(nand && nand->raw);

  /* Retrieve the model */

  raw    = nand->raw;
  model  = &raw->model;

  /* Get the number of pages in one block, the size of one page, and
   * the number of blocks on the device.
   */

  pagesperblock = nandmodel_pagesperblock(model);
  pagesize      = nandmodel_getpagesize(model);
  maxblock      = nandmodel_getdevblocks(model);

  /* Get the block and page offset associated with the startpage */

  block = startpage / pagesperblock;
  page  = startpage % pagesperblock;

  /* Lock access to the NAND until we complete the read */

  nand_lock(nand);

  /* Then read every page from NAND */

  for (remaining = npages; remaining > 0; remaining--)
    {
      /* Check for attempt to read beyond the end of NAND */

      if (block > maxblock)
        {
          ferr("ERROR: Read beyond the end of FLASH, block=%ld\n",
               (long)block);

          ret = -ESPIPE;
          goto errout_with_lock;
        }

      /* Read the next page from NAND */

      ret = nand_readpage(nand, block, page, buffer);
      if (ret < 0)
        {
          ferr("ERROR: nand_readpage failed block=%ld page=%d: %d\n",
               (long)block, page, ret);
          goto errout_with_lock;
        }

      /* Increment the page number.  If we exceed the number of
       * pages per block, then reset the page number and bump up
       * the block number.
       */

      if (++page >= pagesperblock)
        {
          page = 0;
          block++;
        }

      /* Increment the buffer point by the size of one page */

      buffer += pagesize;
    }

  nand_unlock(nand);
  return npages;

errout_with_lock:
  nand_unlock(nand);
  return ret;
}

/****************************************************************************
 * Name: nand_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t nand_bwrite(struct mtd_dev_s *dev, off_t startpage,
                           size_t npages, const uint8_t *buffer)
{
  FAR struct nand_dev_s *nand = (FAR struct nand_dev_s *)dev;
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  unsigned int pagesperblock;
  unsigned int page;
  uint16_t pagesize;
  size_t remaining;
  off_t maxblock;
  off_t block;
  int ret;

  finfo("startpage: %08lx npages: %d\n", (long)startpage, (int)npages);
  DEBUGASSERT(nand && nand->raw);

  /* Retrieve the model */

  raw    = nand->raw;
  model  = &raw->model;

  /* Get the number of pages in one block, the size of one page, and
   * the number of blocks on the device.
   */

  pagesperblock = nandmodel_pagesperblock(model);
  pagesize      = nandmodel_getpagesize(model);
  maxblock      = nandmodel_getdevblocks(model);

  /* Get the block and page offset associated with the startpage */

  block = startpage / pagesperblock;
  page  = startpage % pagesperblock;

  /* Lock access to the NAND until we complete the write */

  nand_lock(nand);

  /* Then write every page into NAND */

  for (remaining = npages; remaining > 0; remaining--)
    {
      /* Check for attempt to write beyond the end of NAND */

      if (block > maxblock)
        {
          ferr("ERROR: Write beyond the end of FLASH, block=%ld\n",
               (long)block);

          ret = -ESPIPE;
          goto errout_with_lock;
        }

      /* Write the next page into NAND */

      ret = nand_writepage(nand, block, page, buffer);
      if (ret < 0)
        {
          ferr("ERROR: nand_writepage failed block=%ld page=%d: %d\n",
               (long)block, page, ret);
          goto errout_with_lock;
        }

      /* Increment the page number.  If we exceed the number of
       * pages per block, then reset the page number and bump up
       * the block number.
       */

      if (++page >= pagesperblock)
        {
          page = 0;
          block++;
        }

      /* Increment the buffer point by the size of one page */

      buffer += pagesize;
    }

  nand_unlock(nand);
  return npages;

errout_with_lock:
  nand_unlock(nand);
  return ret;
}

/****************************************************************************
 * Name: nand_ioctl
 ****************************************************************************/

static int nand_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct nand_dev_s *nand = (FAR struct nand_dev_s *)dev;
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  DEBUGASSERT(nand && nand->raw);
  raw   = nand->raw;
  model = &raw->model;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information needed to know
               * the capacity and how to access the device.  Returns:
               *
               *   blocksize    Size of one read/write block in bytes
               *   erasesize    Size of one erase block in bytes
               *   neraseblocks The number of erase blocks in the device
               */

              geo->blocksize    = model->pagesize;
              geo->erasesize    = nandmodel_getbyteblocksize(model);
              geo->neraseblocks = nandmodel_getdevblocks(model);
              ret               = OK;
          }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = nand_erase(dev, 0, nandmodel_getdevblocks(model));
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
 * Input Parameters:
 *   raw      - Lower-half, raw NAND FLASH interface
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *   model    - A pointer to the model data (probably in the raw MTD
 *              driver instance.
 *
 * Returned Value:
 *   A non-NULL MTD driver intstance is returned on success.  NULL is
 *   returned on any failaure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *nand_initialize(FAR struct nand_raw_s *raw)
{
  FAR struct nand_dev_s *nand;
  struct onfi_pgparam_s onfi;
  int ret;

  finfo("cmdaddr=%p addraddr=%p dataaddr=%p\n",
        (FAR void *)raw->cmdaddr, (FAR void *)raw->addraddr,
        (FAR void *)raw->dataaddr);

  /* Check if there is NAND connected on the EBI */

  if (!onfi_ebidetect(raw->cmdaddr, raw->addraddr, raw->dataaddr))
    {
      ferr("ERROR: No NAND device detected at: %p %p %p\n",
           (FAR void *)raw->cmdaddr, (FAR void *)raw->addraddr,
           (FAR void *)raw->dataaddr);
      return NULL;
    }

  /* Read the ONFI page parameters from the NAND device */

  ret = onfi_read(raw->cmdaddr, raw->addraddr, raw->dataaddr, &onfi);
  if (ret < 0)
    {
      uint32_t chipid;

      finfo("Failed to get ONFI page parameters: %d\n", ret);

      /* If the ONFI model is not supported, determine the NAND
       * model from a lookup of known FLASH parts.
       */

      chipid = nand_chipid(raw);
      if (nandmodel_find(g_nandmodels, NAND_NMODELS, chipid,
                         &raw->model))
       {
          ferr("ERROR: Could not determine NAND model\n");
          return NULL;
        }
    }
  else
    {
      FAR struct nand_model_s *model = &raw->model;
      uint64_t size;

      finfo("Found ONFI compliant NAND FLASH\n");

      /* Construct the NAND model structure */

      model->devid     = onfi.manufacturer;
      model->options   = onfi.buswidth ? NANDMODEL_DATAWIDTH16 : NANDMODEL_DATAWIDTH8;
      model->pagesize  = onfi.pagesize;
      model->sparesize = onfi.sparesize;

      size             = (uint64_t)onfi.pagesperblock *
                         (uint64_t)onfi.blocksperlun *
                         (uint64_t)onfi.pagesize;

      DEBUGASSERT(size < ((uint64_t)1 << 36));
      model->devsize   = (uint16_t)(size >> 20);

      size             = (uint64_t)onfi.pagesperblock *
                         (uint64_t)onfi.pagesize;

      DEBUGASSERT(size < ((uint64_t)1 << 26));
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

      onfi_embeddedecc(&onfi, cmdaddr, addraddr, dataaddr, false);
    }

  /* Allocate an NAND MTD device structure */

  nand = (FAR struct nand_dev_s *)kmm_zalloc(sizeof(struct nand_dev_s));
  if (!nand)
    {
      ferr("ERROR: Failed to allocate the NAND MTD device structure\n");
      return NULL;
    }

  /* Initialize the NAND MTD device structure */

  nand->mtd.erase  = nand_erase;
  nand->mtd.bread  = nand_bread;
  nand->mtd.bwrite = nand_bwrite;
  nand->mtd.ioctl  = nand_ioctl;
  nand->raw        = raw;

  nxsem_init(&nand->exclsem, 0, 1);

  /* Scan the device for bad blocks */

  nand_devscan(nand);

  /* Return the implementation-specific state structure as the MTD device */

  return &nand->mtd;
}
