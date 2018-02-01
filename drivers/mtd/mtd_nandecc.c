/****************************************************************************
 * drivers/mtd/mtd_nandecc.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/hamming.h>
#include <nuttx/mtd/nand_scheme.h>
#include <nuttx/mtd/nand_ecc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nandecc_readpage
 *
 * Description:
 *   Reads the data and/or spare areas of a page of a NAND FLASH chip and
 *   verifies that the data is valid using the ECC information contained in
 *   the spare area. If a buffer pointer is NULL, then the corresponding area
 *   is not saved.
 *
 * Input Parameters:
 *   nand  - Upper-half, NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   spare - Buffer where the spare area will be stored.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

int nandecc_readpage(FAR struct nand_dev_s *nand, off_t block,
                     unsigned int page, FAR void *data, FAR void *spare)
{
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  FAR const struct nand_scheme_s *scheme;
  unsigned int pagesize;
  unsigned int sparesize;
  int ret;

  finfo("block=%d page=%d data=%p spare=%d\n", (int)block, page, data, spare);

  /* Get convenience pointers */

  DEBUGASSERT(nand && nand->raw);
  raw   = nand->raw;
  model = &raw->model;

  /* Get size parameters */

  pagesize  = nandmodel_getpagesize(model);
  sparesize = nandmodel_getsparesize(model);

  /* Store code in spare buffer, either the buffer provided by the caller or
   * the scatch buffer in the raw NAND structure.
   */

  if (!spare)
    {
      spare = raw->spare;
      memset(spare, 0xff, sparesize);
    }

  /* Start by reading the spare data */

  ret = NAND_RAWREAD(raw, block, page, 0, spare);
  if (ret < 0)
    {
      ferr("ERROR: Failed to read page:d\n", ret);
      return ret;
    }

  /* Then reading the data */

  ret = NAND_RAWREAD(nand->raw, block, page, data, 0);
  if (ret < 0)
    {
      ferr("ERROR: Failed to read page:d\n", ret);
      return ret;
    }

  /* Retrieve ECC information from page */

  scheme = nandmodel_getscheme(model);
  nandscheme_readecc(scheme, spare, raw->ecc);

  /* Use the ECC data to verify the page */

  ret = hamming_verify256x(data, pagesize, raw->ecc);
  if (ret && (ret != HAMMING_ERROR_SINGLEBIT))
    {
      ferr("ERROR: Block=%d page=%d Unrecoverable error: %d\n",
           block, page, ret);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: nandecc_writepage
 *
 * Description:
 *   Writes the data and/or spare area of a NAND FLASH page after
 *   calculating an ECC for the data area and storing it in the spare. If no
 *   data buffer is provided, the ECC is read from the existing page spare.
 *   If no spare buffer is provided, the spare area is still written with the
 *   ECC information calculated on the data buffer.
 *
 * Input Parameters:
 *   nand  - Upper-half, NAND FLASH interface
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   spare - Buffer containing the spare data to be written.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

int nandecc_writepage(FAR struct nand_dev_s *nand, off_t block,
                      unsigned int page,  FAR const void *data,
                      FAR void *spare)
{
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  FAR const struct nand_scheme_s *scheme;
  unsigned int pagesize;
  unsigned int sparesize;
  int ret;

  finfo("block=%d page=%d data=%p spare=%d\n", (int)block, page, data, spare);

  /* Get convenience pointers */

  DEBUGASSERT(nand && nand->raw);
  raw   = nand->raw;
  model = &raw->model;

  /* Get size parameters */

  pagesize  = nandmodel_getpagesize(model);
  sparesize = nandmodel_getsparesize(model);

  /* Set hamming code set to 0xffff.. to keep existing bytes */

  memset(raw->ecc, 0xff, CONFIG_MTD_NAND_MAXSPAREECCBYTES);

  /* Compute ECC on the new data, if provided */

  if (data)
    {
      /* Compute hamming code on data */

      hamming_compute256x(data, pagesize, raw->ecc);
    }

  /* Store code in spare buffer, either the buffer provided by the caller or
   * the scatch buffer in the raw NAND structure.
   */

  if (!spare)
    {
      spare = raw->spare;
      memset(spare, 0xff, sparesize);
    }

  /* Write the ECC */

  scheme = nandmodel_getscheme(model);
  nandscheme_writeecc(scheme, spare, raw->ecc);

  /* Perform page write operation */

  ret = NAND_RAWWRITE(nand->raw, block, page, data, spare);
  if (ret < 0)
    {
      ferr("ERROR: Failed to write page:d\n", ret);
    }

  return ret;
}
