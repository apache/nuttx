/****************************************************************************
 * drivers/mtd/mtd_nandmodel.c
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
#include <debug.h>

#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_model.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nandmodel_find
 *
 * Description:
 *   Looks for a nand_model_s corresponding to the given ID inside a list of
 *   model. If found, the model variable is filled with the correct values.
 *
 * Input Parameters:
 *   modeltab  List of nand_model_s instances.
 *   size      Number of models in list.
 *   chipid    Identifier returned by the Nand(id1|(id2<<8)|(id3<<16)|(id4<<24)).
 *   model     nand_model_s instance to update with the model parameters.
 *
 * Returned Value:
 *   OK is returned on success; -ENODEV is returned on failure.
 *
 ****************************************************************************/

int nandmodel_find(FAR const struct nand_model_s *modeltab, size_t size,
                   uint32_t chipid, FAR struct nand_model_s *model)
{
  uint8_t id2;
  uint8_t id4;
  bool found = false;
  int i;

  id2 = (uint8_t)(chipid >> 8);
  id4 = (uint8_t)(chipid >> 24);

  finfo("NAND ID is 0x%08x\n", (int)chipid);

  for (i = 0; i < size; i++)
    {
      if (modeltab[i].devid == id2)
        {
          finfo("NAND Model found: ID2=0x%02x ID4=0x%02x\n", id2, id4);
          found = true;

          if (model)
            {
              memcpy(model, &modeltab[i], sizeof(struct nand_model_s));

              if (model->blocksize == 0 || model->pagesize == 0)
                {
                  /* Decode the extended ID4
                   *
                   * ID4 D5  D4 BlockSize || D1  D0  PageSize
                   *     0   0   64K      || 0   0   1K
                   *     0   1   128K     || 0   1   2K
                   *     1   0   256K     || 1   0   4K
                   *     1   1   512K     || 1   1   8k
                   */

                  switch (id4 & 0x03)
                    {
                      case 0x00:
                        model->pagesize = 1024;
                        break;

                      case 0x01:
                        model->pagesize = 2048;
                        break;

                      case 0x02:
                        model->pagesize = 4096;
                        break;

                      case 0x03:
                        model->pagesize = 8192;
                        break;
                    }

                  switch (id4 & 0x30)
                    {
                      case 0x00:
                        model->blocksize = 64;
                        break;

                      case 0x10:
                        model->blocksize = 128;
                        break;

                      case 0x20:
                        model->blocksize = 256;
                        break;

                      case 0x30:
                        model->blocksize = 512;
                        break;
                    }
                }

              finfo("  devid:     0x%02x\n",  model->devid);
              finfo("  devsize:   %d (MB)\n", model->devsize);
              finfo("  blocksize: %d (KB)\n", model->blocksize);
              finfo("  pagesize:  %d (B)\n",  model->pagesize);
              finfo("  options:   0x%02x\n",  model->options);
            }
          break;
        }
    }

  /* Check if chip has been detected */

  return found ? OK : -ENODEV;
}

/****************************************************************************
 * Name: nandmodel_translate
 *
 * Description:
 *   Translates address/size access of a nand_model_s to block, page and
 *   offset values. The values are stored in the provided variables if their
 *   pointer is not 0.
 *
 * Input Parameters:
 *   model   Pointer to a nand_model_s instance.
 *   address Access address.
 *   size    Access size in bytes.
 *   block   Stores the first accessed block number.
 *   page    Stores the first accessed page number inside the first block.
 *   offset  Stores the byte offset inside the first accessed page.
 *
 * Returned Value:
 *   OK on success; -EPIPE on failure.
 *
 ****************************************************************************/

int nandmodel_translate(FAR const struct nand_model_s *model, off_t address,
                        size_t size, FAR off_t *block, off_t *page,
                        off_t *offset)
{
  size_t blocksize;
  size_t pagesize;
  off_t  tmpblock;
  off_t  tmppage;
  off_t  tmpoffset;

  /* Check that access is not too big */

  if ((address + size) > nandmodel_getdevbytesize(model))
    {
      finfo("nandmodel_translate: out-of-bounds access.\n");
      return -ESPIPE;
    }

  /* Get Nand info */

  blocksize = nandmodel_getbyteblocksize(model);
  pagesize  = nandmodel_getpagesize(model);

  /* Translate address */

  tmpblock  = address / blocksize;
  address  -= tmpblock * blocksize;

  tmppage   = address / pagesize;
  address  -= tmppage * pagesize;
  tmpoffset = address;

  /* Save results */

  if (block)
    {
      *block = tmpblock;
    }

  if (page)
    {
      *page = tmppage;
    }

  if (offset)
    {
      *offset = tmpoffset;
    }

  return OK;
}

/****************************************************************************
 * Name: nandmodel_getsparesize
 *
 * Description:
 *   Returns the size of the spare area of a page in bytes.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Value:
 *  size of spare area in bytes
 *
 ****************************************************************************/

unsigned int nandmodel_getsparesize(FAR const struct nand_model_s *model)
{
  if (model->sparesize)
    {
      return model->sparesize;
    }
  else
    {
      return (model->pagesize >> 5); /* Spare size is 16/512 of data size */
    }
}
