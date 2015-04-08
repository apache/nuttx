/****************************************************************************
 * include/nuttx/mtd/nand_model.h
 *
 * ONFI Support.  The Open NAND Flash Interface (ONFI) is an industry
 * Workgroup made up of more than 100 companies that build, design-in, or
 * enable NAND Flash memory. This file provides definitions for standardized
 * ONFI NAND interfaces.
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was based largely on Atmel sample code for the SAMA5D3x with
 * modifications for better integration with NuttX.  The Atmel sample code
 * has a BSD compatibile license that requires this copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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

#ifndef __INCLUDE_NUTTX_MTD_NAND_MODEL_H
#define __INCLUDE_NUTTX_MTD_NAND_MODEL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of NAND FLASH models inside the model list */

#define NAND_NMODELS 60

/* Bit definitions for the NAND model options field */

#define NANDMODEL_DATAWIDTH8  (0 << 0)  /* NAND uses an 8-bit databus */
#define NANDMODEL_DATAWIDTH16 (1 << 0)  /* NAND uses a 16-bit databus */
#define NANDMODEL_COPYBACK    (1 << 1)  /* NAND supports the copy-back function
                                         * (internal page-to-page copy) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Describes a particular model of NAND FLASH device. */

struct nand_model_s
{
  uint8_t  devid;         /* Identifier for the device */
  uint8_t  options;       /* Special options for the NandFlash */
  uint16_t pagesize;      /* Size of the data area of a page in bytes */
  uint16_t sparesize;     /* Size of the spare area of a page in bytes */
  uint16_t devsize;       /* Size of the device in MB */
  uint16_t blocksize;     /* Size of one block in kilobytes */

  /* Spare area placement scheme */

  FAR const struct nand_scheme_s *scheme;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* List of NandFlash models which can be recognized by the software */

EXTERN const struct nand_model_s g_nandmodels[NAND_NMODELS];

/****************************************************************************
 * Public Function Prototypes
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
 * Returned Values:
 *   OK is returned on success; -ENODEV is returned on failure.
 *
 ****************************************************************************/

int nandmodel_find(FAR const struct nand_model_s *modeltab, size_t size,
                   uint32_t chipid, FAR struct nand_model_s *model);

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
 * Returned Values:
 *   OK on success; -EPIPE on failure.
 *
 ****************************************************************************/

int nandmodel_translate(FAR const struct nand_model_s *model, off_t address,
                        size_t size, FAR off_t *block, off_t *page,
                        off_t *offset);

/****************************************************************************
 * Name: nandmodel_getscheme
 *
 * Description:
 *   Returns the spare area placement scheme used by a particular nandflash
 *   model.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *  Spare placement scheme
 *
 ****************************************************************************/

#define nandmodel_getscheme(m) ((m)->scheme)

/****************************************************************************
 * Name: nandmodel_getdevid
 *
 * Description:
 *   Returns the device ID of a particular NAND FLASH model.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *   Device ID
 *
 ****************************************************************************/

#define nandmodel_getdevid(m) ((m)->devid)

/****************************************************************************
 * Name: nandmodel_getdevblocks
 *
 * Description:
 *   Returns the number of blocks in the entire device.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *   Number of blocks in the device
 *
 ****************************************************************************/

#define nandmodel_getdevblocks(m) \
  ((off_t)((m)->devsize << 10) / (m)->blocksize)

/****************************************************************************
 * Name: nandmodel_getdevbytesize
 *
 * Description:
 *   Returns the size of the whole device in bytes (this does not include
 *   the size of the spare zones).
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *   Size of the device in bytes
 *
 ****************************************************************************/

#define nandmodel_getdevbytesize(m) (((uint64_t)((m)->devsize) << 20))

/****************************************************************************
 * Name: nandmodel_getdevmbsize
 *
 * Description:
 *   Returns the size of the whole device in Mega bytes (this does not
 *   include the size of the spare zones).
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *  size of the device in MB.
 *
 ****************************************************************************/

#define nandmodel_getdevmbsize(m) ((uint32_t)((m)->devsize))

/****************************************************************************
 * Name: nandmodel_pagesperblock
 *
 * Description:
 *   Returns the number of pages in one single block of a device.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *    Block size in pages
 *
 ****************************************************************************/

#define nandmodel_pagesperblock(m) \
  ((uint32_t)((m)->blocksize << 10) / (m)->pagesize)

/****************************************************************************
 * Name: nandmodel_getdevpagesize
 *
 * Description:
 *   Returns the number of pages in the entire device.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *   Number of pages in the device
 *
 ****************************************************************************/

#define nandmodel_getdevpagesize(m) \
  ((uint32_t)nandmodel_getdevblocks(m) * nandmodel_pagesperblock(m))

/****************************************************************************
 * Name: nandmodel_getbyteblocksize
 *
 * Description:
 *   Returns the size in bytes of one single block of a device. This does not
 *   take into account the spare zones size.
 *
 * Input Parameters:
 *  model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *  Block size in bytes
 *
 ****************************************************************************/

#define nandmodel_getbyteblocksize(m) ((unsigned int)(m)->blocksize << 10)

/****************************************************************************
 * Name: nandmodel_getpagesize
 *
 * Description:
 *   Returns the size of the data area of a page in bytes.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *   Size of data area in bytes
 *
 ****************************************************************************/

#define nandmodel_getpagesize(m) ((m)->pagesize)

/****************************************************************************
 * Name: nandmodel_getsparesize
 *
 * Description:
 *   Returns the size of the spare area of a page in bytes.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *  size of spare area in bytes
 *
 ****************************************************************************/

unsigned int nandmodel_getsparesize(FAR const struct nand_model_s *model);

/****************************************************************************
 * Name: nandmodel_getbuswidth
 *
 * Description:
 *   Returns the number of bits used by the data bus of a NAND FLASH device.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *  data width
 *
 ****************************************************************************/

#define nandmodel_getbuswidth(m) \
  (((m)->options & NANDMODEL_DATAWIDTH16) ? 16 : 8)

/****************************************************************************
 * Name: nandmodel_havesmallblocks
 *
 * Description:
 *   Returns true if the given NAND FLASH model uses the "small blocks/pages"
 *   command set; otherwise returns false.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *   Returns true if the given NAND FLASH model uses the "small blocks/pages"
 *   command set; otherwise returns false.
 *
 ****************************************************************************/

#define nandmodel_havesmallblocks(m) ((m)->pagesize <= 512)

/****************************************************************************
 * Name: nandmodel_havecopyback
 *
 * Description:
 *   Returns true if the device supports the copy-back operation. Otherwise
 *   returns false.
 *
 * Input Parameters:
 *   model  Pointer to a nand_model_s instance.
 *
 * Returned Values:
 *   Returns true if the device supports the copy-back operation. Otherwise
 *   returns false.
 *
 ****************************************************************************/

#define nandmodel_havecopyback(m) (((m)->options & NANDMODEL_COPYBACK) != 0)

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MTD_NAND_MODEL_H */
