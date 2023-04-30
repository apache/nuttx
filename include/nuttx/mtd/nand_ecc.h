/****************************************************************************
 * include/nuttx/mtd/nand_ecc.h
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

#ifndef __INCLUDE_NUTTX_MTD_NAND_ECC_H
#define __INCLUDE_NUTTX_MTD_NAND_ECC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

/****************************************************************************
 * Public Function Prototypes
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
                     unsigned int page, FAR void *data, FAR void *spare);

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
 *   data  - Buffer containing the data to be writing
 *   spare - Buffer containing the spare data to be written.
 *
 * Returned Value:
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

int nandecc_writepage(FAR struct nand_dev_s *nand, off_t block,
                      unsigned int page,  FAR const void *data,
                      FAR void *spare);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MTD_NAND_ECC_H */
