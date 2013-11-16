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

#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/onfi.h>
#include <nuttx/mtd/nand_scheme.h>
#include <nuttx/mtd/nand_model.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *
 * Returned value.
 *   OK is returned on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nand_initialize(uintptr_t cmdaddr, uintptr_t addraddr, uintptr_t dataaddr)
{
  struct onfi_pgparam_s onfi;
  struct nand_model_s model;
  bool compatible;
  int ret;

  fvdbg("cmdaddr=%p addraddr=%p dataaddr=%p\n",
        (FAR void *)cmdaddr, (FAR void *)addraddr, (FAR void *)dataaddr);

  /* Check if there is NAND connected on the EBI */

  if (!onfi_ebidetect(cmdaddr, addraddr, dataaddr))
    {
      fdbg("ERROR: No NAND device detected at: %p %p %p\n",
           (FAR void *)cmdaddr, (FAR void *)addraddr, (FAR void *)dataaddr);
      return -ENODEV;
    }

  /* Read the ONFI page parameters from the NAND device */

  ret = onfi_read(cmdaddr, addraddr, dataaddr, &onfi);
  if (ret < 0)
    {
      fvdbg("ERROR: Failed to get ONFI page parameters: %d\n", ret);
      compatible = false;
    }
  else
    {
      uint64_t size;

      fvdbg("Found ONFI compliant NAND FLASH\n");
      compatible = true;

      /* Construct the NAND model structure */

      model.devid     = onfi.manufacturer;
      model.options   = onfi.buswidth ? NANDMODEL_DATAWIDTH16 : NANDMODEL_DATAWIDTH8;
      model.pagesize  = onfi.pagesize;
      model.sparesize = onfi.sparesize;

      size                  = (uint64_t)onfi.pagesperblock *
                              (uint64_t)onfi.blocksperlun *
                              (uint64_t)onfi.pagesize;
      DEBUGASSERT(size < (uint64_t)(1 << 21));

      model.devsize   = (uint16_t)(size >> 20);

      size                  = (uint64_t)onfi.pagesperblock *
                              (uint64_t)onfi.pagesize;
      DEBUGASSERT(size < (uint64_t)(1 << 11));

      model.blocksize = (uint16_t)(size >> 10);

      switch (onfi.pagesize)
        {
          case 256:
            model.scheme = &g_nand_sparescheme256;
            break;

          case 512:
            model.scheme = &g_nand_sparescheme512;
            break;

          case 2048:
            model.scheme = &g_nand_sparescheme2048;
            break;

          case 4096:
            model.scheme = &g_nand_sparescheme4096;
            break;
        }

      /* Disable any internal, embedded ECC function */

      (void)onfi_embeddedecc(&onfi, cmdaddr, addraddr, dataaddr, false);
    }

#warning Missing logic

  /* Return the implementation-specific state structure as the MTD device */

  return OK;
}
