/****************************************************************************
 * configs/olimex-lpc-h3131/src/lpc31_mmcsd.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "lpc31.h"

#include "lpc_h3131.h"

#ifdef HAVE_MMCSD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_mmcsd_initialize
 *
 * Description:
 *   Create the SDIO-based MMC/SD device
 *
 ****************************************************************************/

int lpc31_mmcsd_initialize(int slot, int minor)
{
  FAR struct sdio_dev_s *sdio;
  int ret;

  /* First, get an instance of the SDIO interface */

  finfo("Initializing SDIO slot %d\n", slot);
  sdio = sdio_initialize(slot);
  if (!sdio)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n", slot);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the MMC/SD driver */

  finfo("Bind SDIO to the MMC/SD driver, minor=%d\n", minor);
  ret = mmcsd_slotinitialize(minor, sdio);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  finfo("Successfully bound SDIO to the MMC/SD driver\n");

  /* Then let's guess and say that there is a card in the slot.  I need to check to
   * see if the LPC-H3131 board supports a GPIO to detect if there is a card in
   * the slot.
   */

  sdio_mediachange(sdio, true);
  return OK;
}

#endif /* HAVE_MMCSD */
