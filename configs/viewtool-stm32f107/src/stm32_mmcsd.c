/****************************************************************************
 * config/viewtool-stm32f107/src/up_mmcsd.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "stm32_sdio.h"
#include "viewtool_stm32f107.h"

/* Only the STM32F103 supports the SDIO interface */

#ifdef CONFIG_ARCH_CHIP_STM32F103VC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#define HAVE_MMCSD           1 /* Assume that we have SD support */
#define STM32_MMCSDSLOTNO    0 /* There is only one slot */

/* Can't support MMC/SD features if the SDIO peripheral is disabled */

#ifndef CONFIG_STM32_SDIO
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_MMCSD
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires CONFIG_DISABLE_MOUNTPOINT=n
 *   and CONFIG_STM32_SDIO=y
 *
 ****************************************************************************/

int stm32_sdinitialize(int minor)
{
#ifdef HAVE_MMCSD
  FAR struct sdio_dev_s *sdio;
  int ret;

  /* Configure the card-detect GPIO */
#warning REVISIT: Missing logic

  /* First, get an instance of the SDIO interface */

  sdio = sdio_initialize(STM32_MMCSDSLOTNO);
  if (!sdio)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n", STM32_MMCSDSLOTNO);
      return -ENODEV;
    }

  finfo("Initialized SDIO slot %d\n", STM32_MMCSDSLOTNO);

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(minor, sdio);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO slot %d to the MMC/SD driver, minor=%d\n",
           STM32_MMCSDSLOTNO, minor);
    }

  finfo("Bound SDIO slot %d to the MMC/SD driver, minor=%d\n",
         STM32_MMCSDSLOTNO, minor);

  /* Then let's guess and say that there is a card in the slot.  I need to check to
   * see if the M3 Wildfire board supports a GPIO to detect if there is a card in
   * the slot.
   */
#warning REVISIT: Need to read the current state of the card-detect pin
#warning REVISIT: Need to support interrupts from the card-detect pin
  sdio_mediachange(sdio, true);
#endif
  return OK;
}

#endif /* CONFIG_ARCH_CHIP_STM32F103VC */

