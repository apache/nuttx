/****************************************************************************
 * boards/z80/ez80/makerlisp/src/ez80_spimmcsd.c
 *
 *   Copyright (C) 2019 Greg Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>

#include "chip.h"
#include "ez80_spi.h"
#include "makerlisp.h"

#ifdef HAVE_MMCSD

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NOTES:
 *
 * 1. We are using a SDCard adapter/module without Card Detect pin!
 *    Then we don't need to Card Detect callback here.
 * 2. Media Change callbacks are not yet implemented in the SPI driver.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card.
 *
 ****************************************************************************/

int ez80_mmcsd_initialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  mcinfo("INFO: Initializing mmcsd card\n");

  /* Get/initialize the SPI interface */

  spi = ez80_spibus_initialize(1);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI\n");
      return -ENODEV;
    }

  /* Register the MMC/SD block driver for slot 0 with device minor number 0. */

  ret = mmcsd_spislotinitialize(0, 0, spi);
  if (ret < 0)
    {
      mcerr("ERROR: Failed to bind SPI to SD slot 0\n");
      return ret;
    }

  mcinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}

#endif /* HAVE_MMCSD */
