/*****************************************************************************
 * boards/arm/max326xx/max32660-evsys/src/max326_mmcsd.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>

#include "max326_spim.h"

#ifdef CONFIG_MMCSD_SPI

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MAX326XX_HAVE_SPIM
#  error "SD driver requires SPI master mode support"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

#if defined(CONFIG_MAX326XX_SPIM0)
#  define MMCSD_PORT 0
#elif defined(CONFIG_MAX326XX_SPIM1)
#  define MMCSD_PORT 1
#endif

#define MMCSD_SLOT 0

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef MICROSD_CD
/* NOTE:  We are using a SDCard adapter/module without Card Detect pin!
 * Then we don't need to Card Detect callback here.
 */

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#  error "No card detect pin"
#endif
#else
#  error "Missing card detect logic"
#endif

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: max326_spiregister
 *
 * Description:
 *   Media change callback
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#if defined(CONFIG_MAX326XX_SPIM0)
int max326_spi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                        void *arg)
#elif defined(CONFIG_MAX326XX_SPIM1)
int max326_spi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                        void *arg)
#endif
{
  spiinfo("INFO:  SPI callback\n");
  return OK;
}
#endif

/*****************************************************************************
 * Name: max326_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

int max326_mmcsd_initialize(int minor)
{
  struct spi_dev_s *spi;
  int rv;

  mcinfo("INFO: Initializing mmcsd card\n");

  spi = max326_spibus_initialize(MMCSD_PORT);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n", MMCSD_PORT);
      return -ENODEV;
    }

  rv = mmcsd_spislotinitialize(minor, MMCSD_SLOT, spi);
  if (rv < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
            MMCSD_PORT, MMCSD_SLOT);
      return rv;
    }

  spiinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}

#endif /* CONFIG_MMCSD_SPI */
