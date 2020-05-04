/*****************************************************************************
 * boards/arm/imxrt/imxrt1050-evk/src/imxrt_mmcsd_spi.c
 *
 *   Copyright (C) 2018 Greg Nutt. All rights reserved.
 *   Author: ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
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

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/mmcsd.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "imxrt_lpspi.h"

#ifdef CONFIG_MMCSD_SPI

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_IMXRT_LPSPI1
#  error "SD driver requires CONFIG_IMXRT_LPSPI1 to be enabled"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

/*****************************************************************************
 * Private Definitions
 ****************************************************************************/

static const int SD_SPI_PORT = CONFIG_NSH_MMCSDSPIPORTNO; /* SD is connected to SPI1 port */
static const int SD_SLOT_NO  = 0; /* There is only one SD slot */

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NOTE:  We are using a SDCard adapter/module without Card Detect pin!
 * Then we don't need to Card Detect callback here.
 */

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: imxrt_spi1register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI1
int imxrt_lpspi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  spiinfo("INFO: Registering spi1 device\n");
  return OK;
}
#endif

/*****************************************************************************
 * Name: imxrt_mmcsd_spi_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

int imxrt_mmcsd_spi_initialize(int minor)
{
  struct spi_dev_s *spi;
  int rv;

  mcinfo("INFO: Initializing mmcsd card\n");

  spi = imxrt_lpspibus_initialize(SD_SPI_PORT);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n", SD_SPI_PORT);
      return -ENODEV;
    }

  rv = mmcsd_spislotinitialize(minor, SD_SLOT_NO, spi);
  if (rv < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
            SD_SPI_PORT, SD_SLOT_NO);
      return rv;
    }

  spiinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}

#endif /* CONFIG_MMCSD_SPI */
