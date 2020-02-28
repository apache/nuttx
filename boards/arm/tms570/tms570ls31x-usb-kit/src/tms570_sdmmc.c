/*****************************************************************************
 * boards/arm/tms570/tms570ls31x-usb-kit/src/tms570_mmcsd.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
#include <nuttx/spi/spi.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>

#include "tms570ls31x_usb_kit.h"
#include "tms570_spi.h"

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_TMS570_SPI1
#  error "SD driver requires CONFIG_STM32_SPI1 to be enabled"
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
 * Name: tms570_spi1register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

int tms570_spi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  spiinfo("INFO: Registering spi1 device\n");
  return OK;
}

/*****************************************************************************
 * Name: tms570_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 ****************************************************************************/

int tms570_mmcsd_initialize(int minor)
{
  struct spi_dev_s *spi;
  int rv;

  mcinfo("INFO: Initializing mmcsd card\n");

  spi = tms570_spibus_initialize(SD_SPI_PORT);
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
