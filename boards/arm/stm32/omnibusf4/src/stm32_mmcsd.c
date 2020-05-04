/*****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_mmcsd.c
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Copyright (C) 2017 Greg Nutt. All rights reserved.
 *   Author: Bill Gatliff <bgat@billgatliff.com>
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

#include "arm_arch.h"
#include "chip.h"
#include "stm32.h"

#include <arch/board/board.h>
#include "omnibusf4.h"

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32_spi1register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

int stm32_spi2register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  /* TODO: media change callback */

  return OK;
}

/*****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 ****************************************************************************/

int stm32_mmcsd_initialize(int port, int minor)
{
  struct spi_dev_s *spi;
  int rv;

  stm32_configgpio(GPIO_MMCSD_NCD);  /* SD_DET */
  stm32_configgpio(GPIO_MMCSD_NSS);  /* CS */

  mcinfo("INFO: Initializing mmcsd port %d minor %d SD_DET %x\n",
         port, minor, stm32_gpioread(GPIO_MMCSD_NCD));

  spi = stm32_spibus_initialize(port);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n", port);
      return -ENODEV;
    }

  rv = mmcsd_spislotinitialize(minor, minor, spi);
  if (rv < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
             port, minor);
      return rv;
    }

  spiinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}
