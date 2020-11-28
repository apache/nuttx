/****************************************************************************
 * boards/arm/stm32/stm32-l152re/src/stm32_spisd.c
 *
 *   Copyright 2019 Sony Home Entertainment & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *
 * Based on omnibusf4/src/stm32_mmcsd.c
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

/****************************************************************************
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
#include "stm32_spi.h"
#include "stm32_gpio.h"

#include <arch/board/board.h>
#include "nucleo-l152re.h"

#if defined(CONFIG_STM32_SPI1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:
 *   stm32_spi1register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

int stm32_spi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  /* TODO: media change callback */

  return OK;
}

void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                                                  bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
                                                     "de-assert");
#if defined(CONFIG_MMCSD_SPI)
  stm32_gpiowrite(GPIO_SPI1_CS, !selected);
#endif
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;
#if defined(CONFIG_MMCSD_SPI)
  if (devid == SPIDEV_MMCSD(0))
    {
      ret |= SPI_STATUS_PRESENT;
    }
#endif

  return ret;
}

int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool cmd)
{
  return -ENODEV;
}

/****************************************************************************
 * Name: stm32_spisd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 ****************************************************************************/

int stm32_spisd_initialize(int port, int minor)
{
  struct spi_dev_s *spi;
  int rv;
  stm32_configgpio(GPIO_SPI1_SCK);
  stm32_configgpio(GPIO_SPI1_CS);   /* Assign CS */
  stm32_gpiowrite(GPIO_SPI1_CS, 1); /* Ensure the CS is inactive */

  mcinfo("INFO: Initializing mmcsd port %d minor %d \n",
         port, minor);

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

#endif /* CONFIG_STM32_SPI1 */