/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_max7219_leds.c
 *
 *   Copyright (C) 2018 Alan Carvalho de Assis. All rights reserved.
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/leds/max7219.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "stm32f4discovery.h"

#if defined(CONFIG_SPI) && defined(CONFIG_STM32_SPI1) && \
    defined(CONFIG_LEDS_MAX7219)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX7219_SPI_PORTNO 1   /* On SPI1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_max7219init
 *
 * Description:
 *   Initialize the max7219 to control 7-segment numeric display
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/numdisp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_max7219init(FAR const char *devpath)
{
  FAR struct spi_dev_s *spi;
  int ret;

  spi = stm32_spibus_initialize(MAX7219_SPI_PORTNO);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  /* Register the MAX7219 Driver at the specified location. */

  ret = max7219_leds_register(devpath, spi);
  if (ret < 0)
    {
      lederr("ERROR: max7219_leds_register(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_LEDS_MAX7219 */
