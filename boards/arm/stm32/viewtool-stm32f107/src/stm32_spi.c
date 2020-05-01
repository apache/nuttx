/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/stm32_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "stm32.h"
#include "viewtool_stm32f107.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Viewtool stm32f103/107
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
  /* NOTE: Clocking for SPIx was already provided in stm32_rcc.c.
   *       Configurations of SPI pins is performed in stm32_spi.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

#if defined(CONFIG_STM32_SPI2) && defined(CONFIG_INPUT_ADS7843E)
  /* Configure the XPT2046 SPI2 CS pin as an output */

  stm32_configgpio(GPIO_LCDTP_CS);
#endif

#if defined(CONFIG_STM32_SPI3) && defined(CONFIG_SENSORS_MPL115A)
  /* Configure the MPL115A SPI3 CS pin as an output */

  stm32_configgpio(GPIO_MPL115A_CS);
#endif

#if defined(CONFIG_VIEWTOOL_FT80X_SPI1) || defined(CONFIG_VIEWTOOL_FT80X_SPI2)
  /* Configure the FT80x CS pin as an input */

  stm32_configgpio(GPIO_FT80X_CS);
#endif

#if defined(CONFIG_VIEWTOOL_MAX3421E_SPI1) || defined(CONFIG_VIEWTOOL_MAX3421E_SPI2)
  /* Configure the MAX3421E CS pin as an input */

  stm32_configgpio(GPIO_MAX3421E_CS);
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including stm32_spibus_initialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#ifdef CONFIG_VIEWTOOL_FT80X_SPI1
  /* Select/de-select the FT80x */

  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(GPIO_FT80X_CS, !selected);
    }
  else
#endif
#ifdef CONFIG_VIEWTOOL_MAX3421E_SPI1
  /* Select/de-select the MAX3421E */

  if (devid == SPIDEV_USBHOST(0))
    {
      stm32_gpiowrite(GPIO_MAX3421E_CS, !selected);
    }
  else
#endif
   {
    spierr("ERROR:  Unrecognized devid: %08lx\n", (unsigned long)devid);
   }
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#ifdef CONFIG_INPUT_ADS7843E
  /* Select/de-select the touchscreen */

  if (devid == SPIDEV_TOUCHSCREEN(0))
    {
      stm32_gpiowrite(GPIO_LCDTP_CS, !selected);
    }
  else
#endif
#ifdef CONFIG_VIEWTOOL_FT80X_SPI2
  /* Select/de-select the FT80x */

  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(GPIO_FT80X_CS, !selected);
    }
  else
#endif
#ifdef CONFIG_VIEWTOOL_MAX3421E_SPI2
  /* Select/de-select the MAX3421E */

  if (devid == SPIDEV_USBHOST(0))
    {
      stm32_gpiowrite(GPIO_MAX3421E_CS, !selected);
    }
  else
#endif
   {
    spierr("ERROR:  Unrecognized devid: %08lx\n", (unsigned long)devid);
   }
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 || CONFIG_STM32_SPI3*/
