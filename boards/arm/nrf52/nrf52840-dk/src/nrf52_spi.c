/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_spi.c
 *
 *   Copyright (C) 2019 Greg Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include "arm_arch.h"
#include "chip.h"
#include "nrf52_gpio.h"
#include "nrf52_spi.h"

#include "nrf52840-dk.h"
#include <arch/board/board.h>

#ifdef CONFIG_NRF52_SPI_MASTER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-144 board.
 *
 ****************************************************************************/

void nrf52_spidev_initialize(void)
{
#ifdef CONFIG_NRF52_SPI0_MASTER
#  ifdef CONFIG_LPWAN_SX127X
  /* Configure the SPI-based SX127X chip select GPIO */

  spiinfo("Configure GPIO for SX127X SPI1/CS\n");

  nrf52_gpio_config(GPIO_SX127X_CS);
  nrf52_gpio_write(GPIO_SX127X_CS, true);
#  endif
#endif
}

/****************************************************************************
 * Name:  nrf52_spi0/1/2/3/select and nrf52_spi0/1/2/3/status
 *
 * Description:
 *   The external functions, nrf52_spi0/1/2/3select and
 *   nrf52_spi0/1/2/3status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including nrf52_spibus_initialize()) are provided
 *   by common NRF52 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in nrf52_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide nrf52_spi0/1/2/3select() and nrf52_spi0/1/2/3status()
 *      functions in your board-specific logic. These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to nrf52_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by nrf52_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_NRF52_SPI0_MASTER
void nrf52_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");

  switch (devid)
    {
#ifdef CONFIG_LPWAN_SX127X
      case SPIDEV_LPWAN(0):
        {
          spiinfo("SX127X device %s\n",
                  selected ? "asserted" : "de-asserted");

          /* Set the GPIO low to select and high to de-select */

          nrf52_gpio_write(GPIO_SX127X_CS, !selected);
          break;
        }
#endif

      default:
        {
          break;
        }
    }
}

uint8_t nrf52_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
#ifdef CONFIG_LPWAN_SX127X
      case SPIDEV_LPWAN(0):
        {
          status |= SPI_STATUS_PRESENT;
          break;
        }
#endif

      default:
        {
          break;
        }
    }

  return status;
}
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
void nrf52_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf52_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_nrf52_SPI2_MASTER
void nrf52_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf52_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
void nrf52_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf52_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#endif /* CONFIG_NRF52_SPI_MASTER */
