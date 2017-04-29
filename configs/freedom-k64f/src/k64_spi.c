/************************************************************************************
 * configs/freedom-k64f/src/k64_spi.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "kinetis.h"
#include "freedom-k64f.h"

#if defined(CONFIG_KINETIS_SPI0) || defined(CONFIG_KINETIS_SPI1) || \
	defined(CONFIG_KINETIS_SPI2)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: k64_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the FREEDOM-K64F board.
 *
 ************************************************************************************/

void weak_function k64_spidev_initialize(void)
{
# warning "Missing logic"
}

/************************************************************************************
 * Name:  kinetis_spi[n]select, kinetis_spi[n]status, and kinetis_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including kinetis_spibus_initialize()) are provided by common Kinetis logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in kinetis_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide kinetis_spi[n]select() and kinetis_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      kinetis_spi[n]cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to kinetis_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by kinetis_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_KINETIS_SPI0
void kinetis_spi0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_KINETIS_SPI1
void kinetis_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_KINETIS_SPI2
void kinetis_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#endif /* CONFIG_KINETIS_SPI0 || CONFIG_KINETIS_SPI1 || CONFIG_KINETIS_SPI2 */
