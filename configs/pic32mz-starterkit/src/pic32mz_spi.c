/************************************************************************************
 * configs/pic32mz-starterkit/src/pic32mz_spi.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include "pic32mz-starterkit.h"

#ifdef CONFIG_PIC32MZ_SPI

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mz_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Sure PIC32MZ board.
 *
 ************************************************************************************/

void weak_function pic32mz_spidev_initialize(void)
{
  /* Configure the SPI chip select GPIOs */

#warning "Missing logic"
}

/************************************************************************************
 * Name:  pic32mz_spiNselect, pic32mz_spiNstatus, and pic32mz_spiNcmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including pic32mz_spibus_initialize()) are provided by common PIC32MZ logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mz_boardinitialize() to configure SPI/SPI chip select
 *      pins.
 *   2. Provide pic32mz_spiNselect() and pic32mz_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mz_spiNcmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to pic32mz_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by pic32mz_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

struct spi_dev_s;

#ifdef CONFIG_PIC32MZ_SPI1
void  pic32mz_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI2
void  pic32mz_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI3
void  pic32mz_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI4
void  pic32mz_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI5
void  pic32mz_spi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI6
void  pic32mz_spi6select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi6status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi6cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#endif /* CONFIG_PIC32MZ_SPI */
