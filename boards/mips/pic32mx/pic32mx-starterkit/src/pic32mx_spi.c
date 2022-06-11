/****************************************************************************
 * boards/mips/pic32mx/pic32mx-starterkit/src/pic32mx_spi.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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

#include "mips_internal.h"
#include "chip.h"
#include "pic32mx.h"
#include "pic32mx-starterkit.h"

#if defined(CONFIG_PIC32MX_SPI1) || defined(CONFIG_PIC32MX_SPI2) || \
    defined(CONFIG_PIC32MX_SPI3) || defined(CONFIG_PIC32MX_SPI4)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Sure PIC32MX
 *   board.
 *
 ****************************************************************************/

void weak_function pic32mx_spidev_initialize(void)
{
  /* Configure the SPI chip select GPIOs */

#warning "Missing logic"
}

/****************************************************************************
 * Name:  pic32mx_spiNselect, pic32mx_spiNstatus, and pic32mx_spiNcmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s
 *   (see include/nuttx/spi/spi.h).
 *   All other methods including pic32mx_spibus_initialize()) are provided by
 *   common PIC32MX logic.
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mx_boardinitialize() to configure SPI/SSP chip
 *      select pins.
 *   2. Provide pic32mx_spiNselect() and pic32mx_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mx_spiNcmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to pic32mx_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by pic32mx_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

struct spi_dev_s;

#ifdef CONFIG_PIC32MX_SPI1
void  pic32mx_spi1select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mx_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi1cmddata(struct spi_dev_s *dev,
                        uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI1
void  pic32mx_spi1select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mx_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI3
void  pic32mx_spi3select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mx_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI4
void  pic32mx_spi4select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mx_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#endif /* CONFIG_PIC32MX_SPI1..4 */
