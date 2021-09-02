/****************************************************************************
 * boards/mips/pic32mz/pic32mz-starterkit/src/pic32mz_spi.c
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
#include "pic32mz-starterkit.h"

#ifdef CONFIG_PIC32MZ_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Sure PIC32MZ
 *   board.
 *
 ****************************************************************************/

void weak_function pic32mz_spidev_initialize(void)
{
  /* Configure the SPI chip select GPIOs */

#warning "Missing logic"
}

/****************************************************************************
 * Name:  pic32mz_spiNselect, pic32mz_spiNstatus, and pic32mz_spiNcmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s
 *   (see include/nuttx/spi/spi.h).
 *   All other methods including pic32mz_spibus_initialize()) are provided
 *   by common PIC32MZ logic.
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mz_boardinitialize() to configure SPI/SPI chip
 *      select pins.
 *   2. Provide pic32mz_spiNselect() and pic32mz_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mz_spiNcmddata() functions in your board-specific logic.
 *      These functions will perform cmd/data selection operations using
 *      GPIOs in the way your board is configured.
 *   3. Add a call to pic32mz_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by pic32mz_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

struct spi_dev_s;

#ifdef CONFIG_PIC32MZ_SPI1
void  pic32mz_spi1select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI2
void  pic32mz_spi2select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI3
void  pic32mz_spi3select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI4
void  pic32mz_spi4select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI5
void  pic32mz_spi5select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi5cmddata(struct spi_dev_s *dev, uint32_t devid,
                        bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI6
void  pic32mz_spi6select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mz_spi6status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi6cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#endif /* CONFIG_PIC32MZ_SPI */
