/****************************************************************************
 * boards/mips/pic32mz/flipnclick-pic32mz/src/pic32mz_spi.c
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
#include "pic32mz_gpio.h"

#include "flipnclick-pic32mz.h"

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
#ifdef HAVE_SSD1306
  /* Configure the SSD1306 chip select and command/data GPIOs */

  pic32mz_configgpio(GPIO_SSD1306_CS);
  pic32mz_configgpio(GPIO_SSD1306_DC);
#endif
}

/****************************************************************************
 * Name:  pic32mz_spiNselect, pic32mz_spiNstatus, and pic32mz_spiNcmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s
 *   (see include/nuttx/spi/spi.h).
 *   All other methods including pic32mz_spibus_initialize()) are provided by
 *   common PIC32MZ logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mz_boardinitialize() to configure SPI/SPI chip
 *      select pins.
 *   2. Provide pic32mz_spiNselect() and pic32mz_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mz_spiNcmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
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
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

#if defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBC) || \
    defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBD)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Low:  the display is selected
       * High: the display is deselected
       */

      pic32mz_gpiowrite(GPIO_SSD1306_CS, !selected);
    }
#endif
}

uint8_t pic32mz_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBC) || \
    defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBD)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* High: the inputs are treated as display data.
       * Low:  the inputs are transferred to the command registers.
       */

      pic32mz_gpiowrite(GPIO_SSD1306_DC, !cmd);
    }

#endif
  return 0;
}
#endif
#endif /* CONFIG_PIC32MZ_SPI1 */

#ifdef CONFIG_PIC32MZ_SPI2
void  pic32mz_spi2select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

#if defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBA) || \
    defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBB)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Low:  the display is selected
       * High: the display is deselected
       */

      pic32mz_gpiowrite(GPIO_SSD1306_CS, !selected);
    }
#endif
}

uint8_t pic32mz_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBA) || \
    defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBB)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* High: the inputs are treated as display data.
       * Low:  the inputs are transferred to the command registers.
       */

      pic32mz_gpiowrite(GPIO_SSD1306_DC, !cmd);
    }
#endif

  return 0;
}
#endif
#endif /* CONFIG_PIC32MZ_SPI2 */

#ifdef CONFIG_PIC32MZ_SPI3
void  pic32mz_spi3select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");
}

uint8_t pic32mz_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif
#endif /* CONFIG_PIC32MZ_SPI3 */

#ifdef CONFIG_PIC32MZ_SPI4
void  pic32mz_spi4select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");
}

uint8_t pic32mz_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif
#endif /* CONFIG_PIC32MZ_SPI4 */

#ifdef CONFIG_PIC32MZ_SPI5
void  pic32mz_spi5select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");
}

uint8_t pic32mz_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif /* CONFIG_PIC32MZ_SPI5 */

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
#endif /* CONFIG_PIC32MZ_SPI6 */

/****************************************************************************
 * Name: pic32mz_spi1/2/...register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD driver when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s)
 *   must be implemented.  These functions implements the registercallback
 *   method of the SPI interface (see include/nuttx/spi/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_PIC32MZ_SPI1
int pic32mz_spi1register(struct spi_dev_s *dev,
                         spi_mediachange_t callback, void *arg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_PIC32MZ_SPI1 */

#ifdef CONFIG_PIC32MZ_SPI2
int pic32mz_spi2register(struct spi_dev_s *dev,
                         spi_mediachange_t callback, void *arg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_PIC32MZ_SPI2 */

#ifdef CONFIG_PIC32MZ_SPI3
int pic32mz_spi3register(struct spi_dev_s *dev,
                         spi_mediachange_t callback, void *arg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_PIC32MZ_SPI3 */

#ifdef CONFIG_PIC32MZ_SPI4
int pic32mz_spi4register(struct spi_dev_s *dev,
                         spi_mediachange_t callback, void *arg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_PIC32MZ_SPI4 */

#ifdef CONFIG_PIC32MZ_SPI5
int pic32mz_spi5register(struct spi_dev_s *dev,
                         spi_mediachange_t callback, void *arg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_PIC32MZ_SPI5 */

#ifdef CONFIG_PIC32MZ_SPI6
int pic32mz_spi6register(struct spi_dev_s *dev,
                         spi_mediachange_t callback, void *arg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif /* CONFIG_PIC32MZ_SPI6 */
#endif /* CONFIG_SPI_CALLBACK */

#endif /* CONFIG_PIC32MZ_SPI */
