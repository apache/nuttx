/****************************************************************************
 * boards/arm/stm32l4/b-l475e-iot01a/src/stm32_spi.c
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
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "stm32l4_gpio.h"
#include "stm32l4_spi.h"

#include "b-l475e-iot01a.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Currently no devices are defined on SPI1 or SPI2 */

#undef CONFIG_STM32L4_SPI1
#undef CONFIG_STM32L4_SPI2

/* Only the SPSGRF is currently supported on SPI3 */

#ifndef HAVE_SPSGRF
#  undef CONFIG_STM32L4_SPI3
#endif

#if defined(CONFIG_STM32L4_SPI1) || defined(CONFIG_STM32L4_SPI2) || defined(CONFIG_STM32L4_SPI3)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32L4_SPI1
struct spi_dev_s *g_spi1;
#endif

#ifdef CONFIG_STM32L4_SPI2
struct spi_dev_s *g_spi2;
#endif

#ifdef CONFIG_STM32L4_SPI3
struct spi_dev_s *g_spi3;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-F401RE and
 *   Nucleo-F411RE boards.
 *
 ****************************************************************************/

void weak_function stm32l4_spidev_initialize(void)
{
#ifdef CONFIG_STM32L4_SPI1
  /* Configure SPI-based devices */

  g_spi1 = stm32l4_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: [boot] FAILED to initialize SPI port 1\n");
    }

  /* Configure chip select GPIOs */
#endif

#ifdef CONFIG_STM32L4_SPI2
  /* Configure SPI-based devices */

  g_spi2 = stm32l4_spibus_initialize(2);

  /* Configure chip select GPIOs */
#endif

#ifdef CONFIG_STM32L4_SPI3
  /* Configure SPI-based devices */

  g_spi3 = stm32l4_spibus_initialize(3);

  /* Configure chip select GPIOs */

#ifdef HAVE_SPSGRF
  stm32l4_configgpio(GPIO_SPSGRF_CS);
#endif
#endif
}

/****************************************************************************
 * Name:  stm32l4_spi1/2/3select and stm32l4_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32l4_spi1/2/3select and
 *   stm32l4_spi1/2/3status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including up_spiinitialize()) are provided by
 *   common STM32 logic.  To use this common SPI logic on your  board:
 *
 *   1. Provide logic in stm32l4_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide stm32l4_spi1/2/3select() and stm32l4_spi1/2/3status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind
 *      the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SPI1
void stm32l4_spi1select(FAR struct spi_dev_s *dev,
                        uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32l4_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32L4_SPI2
void stm32l4_spi2select(FAR struct spi_dev_s *dev,
                        uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32l4_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32L4_SPI3
void stm32l4_spi3select(FAR struct spi_dev_s *dev,
                        uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");

#ifdef HAVE_SPSGRF
  if (devid == SPIDEV_WIRELESS(0))
    {
      stm32l4_gpiowrite(GPIO_SPSGRF_CS, !selected);
    }
#endif
}

uint8_t stm32l4_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32l4_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32L4_SPI1
int stm32l4_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef CONFIG_STM32L4_SPI2
int stm32l4_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef CONFIG_STM32L4_SPI3
int stm32l4_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_STM32L4_SPI1 || CONFIG_STM32L4_SPI2 || CONFIG_STM32L4_SPI3 */
