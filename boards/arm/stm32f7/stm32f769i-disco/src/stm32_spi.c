/****************************************************************************
 * boards/arm/stm32f7/stm32f769i-disco/src/stm32_spi.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_spi.h"

#include "stm32f769i-disco.h"

#if defined(CONFIG_STM32F7_SPI1) || defined(CONFIG_STM32F7_SPI2) || \
    defined(CONFIG_STM32F7_SPI3) || defined(CONFIG_STM32F7_SPI4) || \
    defined(CONFIG_STM32F7_SPI5)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f769i-disco
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
}

/****************************************************************************
 * Name:  stm32_spi1/2/3/4/5select and stm32_spi1/2/3/4/5status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   (including stm32_spibus_initialize()) are provided by common STM32
 *   logic. To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32F7_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32F7_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32F7_SPI4
void stm32_spi4select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32F7_SPI5
void stm32_spi5select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
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
#ifdef CONFIG_STM32F7_SPI1
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32F7_SPI2
int stm32_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32F7_SPI3
int stm32_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32F7_SPI4
int stm32_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32F7_SPI5
int stm32_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */
#endif /* CONFIG_STM32F7_SPI1 || ... CONFIG_STM32F7_SPI5 */
