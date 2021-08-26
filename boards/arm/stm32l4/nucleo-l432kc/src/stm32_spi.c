/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_spi.c
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
#include <nuttx/spi/spi_transfer.h>
#include <arch/board/board.h>

#include "chip.h"
#include <stm32l4.h>

#include "nucleo-l432kc.h"

#if defined(CONFIG_STM32L4_SPI1) || defined(CONFIG_STM32L4_SPI2)

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_spiregister
 *
 * Description:
 *   Called to register spi character driver of initialized spi device for the Nucleo-L432KC board.
 *
 ****************************************************************************/

void stm32l4_spiregister(void)
{
#ifdef CONFIG_STM32L4_SPI1
      int ret = spi_register(g_spi1, 1);
      if (ret < 0)
      {
        spierr("ERROR: FAILED to register driver of SPI port 1\n");
      }
#endif
#ifdef CONFIG_STM32L4_SPI2
      int ret = spi_register(g_spi2, 2);
      if (ret < 0)
      {
        spierr("ERROR: FAILED to register driver of SPI port 2\n");
      }
#endif
}

/****************************************************************************
 * Name: stm32l4_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-L432KC
 *   board.
 *
 ****************************************************************************/

void stm32l4_spiinitialize(void)
{
#ifdef CONFIG_STM32L4_SPI1
  /* Configure SPI1-based devices */

  g_spi1 = stm32l4_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: FAILED to initialize SPI port 1\n");
    }
  else
    {
      spiinfo("INFO: SPI port 1 initialized\n");
    }

  /* Setup CS, EN & IRQ line IOs */

#ifdef CONFIG_MTD_AT45DB
  stm32l4_configgpio(AT45DB_SPI1_CS);      /* FLASH chip select */
#endif
#endif

#ifdef CONFIG_STM32L4_SPI2
  /* Configure SPI2-based devices */

  g_spi2 = stm32l4_spibus_initialize(2);
  if (!g_spi2)
    {
      spierr("ERROR: FAILED to initialize SPI port 2\n");
    }
  else
    {
      spiinfo("INFO: SPI port 2 initialized\n");
    }

  /* Setup CS, EN & IRQ line IOs */
#endif
}

/****************************************************************************
 * Name:  stm32l4_spi1/2select and stm32l4_spi1/2status
 *
 * Description:
 *   The external functions, stm32l4_spi1/2select and stm32l4_spi1/2status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   up_spiinitialize()) are provided by common STM32 logic.  To use this
 *   common SPI logic on your board:
 *
 *   1. Provide logic in stm32l4_board_initialize() to configure SPI chip
 *      select pins.
 *   2. Provide stm32l4_spi1/2select() and stm32l4_spi1/2status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind
 *      the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SPI1
void stm32l4_spi1select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %08X CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

#ifdef CONFIG_MTD_AT45DB
  if (devid == SPIDEV_FLASH(0))
    {
      stm32l4_gpiowrite(AT45DB_SPI1_CS, !selected);
    }
#endif
}

uint8_t stm32l4_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32L4_SPI2
void stm32l4_spi2select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t stm32l4_spi2status(struct spi_dev_s *dev, uint32_t devid)
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
int stm32l4_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef CONFIG_STM32L4_SPI2
int stm32l4_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_STM32L4_SPI1 || CONFIG_STM32L4_SPI2 */
