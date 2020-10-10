/****************************************************************************
 * boards/arm/stm32f0l0g0/b-l072z-lrwan1/src/stm32_spi.c
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

#include "arm_arch.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "b-l072z-lrwan1.h"
#include <arch/board/board.h>

#ifdef CONFIG_STM32F0L0G0_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-144 board.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI2 was already provided in stm32_rcc.c.
   *       Configurations of SPI pins is performed in stm32_spi.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

#ifdef CONFIG_STM32F0L0G0_SPI1
#  ifdef CONFIG_LPWAN_SX127X
  /* Configure the SPI-based SX127X chip select GPIO */

  spiinfo("Configure GPIO for SX127X SPI1/CS\n");

  stm32_configgpio(GPIO_SX127X_CS);
  stm32_gpiowrite(GPIO_SX127X_CS, true);
#  endif
#endif
}

/****************************************************************************
 * Name: stm32_spi1/2/select and stm32_spi1/2/status
 *
 * Description:
 *   The external functions, stm32_spi1/2select and stm32_spi1/2status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).  All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2select() and stm32_spi1/2status() functions
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

#ifdef CONFIG_STM32F0L0G0_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

  switch (devid)
    {
#ifdef CONFIG_LPWAN_SX127X
      case SPIDEV_LPWAN(0):
        {
          spiinfo("SX127X device %s\n",
                  selected ? "asserted" : "de-asserted");

          /* Set the GPIO low to select and high to de-select */

          stm32_gpiowrite(GPIO_SX127X_CS, !selected);
          break;
        }
#endif

      default:
        {
          break;
        }
    }
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
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
#endif /* CONFIG_STM32F0L0G0_SPI1 */

#ifdef CONFIG_STM32F0L0G0_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_STM32F0L0G0_SPI2 */

#endif /* CONFIG_STM32F0L0G0_SPI */
