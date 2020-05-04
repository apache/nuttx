/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-f091rc/src/stm32_spi.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "nucleo-f091rc.h"
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
  /* NOTE: Clocking for SPI1 and/or SPI3 was already provided in stm32_rcc.c.
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

#endif /* CONFIG_STM32F0L0G0_SPI1 */
}

/****************************************************************************
 * Name: stm32_spi1/2/select and stm32_spi1/2/status
 *
 * Description:
 *   The external functions, stm32_spi1/2select and stm32_spi1/2status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).  All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.  To use this
 *   common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2select() and stm32_spi1/2status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to bind
 *      the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F0L0G0_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  switch (devid)
    {
#ifdef CONFIG_LPWAN_SX127X
      case SPIDEV_LPWAN(0):
        {
          spiinfo("SX127X device %s\n", selected ? "asserted" : "de-asserted");

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
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_STM32F0L0G0_SPI2 */

#endif
