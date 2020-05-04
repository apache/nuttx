/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32l4_spi.c
 *
 *   Copyright (C) 2014, 2018 Gregory Nutt. All rights reserved.
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

#include <arm_arch.h>
#include "chip.h"
#include <stm32l4.h>

#include "nucleo-l476rg.h"

#if defined(CONFIG_STM32L4_SPI1) || defined(CONFIG_STM32L4_SPI2) || \
    defined(CONFIG_STM32L4_SPI3)

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
 * Name: stm32l4_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-L476RG board.
 *
 ****************************************************************************/

void weak_function stm32l4_spiinitialize(void)
{
#ifdef CONFIG_STM32L4_SPI1
  /* Configure SPI-based devices */

  g_spi1 = stm32l4_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: FAILED to initialize SPI port 1\n");
    }

#ifdef HAVE_MMCSD_SPI
  stm32l4_configgpio(GPIO_SPI_CS_SD_CARD);
#endif

#ifdef CONFIG_LCD_PCD8544
  stm32l4_configgpio(STM32_LCD_CS);       /* PCD8544 chip select */
#endif
#endif

#ifdef CONFIG_STM32L4_SPI2
  /* Configure SPI-based devices */

  g_spi2 = stm32l4_spibus_initialize(2);

#ifdef CONFIG_WL_CC1101
  /* Setup CS, IRQ(gdo2) line IOs */

  stm32l4_configgpio(GPIO_CC1101_PWR);
  stm32l4_configgpio(GPIO_CC1101_CS);
  stm32l4_configgpio(GPIO_CC1101_GDO2);
#endif

#endif
}

/****************************************************************************
 * Name:  stm32l4_spi1/2/3select and stm32l4_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32l4_spi1/2/3select and stm32l4_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32l4_board_initialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32l4_spi1/2/3select() and stm32l4_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SPI1
void stm32l4_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#ifdef HAVE_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
      stm32l4_gpiowrite(GPIO_SPI_CS_SD_CARD, !selected);
    }
#endif

#ifdef CONFIG_LCD_PCD8544
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32l4_gpiowrite(STM32_LCD_CS, !selected);
    }
#endif
}

uint8_t stm32l4_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  #ifdef HAVE_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
       status |= SPI_STATUS_PRESENT;
    }
  #endif

  return status;
}
#endif

#ifdef CONFIG_STM32L4_SPI2
void stm32l4_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#ifdef CONFIG_WL_CC1101
  if (devid == SPIDEV_WIRELESS(5))
    {
      stm32l4_gpiowrite(GPIO_CC1101_CS, !selected);
    }
#endif
}

uint8_t stm32l4_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32L4_SPI3
void stm32l4_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)

  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
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
#ifdef CONFIG_LCD_PCD8544
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      stm32l4_gpiowrite(STM32_LCD_CD, !cmd);

      return OK;
    }
#endif

  return -ENODEV;
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
