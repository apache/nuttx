/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/src/imxrt_spi.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
 *            David Sidrane <david_s5@nscdg.com>
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
#include <arch/board/board.h>

#include "arm_arch.h"

#include "imxrt_config.h"
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "imxrt1060-evk.h"

#if defined(CONFIG_IMXRT_LPSPI1) || defined(CONFIG_IMXRT_LPSPI2) || \
    defined(CONFIG_IMXRT_LPSPI3) || defined(CONFIG_IMXRT_LPSPI4)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the imxrt1060-evk
 *   board.
 *
 ****************************************************************************/

void weak_function imxrt_spidev_initialize(void)
{
#ifdef CONFIG_IMXRT_LPSPI1
  imxrt_config_gpio(GPIO_LPSPI1_CS); /* LPSPI1 chip select */
  imxrt_config_gpio(GPIO_MMCSD_EN);
#endif
#ifdef CONFIG_IMXRT_LPSPI3
  imxrt_config_gpio(GPIO_LPSPI3_CS); /* LPSPI3 chip select */
#endif
}

/****************************************************************************
 * Name:  imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status
 *
 * Description:
 *  The external functions, imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status
 *  must be provided by board-specific logic.
 *  They are implementations of the select and status methods of the SPI
 *  interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *  All other methods (including imxrt_lpspibus_initialize())
 *  are provided by common STM32 logic.
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in imxrt_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide imxrt_lpspi1/2/3select() and imxrt_lpspi1/2/3status()
 *      functions in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to imxrt_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by imxrt_lpspibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI1
void imxrt_lpspi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI1_CS, !selected);
}

uint8_t imxrt_lpspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI2
void imxrt_lpspi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI2_CS, !selected);
}

uint8_t imxrt_lpspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI3
void imxrt_lpspi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI3_CS, !selected);
}

uint8_t imxrt_lpspi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI4
void imxrt_lpspi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI4_CS, !selected);
}

uint8_t imxrt_lpspi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: imxrt_lpspi1cmddata
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
#ifdef CONFIG_IMXRT_LPSPI1
int imxrt_lpspi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI2
int imxrt_lpspi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI3
int imxrt_lpspi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI4
int imxrt_lpspi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_IMXRT_LPSPI1 || CONFIG_IMXRT_LPSPI2 */
