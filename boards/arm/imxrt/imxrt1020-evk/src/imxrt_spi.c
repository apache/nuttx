/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/src/imxrt_spi.c
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
#include "imxrt_config.h"
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "imxrt1020-evk.h"

#if defined(CONFIG_IMXRT_LPSPI1) || defined(CONFIG_IMXRT_LPSPI2) || \
    defined(CONFIG_IMXRT_LPSPI3) || defined(CONFIG_IMXRT_LPSPI4)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the versiboard2.
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
 *   The external functions, imxrt_lpspi1/2/3select and
 *   imxrt_lpspi1/2/3status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined bystruct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including imxrt_lpspibus_initialize()) are
 *   provided by common logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in imxrt_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide imxrt_lpspi1/2/3select() and imxrt_lpspi1/2/3status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to imxrt_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by imxrt_lpspibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI1
void imxrt_lpspi1select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI1_CS, !selected);
}

uint8_t imxrt_lpspi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI2
void imxrt_lpspi2select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI2_CS, !selected);
}

uint8_t imxrt_lpspi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI3
void imxrt_lpspi3select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI3_CS, !selected);
}

uint8_t imxrt_lpspi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI4
void imxrt_lpspi4select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI4_CS, !selected);
}

uint8_t imxrt_lpspi4status(struct spi_dev_s *dev, uint32_t devid)
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
int imxrt_lpspi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI2
int imxrt_lpspi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI3
int imxrt_lpspi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI4
int imxrt_lpspi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_IMXRT_LPSPI1 || CONFIG_IMXRT_LPSPI2 */
