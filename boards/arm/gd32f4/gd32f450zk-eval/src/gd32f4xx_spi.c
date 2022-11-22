/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f4xx_spi.c
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

#include "chip.h"
#include "gd32f4xx.h"
#include "gd32f450z_eval.h"

#if defined(CONFIG_SPI)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the GD32F450Z-EVAL.
 *
 ****************************************************************************/

void weak_function gd32_spidev_initialize(void)
{
#ifdef CONFIG_GD32F4_SPI0
  /* Configure SPI0 CS GPIO for output */

  gd32_gpio_config(GPIO_SPI0_CSPIN);
  gd32_gpio_write(GPIO_SPI0_CSPIN, 1);
#endif
#ifdef CONFIG_GD32F4_SPI1
  /* Configure SPI1 CS GPIO for output */

  gd32_gpio_config(GPIO_SPI1_CSPIN);
  gd32_gpio_write(GPIO_SPI1_CSPIN, 1);
#endif
#ifdef CONFIG_GD32F4_SPI2
  /* Configure SPI2 CS GPIO for output */

  gd32_gpio_config(GPIO_SPI2_CSPIN);
  gd32_gpio_write(GPIO_SPI2_CSPIN, 1);
#endif
#ifdef CONFIG_GD32F4_SPI3
  /* Configure SPI3 CS GPIO for output */

  gd32_gpio_config(GPIO_SPI3_CSPIN);
  gd32_gpio_write(GPIO_SPI3_CSPIN, 1);
#endif
#ifdef CONFIG_GD32F4_SPI4
  /* Configure SPI4 CS GPIO for output */

  gd32_gpio_config(GPIO_SPI4_CSPIN);
  gd32_gpio_write(GPIO_SPI4_CSPIN, 1);
#endif
#ifdef CONFIG_GD32F4_SPI5
  /* Configure SPI5 CS GPIO for output */

  gd32_gpio_config(GPIO_SPI5_CSPIN);
  gd32_gpio_write(GPIO_SPI5_CSPIN, 1);
#endif
}

/****************************************************************************
 * Name:  gd32_spi[n]select and gd32_spi[n]status
 *
 * Description:
 * The external functions, gd32_spi0-5select and gd32_spi0-5 status
 * must be provided by board-specific logic.  They are implementations of the
 * select and status methods of the SPI interface defined by struct spi_ops_s
 * (see include/nuttx/spi/spi.h).
 * All other methods (including gd32_spibus_initialize())  are provided by
 * common GD32F4 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in gd32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide gd32_spi[n]select() and gd32_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to gd32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by gd32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_SPI0
void gd32_spi0select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI0_CSPIN, !selected);
}

uint8_t gd32_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_GD32F4_SPI1
void gd32_spi1select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI1_CSPIN, !selected);
}

uint8_t gd32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_GD32F4_SPI2
void gd32_spi2select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI2_CSPIN, !selected);
}

uint8_t gd32_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_GD32F4_SPI3
void gd32_spi3select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI3_CSPIN, !selected);
}

uint8_t gd32_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_GD32F4_SPI4
void gd32_spi4select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI4_CSPIN, !selected);
}

uint8_t gd32_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_GD32F4_SPI5
void gd32_spi5select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI5_CSPIN, !selected);
}

uint8_t gd32_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: gd32_spi[n]cmddata
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
#ifdef CONFIG_GD32F4_SPI0
int gd32_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_GD32F4_SPI1
int gd32_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_GD32F4_SPI2
int gd32_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_GD32F4_SPI3
int gd32_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_GD32F4_SPI5
int gd32_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_GD32F4_SPI5
int gd32_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */

#endif /* defined(CONFIG_SPI) */
