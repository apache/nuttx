/****************************************************************************
 * boards/arm/gd32w51x/gd-xd-w515-eval/src/gd32_spi.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "gd32w51x_gpio.h"
#include "gd-xd-w515-eval.h"

#ifdef CONFIG_GD32W51X_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spidev_initialize
 *
 * Description:
 *   Called to configure the SPI chip select and control GPIO pins of the
 *   board: SPI0 carries the ST7735S LCD and SPI1 the GSL6157 fingerprint
 *   sensor.
 *
 ****************************************************************************/

void gd32_spidev_initialize(void)
{
#ifdef CONFIG_GD32W51X_SPI0
  gd32_gpio_config(GPIO_LCD_CS);
  gd32_gpio_config(GPIO_LCD_DC);
#endif

#ifdef CONFIG_GD32W51X_SPI1
  /* Fingerprint sensor deselected and out of reset */

  gd32_gpio_config(GPIO_FP_CS);
  gd32_gpio_config(GPIO_FP_SHUTDOWN);
#endif
}

/****************************************************************************
 * Name:  gd32_spi0select, gd32_spi0status and gd32_spi0cmddata
 *
 * Description:
 *   The select, status and cmddata methods of SPI0.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32W51X_SPI0
void gd32_spi0select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %" PRIu32 " CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  if (devid == SPIDEV_DISPLAY(0))
    {
      gd32_gpio_write(GPIO_LCD_CS, !selected);
    }
}

uint8_t gd32_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int gd32_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* The data/command line is low for a command */

      gd32_gpio_write(GPIO_LCD_DC, !cmd);
      return OK;
    }

  return -ENODEV;
}
#endif

#ifdef CONFIG_SPI_CALLBACK
int gd32_spi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                      void *arg)
{
  return OK;
}
#endif
#endif /* CONFIG_GD32W51X_SPI0 */

/****************************************************************************
 * Name:  gd32_spi1select, gd32_spi1status and gd32_spi1cmddata
 *
 * Description:
 *   The select, status and cmddata methods of SPI1.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32W51X_SPI1
void gd32_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %" PRIu32 " CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_FP_CS, !selected);
}

uint8_t gd32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int gd32_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  /* The fingerprint sensor has no data/command line */

  return OK;
}
#endif

#ifdef CONFIG_SPI_CALLBACK
int gd32_spi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                      void *arg)
{
  return OK;
}
#endif
#endif /* CONFIG_GD32W51X_SPI1 */

#endif /* CONFIG_GD32W51X_SPI */
