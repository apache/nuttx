/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/mx8mp_spi.c
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

#include "arm_internal.h"
#include "chip.h"
#include "mx8mp_gpio.h"

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mx8mp_spi1/2/3select and mx8mp_spi1/2/3status
 *
 * Description:
 *   The external functions, mx8mp_spi1/2/3select and mx8mp_spi1/2/3status
 *   must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 ****************************************************************************/

#ifdef CONFIG_MX8MP_SPI1
void mx8mp_spi1_select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  /* nothing to do : CS handled by module */
}

uint8_t mx8mp_spi1_status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int mx8mp_spi1_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif

#ifdef CONFIG_MX8MP_SPI2
void mx8mp_spi2_select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  mx8mp_gpio_write(GPIO_SPI2_CS, !selected);
}

uint8_t mx8mp_spi2_status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int mx8mp_spi2_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif

#ifdef CONFIG_MX8MP_SPI3
void mx8mp_spi3_select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  mx8mp_gpio_write(GPIO_SPI3_CS, !selected);
}

uint8_t mx8mp_spi3_status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int mx8mp_spi3_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif
