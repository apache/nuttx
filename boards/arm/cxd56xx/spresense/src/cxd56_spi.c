/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_spi.c
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
#include "hardware/cxd56_spi.h"
#include "cxd56_clock.h"
#include "cxd56_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  cxd56_spi0/3/4/5select and cxd56_spi0/3/4/5status
 *
 * Description:
 *   The external functions, cxd56_spi1/2/3select and cxd56_spi1/2/3 status
 *   must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including cxd56_spibus_initialize()) are provided by
 *   common CXD56 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in cxd56_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide cxd56_spi0/3/4/5select() and cxd56_spi0/3/4/5status()
 *      functions in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to cxd56_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by cxd56_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_SPI0
void cxd56_spi0select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t cxd56_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_CXD56_SPI3
void cxd56_spi3select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  /* Disable clock gating (clock enable) */

  cxd56_spi_clock_gate_disable(3);

  if (selected)
    {
      putreg32(0, CXD56_SPI3_CS);
    }
  else
    {
      putreg32(1, CXD56_SPI3_CS);
    }

  /* Enable clock gating (clock disable) */

  cxd56_spi_clock_gate_enable(3);
}

uint8_t cxd56_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_CXD56_SPI4
void cxd56_spi4select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t cxd56_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#  if defined(CONFIG_CXD56_SPISD) && (CONFIG_CXD56_SPISD_SPI_CH == 4)
  ret = board_spisd_status(dev, devid);
#  endif
  return ret;
}
#endif

#ifdef CONFIG_CXD56_SPI5
void cxd56_spi5select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t cxd56_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#  if defined(CONFIG_CXD56_SPISD) && (CONFIG_CXD56_SPISD_SPI_CH == 5)
  ret = board_spisd_status(dev, devid);
#  endif
  return ret;
}
#endif
