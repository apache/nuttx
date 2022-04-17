/****************************************************************************
 * boards/arm/lpc31xx/olimex-lpc-h3131/src/lpc31_spi.c
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

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc31.h"
#include "lpc_h3131.h"

#ifdef CONFIG_LPC31_SPI
#if 0

/* At present, LPC-H3131 specific logic is hard-coded in the file lpc31_spi.c
 * in arch/arm/src/lpc31xx
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC-H3131 board.
 *
 ****************************************************************************/

void weak_function lpc31_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI has already been provided.
   * Pin configuration is performed on-the-fly, so no additional setup
   * is required.
   */
}

/****************************************************************************
 * Name:  lpc31_spiselect and lpc31_spistatus
 *
 * Description:
 *   The external functions, lpc31_spiselect and lpc31_spistatus must be
 *   provided by board-specific logic.  They are implementations of the
 *   select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including lpc31_spibus_initialize()) are provided
 *   by common LPC31XX logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in lpc31_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide lpc31_spiselect() and lpc31_spistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to lpc31_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by lpc31_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI
 *      driver to the SPI MMC/SD driver).
 *
 ****************************************************************************/

void lpc31_spiselect(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t lpc31_spistatus(struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

#endif /* 0 */
#endif /* CONFIG_LPC31_SPI  */
