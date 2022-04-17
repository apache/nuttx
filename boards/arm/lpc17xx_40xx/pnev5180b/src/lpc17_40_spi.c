/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_spi.c
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
#include "lpc17_40_spi.h"
#include "lpc17_40_gpio.h"
#include "pnev5180b.h"

#if defined(CONFIG_LPC17_40_SPI)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pnev5180b_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select and card detect GPIO pins for the
 *   NXP PNEV5180B NFC Frontend Development Kit.
 *
 ****************************************************************************/

void weak_function pnev5180b_spidev_initialize(void)
{
}

/****************************************************************************
 * Name:  lpc17_40_spiselect and lpc17_40_spistatus
 *
 * Description:
 *   The external functions, lpc17_40_spiselect and  lpc17_40_spistatus
 *   must be provided by board-specific logic. They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   lpc17_40_spibus_initialize()) are provided by common LPC17xx/LPC40xx
 *   logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in lpc17_40_boardinitialize() to configure SPI/SSP chip
 *      select pins.
 *   2. Provide lpc17_40_spiselect and  lpc17_40_spistatus functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to lpc17_40_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by lpc17_40_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void lpc17_40_spiselect(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");
}

uint8_t lpc17_40_spistatus(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("devid: %d\n", (int)devid);
  return 0;
}

#endif /* CONFIG_LPC17_40_SPI */
