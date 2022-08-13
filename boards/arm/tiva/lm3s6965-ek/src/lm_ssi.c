/****************************************************************************
 * boards/arm/tiva/lm3s6965-ek/src/lm_ssi.c
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
#include "tiva_gpio.h"
#include "lm3s6965-ek.h"

/* The LM3S6965 Eval Kit microSD CS is on SSI0 */

#if defined(CONFIG_TIVA_SSI0) /* || defined(CONFIG_TIVA_SSI1) */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_SPI enables debug output from this file */

#ifdef CONFIG_DEBUG_SPI
#  define ssi_dumpgpio(m) tiva_dumpgpio(SDCCS_GPIO, m)
#else
#  define ssi_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm_ssidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LM3S6965 Eval Kit.
 *
 ****************************************************************************/

void weak_function lm_ssidev_initialize(void)
{
  /* Configure the SPI-based microSD CS GPIO */

  ssi_dumpgpio("lm_ssidev_initialize() Entry)");
  tiva_configgpio(SDCCS_GPIO);
#ifdef CONFIG_NX_LCDDRIVER
  tiva_configgpio(OLEDCS_GPIO);
#endif
  ssi_dumpgpio("lm_ssidev_initialize() Exit");
}

/****************************************************************************
 * The external functions, tiva_ssiselect and tiva_ssistatus must be provided
 * by board-specific logic.  The are implementations of the select and status
 * methods SPI interface defined by struct spi_ops_s
 * (see include/nuttx/spi/spi.h).
 * All othermethods (including tiva_ssibus_initialize()) are provided by
 * common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide tiva_ssiselect() and tiva_ssistatus() functions in your
 *      board-specific logic.  This function will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. Add a call to tiva_ssibus_initialize() in your low level
 *      initialization logic
 *   3. The handle returned by tiva_ssibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void tiva_ssiselect(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  ssi_dumpgpio("tiva_ssiselect() Entry");

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Assert the CS pin to the card */

      tiva_gpiowrite(SDCCS_GPIO, !selected);
    }
#ifdef CONFIG_NX_LCDDRIVER
  else if (devid == SPIDEV_DISPLAY(0))
    {
      /* Assert the CS pin to the display */

      tiva_gpiowrite(OLEDCS_GPIO, !selected);
    }
#endif

  ssi_dumpgpio("tiva_ssiselect() Exit");
}

uint8_t tiva_ssistatus(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}

#endif /* CONFIG_TIVA_SSI0 || CONFIG_TIVA_SSI1 */
