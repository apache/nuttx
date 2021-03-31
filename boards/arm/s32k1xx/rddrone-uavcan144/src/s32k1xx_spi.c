/****************************************************************************
 * boards/arm/s32k1xx/rddrone-uavcan144/src/s32k1xx_spi.c
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
#include <nuttx/spi/spi_transfer.h>
#include <arch/board/board.h>

#include "arm_arch.h"

#include "s32k1xx_config.h"
#include "s32k1xx_lpspi.h"
#include "s32k1xx_pin.h"
#include "rddrone-uavcan144.h"

#if defined(CONFIG_S32K1XX_LPSPI0) || defined(CONFIG_S32K1XX_LPSPI1) || \
    defined(CONFIG_S32K1XX_LPSPI2)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the RDDRONE-UAVCAN144
 *   board.
 *
 ****************************************************************************/

void weak_function s32k1xx_spidev_initialize(void)
{
#ifdef CONFIG_S32K1XX_LPSPI0
  s32k1xx_pinconfig(PIN_LPSPI0_PCS);

#ifdef CONFIG_SPI_DRIVER
  struct spi_dev_s *g_lpspi0;
  g_lpspi0 = s32k1xx_lpspibus_initialize(0);

  if (!g_lpspi0)
    {
      spierr("ERROR: [boot] FAILED to initialize LPSPI0\n");
    }

  spi_register(g_lpspi0, 0);
#endif
#endif

#ifdef CONFIG_S32K1XX_LPSPI1
  s32k1xx_pinconfig(PIN_LPSPI1_PCS);

#ifdef CONFIG_SPI_DRIVER
  struct spi_dev_s *g_lpspi1;
  g_lpspi1 = s32k1xx_lpspibus_initialize(1);

  if (!g_lpspi1)
    {
      spierr("ERROR: [boot] FAILED to initialize LPSPI1\n");
    }

  spi_register(g_lpspi1, 1);
#endif
#endif

#ifdef CONFIG_S32K1XX_LPSPI2
  s32k1xx_pinconfig(PIN_LPSPI2_PCS);

#ifdef CONFIG_SPI_DRIVER
  struct spi_dev_s *g_lpspi2;
  g_lpspi2 = s32k1xx_lpspibus_initialize(2);

  if (!g_lpspi2)
    {
      spierr("ERROR: [boot] FAILED to initialize LPSPI2\n");
    }

  spi_register(g_lpspi2, 2);
#endif
#endif
}

/****************************************************************************
 * Name:  s32k1xx_lpspi0/1/2select and s32k1xx_lpspi0/1/2status
 *
 * Description:
 *   The external functions, s32k1xx_lpspi0/1/2select and
 *   s32k1xx_lpspi0/1/2status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including s32k1xx_lpspibus_initialize()) are provided
 *   by common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in s32k1xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k1xx_lpspi0/1/2select() and s32k1xx_lpspi0/1/2status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to s32k1xx_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by s32k1xx_lpspibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI0
void s32k1xx_lpspi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  s32k1xx_gpiowrite(PIN_LPSPI0_PCS, !selected);
}

uint8_t s32k1xx_lpspi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_S32K1XX_LPSPI1
void s32k1xx_lpspi1select(FAR struct spi_dev_s *dev,
                        uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  s32k1xx_gpiowrite(PIN_LPSPI1_PCS, !selected);
}

uint8_t s32k1xx_lpspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_S32K1XX_LPSPI2
void s32k1xx_lpspi2select(FAR struct spi_dev_s *dev,
                        uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  s32k1xx_gpiowrite(PIN_LPSPI2_PCS, !selected);
}

uint8_t s32k1xx_lpspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#endif /* CONFIG_S32K1XX_LPSPI0 || CONFIG_S32K1XX_LPSPI01 || CONFIG_S32K1XX_LPSPI2 */
