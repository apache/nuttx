/****************************************************************************
 * boards/arm/s32k1xx/s32k148evb/src/s32k1xx_spi.c
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
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "s32k1xx_pin.h"
#include "s32k1xx_lpspi.h"

#include <arch/board/board.h>

#include "s32k148evb.h"

#ifdef CONFIG_S32K1XX_LPSPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Configure chip select pins, initialize the SPI driver and register
 *   /dev/spiN devices.
 *
 ****************************************************************************/

int weak_function s32k1xx_spidev_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_S32K1XX_LPSPI0
  /* LPSPI0 *****************************************************************/

  /* Configure LPSPI0 peripheral chip select */

  s32k1xx_pinconfig(PIN_LPSPI0_PCS);

#  ifdef CONFIG_SPI_DRIVER
  /* Initialize the SPI driver for LPSPI0 */

  struct spi_dev_s *g_lpspi0 = s32k1xx_lpspibus_initialize(0);
  if (g_lpspi0 == NULL)
    {
      spierr("ERROR: FAILED to initialize LPSPI0\n");
      return -ENODEV;
    }

  ret = spi_register(g_lpspi0, 0);
  if (ret < 0)
    {
      spierr("ERROR: FAILED to register LPSPI0 driver\n");
      return ret;
    }
#  endif /* CONFIG_SPI_DRIVER */
#endif /* CONFIG_S32K1XX_LPSPI0 */

#ifdef CONFIG_S32K1XX_LPSPI1
  /* LPSPI1 *****************************************************************/

  /* Configure LPSPI1 peripheral chip select */

  s32k1xx_pinconfig(PIN_LPSPI1_PCS);

#  ifdef CONFIG_SPI_DRIVER
  /* Initialize the SPI driver for LPSPI1 */

  struct spi_dev_s *g_lpspi1 = s32k1xx_lpspibus_initialize(1);
  if (g_lpspi1 == NULL)
    {
      spierr("ERROR: FAILED to initialize LPSPI1\n");
      return -ENODEV;
    }

  ret = spi_register(g_lpspi1, 1);
  if (ret < 0)
    {
      spierr("ERROR: FAILED to register LPSPI1 driver\n");
      return ret;
    }
#  endif /* CONFIG_SPI_DRIVER */
#endif /* CONFIG_S32K1XX_LPSPI1 */

#ifdef CONFIG_S32K1XX_LPSPI2
  /* LPSPI2 *****************************************************************/

  /* Configure LPSPI2 peripheral chip select */

  s32k1xx_pinconfig(PIN_LPSPI2_PCS);

#  ifdef CONFIG_SPI_DRIVER
  /* Initialize the SPI driver for LPSPI2 */

  struct spi_dev_s *g_lpspi2 = s32k1xx_lpspibus_initialize(2);
  if (g_lpspi2 == NULL)
    {
      spierr("ERROR: FAILED to initialize LPSPI2\n");
      return -ENODEV;
    }

  ret = spi_register(g_lpspi2, 2);
  if (ret < 0)
    {
      spierr("ERROR: FAILED to register LPSPI2 driver\n");
      return ret;
    }
#  endif /* CONFIG_SPI_DRIVER */
#endif /* CONFIG_S32K1XX_LPSPI2 */

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_lpspiNselect and s32k1xx_lpspiNstatus
 *
 * Description:
 *   The external functions, s32k1xx_lpspiNselect and s32k1xx_lpspiNstatus
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).  All other methods (including
 *   s32k1xx_lpspibus_initialize()) are provided by common logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in s32k1xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k1xx_lpspiNselect() and s32k1xx_lpspiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to s32k1xx_lpspibus_initialize() in your low level
 *      application initialization logic.
 *   4. The handle returned by s32k1xx_lpspibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI0
/* LPSPI0 *******************************************************************/

void s32k1xx_lpspi0select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  s32k1xx_gpiowrite(PIN_LPSPI0_PCS, !selected);
}

uint8_t s32k1xx_lpspi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_S32K1XX_LPSPI0 */

#ifdef CONFIG_S32K1XX_LPSPI1
/* LPSPI1 *******************************************************************/

void s32k1xx_lpspi1select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  s32k1xx_gpiowrite(PIN_LPSPI1_PCS, !selected);
}

uint8_t s32k1xx_lpspi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_S32K1XX_LPSPI1 */

#ifdef CONFIG_S32K1XX_LPSPI2
/* LPSPI2 *******************************************************************/

void s32k1xx_lpspi2select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  s32k1xx_gpiowrite(PIN_LPSPI2_PCS, !selected);
}

uint8_t s32k1xx_lpspi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_S32K1XX_LPSPI2 */
#endif /* CONFIG_S32K1XX_LPSPI */
