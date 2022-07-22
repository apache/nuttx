/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/s32k3xx_spi.c
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

/* Copyright 2022 NXP */

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

#include "s32k3xx_pin.h"
#include "s32k3xx_lpspi.h"

#include <arch/board/board.h>

#include "mr-canhubk3.h"

#ifdef CONFIG_S32K3XX_FS26
#include "s32k3xx_fs26.h"
#endif

#if defined(CONFIG_S32K3XX_LPSPI2) && defined(CONFIG_MMCSD)
#include <nuttx/mmcsd.h>
#endif

#ifdef CONFIG_S32K3XX_LPSPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_spidev_initialize
 *
 * Description:
 *   Configure chip select pins, initialize the SPI driver and register
 *   /dev/spiN devices.
 *
 ****************************************************************************/

int weak_function s32k3xx_spidev_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_S32K3XX_LPSPI3
  /* LPSPI3 *****************************************************************/

  /* Configure LPSPI3 peripheral chip select */

  s32k3xx_pinconfig(PIN_LPSPI3_PCS);

#  ifdef CONFIG_SPI_DRIVER
  /* Initialize the SPI driver for LPSPI3 */

  struct spi_dev_s *g_lpspi3 = s32k3xx_lpspibus_initialize(3);
  if (g_lpspi3 == NULL)
    {
      spierr("ERROR: FAILED to initialize LPSPI3\n");
      return -ENODEV;
    }

  ret = spi_register(g_lpspi3, 3);
  if (ret < 0)
    {
      spierr("ERROR: FAILED to register LPSPI3 driver\n");
      return ret;
    }
#  endif /* CONFIG_SPI_DRIVER */

#  ifdef CONFIG_S32K3XX_FS26
  fs26_initialize(g_lpspi3);
#  endif

#endif /* CONFIG_S32K3XX_LPSPI3 */

#ifdef CONFIG_S32K3XX_LPSPI0
  /* LPSPI0 *****************************************************************/

  /* Configure LPSPI0 peripheral chip select */

  s32k3xx_pinconfig(PIN_LPSPI0_PCS);

#  ifdef CONFIG_SPI_DRIVER
  /* Initialize the SPI driver for LPSPI0 */

  struct spi_dev_s *g_lpspi0 = s32k3xx_lpspibus_initialize(0);
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
#endif /* CONFIG_S32K3XX_LPSPI0 */

#ifdef CONFIG_S32K3XX_LPSPI1
  /* LPSPI1 *****************************************************************/

  /* Configure LPSPI1 peripheral chip select */

  s32k3xx_pinconfig(PIN_LPSPI1_PCS);

#  ifdef CONFIG_SPI_DRIVER
  /* Initialize the SPI driver for LPSPI1 */

  struct spi_dev_s *g_lpspi1 = s32k3xx_lpspibus_initialize(1);
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
#endif /* CONFIG_S32K3XX_LPSPI1 */

#ifdef CONFIG_S32K3XX_LPSPI2
  /* LPSPI2 *****************************************************************/

  /* Configure LPSPI2 peripheral chip select */

  s32k3xx_pinconfig(PIN_LPSPI2_PCS);

  /* Initialize the SPI driver for LPSPI2 */

  struct spi_dev_s *g_lpspi2 = s32k3xx_lpspibus_initialize(2);
  if (g_lpspi2 == NULL)
    {
      spierr("ERROR: FAILED to initialize LPSPI2\n");
      return -ENODEV;
    }

#if defined(CONFIG_S32K3XX_LPSPI2) && defined(CONFIG_MMCSD)

#if defined(CONFIG_FAT_DMAMEMORY)
  if (s32k3xx_dma_alloc_init() < 0)
    {
      spierr("ERROR: DMA alloc FAILED");
    }
#endif

  if (g_lpspi2 == NULL)
    {
      spierr("ERROR: FAILED to initialize LPSPI2\n");
    }

  ret = mmcsd_spislotinitialize(0, 0, g_lpspi2);

  if (ret < 0)
    {
      spierr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
                2, 0);
    }
#  endif

#  ifdef CONFIG_SPI_DRIVER
  ret = spi_register(g_lpspi2, 2);
  if (ret < 0)
    {
      spierr("ERROR: FAILED to register LPSPI2 driver\n");
      return ret;
    }
#  endif /* CONFIG_SPI_DRIVER */

#endif /* CONFIG_S32K3XX_LPSPI2 */

#ifdef CONFIG_S32K3XX_LPSPI4
  /* LPSPI4 *****************************************************************/

  /* Configure LPSPI4 peripheral chip select */

  s32k3xx_pinconfig(PIN_LPSPI4_PCS);

#  ifdef CONFIG_SPI_DRIVER
  /* Initialize the SPI driver for LPSPI4 */

  struct spi_dev_s *g_lpspi4 = s32k3xx_lpspibus_initialize(4);
  if (g_lpspi4 == NULL)
    {
      spierr("ERROR: FAILED to initialize LPSPI4\n");
      return -ENODEV;
    }

  ret = spi_register(g_lpspi4, 4);
  if (ret < 0)
    {
      spierr("ERROR: FAILED to register LPSPI4 driver\n");
      return ret;
    }
#  endif /* CONFIG_SPI_DRIVER */
#endif /* CONFIG_S32K3XX_LPSPI4 */

#ifdef CONFIG_S32K3XX_LPSPI5
  /* LPSPI5 *****************************************************************/

  /* Configure LPSPI5 peripheral chip select */

  s32k3xx_pinconfig(PIN_LPSPI5_PCS);

#  ifdef CONFIG_SPI_DRIVER
  /* Initialize the SPI driver for LPSPI5 */

  struct spi_dev_s *g_lpspi5 = s32k3xx_lpspibus_initialize(5);
  if (g_lpspi5 == NULL)
    {
      spierr("ERROR: FAILED to initialize LPSPI5\n");
      return -ENODEV;
    }

  ret = spi_register(g_lpspi5, 5);
  if (ret < 0)
    {
      spierr("ERROR: FAILED to register LPSPI5 driver\n");
      return ret;
    }
#  endif /* CONFIG_SPI_DRIVER */
#endif /* CONFIG_S32K3XX_LPSPI5 */

  return ret;
}

/****************************************************************************
 * Name: s32k3xx_lpspiNselect and s32k3xx_lpspiNstatus
 *
 * Description:
 *   The external functions, s32k3xx_lpspiNselect and s32k3xx_lpspiNstatus
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).  All other methods (including
 *   s32k3xx_lpspibus_initialize()) are provided by common logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in s32k3xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k3xx_lpspiNselect() and s32k3xx_lpspiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to s32k3xx_lpspibus_initialize() in your low level
 *      application initialization logic.
 *   4. The handle returned by s32k3xx_lpspibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI1
/* LPSPI1 *******************************************************************/

void s32k3xx_lpspi1select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  s32k3xx_gpiowrite(PIN_LPSPI1_PCS, !selected);
}

uint8_t s32k3xx_lpspi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_S32K3XX_LPSPI1 */

#ifdef CONFIG_S32K3XX_LPSPI2
/* LPSPI2 *******************************************************************/

void s32k3xx_lpspi2select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  s32k3xx_gpiowrite(PIN_LPSPI2_PCS, !selected);
}

uint8_t s32k3xx_lpspi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 1;
}
#endif /* CONFIG_S32K3XX_LPSPI2 */

#ifdef CONFIG_S32K3XX_LPSPI3
/* LPSPI3 *******************************************************************/

void s32k3xx_lpspi3select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  s32k3xx_gpiowrite(PIN_LPSPI3_PCS, !selected);
}

uint8_t s32k3xx_lpspi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_S32K3XX_LPSPI3 */

#ifdef CONFIG_S32K3XX_LPSPI4
/* LPSPI4 *******************************************************************/

void s32k3xx_lpspi4select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  s32k3xx_gpiowrite(PIN_LPSPI4_PCS, !selected);
}

uint8_t s32k3xx_lpspi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_S32K3XX_LPSPI4 */

#ifdef CONFIG_S32K3XX_LPSPI5
/* LPSPI5 *******************************************************************/

void s32k3xx_lpspi5select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  s32k3xx_gpiowrite(PIN_LPSPI5_PCS, !selected);
}

uint8_t s32k3xx_lpspi5status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif /* CONFIG_S32K3XX_LPSPI5 */
#endif /* CONFIG_S32K3XX_LPSPI */
