/****************************************************************************
 * boards/arm/stm32f7/common/src/stm32_spitest.c
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

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <sys/param.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32F7_SPI1_TEST)
#  if defined(CONFIG_STM32F7_SPI1_TEST_MODE0)
#    define CONFIG_STM32F7_SPI1_TEST_MODE SPIDEV_MODE0
#  elif defined(CONFIG_STM32F7_SPI1_TEST_MODE1)
#    define CONFIG_STM32F7_SPI1_TEST_MODE SPIDEV_MODE1
#  elif defined(CONFIG_STM32F7_SPI1_TEST_MODE2)
#    define CONFIG_STM32F7_SPI1_TEST_MODE SPIDEV_MODE2
#  elif defined(CONFIG_STM32F7_SPI1_TEST_MODE3)
#    define CONFIG_STM32F7_SPI1_TEST_MODE SPIDEV_MODE3
#  else
#    error "No CONFIG_STM32F7_SPI1_TEST_MODEx defined"
#  endif
#endif

#if defined(CONFIG_STM32F7_SPI2_TEST)
#  if defined(CONFIG_STM32F7_SPI2_TEST_MODE0)
#    define CONFIG_STM32F7_SPI2_TEST_MODE SPIDEV_MODE0
#  elif defined(CONFIG_STM32F7_SPI2_TEST_MODE1)
#    define CONFIG_STM32F7_SPI2_TEST_MODE SPIDEV_MODE1
#  elif defined(CONFIG_STM32F7_SPI2_TEST_MODE2)
#    define CONFIG_STM32F7_SPI2_TEST_MODE SPIDEV_MODE2
#  elif defined(CONFIG_STM32F7_SPI2_TEST_MODE3)
#    define CONFIG_STM32F7_SPI2_TEST_MODE SPIDEV_MODE3
#  else
#    error "No CONFIG_STM32F7_SPI2_TEST_MODEx defined"
#  endif
#endif

#if defined(CONFIG_STM32F7_SPI3_TEST)
#  if defined(CONFIG_STM32F7_SPI3_TEST_MODE0)
#    define CONFIG_STM32F7_SPI3_TEST_MODE SPIDEV_MODE0
#  elif defined(CONFIG_STM32F7_SPI3_TEST_MODE1)
#    define CONFIG_STM32F7_SPI3_TEST_MODE SPIDEV_MODE1
#  elif defined(CONFIG_STM32F7_SPI3_TEST_MODE2)
#    define CONFIG_STM32F7_SPI3_TEST_MODE SPIDEV_MODE2
#  elif defined(CONFIG_STM32F7_SPI3_TEST_MODE3)
#    define CONFIG_STM32F7_SPI3_TEST_MODE SPIDEV_MODE3
#  else
#    error "No CONFIG_STM32F7_SPI3_TEST_MODEx defined"
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_STM32F7_SPI1)
struct spi_dev_s *g_spi1;
#endif
#if defined(CONFIG_STM32F7_SPI2)
struct spi_dev_s *g_spi2;
#endif
#if defined(CONFIG_STM32F7_SPI3)
struct spi_dev_s *g_spi3;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_bus_test
 *
 * Description:
 *   Called to create the defined SPI buses and test them by initializing
 *   them and sending the CONFIG_STM32F7_SPI_TEST_MESSAGE (no chip select).
 *
 ****************************************************************************/

int stm32_spidev_bus_test(void)
{
  /* Configure and test SPI- */

  uint8_t *tx = (uint8_t *)CONFIG_STM32F7_SPI_TEST_MESSAGE;

#if defined(CONFIG_STM32F7_SPI1_TEST)
  g_spi1 = stm32_spibus_initialize(1);

  if (!g_spi1)
    {
      syslog(LOG_ERR, "ERROR Failed to initialize SPI port 1\n");
      return -ENODEV;
    }

  /* Default SPI1 to STM32F7_SPI1_FREQ and mode */

  SPI_SETFREQUENCY(g_spi1, CONFIG_STM32F7_SPI1_TEST_FREQ);
  SPI_SETBITS(g_spi1, CONFIG_STM32F7_SPI1_TEST_BITS);
  SPI_SETMODE(g_spi1, CONFIG_STM32F7_SPI1_TEST_MODE);
  SPI_EXCHANGE(g_spi1, tx, NULL,
               nitems(CONFIG_STM32F7_SPI_TEST_MESSAGE));
#endif

#if defined(CONFIG_STM32F7_SPI2_TEST)
  g_spi2 = stm32_spibus_initialize(2);

  if (!g_spi2)
    {
      syslog(LOG_ERR, "ERROR Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  /* Default SPI2 to STM32F7_SPI2_FREQ and mode */

  SPI_SETFREQUENCY(g_spi2, CONFIG_STM32F7_SPI2_TEST_FREQ);
  SPI_SETBITS(g_spi2, CONFIG_STM32F7_SPI2_TEST_BITS);
  SPI_SETMODE(g_spi2, CONFIG_STM32F7_SPI2_TEST_MODE);
  SPI_EXCHANGE(g_spi2, tx, NULL,
               nitems(CONFIG_STM32F7_SPI_TEST_MESSAGE));
#endif

#if defined(CONFIG_STM32F7_SPI3_TEST)
  g_spi3 = stm32_spibus_initialize(3);

  if (!g_spi3)
    {
      syslog(LOG_ERR, "ERROR Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  /* Default SPI3 to STM32F7_SPI3_FREQ and mode */

  SPI_SETFREQUENCY(g_spi3, CONFIG_STM32F7_SPI3_TEST_FREQ);
  SPI_SETBITS(g_spi3, CONFIG_STM32F7_SPI3_TEST_BITS);
  SPI_SETMODE(g_spi3, CONFIG_STM32F7_SPI3_TEST_MODE);
  SPI_EXCHANGE(g_spi3, tx, NULL,
               nitems(CONFIG_STM32F7_SPI_TEST_MESSAGE));
#endif

  return OK;
}
