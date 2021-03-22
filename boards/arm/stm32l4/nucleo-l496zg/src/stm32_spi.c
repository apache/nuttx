/****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/src/stm32_spi.c
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

#include "arm_arch.h"
#include "chip.h"
#include "stm32l4_gpio.h"
#include "stm32l4_spi.h"

#include "nucleo-144.h"

#if defined(CONFIG_SPI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x) (sizeof((x)) / sizeof((x)[0]))

#if defined(CONFIG_NUCLEO_SPI1_TEST)
#  if defined(CONFIG_NUCLEO_SPI1_TEST_MODE0)
#    define CONFIG_NUCLEO_SPI1_TEST_MODE SPIDEV_MODE0
#  elif defined(CONFIG_NUCLEO_SPI1_TEST_MODE1)
#    define CONFIG_NUCLEO_SPI1_TEST_MODE SPIDEV_MODE1
#  elif defined(CONFIG_NUCLEO_SPI1_TEST_MODE2)
#    define CONFIG_NUCLEO_SPI1_TEST_MODE SPIDEV_MODE2
#  elif defined(CONFIG_NUCLEO_SPI1_TEST_MODE3)
#    define CONFIG_NUCLEO_SPI1_TEST_MODE SPIDEV_MODE3
#  else
#    error "No CONFIG_NUCLEO_SPI1_TEST_MODEx defined"
# endif
#endif

#if defined(CONFIG_NUCLEO_SPI2_TEST)
#  if defined(CONFIG_NUCLEO_SPI2_TEST_MODE0)
#    define CONFIG_NUCLEO_SPI2_TEST_MODE SPIDEV_MODE0
#  elif defined(CONFIG_NUCLEO_SPI2_TEST_MODE1)
#    define CONFIG_NUCLEO_SPI2_TEST_MODE SPIDEV_MODE1
#  elif defined(CONFIG_NUCLEO_SPI2_TEST_MODE2)
#    define CONFIG_NUCLEO_SPI2_TEST_MODE SPIDEV_MODE2
#  elif defined(CONFIG_NUCLEO_SPI2_TEST_MODE3)
#    define CONFIG_NUCLEO_SPI2_TEST_MODE SPIDEV_MODE3
#  else
#    error "No CONFIG_NUCLEO_SPI2_TEST_MODEx defined"
# endif
#endif

#if defined(CONFIG_NUCLEO_SPI3_TEST)
#  if defined(CONFIG_NUCLEO_SPI3_TEST_MODE0)
#    define CONFIG_NUCLEO_SPI3_TEST_MODE SPIDEV_MODE0
#  elif defined(CONFIG_NUCLEO_SPI3_TEST_MODE1)
#    define CONFIG_NUCLEO_SPI3_TEST_MODE SPIDEV_MODE1
#  elif defined(CONFIG_NUCLEO_SPI3_TEST_MODE2)
#    define CONFIG_NUCLEO_SPI3_TEST_MODE SPIDEV_MODE2
#  elif defined(CONFIG_NUCLEO_SPI3_TEST_MODE3)
#    define CONFIG_NUCLEO_SPI3_TEST_MODE SPIDEV_MODE3
#  else
#    error "No CONFIG_NUCLEO_SPI3_TEST_MODEx defined"
# endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Indexed by NUCLEO_SPI_BUSx_CSx */

static const uint32_t g_spigpio[] =
{
#if defined(GPIO_SPI1_CS0)
  GPIO_SPI1_CS0,
#endif
#if defined(GPIO_SPI1_CS1)
  GPIO_SPI1_CS1,
#endif
#if defined(GPIO_SPI1_CS2)
  GPIO_SPI1_CS2,
#endif
#if defined(GPIO_SPI1_CS3)
  GPIO_SPI1_CS3,
#endif
#if defined(GPIO_SPI2_CS0)
  GPIO_SPI2_CS0,
#endif
#if defined(GPIO_SPI2_CS1)
  GPIO_SPI2_CS1,
#endif
#if defined(GPIO_SPI2_CS2)
  GPIO_SPI2_CS2,
#endif
#if defined(GPIO_SPI2_CS3)
  GPIO_SPI2_CS3,
#endif
#if defined(GPIO_SPI3_CS0)
  GPIO_SPI3_CS0,
#endif
#if defined(GPIO_SPI3_CS1)
  GPIO_SPI3_CS1,
#endif
#if defined(GPIO_SPI3_CS2)
  GPIO_SPI3_CS2,
#endif
#if defined(GPIO_SPI3_CS3)
  GPIO_SPI3_CS3,
#endif
};

#if defined(CONFIG_NUCLEO_SPI_TEST)
#  if defined(CONFIG_STM32L4_SPI1)
struct spi_dev_s *spi1;
#  endif
#  if defined(CONFIG_STM32L4_SPI2)
struct spi_dev_s *spi2;
#  endif
#  if defined(CONFIG_STM32L4_SPI3)
struct spi_dev_s *spi3;
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-144 board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
  int i;

  /* Configure SPI CS GPIO for output */

  for (i = 0; i < ARRAYSIZE(g_spigpio); i++)
    {
      stm32l4_configgpio(g_spigpio[i]);
    }
}

/****************************************************************************
 * Name:  stm32_spi1/2/3/4/5select and stm32_spi1/2/3/4/5status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver
 *      to the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");
  stm32l4_gpiowrite(g_spigpio[devid], !selected);
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32L4_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
  stm32l4_gpiowrite(g_spigpio[devid], !selected);
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32L4_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
  stm32l4_gpiowrite(g_spigpio[devid], !selected);
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32L4_SPI1
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32L4_SPI2
int stm32_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32L4_SPI3
int stm32_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */

#if defined(CONFIG_NUCLEO_SPI_TEST)
int stm32_spidev_bus_test(void)
{
  /* Configure and test SPI- */

  uint8_t *tx = (uint8_t *)CONFIG_NUCLEO_SPI_TEST_MESSAGE;

#if defined(CONFIG_NUCLEO_SPI1_TEST)
  spi1 = stm32l3_spibus_initialize(1);

  if (!spi1)
    {
      syslog(LOG_ERR, "ERROR Failed to initialize SPI port 1\n");
      return -ENODEV;
    }

  /* Default SPI1 to NUCLEO_SPI1_FREQ and mode */

  SPI_SETFREQUENCY(spi1, CONFIG_NUCLEO_SPI1_TEST_FREQ);
  SPI_SETBITS(spi1, CONFIG_NUCLEO_SPI1_TEST_BITS);
  SPI_SETMODE(spi1, CONFIG_NUCLEO_SPI1_TEST_MODE);
  SPI_EXCHANGE(spi1, tx, NULL, ARRAYSIZE(CONFIG_NUCLEO_SPI_TEST_MESSAGE));
#endif

#if defined(CONFIG_NUCLEO_SPI2_TEST)
  spi2 = stm32l4_spibus_initialize(2);

  if (!spi2)
    {
      syslog(LOG_ERR, "ERROR Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  /* Default SPI2 to NUCLEO_SPI2_FREQ and mode */

  SPI_SETFREQUENCY(spi2, CONFIG_NUCLEO_SPI2_TEST_FREQ);
  SPI_SETBITS(spi2, CONFIG_NUCLEO_SPI2_TEST_BITS);
  SPI_SETMODE(spi2, CONFIG_NUCLEO_SPI2_TEST_MODE);
  SPI_EXCHANGE(spi2, tx, NULL, ARRAYSIZE(CONFIG_NUCLEO_SPI_TEST_MESSAGE));
#endif

#if defined(CONFIG_NUCLEO_SPI3_TEST)
  spi3 = stm32l4_spibus_initialize(3);

  if (!spi3)
    {
      syslog(LOG_ERR, "ERROR Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  /* Default SPI3 to NUCLEO_SPI3_FREQ and mode */

  SPI_SETFREQUENCY(spi3, CONFIG_NUCLEO_SPI3_TEST_FREQ);
  SPI_SETBITS(spi3, CONFIG_NUCLEO_SPI3_TEST_BITS);
  SPI_SETMODE(spi3, CONFIG_NUCLEO_SPI3_TEST_MODE);
  SPI_EXCHANGE(spi3, tx, NULL, ARRAYSIZE(CONFIG_NUCLEO_SPI_TEST_MESSAGE));
#endif

  return OK;
}
#endif /* NUCLEO_SPI_TEST */
#endif /* defined(CONFIG_SPI) */
