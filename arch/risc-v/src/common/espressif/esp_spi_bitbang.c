/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spi_bitbang.c
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

#ifdef CONFIG_ESPRESSIF_SPI_BITBANG
#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_bitbang.h>

#include "riscv_internal.h"
#include "espressif/esp_gpio.h"
#include "soc/gpio_sig_map.h"
#include "esp_spi_bitbang.h"
#include <nuttx/spi/spi_bitbang.c>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_SPI_BITBANG_MODE0)
#undef  SPI_BITBANG_DISABLEMODE0
#define SPI_BITBANG_DISABLEMODE1 1
#define SPI_BITBANG_DISABLEMODE2 1
#define SPI_BITBANG_DISABLEMODE3 1
#elif defined(CONFIG_ESPRESSIF_SPI_BITBANG_MODE1)
#undef  SPI_BITBANG_DISABLEMODE1
#define SPI_BITBANG_DISABLEMODE0 1
#define SPI_BITBANG_DISABLEMODE2 1
#define SPI_BITBANG_DISABLEMODE3 1
#elif defined(CONFIG_ESPRESSIF_SPI_BITBANG_MODE2)
#undef  SPI_BITBANG_DISABLEMODE2
#define SPI_BITBANG_DISABLEMODE0 1
#define SPI_BITBANG_DISABLEMODE1 1
#define SPI_BITBANG_DISABLEMODE3 1
#else
#undef  SPI_BITBANG_DISABLEMODE3
#define SPI_BITBANG_DISABLEMODE0 1
#define SPI_BITBANG_DISABLEMODE1 1
#define SPI_BITBANG_DISABLEMODE2 1
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Lower-half SPI */

static void spi_select(struct spi_bitbang_s *priv, uint32_t devid,
                       bool selected);
static uint8_t spi_status(struct spi_bitbang_s *priv, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_bitbang_s *priv, uint32_t devid,
                       bool cmd);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct spi_bitbang_ops_s esp_spi_bitbang_ops =
{
  .select = spi_select,
  .status = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata = spi_cmddata,
#endif
  .setfrequency = spi_setfrequency,
  .setmode = spi_setmode,
  .exchange = spi_exchange,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Select or de-selected the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   priv     - An instance of the bit-bang driver structure
 *   devid    - The device to select or de-select
 *   selected - True:select false:de-select
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(struct spi_bitbang_s *priv, uint32_t devid,
                       bool selected)
{
  if (selected)
    {
      SPI_CLRCS;
    }
  else
    {
      SPI_SETCS;
    }
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Return status of the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   priv     - An instance of the bit-bang driver structure
 *   devid    - The device to select or de-select
 *
 * Returned Value:
 *   An 8-bit, bit-encoded status byte
 *
 ****************************************************************************/

static uint8_t spi_status(struct spi_bitbang_s *priv, uint32_t devid)
{
  if (devid == SPIDEV_MMCSD(0))
    {
      return SPI_STATUS_PRESENT;
    }

  return 0;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   If there were was a CMD/DATA line, this function would manage it
 *
 * Input Parameters:
 *   priv  - An instance of the bit-bang driver structure
 *   devid - The device to use
 *   cmd   - True=MCD false=DATA
 *
 * Returned Value:
 *  OK
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_bitbang_s *priv, uint32_t devid,
                       bool cmd)
{
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_spi_bitbang_init
 *
 * Description:
 *   Initialize the SPI bit-bang driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-NULL reference to the SPI driver on success
 *
 ****************************************************************************/

struct spi_dev_s *esp_spi_bitbang_init(void)
{
  /* Configure the SPI bit-bang pins */

  esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_CSPIN, true);
  esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_MOSIPIN, true);
  esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_CLKPIN, true);

#if CONFIG_ESPRESSIF_SPI_SWCS
  esp_configgpio(CONFIG_ESPRESSIF_SPI_BITBANG_CSPIN, OUTPUT_FUNCTION_1);
  esp_gpio_matrix_out(CONFIG_ESPRESSIF_SPI_BITBANG_CSPIN, SIG_GPIO_OUT_IDX,
                      0, 0);
#endif
  esp_configgpio(CONFIG_ESPRESSIF_SPI_BITBANG_MOSIPIN, OUTPUT_FUNCTION_1);
  esp_gpio_matrix_out(CONFIG_ESPRESSIF_SPI_BITBANG_MOSIPIN, SIG_GPIO_OUT_IDX,
                      0, 0);

  esp_configgpio(CONFIG_ESPRESSIF_SPI_BITBANG_MISOPIN,
                 INPUT_FUNCTION_1 | PULLUP);
  esp_gpio_matrix_out(CONFIG_ESPRESSIF_SPI_BITBANG_MISOPIN, SIG_GPIO_OUT_IDX,
                      0, 0);

  esp_configgpio(CONFIG_ESPRESSIF_SPI_BITBANG_CLKPIN, OUTPUT_FUNCTION_1);
  esp_gpio_matrix_out(CONFIG_ESPRESSIF_SPI_BITBANG_CLKPIN, SIG_GPIO_OUT_IDX,
                      0, 0);

  /* Create the SPI driver instance */

  return spi_create_bitbang(&esp_spi_bitbang_ops, NULL);
}

/****************************************************************************
 * Name:  esp_spi_bitbang_uninitialize
 *
 * Description:
 *   Destroy an instance of the SPI bit-bang driver.
 *
 * Input Parameters:
 *   dev - device instance, target driver to destroy.
 *
 ****************************************************************************/

void esp_spi_bitbang_uninitialize(struct spi_dev_s *dev)
{
  spi_destroy_bitbang(dev);
}

#endif /* CONFIG_ESPRESSIF_SPI_BITBANG */
