/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_cxd5602pwbimu.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <arch/chip/pin.h>
#include <arch/board/board.h>

#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"
#include "cxd56_spi.h"
#include "cxd56_dmac.h"
#include "cxd56_i2c.h"

#include <nuttx/sensors/cxd5602pwbimu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PIN_POWER    PIN_EMMC_DATA2
#define PIN_XRST     PIN_I2S0_BCK
#define PIN_SPI_DRDY PIN_EMMC_DATA3
#define PIN_SPI_CSX  PIN_I2S0_DATA_IN

#define SETUP_PIN_INPUT(pin) \
do \
  { \
    board_gpio_write(pin, -1); \
    board_gpio_config(pin, 0, true, false, PIN_PULLDOWN); \
  } \
while (0)

#define SETUP_PIN_OUTPUT(pin) \
do \
  { \
    board_gpio_write(pin, -1); \
    board_gpio_config(pin, 0, false, true, PIN_FLOAT); \
  } \
while (0)

#if defined(CONFIG_CXD56_CXD5602PWBIMU_SPI5_DMAC)
#define PWBIMU_MAX_PACKET_SIZE (34)
#define PWBIMU_DMA_TXCH        (4)
#define PWBIMU_DMA_RXCH        (5)
#define PWBIMU_DMA_TXCHCHG     (CXD56_DMA_PERIPHERAL_SPI5_TX)
#define PWBIMU_DMA_RXCHCFG     (CXD56_DMA_PERIPHERAL_SPI5_RX)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  cxd5602pwbimu_irq_attach(const cxd5602pwbimu_config_t *state,
                                     xcpt_t isr, void *arg);
static void cxd5602pwbimu_irq_enable(const cxd5602pwbimu_config_t *state,
                                     bool enable);
static void cxd5602pwbimu_csx(const cxd5602pwbimu_config_t *state,
                              bool pol);
static void cxd5602pwbimu_power(const cxd5602pwbimu_config_t *state,
                                bool pol);
static void cxd5602pwbimu_reset(const cxd5602pwbimu_config_t *state,
                                bool assert);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static cxd5602pwbimu_config_t g_config =
{
  .irq_attach = cxd5602pwbimu_irq_attach,
  .irq_enable = cxd5602pwbimu_irq_enable,
  .csx = cxd5602pwbimu_csx,
  .power = cxd5602pwbimu_power,
  .reset = cxd5602pwbimu_reset,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Attach the CXD5602PWBIMU interrupt handler to the GPIO interrupt */

static int cxd5602pwbimu_irq_attach(const cxd5602pwbimu_config_t *state,
                                    xcpt_t isr, void *arg)
{
  sninfo("cxd5602pwbimu_irq_attach\n");
  cxd56_gpioint_config(PIN_SPI_DRDY, GPIOINT_LEVEL_HIGH, isr, arg);

  return OK;
}

/* Enable or disable the GPIO interrupt */

static void cxd5602pwbimu_irq_enable(const cxd5602pwbimu_config_t *state,
                                     bool enable)
{
  sninfo("%d\n", enable);
  board_gpio_int(PIN_SPI_DRDY, enable);
}

/* Toggle the csx pin level */

static void cxd5602pwbimu_csx(const cxd5602pwbimu_config_t *state,
                              bool pol)
{
  sninfo("%d\n", pol);

  /* delay for csx rising edge */

  if (pol == true)
    {
      up_udelay(5);
    }

  /* toggle csx pin */

  board_gpio_write(PIN_SPI_CSX, pol);

  /* delay for csx falling edge */

  if (pol == false)
    {
      up_udelay(5);
    }
}

static void cxd5602pwbimu_power(const cxd5602pwbimu_config_t *state,
                                bool pol)
{
  sninfo("%d\n", pol);

  /* toggle power pin */

  board_gpio_write(PIN_POWER, pol);

  /* wait power level settling */

  up_mdelay(2);
}

static void cxd5602pwbimu_reset(const cxd5602pwbimu_config_t *state,
                                bool assert)
{
  board_gpio_write(PIN_XRST, !assert);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_CXD56_SPI) && defined(CONFIG_SENSORS_CXD5602PWBIMU)

int board_cxd5602pwbimu_initialize(int bus)
{
  int ret;
  struct spi_dev_s *spi;
  struct i2c_master_s *i2c;
#if defined(CONFIG_CXD56_CXD5602PWBIMU_SPI5_DMAC)
  DMA_HANDLE            hdl;
  dma_config_t          conf;
#endif
  cxd5602pwbimu_config_t *config = &g_config;

  sninfo("Initializing CXD5602PWBIMU..\n");

  /* Initialize pins */

  SETUP_PIN_OUTPUT(PIN_POWER);

  SETUP_PIN_OUTPUT(PIN_XRST);
  config->reset(config, true);

  SETUP_PIN_OUTPUT(PIN_SPI_CSX);
  config->csx(config, true);

  SETUP_PIN_INPUT(PIN_SPI_DRDY);

  /* Initialize spi device */

  spi = cxd56_spibus_initialize(bus);
  if (!spi)
    {
      snerr("ERROR: Failed to initialize spi%d.\n", bus);
      return -ENODEV;
    }

#if defined(CONFIG_CXD56_CXD5602PWBIMU_SPI5_DMAC)
  hdl = cxd56_dmachannel(PWBIMU_DMA_TXCH,
                         PWBIMU_MAX_PACKET_SIZE);
  if (hdl)
    {
      conf.channel_cfg = PWBIMU_DMA_TXCHCHG;
      conf.dest_width  = CXD56_DMAC_WIDTH8;
      conf.src_width   = CXD56_DMAC_WIDTH8;
      cxd56_spi_dmaconfig(bus, CXD56_SPI_DMAC_CHTYPE_TX, hdl, &conf);
    }

  hdl = cxd56_dmachannel(PWBIMU_DMA_RXCH,
                         PWBIMU_MAX_PACKET_SIZE);
  if (hdl)
    {
      conf.channel_cfg = PWBIMU_DMA_RXCHCFG;
      conf.dest_width  = CXD56_DMAC_WIDTH8;
      conf.src_width   = CXD56_DMAC_WIDTH8;
      cxd56_spi_dmaconfig(bus, CXD56_SPI_DMAC_CHTYPE_RX, hdl, &conf);
    }
#endif

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(0);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", 0);
      return -ENODEV;
    }

  ret = cxd5602pwbimu_register("/dev/imu0", spi, i2c, config);
  if (ret < 0)
    {
      snerr("Error registering CXD5602PWBIMU\n");
    }

  return ret;
}

#endif
