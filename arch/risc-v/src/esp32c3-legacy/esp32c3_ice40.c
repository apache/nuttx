/****************************************************************************
 * arch/risc-v/src/esp32c3-legacy/esp32c3_ice40.c
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
#include <nuttx/nuttx.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include <arch/board/board.h>
#include <arch/esp32c3-legacy/chip.h>
#include <arch/irq.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/ice40.h>
#include <nuttx/spi/spi.h>

#include "esp32c3_spi.h"
#include "hardware/esp32c3_spi.h"

#include "hardware/esp32c3_gpio.h"
#include "hardware/esp32c3_gpio_sigmap.h"

#include "esp32c3_ice40.h"

#include "esp32c3_gpio.h"

#include <nuttx/spi/ice40.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ice40_reset(struct ice40_dev_s *dev, bool reset);
static void ice40_select(struct ice40_dev_s *dev, bool select);
static bool ice40_get_status(struct ice40_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ice40_ops_s ice40_ops =
{
  .reset = ice40_reset,
  .select = ice40_select,
  .get_status = ice40_get_status,
};

struct esp32c3_ice40_dev_s
{
  struct ice40_dev_s base;
  uint16_t cdone_gpio;
  uint16_t crst_gpio;
  uint16_t cs_gpio;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ice40_reset
 *
 * Description:
 *  Reset ICE40 FPGA
 *
 * Input Parameters:
 *  reset - true to reset, false to release reset (inverse logic, active low)
 *
 ****************************************************************************/

static void
ice40_reset(struct ice40_dev_s *dev, bool reset)
{
  struct esp32c3_ice40_dev_s *priv
      = container_of (dev, struct esp32c3_ice40_dev_s, base);
  esp32c3_gpiowrite(priv->crst_gpio, !reset);
}

/****************************************************************************
 * Name: ice40_select
 *
 * Description:
 *  Select ICE40 FPGA
 *
 * Input Parameters:
 *  select - true to select, false to deselect (inverse logic, active low)
 *
 ****************************************************************************/

static void
ice40_select(struct ice40_dev_s *dev, bool select)
{
  struct esp32c3_ice40_dev_s *priv
      = container_of (dev, struct esp32c3_ice40_dev_s, base);
  esp32c3_gpiowrite(priv->cs_gpio, !select);
}

/****************************************************************************
 * Name: ice40_get_status
 *
 * Description:
 *  Get ICE40 FPGA status via CDONE pin. Important to know if the FPGA is
 *  programmed and ready to use.
 *
 * Returned Value:
 *  true if the FPGA is programmed and ready to use, false otherwise.
 *
 ****************************************************************************/

static bool
ice40_get_status(struct ice40_dev_s *dev)
{
  struct esp32c3_ice40_dev_s *priv
      = container_of(dev, struct esp32c3_ice40_dev_s, base);
  return esp32c3_gpioread (priv->cdone_gpio);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_ice40_initialize
 *
 * Description:
 *  Initialize ICE40 FPGA GPIOs and SPI.
 *
 * Input Parameters:
 *
 *
 ****************************************************************************/

FAR struct ice40_dev_s *
esp32c3_ice40_initialize(const uint16_t cdone_gpio,
                          const uint16_t crst_gpio,
                          const uint16_t cs_gpio,
                          const uint16_t spi_port)
{
  struct esp32c3_ice40_dev_s *ice40_ptr;

  ice40_ptr = kmm_malloc(sizeof(struct esp32c3_ice40_dev_s));
  if (ice40_ptr == NULL)
    {
      spierr("ERROR: Failed to allocate memory for ICE40 driver\n");
      return NULL;
    }

  memset(ice40_ptr, 0, sizeof (struct esp32c3_ice40_dev_s));

  ice40_ptr->base.ops = &ice40_ops;

  /* Configure GPIO pins */

  DEBUGASSERT(0 <= cdone_gpio && cdone_gpio < 32);
  DEBUGASSERT(0 <= crst_gpio && crst_gpio < 32);
  DEBUGASSERT(0 <= cs_gpio && cs_gpio < 32);
  DEBUGASSERT(0 <= spi_port && spi_port < 3);

  esp32c3_gpio_matrix_out(cdone_gpio, SIG_GPIO_OUT_IDX, 0, 0);
  esp32c3_gpio_matrix_out(crst_gpio, SIG_GPIO_OUT_IDX, 0, 0);
  esp32c3_gpio_matrix_out(cs_gpio, SIG_GPIO_OUT_IDX, 0, 0);

  esp32c3_configgpio(cdone_gpio, INPUT_FUNCTION_1 | PULLDOWN);
  esp32c3_configgpio(crst_gpio, OUTPUT_FUNCTION_1);
  esp32c3_configgpio(cs_gpio, OUTPUT_FUNCTION_1);

  esp32c3_gpiowrite(crst_gpio, 1);
  esp32c3_gpiowrite(cs_gpio, 1);

  ice40_ptr->cdone_gpio = cdone_gpio;
  ice40_ptr->crst_gpio = crst_gpio;
  ice40_ptr->cs_gpio = cs_gpio;

  /* Configure SPI */

  ice40_ptr->base.spi = esp32c3_spibus_initialize(spi_port);
  if (ice40_ptr->base.spi == NULL)
    {
      spierr("ERROR: Failed to initialize SPI port %d\n",
              ice40_ptr->base.spi);
      return NULL;
    }

  return &ice40_ptr->base;
}
