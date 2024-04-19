/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_w5500.c
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/w5500.h>

#include "rp2040_spi.h"
#include "rp2040_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sanity Check *************************************************************/

#if (CONFIG_RP2040_W5500_SPI_CH == 0) && !defined(CONFIG_RP2040_SPI0)
# error "W5500 configured to use SPI0, but SPI0 is not enabled"
#endif

#if (CONFIG_RP2040_W5500_SPI_CH == 1) && !defined(CONFIG_RP2040_SPI1)
# error "W5500 configured to use SPI1, but SPI1 is not enabled"
#endif

/* SPI Assumptions **********************************************************/

#define W5500_DEVNO      0   /* Only one W5500 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp2040_lower_s
{
  const struct w5500_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                     handler;  /* W5500 interrupt handler */
  void                      *arg;      /* Argument that accompanies IRQ */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  rp2040_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                          void *arg);
static void rp2040_enable(const struct w5500_lower_s *lower, bool enable);
static void rp2040_reset(const struct w5500_lower_s *lower, bool reset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rp2040_lower_s g_enclower =
{
  .lower =
  {
    .frequency = (CONFIG_RP2040_W5500_SPI_FREQ * 1000),
    .spidevid  = 0,
    .mode      = SPIDEV_MODE0,
    .attach    = rp2040_attach,
    .enable    = rp2040_enable,
    .reset     = rp2040_reset,
  },
  .handler = NULL,
  .arg     = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_attach
 *
 * Description:
 *   Attaches the interrupt handler to the GPIO.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   handler - The handler function
 *   arg - Argument to pass to handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

static int rp2040_attach(const struct w5500_lower_s *lower,
                         xcpt_t handler,
                         void *arg)
{
  struct rp2040_lower_s *priv = (struct rp2040_lower_s *)lower;

  priv->handler = handler;
  priv->arg     = arg;
  rp2040_gpio_irq_attach(CONFIG_RP2040_W5500_INT_GPIO,
                         RP2040_GPIO_INTR_LEVEL_LOW,
                         priv->handler, priv->arg);
  return OK;
}

/****************************************************************************
 * Name: rp2040_enable
 *
 * Description:
 *   Enables the W5500 interrupt handler.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   enable - true to enable, false to disable
 *
 ****************************************************************************/

static void rp2040_enable(const struct w5500_lower_s *lower, bool enable)
{
  struct rp2040_lower_s *priv = (struct rp2040_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  if (enable)
    {
      rp2040_gpio_enable_irq(CONFIG_RP2040_W5500_INT_GPIO);
    }
  else
    {
      rp2040_gpio_disable_irq(CONFIG_RP2040_W5500_INT_GPIO);
    }
}

/****************************************************************************
 * Name: rp2040_reset
 *
 * Description:
 *   Brings the W5500 in or out of reset.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   reset - true to reset, false to enable
 *
 ****************************************************************************/

static void rp2040_reset(const struct w5500_lower_s *lower, bool reset)
{
  rp2040_gpio_put(CONFIG_RP2040_W5500_RST_GPIO, !reset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initializes the SPI and W5500 drivers.
 *
 ****************************************************************************/

void arm_netinitialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  spi = rp2040_spibus_initialize(CONFIG_RP2040_W5500_SPI_CH);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n",
           CONFIG_RP2040_W5500_SPI_CH);
      return;
    }

  ret = w5500_initialize(spi, &g_enclower.lower, W5500_DEVNO);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI%d W5500 device %d: %d\n",
           CONFIG_RP2040_W5500_SPI_CH, W5500_DEVNO, ret);
      return;
    }

  ninfo("Bound SPI%d to W5500 device %d\n",
        CONFIG_RP2040_W5500_SPI_CH, W5500_DEVNO);
}

