/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_w5500.c
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

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_spi.h"

#include "stm32f4discovery.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* W5500 is on SPI1 */

#ifndef CONFIG_STM32_SPI1
# error "Need CONFIG_STM32_SPI1 in the configuration"
#endif

/* SPI Assumptions **********************************************************/

#define W5500_SPI_PORTNO 1   /* On SPI1 */
#define W5500_DEVNO      0   /* Only one W5500 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lower_s
{
  const struct w5500_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                     handler;  /* W5500 interrupt handler */
  void                      *arg;      /* Argument that accompanies IRQ */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                      void *arg);
static void up_enable(const struct w5500_lower_s *lower, bool enable);
static void up_reset(const struct w5500_lower_s *lower, bool reset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The W5500 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanism for controlling
 * the W5500 GPIO interrupt.
 */

static struct stm32_lower_s g_enclower =
{
  .lower =
  {
    .frequency = 10000000,
    .spidevid  = 0,
    .mode      = SPIDEV_MODE0,
    .attach    = up_attach,
    .enable    = up_enable,
    .reset     = up_reset,
  },
  .handler = NULL,
  .arg     = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct w5500_lower_s methods
 ****************************************************************************/

static int up_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                     void *arg)
{
  struct stm32_lower_s *priv = (struct stm32_lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void up_enable(const struct w5500_lower_s *lower, bool enable)
{
  struct stm32_lower_s *priv = (struct stm32_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  if (enable)
    {
      stm32_gpiosetevent(GPIO_W5500_INTR, false, true, true,
                         priv->handler, priv->arg);
    }
  else
    {
      stm32_gpiosetevent(GPIO_W5500_INTR, false, true, true,
                         NULL, NULL);
    }
}

/* REVISIT:  Since the interrupt is completely torn down, not just disabled,
 * in interrupt requests that occurs while the interrupt is disabled will be
 * lost.
 */

static void up_reset(const struct w5500_lower_s *lower, bool reset)
{
  /* Take W5500 out of reset (active low) */

  stm32_gpiowrite(GPIO_W5500_RESET, !reset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 ****************************************************************************/

void arm_netinitialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  /* Assumptions:
   * 1) W5500 pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in
   *    boot-up.
   */

  spi = stm32_spibus_initialize(W5500_SPI_PORTNO);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n", W5500_SPI_PORTNO);
      return;
    }

  /* Bind the SPI port to the W5500 driver */

  ret = w5500_initialize(spi, &g_enclower.lower, W5500_DEVNO);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI port %d W5500 device %d: %d\n",
           W5500_SPI_PORTNO, W5500_DEVNO, ret);
      return;
    }

  ninfo("Bound SPI port %d to W5500 device %d\n",
        W5500_SPI_PORTNO, W5500_DEVNO);
}

