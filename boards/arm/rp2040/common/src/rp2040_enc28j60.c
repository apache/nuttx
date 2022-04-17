/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_enc28j60.c
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
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/enc28j60.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "rp2040_spi.h"
#include "rp2040_gpio.h"

#ifdef CONFIG_ENC28J60

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_RP2040_ENC28J60_INTR_GPIO < 0
# error "Need CONFIG_RP2040_ENC28J60_INTR_GPIO to specify the interrupt pin"
#endif

#define ENC28J60_DEVNO      0   /* Only one ENC28J60 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp2040_lower_s
{
  const struct enc_lower_s lower;    /* Low-level MCU interface */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(const struct enc_lower_s *lower, xcpt_t handler,
                      void *arg);
static void up_enable(const struct enc_lower_s *lower);
static void up_disable(const struct enc_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The ENC28J60 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanism for controlling
 * the ENC28J60 GPIO interrupt.
 */

static struct rp2040_lower_s g_enclower =
{
  .lower =
  {
    .attach  = up_attach,
    .enable  = up_enable,
    .disable = up_disable
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct enc_lower_s methods
 ****************************************************************************/

static int up_attach(const struct enc_lower_s *lower, xcpt_t handler,
                     void *arg)
{
  rp2040_gpio_irq_attach(CONFIG_RP2040_ENC28J60_INTR_GPIO,
                         RP2040_GPIO_INTR_EDGE_LOW, handler, arg);
  return OK;
}

static void up_enable(const struct enc_lower_s *lower)
{
  rp2040_gpio_enable_irq(CONFIG_RP2040_ENC28J60_INTR_GPIO);
}

static void up_disable(const struct enc_lower_s *lower)
{
  rp2040_gpio_disable_irq(CONFIG_RP2040_ENC28J60_INTR_GPIO);
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

  spi = rp2040_spibus_initialize(CONFIG_RP2040_ENC28J60_SPI_CH);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n",
           CONFIG_RP2040_ENC28J60_SPI_CH);
      return;
    }

  /* Take ENC28J60 out of reset (active low) */

#if CONFIG_RP2040_ENC28J60_RESET_GPIO >= 0
  rp2040_gpio_init(CONFIG_RP2040_ENC28J60_RESET_GPIO);
  rp2040_gpio_setdir(CONFIG_RP2040_ENC28J60_RESET_GPIO, true);
  rp2040_gpio_put(CONFIG_RP2040_ENC28J60_RESET_GPIO, true);
#endif

  rp2040_gpio_init(CONFIG_RP2040_ENC28J60_INTR_GPIO);
  rp2040_gpio_set_pulls(CONFIG_RP2040_ENC28J60_INTR_GPIO, true, false);

  /* Bind the SPI port to the ENC28J60 driver */

  ret = enc_initialize(spi, &g_enclower.lower, ENC28J60_DEVNO);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI port %d ENC28J60 device %d: %d\n",
           CONFIG_RP2040_ENC28J60_SPI_CH, ENC28J60_DEVNO, ret);
      return;
    }

  ninfo("Bound SPI port %d to ENC28J60 device %d\n",
        CONFIG_RP2040_ENC28J60_SPI_CH, ENC28J60_DEVNO);
}

#endif /* CONFIG_ENC28J60 */
