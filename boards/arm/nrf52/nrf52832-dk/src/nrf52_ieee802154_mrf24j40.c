/****************************************************************************
 * boards/arm/nrf52/nrf52832-dk/src/nrf52_ieee802154_mrf24j40.c
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

#include <debug.h>

#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"
#include "nrf52_spi.h"

#include "nrf52_mrf24j40.h"

#include "nrf52832-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF52_SPI0_MASTER
#  error this driver requires CONFIG_NRF52_SPI0_MASTER
#endif

#define NRF52_MRF24J40_SPI (0)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the MRF24J40 driver from differences in GPIO interrupt handling
 * varying boards and MCUs.
 *
 *   irq_attach - Attach the MRF24J40 interrupt handler to the GPIO
 *                interrupt
 *   irq_enable - Enable or disable the GPIO interrupt
 */

static int  nrf52_attach_irq(const struct mrf24j40_lower_s *lower,
                             xcpt_t handler, void *arg);
static void nrf52_enable_irq(const struct mrf24j40_lower_s *lower,
                             bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the MRF24J40
 * driver.  This structure provides information about the configuration
 * of the MRF24J40 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct nrf52_mrf24j40_s g_mrf24j40_priv =
{
  .dev.attach  = nrf52_attach_irq,
  .dev.enable  = nrf52_enable_irq,
  .handler     = NULL,
  .arg         = NULL,
  .intcfg      = GPIO_MRF24J40_INT,
  .spidev      = NRF52_MRF24J40_SPI,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_attach_irq
 *
 * Description:
 *   Attach the MRF24J40 interrupt handler to the GPIO interrupt.
 *
 ****************************************************************************/

static int nrf52_attach_irq(const struct mrf24j40_lower_s *lower,
                            xcpt_t handler, void *arg)
{
  struct nrf52_mrf24j40_s *priv = (struct nrf52_mrf24j40_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

/****************************************************************************
 * Name: nrf52_enable_irq
 *
 * Description:
 *   Enable or disable the GPIO interrupt
 *
 ****************************************************************************/

static void nrf52_enable_irq(const struct mrf24j40_lower_s *lower,
                             bool state)
{
  struct nrf52_mrf24j40_s *priv = (struct nrf52_mrf24j40_s *)lower;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv != NULL && (priv->handler != NULL || !state));

  wlinfo("state:%d\n", (int)state);

  /* Attach and enable, or detach and disable */

  if (state)
    {
      nrf52_gpiote_set_event(priv->intcfg, false, true,
                             priv->handler, priv->arg);
    }
  else
    {
      nrf52_gpiote_set_event(priv->intcfg, false, false,
                             priv->handler, priv->arg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_mrf24j40_initialize
 *
 * Description:
 *   Initialize the MRF24J40 device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nrf52_mrf24j40_initialize(void)
{
  int ret;

  ret = nrf52_mrf24j40_devsetup(&g_mrf24j40_priv);
  if (ret < 0)
    {
      wlerr("Failed to initialize mrf24j40: %d\n", ret);
    }

  return ret;
}
