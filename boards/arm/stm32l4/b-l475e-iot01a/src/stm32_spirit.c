/****************************************************************************
 * boards/arm/stm32l4/b-l475e-iot01a/src/stm32_spirit.c
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
#include <stdint.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wireless/spirit.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "stm32l4_gpio.h"
#include "stm32l4_exti.h"
#include "stm32l4_spi.h"

#include "b-l475e-iot01a.h"

#ifdef HAVE_SPSGRF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DRIVERS_WIRELESS
#  error Wireless support requires CONFIG_DRIVERS_WIRELESS
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32l4_priv_s
{
  struct spirit_lower_s dev;
  xcpt_t handler;
  void *arg;
  uint32_t intcfg;
  uint32_t sdncfg;
  uint8_t spidev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the Spirit driver from differences in GPIO interrupt handling
 * varying boards and MCUs.
 *
 *   stm32l4_reset      - Reset the Spirit part.
 *   stm32l4_attach_irq - Attach the Spirit interrupt handler to the GPIO
 *                        interrupt
 *   stm32l4_enable_irq - Enable or disable the GPIO interrupt
 */

static int  stm32l4_reset(const struct spirit_lower_s *lower);
static int  stm32l4_attach_irq(const struct spirit_lower_s *lower,
                               xcpt_t handler, void *arg);
static void stm32l4_enable_irq(const struct spirit_lower_s *lower,
                               bool state);
static int  stm32l4_spirit_devsetup(struct stm32l4_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the Spirit
 * driver.  This structure provides information about the configuration
 * of the Spirit and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32l4_priv_s g_spirit =
{
  .dev.reset   = stm32l4_reset,
  .dev.attach  = stm32l4_attach_irq,
  .dev.enable  = stm32l4_enable_irq,
  .handler     = NULL,
  .arg         = NULL,
  .intcfg      = GPIO_SPSGRF_INT,
  .sdncfg      = GPIO_SPSGRF_SDN,
  .spidev      = 3,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Reset the Spirit 1 part */

static int stm32l4_reset(const struct spirit_lower_s *lower)
{
  struct stm32l4_priv_s *priv = (struct stm32l4_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Reset pulse */

  stm32l4_gpiowrite(priv->sdncfg, true);
  stm32l4_gpiowrite(priv->sdncfg, false);

  /* Wait minimum 1.5 ms to allow Spirit a proper boot-up sequence */

  nxsig_usleep(1500);
  return OK;
}

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the Spirit driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   stm32l4_attach_irq - Attach the Spirit interrupt handler to the GPIO
 *                        interrupt
 *   stm32l4_enable_irq - Enable or disable the GPIO interrupt
 */

static int stm32l4_attach_irq(const struct spirit_lower_s *lower,
                              xcpt_t handler, void *arg)
{
  struct stm32l4_priv_s *priv = (struct stm32l4_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void stm32l4_enable_irq(const struct spirit_lower_s *lower,
                               bool state)
{
  struct stm32l4_priv_s *priv = (struct stm32l4_priv_s *)lower;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv != NULL && (priv->handler != NULL || !state));

  wlinfo("state:%d\n", (int)state);

  /* Attach and enable, or detach and disable */

  if (state)
    {
      /* Enable interrupts on falling edge (active low) */

      stm32l4_gpiosetevent(priv->intcfg, false, true, false,
                           priv->handler, priv->arg);
    }
  else
    {
      /* Disable interrupts */

      stm32l4_gpiosetevent(priv->intcfg, false, false, false,
                           NULL, NULL);
    }
}

/****************************************************************************
 * Name: stm32l4_spirit_devsetup
 *
 * Description:
 *   Initialize one the Spirit device
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int stm32l4_spirit_devsetup(struct stm32l4_priv_s *priv)
{
  struct spi_dev_s *spi;
  int ret;

  /* Configure the interrupt pin and SDN pins.  Innitializing the SDN to '1'
   * powers down the Spirit.
   */

  stm32l4_configgpio(priv->intcfg);
  stm32l4_configgpio(priv->sdncfg);

  /* Initialize the SPI bus and get an instance of the SPI interface */

  spi = stm32l4_spibus_initialize(priv->spidev);
  if (spi == NULL)
    {
      wlerr("ERROR: Failed to initialize SPI bus %d\n", priv->spidev);
      return -ENODEV;
    }

  /* Initialize and register the SPI Spirit device */

  ret = spirit_netdev_initialize(spi, &priv->dev);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_netdev_initialize failed %d\n", priv->spidev);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_spirit_initialize
 *
 * Description:
 *   Initialize the Spirit device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32l4_spirit_initialize(void)
{
  int ret;

  wlinfo("Configuring Spirit\n");

  ret = stm32l4_spirit_devsetup(&g_spirit);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize Spirit: %d\n", ret);
    }

  return OK;
}
#endif /* HAVE_SPSGRF */
