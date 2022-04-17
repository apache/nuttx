/****************************************************************************
 * boards/arm/samv7/same70-xplained/src/sam_xbee.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/xbee.h>

#include "sam_gpio.h"
#include "sam_spi.h"

#include "same70-xplained.h"

#ifdef HAVE_XBEE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DRIVERS_WIRELESS
#  error Wireless support requires CONFIG_DRIVERS_WIRELESS
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_priv_s
{
  struct xbee_lower_s dev;
  uint32_t attncfg;
  uint32_t rstcfg;
  uint8_t irq;
  uint8_t csno;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the XBee driver from differences in GPIO interrupt handling
 * varying boards and MCUs.
 *
 *   sam_reset  - Reset the XBee
 *   irq_attach - Attach the XBee ATTN handler to the GPIO interrupt
 *   irq_enable - Enable or disable the GPIO interrupt
 */

static void sam_reset(const struct xbee_lower_s *lower);
static int  sam_attach_irq(const struct xbee_lower_s *lower,
                           xcpt_t handler, void *arg);
static void sam_enable_irq(const struct xbee_lower_s *lower,
                           bool state);
static bool sam_poll_attn(const struct xbee_lower_s *lower);
static int  sam_xbee_devsetup(struct sam_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the XBee
 * driver.  This structure provides information about the configuration
 * of the XBee and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

#ifdef CONFIG_SAME70XPLAINED_MB1_XBEE
static struct sam_priv_s g_xbee_mb1_priv =
{
  .dev.reset   = sam_reset,
  .dev.attach  = sam_attach_irq,
  .dev.enable  = sam_enable_irq,
  .dev.poll    = sam_poll_attn,
  .attncfg     = CLICK_MB1_INTR,
  .rstcfg      = CLICK_MB1_RESET,
  .irq         = IRQ_MB1,
  .csno        = MB1_CSNO,
};
#endif

#ifdef CONFIG_SAME70XPLAINED_MB2_XBEE
static struct sam_priv_s g_xbee_mb2_priv =
{
  .dev.reset   = sam_reset,
  .dev.attach  = sam_attach_irq,
  .dev.enable  = sam_enable_irq,
  .dev.poll    = sam_poll_attn,
  .attncfg     = CLICK_MB2_INTR,
  .rstcfg      = CLICK_MB2_RESET,
  .irq         = IRQ_MB2,
  .csno        = MB2_CSNO,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sam_reset(const struct xbee_lower_s *lower)
{
  struct sam_priv_s *priv = (struct sam_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Reset pulse */

  sam_gpiowrite(priv->rstcfg, false);
  up_udelay(1);
  sam_gpiowrite(priv->rstcfg, true);

  up_mdelay(100);
}

static int sam_attach_irq(const struct xbee_lower_s *lower,
                          xcpt_t handler, void *arg)
{
  struct sam_priv_s *priv = (struct sam_priv_s *)lower;
  int ret;

  DEBUGASSERT(priv != NULL);

  ret = irq_attach(priv->irq, handler, arg);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to attach XBee interrupt: %d\n", ret);
    }

  return ret;
}

static void sam_enable_irq(const struct xbee_lower_s *lower,
                           bool state)
{
  struct sam_priv_s *priv = (struct sam_priv_s *)lower;
  static bool enabled;
  irqstate_t flags;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv != NULL);
  wlinfo("state: %d irq: %u\n", (int)state, priv->irq);

  /* Has the interrupt state changed */

  flags = enter_critical_section();
  if (state != enabled)
    {
      /* Enable or disable interrupts */

      if (state)
        {
          wlinfo("Enabling\n");
          sam_gpioirqenable(priv->irq);
          enabled = true;
        }
      else
        {
          wlinfo("Disabling\n");
          sam_gpioirqdisable(priv->irq);
          enabled = false;
        }
    }

  leave_critical_section(flags);
}

static bool sam_poll_attn(const struct xbee_lower_s *lower)
{
  struct sam_priv_s *priv = (struct sam_priv_s *)lower;

  return !sam_gpioread(priv->attncfg);
}

/****************************************************************************
 * Name: sam_xbee_devsetup
 *
 * Description:
 *   Initialize one the XBee device in one mikroBUS slot
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int sam_xbee_devsetup(struct sam_priv_s *priv)
{
  struct xbee_mac_s *xbee;
  struct spi_dev_s *spi;
  int ret;

  sam_configgpio(priv->rstcfg);
  sam_configgpio(priv->attncfg);
  sam_gpioirq(priv->attncfg);

  /* Initialize the SPI bus and get an instance of the SPI interface */

  spi = sam_spibus_initialize(priv->csno);
  if (spi == NULL)
    {
      wlerr("ERROR: Failed to initialize SPI bus %d\n", priv->csno);
      return -ENODEV;
    }

  /* Initialize and register the SPI XBee device */

  xbee = xbee_init(spi, &priv->dev);
  if (xbee == NULL)
    {
      wlerr("ERROR: Failed to initialize XBee radio\n");
      return -ENODEV;
    }

  ret = xbee_netdev_register(xbee);
  if (ret < 0)
    {
      wlerr("ERROR: "
            "Failed to register the XBee MAC network driver wpan%d: %d\n",
            0, ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_xbee_initialize
 *
 * Description:
 *   Initialize the XBee device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_xbee_initialize(void)
{
  int ret;

#ifdef CONFIG_SAME70XPLAINED_MB1_XBEE
  wlinfo("Configuring XBee in mikroBUS1\n");

  ret = sam_xbee_devsetup(&g_xbee_mb1_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize XBee on mikroBUS1: %d\n", ret);
    }
#endif

#ifdef CONFIG_SAME70XPLAINED_MB2_XBEE
  wlinfo("Configuring XBee in mikroBUS2\n");

  ret = sam_xbee_devsetup(&g_xbee_mb2_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize XBee on mikroBUS2: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
#endif /* HAVE_XBEE */
