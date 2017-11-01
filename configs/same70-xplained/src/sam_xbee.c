/****************************************************************************
 * configs/same70-xplained/src/sam_xbee.c
 *
 *   Copyright (C) 2017 Verge Inc, All rights reserver
 *   Author: Anthony Merlino <anthony@vergeaero.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

static void sam_reset(FAR const struct xbee_lower_s *lower);
static int  sam_attach_irq(FAR const struct xbee_lower_s *lower,
                           xcpt_t handler, FAR void *arg);
static void sam_enable_irq(FAR const struct xbee_lower_s *lower,
                           bool state);
static bool sam_poll_attn(FAR const struct xbee_lower_s *lower);
static int  sam_xbee_devsetup(FAR struct sam_priv_s *priv);

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

static void sam_reset(FAR const struct xbee_lower_s *lower)
{
  FAR struct sam_priv_s *priv = (FAR struct sam_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Reset pulse */

  sam_gpiowrite(priv->rstcfg, false);
  up_udelay(1);
  sam_gpiowrite(priv->rstcfg, true);

  up_mdelay(100);
}

static int sam_attach_irq(FAR const struct xbee_lower_s *lower,
                            xcpt_t handler, FAR void *arg)
{
  FAR struct sam_priv_s *priv = (FAR struct sam_priv_s *)lower;
  int ret;

  DEBUGASSERT(priv != NULL);

  ret = irq_attach(priv->irq, handler, arg);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to attach XBee interrupt: %d\n", ret);
    }

  return ret;
}

static void sam_enable_irq(FAR const struct xbee_lower_s *lower,
                             bool state)
{
  FAR struct sam_priv_s *priv = (FAR struct sam_priv_s *)lower;
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

static bool sam_poll_attn(FAR const struct xbee_lower_s *lower)
{
  FAR struct sam_priv_s *priv = (FAR struct sam_priv_s *)lower;

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

static int sam_xbee_devsetup(FAR struct sam_priv_s *priv)
{
  FAR struct xbee_mac_s *xbee;
  FAR struct spi_dev_s *spi;
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
      wlerr("ERROR: Failed to register the XBee MAC network driver wpan%d: %d\n",
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
