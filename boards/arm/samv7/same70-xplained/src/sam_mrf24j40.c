/****************************************************************************
 * boards/arm/samv7/same70-xplained/src/sam_mrf24j40.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/mrf24j40.h>

#include "sam_gpio.h"
#include "sam_spi.h"

#include "same70-xplained.h"

#ifdef HAVE_MRF24J40

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef BEE_RESET

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_priv_s
{
  struct mrf24j40_lower_s dev;
  uint32_t intcfg;
#ifdef BEE_RESET
  uint32_t rstcfg;
#endif
  uint8_t irq;
  uint8_t csno;
};

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

static int  sam_attach_irq(FAR const struct mrf24j40_lower_s *lower,
                           xcpt_t handler, FAR void *arg);
static void sam_enable_irq(FAR const struct mrf24j40_lower_s *lower,
                           bool state);
static int  sam_mrf24j40_devsetup(FAR struct sam_priv_s *priv);

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

#ifdef CONFIG_SAME70XPLAINED_MB1_BEE
static struct sam_priv_s g_mrf24j40_mb1_priv =
{
  .dev.attach  = sam_attach_irq,
  .dev.enable  = sam_enable_irq,
  .intcfg      = CLICK_MB1_INTR,
#ifdef BEE_RESET
  .rstcfg      = CLICK_MB1_RESET,
#endif
  .irq         = IRQ_MB1,
  .csno        = MB1_CSNO,
};
#endif

#ifdef CONFIG_SAME70XPLAINED_MB2_BEE
static struct sam_priv_s g_mrf24j40_mb2_priv =
{
  .dev.attach  = sam_attach_irq,
  .dev.enable  = sam_enable_irq,
  .intcfg      = CLICK_MB2_INTR,
#ifdef BEE_RESET
  .rstcfg      = CLICK_MB2_RESET,
#endif
  .irq         = IRQ_MB2,
  .csno        = MB2_CSNO,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the MRF24J40 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   irq_attach       - Attach the MRF24J40 interrupt handler to the GPIO
 *                      interrupt
 *   irq_enable       - Enable or disable the GPIO interrupt
 */

static int sam_attach_irq(FAR const struct mrf24j40_lower_s *lower,
                            xcpt_t handler, FAR void *arg)
{
  FAR struct sam_priv_s *priv = (FAR struct sam_priv_s *)lower;
  int ret;

  DEBUGASSERT(priv != NULL);

  ret = irq_attach(priv->irq, handler, arg);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to attach WM8904 interrupt: %d\n", ret);
    }

  return ret;
}

static void sam_enable_irq(FAR const struct mrf24j40_lower_s *lower,
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

/****************************************************************************
 * Name: sam_mrf24j40_devsetup
 *
 * Description:
 *   Initialize one the MRF24J40 device in one mikroBUS slot
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int sam_mrf24j40_devsetup(FAR struct sam_priv_s *priv)
{
  FAR struct ieee802154_radio_s *radio;
  MACHANDLE mac;
  FAR struct spi_dev_s *spi;
  int ret;

#ifdef BEE_RESET
  /* Bring the MRF24J40 out of reset
   * NOTE: Not necessary.  The RST# input is pulled high on the BEE.
   */

  sam_configgpio(priv->rstcfg);
  sam_gpiowrite(priv->rstcfg, true);
#endif

  /* Configure the interrupt pin */

  sam_configgpio(priv->intcfg);
  sam_gpioirq(priv->intcfg);

  /* Initialize the SPI bus and get an instance of the SPI interface */

  spi = sam_spibus_initialize(priv->csno);
  if (spi == NULL)
    {
      wlerr("ERROR: Failed to initialize SPI bus %d\n", priv->csno);
      return -ENODEV;
    }

  /* Initialize and register the SPI MRF24J40 device */

  radio = mrf24j40_init(spi, &priv->dev);
  if (radio == NULL)
    {
      wlerr("ERROR: Failed to initialize MRF24J40 radio\n");
      return -ENODEV;
    }

  /* Create a 802.15.4 MAC device from a 802.15.4 compatible radio device. */

  mac = mac802154_create(radio);
  if (mac == NULL)
    {
      wlerr("ERROR: Failed to initialize IEEE802.15.4 MAC\n");
      return -ENODEV;
    }

#ifdef CONFIG_IEEE802154_NETDEV
  /* Use the IEEE802.15.4 MAC interface instance to create a 6LoWPAN
   * network interface by wrapping the MAC interface instance in a
   * network device driver via mac802154dev_register().
   */

  ret = mac802154netdev_register(mac);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to register the MAC network driver wpan%d: %d\n",
            0, ret);
      return ret;
    }
#endif

#ifdef CONFIG_IEEE802154_MACDEV
  /* If want to call these APIs from userspace, you have to wrap the MAC
   * interface in a character device via mac802154dev_register().
   */

  ret = mac802154dev_register(mac, 0);
  if (ret < 0)
    {
      wlerr(
      "ERROR: Failed to register the MAC character driver /dev/ieee%d: %d\n",
            0, ret);
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_mrf24j40_initialize
 *
 * Description:
 *   Initialize the MRF24J40 device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_mrf24j40_initialize(void)
{
  int ret;

#ifdef CONFIG_SAME70XPLAINED_MB1_BEE
  wlinfo("Configuring BEE in mikroBUS1\n");

  ret = sam_mrf24j40_devsetup(&g_mrf24j40_mb1_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize BD in mikroBUS1: %d\n", ret);
    }
#endif

#ifdef CONFIG_SAME70XPLAINED_MB2_BEE
  wlinfo("Configuring BEE in mikroBUS2\n");

  ret = sam_mrf24j40_devsetup(&g_mrf24j40_mb2_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize BD in mikroBUS2: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
#endif /* HAVE_MRF24J40 */
