/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/stm32_mrf24j40.c
 *
 *   Copyright (C) 2017 Gregory Nutt, All rights reserver
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <nuttx/fs/fs.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/mrf24j40.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_spi.h"

#include "clicker2-stm32.h"

#ifdef CONFIG_IEEE802154_MRF24J40

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DRIVERS_WIRELESS
#  error Wireless support requires CONFIG_DRIVERS_WIRELESS
#endif

#if !defined(CONFIG_CLICKER2_STM32_MB1_BEE) && \
    !defined(CONFIG_CLICKER2_STM32_MB2_BEE)
#  error Only the Mikroe BEE board is supported
#endif

#ifdef CONFIG_CLICKER2_STM32_MB1_BEE
#  ifndef CONFIG_STM32_SPI3
#    error Mikroe BEE on mikroBUS1 requires CONFIG_STM32_SPI3
#  endif
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_BEE
#  ifndef CONFIG_STM32_SPI2
#    error Mikroe BEE on mikroBUS1 requires CONFIG_STM32_SPI2
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_priv_s
{
  struct mrf24j40_lower_s dev;
  xcpt_t handler;
  FAR void *arg;
  uint32_t intcfg;
  uint8_t spidev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the MRF24J40 driver from differences in GPIO interrupt handling
 * varying boards and MCUs.
 *
 *   irq_attach - Attach the MRF24J40 interrupt handler to the GPIO
                  interrupt
 *   irq_enable - Enable or disable the GPIO interrupt
 */

static int  stm32_attach_irq(FAR const struct mrf24j40_lower_s *lower,
                             xcpt_t handler, FAR void *arg);
static void stm32_enable_irq(FAR const struct mrf24j40_lower_s *lower,
                             bool state);
static int  stm32_mrf24j40_devsetup(FAR struct stm32_priv_s *priv);

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

#ifdef CONFIG_CLICKER2_STM32_MB1_BEE
static struct stm32_priv_s g_mrf24j40_mb1_priv =
{
  .dev.attach  = stm32_attach_irq,
  .dev.enable  = stm32_enable_irq,
  .handler     = NULL,
  .arg         = NULL,
  .intcfg      = GPIO_MB1_INT,
  .spidev      = 3,
};
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_BEE
static struct stm32_priv_s g_mrf24j40_mb2_priv =
{
  .dev.attach  = stm32_attach_irq,
  .dev.enable  = stm32_enable_irq,
  .handler     = NULL,
  .arg         = NULL,
  .intcfg      = GPIO_MB2_INT,
  .spidev      = 2,
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

static int stm32_attach_irq(FAR const struct mrf24j40_lower_s *lower,
                            xcpt_t handler, FAR void *arg)
{
  FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void stm32_enable_irq(FAR const struct mrf24j40_lower_s *lower,
                             bool state)
{
  FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)lower;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv != NULL && (priv->handler != NULL || !state));

#ifdef CONFIG_CLICKER2_STM32_MRF24J40LH_VERBOSE
  wlinfo("state:%d\n", (int)state);
#endif

  /* Attach and enable, or detach and disable */

  if (state)
    {
      stm32_gpiosetevent(priv->intcfg, false, true, true,
                         priv->handler, priv->arg);
    }
  else
    {
      stm32_gpiosetevent(priv->intcfg, false, false, false,
                         NULL, NULL);
    }
}

/****************************************************************************
 * Name: stm32_mrf24j40_devsetup
 *
 * Description:
 *   Initialize one the MRF24J40 device in one mikroBUS slot
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int stm32_mrf24j40_devsetup(FAR struct stm32_priv_s *priv)
{
  FAR struct ieee802154_radio_s *radio;
  MACHANDLE mac;
  FAR struct spi_dev_s *spi;
  int ret;

  /* Configure the interrupt pin */

   stm32_configgpio(priv->intcfg);

  /* Initialize the SPI bus and get an instance of the SPI interface */

  spi = stm32_spibus_initialize(priv->spidev);
  if (spi == NULL)
    {
      wlerr("ERROR: Failed to initialize SPI bus %d\n", priv->spidev);
      return -ENODEV;
    }

  /* Initialize and register the SPI MRF24J40 device */

  radio = mrf24j40_init(spi, &priv->dev);
  if (radio == NULL)
    {
      wlerr("ERROR: Failed to initialize SPI bus %d\n", priv->spidev);
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
   * network interface by wrapping the MAC intrface instance in a
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
   * interface in a character device viamac802154dev_register().
   */

  ret = mac802154dev_register(mac, 0);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to register the MAC character driver /dev/ieee%d: %d\n",
            0, ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mrf24j40_initialize
 *
 * Description:
 *   Initialize the MRF24J40 device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_mrf24j40_initialize(void)
{
  int ret;

#ifdef CONFIG_CLICKER2_STM32_MB1_BEE
  wlinfo("Configuring BEE in mikroBUS1\n");

  ret = stm32_mrf24j40_devsetup(&g_mrf24j40_mb1_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize BD in mikroBUS1: %d\n", ret);
    }
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_BEE
  wlinfo("Configuring BEE in mikroBUS2\n");

  ret = stm32_mrf24j40_devsetup(&g_mrf24j40_mb2_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize BD in mikroBUS2: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
#endif /* CONFIG_IEEE802154_MRF24J40 */
