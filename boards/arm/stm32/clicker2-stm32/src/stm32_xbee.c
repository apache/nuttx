/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/stm32_xbee.c
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
#include <nuttx/wireless/ieee802154/xbee.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_spi.h"

#include "clicker2-stm32.h"

#ifdef CONFIG_IEEE802154_XBEE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DRIVERS_WIRELESS
#  error Wireless support requires CONFIG_DRIVERS_WIRELESS
#endif

#if !defined(CONFIG_CLICKER2_STM32_MB1_XBEE) && \
    !defined(CONFIG_CLICKER2_STM32_MB2_XBEE)
#  error Only the Mikroe XBee board is supported
#endif

#ifdef CONFIG_CLICKER2_STM32_MB1_XBEE
#  ifndef CONFIG_STM32_SPI3
#    error Mikroe XBee on mikroBUS1 requires CONFIG_STM32_SPI3
#  endif
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_XBEE
#  ifndef CONFIG_STM32_SPI2
#    error Mikroe XBee on mikroBUS1 requires CONFIG_STM32_SPI2
#  endif
#endif

/* Reset
 *
 *   mikroBUS1 Reset: PE7-MB1_RST
 *   mikroBUS2 Reset: PE13-MB2_RST
 *
 * Reset line must be configured for opendrain
 */

#define GPIO_MB1_XBEE_RST  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN7)
#define GPIO_MB2_XBEE_RST  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)

/* Interrupts
 *
 *   mikroBUS1 Interrupt: PE10-MB1_INT
 *   mikroBUS2 Interrupt: PE14-MB2_INT
 */

#define GPIO_MB1_XBEE_INT   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN10)
#define GPIO_MB2_XBEE_INT   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN14)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_priv_s
{
  struct xbee_lower_s dev;
  xcpt_t handler;
  void *arg;
  uint32_t rstcfg;
  uint32_t attncfg;
  uint8_t spidev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XBee driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   reset        - Reset the XBee using the reset pin
 *   attn_attach  - Attach the XBee interrupt handler to the GPIO
 *                 interrupt (ATTN)
 *   attn_enable  - Enable or disable the GPIO interrupt
 *   attn_poll    - Poll the current state of the GPIO interrupt (ATTN)
 */

static void stm32_reset(const struct xbee_lower_s *lower);
static int  stm32_attach_attn(const struct xbee_lower_s *lower,
                              xcpt_t handler, void *arg);
static void stm32_enable_attn(const struct xbee_lower_s *lower,
                              bool state);
static bool stm32_poll_attn(const struct xbee_lower_s *lower);
static int  stm32_xbee_devsetup(struct stm32_priv_s *priv);

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

#ifdef CONFIG_CLICKER2_STM32_MB1_XBEE
static struct stm32_priv_s g_xbee_mb1_priv =
{
  .dev.reset    = stm32_reset,
  .dev.attach   = stm32_attach_attn,
  .dev.enable   = stm32_enable_attn,
  .dev.poll     = stm32_poll_attn,
  .handler      = NULL,
  .arg          = NULL,
  .rstcfg       = GPIO_MB1_XBEE_RST,
  .attncfg      = GPIO_MB1_XBEE_INT,
  .spidev       = 3,
};
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_XBEE
static struct stm32_priv_s g_xbee_mb2_priv =
{
  .dev.reset    = stm32_reset,
  .dev.attach   = stm32_attach_attn,
  .dev.enable   = stm32_enable_attn,
  .dev.poll     = stm32_poll_attn,
  .handler      = NULL,
  .arg          = NULL,
  .rstcfg       = GPIO_MB2_XBEE_RST,
  .attncfg      = GPIO_MB2_XBEE_INT,
  .spidev       = 2,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XBee driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   reset        - Reset the XBee using the reset pin
 *   attn_attach  - Attach the XBee interrupt handler to the GPIO
 *                 interrupt (ATTN)
 *   attn_enable  - Enable or disable the GPIO interrupt
 *   attn_poll    - Poll the current state of the GPIO interrupt (ATTN)
 */

static void stm32_reset(const struct xbee_lower_s *lower)
{
  struct stm32_priv_s *priv = (struct stm32_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Hold reset line low for min. 200ns */

  stm32_gpiowrite(priv->rstcfg, false);
  up_udelay(1);
  stm32_gpiowrite(priv->rstcfg, true);

  up_mdelay(100);
}

static int stm32_attach_attn(const struct xbee_lower_s *lower,
                             xcpt_t handler, void *arg)
{
  struct stm32_priv_s *priv = (struct stm32_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void stm32_enable_attn(const struct xbee_lower_s *lower,
                              bool state)
{
  struct stm32_priv_s *priv = (struct stm32_priv_s *)lower;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv != NULL && (priv->handler != NULL || !state));

#ifdef CONFIG_CLICKER2_STM32_XBEELH_VERBOSE
  wlinfo("state:%d\n", (int)state);
#endif

  /* Attach and enable, or detach and disable */

  if (state)
    {
      stm32_gpiosetevent(priv->attncfg, false, true, true,
                         priv->handler, priv->arg);
    }
  else
    {
      stm32_gpiosetevent(priv->attncfg, false, false, false,
                         NULL, NULL);
    }
}

static bool stm32_poll_attn(const struct xbee_lower_s *lower)
{
  struct stm32_priv_s *priv = (struct stm32_priv_s *)lower;

  return !stm32_gpioread(priv->attncfg);
}

/****************************************************************************
 * Name: stm32_xbee_devsetup
 *
 * Description:
 *   Initialize one the XBee device in one mikroBUS slot
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int stm32_xbee_devsetup(struct stm32_priv_s *priv)
{
  struct spi_dev_s *spi;
  XBEEHANDLE xbee;
  int ret;

  /* Configure the Reset and Attention pins */

  stm32_configgpio(priv->rstcfg);
  stm32_configgpio(priv->attncfg);

  /* Initialize the SPI bus and get an instance of the SPI interface */

  spi = stm32_spibus_initialize(priv->spidev);
  if (spi == NULL)
    {
      wlerr("ERROR: Failed to initialize SPI bus %d\n", priv->spidev);
      return -ENODEV;
    }

  /* Initialize and register the SPI XBee device */

  xbee = xbee_init(spi, &priv->dev);
  if (xbee == NULL)
    {
      wlerr("ERROR: Failed to initialize XBee driver%d\n", priv->spidev);
      return -ENODEV;
    }

  /* Register the XBee netdev providing it the XBee MAC layer to interface
   * with
   */

  ret = xbee_netdev_register(xbee);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to register "
            "the XBee MAC network driver wpan%d: %d\n",
            0, ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_xbee_initialize
 *
 * Description:
 *   Initialize the XBee device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_xbee_initialize(void)
{
  int ret;

#ifdef CONFIG_CLICKER2_STM32_MB1_XBEE
  wlinfo("Configuring XBee in mikroBUS1\n");

  ret = stm32_xbee_devsetup(&g_xbee_mb1_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize XBee on mikroBUS1: %d\n", ret);
    }
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_XBEE
  wlinfo("Configuring XBee in mikroBUS2\n");

  ret = stm32_xbee_devsetup(&g_xbee_mb2_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize XBee on mikroBUS2: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
#endif /* CONFIG_IEEE802154_XBEE */
