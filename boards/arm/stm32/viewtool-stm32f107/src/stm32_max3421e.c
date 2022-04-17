/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/stm32_max3421e.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <nuttx/kthread.h>
#include <nuttx/spi/spi.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/max3421e.h>

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "viewtool_stm32f107.h"

#if defined(CONFIG_VIEWTOOL_MAX3421E_SPI1) || defined(CONFIG_VIEWTOOL_MAX3421E_SPI2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct viewtool_max3421elower_s
{
  /* Standard MAX3421E interface */

  struct max3421e_lowerhalf_s config;

  /* Extensions for the viewtool board */

  xcpt_t handler;
  void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the MAX3421E driver from differences in GPIO interrupt handling
 * by varying boards and MCUs.
 *
 * Interrupts should be configured on the falling edge of nINT.
 *
 *   attach      - Attach the ADS7843E interrupt handler to the GPIO
 *                 interrupt
 *   enable      - Enable or disable the GPIO interrupt
 *   acknowledge - Acknowledge/clear any pending GPIO interrupt as necessary.
 */

static int max3421e_attach(const struct max3421e_lowerhalf_s *lower,
                           xcpt_t isr, void *arg);
static void max3421e_enable(const struct max3421e_lowerhalf_s *lower,
                            bool enable);
static void max3421e_acknowledge
                           (const struct max3421e_lowerhalf_s *lower);
static void max3421e_power(const struct max3421e_lowerhalf_s *lower,
                           bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the MAX3421E
 * driver.  This structure provides information about the configuration
 * of the MAX3421E and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify certain values.
 */

static struct viewtool_max3421elower_s g_max3421e_lower =
{
  .config =
  {
    .spi         = NULL,                               /* SPI device instance */
    .frequency   = CONFIG_VIEWTOOL_MAX3421E_FREQUENCY, /* SPI frequency < 26MHz */
    .mode        = SPIDEV_MODE0,                       /* SPI Mode 0 */
    .devid       = 0,                                  /* Only one MAX3421E on SPI bus */
    .intconfig   = 0,                                  /* Falling edge-sensitive. */

    .attach      = max3421e_attach,                    /* Attach interrupt handler */
    .enable      = max3421e_enable,                    /* Enable interrupt */
    .acknowledge = max3421e_acknowledge,               /* Acknowledge/clear interrupt */
    .power       = max3421e_power,                     /* Enable VBUS power */
  },
};

static struct usbhost_connection_s *g_usbconn;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the MAX3421E driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach      - Attach the ADS7843E interrupt handler to the GPIO
 *                 interrupt
 *   enable      - Enable or disable the GPIO interrupt
 *   acknowledge - Acknowledge/clear any pending GPIO interrupt as necessary.
 *   power       - Enable or disable 5V VBUS power
 *
 ****************************************************************************/

static int max3421e_attach(const struct max3421e_lowerhalf_s *lower,
                           xcpt_t isr, void *arg)
{
  struct viewtool_max3421elower_s *priv =
    (struct viewtool_max3421elower_s *)lower;

  if (isr != NULL)
    {
      /* Just save the address of the handler for now.  The new handler will
       * be attached when the interrupt is next enabled.
       */

      uinfo("Attaching %p\n", isr);
      priv->handler = isr;
      priv->arg     = arg;
    }
  else
    {
      uinfo("Detaching %p\n", priv->handler);
      max3421e_enable(lower, false);
      priv->handler = NULL;
      priv->arg     = NULL;
    }

  return OK;
}

static void max3421e_enable(const struct max3421e_lowerhalf_s *lower,
                            bool enable)
{
  struct viewtool_max3421elower_s *priv =
    (struct viewtool_max3421elower_s *)lower;
  irqstate_t flags;

  uinfo("enable=%u handler=%p\n", enable, priv->handler);

  /* Attach and enable, or detach and disable.  Enabling and disabling GPIO
   * interrupts is a multi-step process so the safest thing is to keep
   * interrupts disabled during the reconfiguration.
   */

  flags = enter_critical_section();
  if (enable && priv->handler != NULL)
    {
      /* Configure the EXTI interrupt using the saved handler to generate
       * an interrupt when a falling edge is detected on the INT pin.  An
       * event is also generated (but not used).
       */

      stm32_gpiosetevent(GPIO_MAX3421E_INT, false, true, true,
                         priv->handler, priv->arg);
    }
  else
    {
      /* Configure the EXTI interrupt with a NULL handler to disable it.
       *
       * REVISIT:  There is a problem here... interrupts received while
       * the EXTI is de-configured will not pend but will be lost.
       */

     stm32_gpiosetevent(GPIO_MAX3421E_INT, false, false, false,
                        NULL, NULL);
    }

  leave_critical_section(flags);
}

static void max3421e_acknowledge
            (const struct max3421e_lowerhalf_s *lower)
{
  /* Does nothing */
}

static void max3421e_power(const struct max3421e_lowerhalf_s *lower,
                           bool enable)
{
  /* We currently have no control over VBUS power */

#ifdef CONFIG_VIEWTOOL_MAX3421E_PWR
  stm32_gpiowrite(GPIO_MAX3421E_PWR, enable);
#endif
}

/****************************************************************************
 * Name: usbhost_detect
 *
 * Description:
 *   Wait for USB devices to be connected.
 ****************************************************************************/

static int usbhost_detect(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

  uinfo("Starting USB detect thread\n");

  for (; ; )
    {
      CONN_WAIT(g_usbconn, &hport);

      if (hport->connected)
        {
          CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_max3421e_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   MAX3421E USB host.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_max3421e_setup(void)
{
  struct spi_dev_s *spi;
  int ret;

  /* Configure the MAX3421E interrupt pin as an input and the reset and power
   * pins as an outputs.  Device is initially in reset and 5V power is not
   * provided.
   */

  stm32_configgpio(GPIO_MAX3421E_INT);

#ifdef CONFIG_VIEWTOOL_MAX3421E_RST
  stm32_configgpio(GPIO_MAX3421E_RST);
#endif

#ifdef CONFIG_VIEWTOOL_MAX3421E_PWR
  stm32_configgpio(GPIO_MAX3421E_PWR);
#endif

  /* Get an instance of the SPI interface for the touchscreen chip select */

  spi = stm32_spibus_initialize(MAX3421E_SPIBUS);
  if (spi == NULL)
    {
      lcderr("ERROR: Failed to initialize SPI%d\n", MAX3421E_SPIBUS);
      return -ENODEV;
    }

#ifdef CONFIG_USBHOST_MSC
  uinfo("INFO: Initializing USB MSC class\n");

  ret = usbhost_msc_initialize();
  if (ret < 0)
    {
      uerr("ERROR: Failed to register mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDACM
  uinfo("INFO: Initializing CDCACM usb class\n");

  ret = usbhost_cdacm_initialize();
  if (ret < 0)
    {
      uerr("ERROR: Failed to register CDC/ACM serial class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  uinfo("INFO: Initializing HID Keyboard usb class\n");

  ret = usbhost_kbdinit();
  if (ret < 0)
    {
      uerr("ERROR: Failed to register the KBD class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  uinfo("INFO: Initializing HID Mouse usb class\n");

  ret = usbhost_mouse_init();
  if (ret < 0)
    {
      uerr("ERROR: Failed to register the mouse class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HUB
  uinfo("INFO: Initializing USB HUB class\n");

  ret = usbhost_hub_initialize()
  if (ret < 0)
    {
      uerr("ERROR: Failed to register hub class: %d\n", ret);
    }
#endif

#if defined(CONFIG_RNDIS)
  uint8_t mac[6];

  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;

  ret = usbdev_rndis_initialize(mac);
  if (ret < 0)
    {
      uerr("ERROR: Failed to register RNDIS class: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIEWTOOL_MAX3421E_RST
  /* Take the MAX3412E out of reset
   *
   * REVISIT:  The MAX3421E is not operational immediately after the reset.
   * The internal signal OPERATE indicates when the MAX3421E is fully out of
   * reset an operational.  The reset forces the OPERATE signal to be
   * visible on the GPX pin.  Hence, it many be necessary to poll the GPX
   * pin here to assure that the MAX3421E is operational before continuing.
   */

  stm32_gpiowrite(GPIO_MAX3421E_RST, true);
#endif

  /* Initialize and register the MAX3421E device */

  g_max3421e_lower.config.spi = spi;

  g_usbconn = max3421e_usbhost_initialize(&g_max3421e_lower.config);
  if (g_usbconn == NULL)
    {
      uerr("ERROR: Failed to register MAX3421E device\n");
      return -ENODEV;
    }

  /* Start the USB connection monitor kernel thread */

  ret = kthread_create("MAX3421E ConnMon",
                       CONFIG_VIEWTOOL_MAX3421E_CONNMON_PRIORITY,
                       CONFIG_VIEWTOOL_MAX3421E_CONNMON_STACKSIZE,
                       usbhost_detect, NULL);
  if (ret < 0)
    {
      uerr("ERROR: Failed to start connection monitor: %d\n", ret);
    }

  return OK;
}

#endif /* CONFIG_VIEWTOOL_MAX3421E_SPI1 || CONFIG_VIEWTOOL_MAX3421E_SPI2 */
