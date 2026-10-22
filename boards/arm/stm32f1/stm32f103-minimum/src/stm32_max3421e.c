/****************************************************************************
 * boards/arm/stm32f1/stm32f103-minimum/src/stm32_max3421e.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/max3421e.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "stm32f103_minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX3421E_SPIBUS   1

/* SPI command byte:  register number in bits 7-3, bit 1 = write */

#define MAX3421E_DIR_WRITE 0x02

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_max3421elower_s
{
  /* Standard MAX3421E interface */

  struct max3421e_lowerhalf_s config;

  /* Extensions for the stm32f103-minimum board */

  xcpt_t handler;
  void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int max3421e_attach(const struct max3421e_lowerhalf_s *lower,
                           xcpt_t isr, void *arg);
static void max3421e_enable(const struct max3421e_lowerhalf_s *lower,
                            bool enable);
static void max3421e_acknowledge(const struct max3421e_lowerhalf_s *lower);
static void max3421e_power(const struct max3421e_lowerhalf_s *lower,
                           bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_max3421elower_s g_max3421e_lower =
{
  .config =
  {
    .spi         = NULL,
    .frequency   = CONFIG_STM32F103MINIMUM_MAX3421E_FREQUENCY,
    .mode        = SPIDEV_MODE0,
    .devid       = 0,
    .intconfig   = 0,                /* Push-pull, falling edge-sensitive */

    .attach      = max3421e_attach,
    .enable      = max3421e_enable,
    .acknowledge = max3421e_acknowledge,
    .power       = max3421e_power,
  },
};

static struct usbhost_connection_s *g_usbconn;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int max3421e_attach(const struct max3421e_lowerhalf_s *lower,
                           xcpt_t isr, void *arg)
{
  struct stm32_max3421elower_s *priv =
    (struct stm32_max3421elower_s *)lower;

  if (isr != NULL)
    {
      /* Save the handler; it is attached when the interrupt is enabled */

      priv->handler = isr;
      priv->arg     = arg;
    }
  else
    {
      max3421e_enable(lower, false);
      priv->handler = NULL;
      priv->arg     = NULL;
    }

  return OK;
}

static void max3421e_enable(const struct max3421e_lowerhalf_s *lower,
                            bool enable)
{
  struct stm32_max3421elower_s *priv =
    (struct stm32_max3421elower_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable && priv->handler != NULL)
    {
      /* Interrupt on the falling edge of INT */

      stm32_gpiosetevent(GPIO_MAX3421E_INT, false, true, true,
                         priv->handler, priv->arg);

      /* The EXTI is disabled while the driver processes an interrupt, so a
       * falling edge in that window is lost and INT just stays low.  If INT
       * is already asserted, dispatch the handler now.
       */

      if (!stm32_gpioread(GPIO_MAX3421E_INT))
        {
          priv->handler(0, NULL, priv->arg);
        }
    }
  else
    {
      stm32_gpiosetevent(GPIO_MAX3421E_INT, false, false, false,
                         NULL, NULL);
    }

  leave_critical_section(flags);
}

static void max3421e_acknowledge(const struct max3421e_lowerhalf_s *lower)
{
  /* Nothing to do:  the EXTI pending bit is cleared by the STM32 logic */
}

static void max3421e_power(const struct max3421e_lowerhalf_s *lower,
                           bool enable)
{
  /* VBUS is wired directly to 5V; there is no power switch */
}

/****************************************************************************
 * Name: max3421e_probe_xfer
 *
 * Description:
 *   Perform one 2-byte register access and return the second byte read.
 *
 ****************************************************************************/

static uint8_t max3421e_probe_xfer(struct spi_dev_s *spi, uint8_t cmd,
                                   uint8_t data)
{
  uint8_t txbuf[2];
  uint8_t rxbuf[2];

  txbuf[0] = cmd;
  txbuf[1] = data;

  SPI_SELECT(spi, SPIDEV_USBHOST(0), true);
  SPI_EXCHANGE(spi, txbuf, rxbuf, 2);
  SPI_SELECT(spi, SPIDEV_USBHOST(0), false);

  return rxbuf[1];
}

/****************************************************************************
 * Name: max3421e_probe
 *
 * Description:
 *   Check that the MAX3421E answers on the SPI bus before handing it to the
 *   USB host driver (the driver waits forever for the oscillator if the
 *   chip is missing).
 *
 ****************************************************************************/

static int max3421e_probe(struct spi_dev_s *spi)
{
  uint8_t revision;
  uint8_t usbirq;
  uint8_t rdback1;
  uint8_t rdback2;
  int i;

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_STM32F103MINIMUM_MAX3421E_FREQUENCY);

  /* Switch to full-duplex SPI so that the chip drives MISO */

  max3421e_probe_xfer(spi, MAX3421E_USBHOST_PINCTL | MAX3421E_DIR_WRITE,
                      USBHOST_PINCTL_FDUPSPI);

  /* Wait up to 100 ms for the 12MHz oscillator to be stable.  Register
   * writes are clocked by the oscillator and are lost until it runs.
   */

  for (i = 0; i < 100; i++)
    {
      usbirq = max3421e_probe_xfer(spi, MAX3421E_USBHOST_USBIRQ, 0);
      if ((usbirq & USBHOST_USBIRQ_OSCOKIRQ) != 0)
        {
          break;
        }

      nxsig_usleep(1000);
    }

  revision = max3421e_probe_xfer(spi, MAX3421E_USBHOST_REVISION, 0);

  /* Write/read-back test using the GPINPOL register (it exists in both
   * peripheral and host modes; the chip is in peripheral mode after reset)
   */

  max3421e_probe_xfer(spi, MAX3421E_USBHOST_GPINPOL | MAX3421E_DIR_WRITE,
                      0x55);
  rdback1 = max3421e_probe_xfer(spi, MAX3421E_USBHOST_GPINPOL, 0);
  max3421e_probe_xfer(spi, MAX3421E_USBHOST_GPINPOL | MAX3421E_DIR_WRITE,
                      0xaa);
  rdback2 = max3421e_probe_xfer(spi, MAX3421E_USBHOST_GPINPOL, 0);
  max3421e_probe_xfer(spi, MAX3421E_USBHOST_GPINPOL | MAX3421E_DIR_WRITE,
                      0);

  SPI_LOCK(spi, false);

  syslog(LOG_INFO, "MAX3421E: REVISION=0x%02x GPINPOL rd=0x%02x/0x%02x "
         "USBIRQ=0x%02x\n", revision, rdback1, rdback2, usbirq);

  if (revision != USBHOST_REVISION)
    {
      syslog(LOG_ERR, "MAX3421E: not responding (bad revision)\n");
      return -ENODEV;
    }

  if (rdback1 != 0x55 || rdback2 != 0xaa)
    {
      syslog(LOG_ERR, "MAX3421E: register read-back failed\n");
      return -EIO;
    }

  if ((usbirq & USBHOST_USBIRQ_OSCOKIRQ) == 0)
    {
      syslog(LOG_ERR, "MAX3421E: oscillator not running\n");
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: usbhost_detect
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

static int usbhost_detect(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

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
 *   Initialize the MAX3421E USB host controller on SPI1 and start the
 *   USB connection monitor thread.
 *
 ****************************************************************************/

int stm32_max3421e_setup(void)
{
  struct spi_dev_s *spi;
  int ret;

  /* INT is an input, RES# starts low (chip held in reset) */

  stm32_configgpio(GPIO_MAX3421E_INT);
#ifdef CONFIG_STM32F103MINIMUM_MAX3421E_RST
  stm32_configgpio(GPIO_MAX3421E_RST);
#endif

  spi = stm32_spibus_initialize(MAX3421E_SPIBUS);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI%d\n",
             MAX3421E_SPIBUS);
      return -ENODEV;
    }

  /* Pulse reset and give the chip time to come out of it */

  nxsig_usleep(10000);
#ifdef CONFIG_STM32F103MINIMUM_MAX3421E_RST
  stm32_gpiowrite(GPIO_MAX3421E_RST, true);
#endif
  nxsig_usleep(10000);

  ret = max3421e_probe(spi);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_USBHOST_MSC
  ret = usbhost_msc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MSC class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  ret = usbhost_kbdinit();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register KBD class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  ret = usbhost_mouse_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register mouse class: %d\n", ret);
    }
#endif

  g_max3421e_lower.config.spi = spi;

  g_usbconn = max3421e_usbhost_initialize(&g_max3421e_lower.config);
  if (g_usbconn == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MAX3421E USB host\n");
      return -ENODEV;
    }

  ret = kthread_create("max3421e_conn",
                       CONFIG_STM32F103MINIMUM_MAX3421E_CONNMON_PRIORITY,
                       CONFIG_STM32F103MINIMUM_MAX3421E_CONNMON_STACKSIZE,
                       usbhost_detect, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start connection monitor: %d\n",
             ret);
      return ret;
    }

  syslog(LOG_INFO, "MAX3421E: USB host initialized\n");
  return OK;
}
