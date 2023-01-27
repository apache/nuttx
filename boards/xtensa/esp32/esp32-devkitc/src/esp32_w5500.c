/****************************************************************************
 * boards/xtensa/esp32/esp32-devkitc/src/esp32_w5500.c
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
#include <syslog.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/irq.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/w5500.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "esp32-devkitc.h"
#include "esp32_spi.h"
#include "esp32_gpio.h"
#include "hardware/esp32_gpio_sigmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* W5500 GPIO pins */

#define GPIO_W5500_INTR   17
#define GPIO_W5500_RESET  18

/* W5500 is on SPI1 */

#ifndef CONFIG_ESP32_SPI2
# error "Need CONFIG_ESP32_SPI2 in the configuration"
#endif

/* SPI Assumptions **********************************************************/

#define W5500_SPI_PORTNO 2   /* On SPI2 */
#define W5500_DEVNO      0   /* Only one W5500 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_lower_s
{
  const struct w5500_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                     handler;  /* W5500 interrupt handler */
  void                      *arg;      /* Argument that accompanies IRQ */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                      void *arg);
static void up_enable(const struct w5500_lower_s *lower, bool enable);
static void up_reset(const struct w5500_lower_s *lower, bool reset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The W5500 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanism for controlling
 * the W5500 GPIO interrupt.
 */

static struct esp32_lower_s g_enclower =
{
  .lower =
  {
    .frequency = 1000000,
    .spidevid  = 0,
    .mode      = SPIDEV_MODE0,
    .attach    = up_attach,
    .enable    = up_enable,
    .reset     = up_reset,
  },
  .handler = NULL,
  .arg     = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct w5500_lower_s methods
 ****************************************************************************/

static int up_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                     void *arg)
{
  struct esp32_lower_s *priv = (struct esp32_lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void up_enable(const struct w5500_lower_s *lower, bool enable)
{
  struct esp32_lower_s *priv = (struct esp32_lower_s *)lower;

  int irq = ESP32_PIN2IRQ(GPIO_W5500_INTR);
  int ret;

  /* Make sure the interrupt is disabled */

  esp32_gpioirqdisable(irq);

  DEBUGASSERT(priv->handler);
  if (enable)
    {
      ret = irq_attach(irq, priv->handler, priv->arg);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
        }

      /* IRQ on rising edge */

      esp32_gpioirqenable(irq, RISING);
    }
  else
    {
      /* Just keep interrupt disabled is enough */
    }
}

/* REVISIT:  Since the interrupt is completely torn down, not just disabled,
 * in interrupt requests that occurs while the interrupt is disabled will be
 * lost.
 */

static void up_reset(const struct w5500_lower_s *lower, bool reset)
{
  /* Take W5500 out of reset (active low) */

  esp32_gpiowrite(GPIO_W5500_RESET, !reset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 ****************************************************************************/

void up_netinitialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  /* Configure the interrupt pin */

  esp32_configgpio(GPIO_W5500_INTR, INPUT_FUNCTION_1 | PULLDOWN);

  /* Configure the reset pin as output */

  esp32_gpio_matrix_out(GPIO_W5500_RESET, SIG_GPIO_OUT_IDX, 0, 0);
  esp32_configgpio(GPIO_W5500_RESET, OUTPUT_FUNCTION_1 |
                     INPUT_FUNCTION_1);

  /* Assumptions:
   * 1) W5500 pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in
   *    boot-up.
   */

  spi = esp32_spibus_initialize(W5500_SPI_PORTNO);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n", W5500_SPI_PORTNO);
      return;
    }

  /* Bind the SPI port to the W5500 driver */

  ret = w5500_initialize(spi, &g_enclower.lower, W5500_DEVNO);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI port %d W5500 device %d: %d\n",
           W5500_SPI_PORTNO, W5500_DEVNO, ret);
      return;
    }

  ninfo("Bound SPI port %d to W5500 device %d\n",
        W5500_SPI_PORTNO, W5500_DEVNO);
}

