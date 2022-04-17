/****************************************************************************
 * boards/arm/stm32/mikroe-stm32f4/src/stm32_vs1053.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/vs1053.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "mikroe-stm32f4.h"

#ifdef CONFIG_AUDIO_VS1053

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VS1053 is on SPI3 */

#ifndef CONFIG_STM32_SPI3
# error "Need CONFIG_STM32_SPI3 in the configuration"
#endif

/* SPI Assumptions **********************************************************/

#define VS1053_SPI_PORTNO   3   /* On SPI3 */
#define VS1053_DEVNO        0   /* Only one VS1053 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lower_s
{
  const struct vs1053_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                      handler;  /* VS1053 interrupt handler */
  void                       *arg;      /* Interrupt handler argument */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(const struct vs1053_lower_s *lower, xcpt_t handler,
                      void *arg);
static void up_enable(const struct vs1053_lower_s *lower);
static void up_disable(const struct vs1053_lower_s *lower);
static void up_reset(const struct vs1053_lower_s *lower, bool state);
static int  up_read_dreq(const struct vs1053_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The VS1053 provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the VS1053 GPIO interrupt.
 */

static struct stm32_lower_s g_vs1053lower =
{
  .lower =
  {
    .attach     = up_attach,
    .enable     = up_enable,
    .disable    = up_disable,
    .reset      = up_reset,
    .read_dreq  = up_read_dreq,
    .irq        = GPIO_VS1053_DREQ_IRQ
  },
  .handler      = NULL,
  .arg          = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct vs1053_lower_s methods
 ****************************************************************************/

static int up_attach(const struct vs1053_lower_s *lower, xcpt_t handler,
                     void *arg)
{
  struct stm32_lower_s *priv = (struct stm32_lower_s *)lower;

  priv->handler = handler;    /* Save the handler for later */
  priv->arg     = arg;        /* Along with the handler argument */
  return 0;
}

static void up_enable(const struct vs1053_lower_s *lower)
{
  struct stm32_lower_s *priv = (struct stm32_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  stm32_gpiosetevent(GPIO_VS1053_DREQ, true, false, false,
                     priv->handler, priv->arg);
}

static void up_disable(const struct vs1053_lower_s *lower)
{
  stm32_gpiosetevent(GPIO_VS1053_DREQ, false, false, false, NULL, NULL);
}

static void up_reset(const struct vs1053_lower_s *lower, bool state)
{
  stm32_gpiowrite(GPIO_VS1053_RST, state);
}

static int up_read_dreq(const struct vs1053_lower_s *lower)
{
  return stm32_gpioread(GPIO_VS1053_DREQ);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_vs1053initialize
 ****************************************************************************/

void up_vs1053initialize(struct spi_dev_s * spi)
{
  int   ret;
  char  name[16];
  struct audio_lowerhalf_s *PVS1053;

  /* Assumptions:
   * 1) SPI pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI3 peripheral was also provided earlier in
   * boot-up.
   */

  /* NOTE:  The RST line should be asserted early in the boot process
   *        during the boardinitialize function because the VS1053
   *        generates a low frequency humming noise from power-on reset
   *        until the RST line is asserted.
   */

  /* stm32_configgpio(GPIO_VS1053_RST); */

  /* Initialize the VS1053 DREQ GPIO line */

  stm32_configgpio(GPIO_VS1053_DREQ);

  /* Bind the SPI port to the VS1053 driver */

  PVS1053 = vs1053_initialize(spi, &g_vs1053lower.lower, VS1053_DEVNO);
  if (PVS1053 == NULL)
    {
      auderr("ERROR: Failed to bind SPI port %d VS1053 device\n",
             VS1053_DEVNO);
      return;
    }

  /* Now register the audio device */

  snprintf(name, sizeof(name), "vs1053d%d", VS1053_DEVNO);
  ret = audio_register(name, PVS1053);
  if (ret < 0)
    {
      auderr("ERROR: Failed to register VS1053 Audio device\n");
    }

  audinfo("Bound SPI port to VS1053 device %s\n", name);
}

#endif /* CONFIG_AUDIO_VS1053 */
