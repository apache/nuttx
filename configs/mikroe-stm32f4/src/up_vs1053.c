/****************************************************************************
 * configs/mikroe-stm32f4/src/up_vs1053.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include <stdint.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/spi.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/vs1053.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "mikroe-stm32f4-internal.h"

#ifdef CONFIG_VS1053

/****************************************************************************
 * Definitions
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
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(FAR const struct vs1053_lower_s *lower, xcpt_t handler);
static void up_enable(FAR const struct vs1053_lower_s *lower);
static void up_disable(FAR const struct vs1053_lower_s *lower);
static void up_reset(FAR const struct vs1053_lower_s *lower, bool state);

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
    .attach  = up_attach,
    .enable  = up_enable,
    .disable = up_disable,
    .reset   = up_reset  
  },
  .handler = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct vs1053_lower_s methods
 ****************************************************************************/

static int up_attach(FAR const struct vs1053_lower_s *lower, xcpt_t handler)
{
  FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void up_enable(FAR const struct vs1053_lower_s *lower)
{
  FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  (void)stm32_gpiosetevent(GPIO_VS1053_DREQ, false, true, true, priv->handler);
}

static void up_disable(FAR const struct vs1053_lower_s *lower)
{
  (void)stm32_gpiosetevent(GPIO_VS1053_DREQ, false, true, true, NULL);
}

static void up_reset(FAR const struct vs1053_lower_s *lower, bool state)
{
  stm32_gpiowrite(GPIO_VS1053_RST, state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 ****************************************************************************/

void up_vs1053initialize(FAR struct spi_dev_s* spi)
{
  int   ret;
  int   x;
  char  name[8];
  FAR struct audio_lowerhalf_s *pVs1053;

  /* Assumptions:
   * 1) SPI pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in boot-up.
   */

  /* Take VS1053 out of reset (active low)*/

  (void)stm32_configgpio(GPIO_VS1053_RST); 
  (void)stm32_configgpio(GPIO_VS1053_DREQ); 

  stm32_gpiowrite(GPIO_VS1053_RST, 0);
  for (x = 0; x < 10000; x++);
  stm32_gpiowrite(GPIO_VS1053_RST, 1);

  /* Bind the SPI port to the VS1053 driver */

  pVs1053 = vs1053_initialize(spi, &g_vs1053lower.lower, VS1053_DEVNO);
  if (ret < 0)
    {
      audlldbg("Failed to bind SPI port %d VS1053 device: %d\n",
             VS1053_DEVNO, ret);
      return;
    }

  /* Now register the audio device */

  sprintf(name, "mp3%d", VS1053_DEVNO);
  ret = audio_register(name, pVs1053);
  if (ret < 0)
    {
      auddbg("up_vs1053initialize: Failed to register VS1053 Audio device\n");
    }
  
  audllvdbg("Bound SPI port to VS1053 device %s\n", name);
}

#endif /* CONFIG_VS1053 */
