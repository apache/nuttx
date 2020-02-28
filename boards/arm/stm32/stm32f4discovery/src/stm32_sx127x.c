/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_boot.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Mateusz Szafoni <raiden00@railab.me>
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

#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <nuttx/wireless/lpwan/sx127x.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_spi.h"

#include "stm32f4discovery.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SX127X on SPI1 bus */

#define SX127X_SPI 1

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sx127x_chip_reset(void);
static int sx127x_opmode_change(int opmode);
static int sx127x_freq_select(uint32_t freq);
static int sx127x_pa_select(bool enable);
static int sx127x_irq0_attach(xcpt_t isr, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sx127x_lower_s lower =
{
  .irq0attach    = sx127x_irq0_attach,
  .reset         = sx127x_chip_reset,
  .opmode_change = sx127x_opmode_change,
  .freq_select   = sx127x_freq_select,
  .pa_select     = sx127x_pa_select,
  .pa_force      = false
};

static bool g_high_power_output = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx127x_irq0_attach
 ****************************************************************************/

static int sx127x_irq0_attach(xcpt_t isr, FAR void *arg)
{
  wlinfo("Attach DIO0 IRQ\n");

  /* IRQ on rising edge */

  stm32_gpiosetevent(GPIO_SX127X_DIO0, true, false, false, isr, arg);
  return OK;
}

/****************************************************************************
 * Name: sx127x_chip_reset
 ****************************************************************************/

static void sx127x_chip_reset(void)
{
  wlinfo("SX127X RESET\n");

  /* Configure reset as output */

  stm32_configgpio(GPIO_SX127X_RESET);

  /* Set pin to zero */

  stm32_gpiowrite(GPIO_SX127X_RESET, false);

  /* Wait 1 ms */

  nxsig_usleep(1000);

  /* Configure reset as input */

  stm32_configgpio(GPIO_SX127X_RESET | GPIO_INPUT | GPIO_FLOAT);

  /* Wait 10 ms */

  nxsig_usleep(10000);
}

/****************************************************************************
 * Name: sx127x_opmode_change
 ****************************************************************************/

static int sx127x_opmode_change(int opmode)
{
  /* Nothing to do */

  return OK;
}

/****************************************************************************
 * Name: sx127x_freq_select
 ****************************************************************************/

static int sx127x_freq_select(uint32_t freq)
{
  int ret = OK;

  /* Only HF supported (BAND3 - 860-930 MHz) */

  if (freq < SX127X_HFBAND_THR)
    {
      ret = -EINVAL;
      wlerr("LF band not supported\n");
    }

  return ret;
}

/****************************************************************************
 * Name: sx127x_pa_select
 ****************************************************************************/

static int sx127x_pa_select(bool enable)
{
  g_high_power_output = enable;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_lpwaninitialize(void)
{
  FAR struct spi_dev_s *spidev;
  int ret = OK;

  wlinfo("Register the sx127x module\n");

  /* Setup DIO0 */

  stm32_configgpio(GPIO_SX127X_DIO0);

  /* Init SPI bus */

  spidev = stm32_spibus_initialize(SX127X_SPI);
  if (!spidev)
    {
      wlerr("ERROR: Failed to initialize SPI %d bus\n", SX127X_SPI);
      ret = -ENODEV;
      goto errout;
    }

  /* Initialize SX127X */

  ret = sx127x_register(spidev, &lower);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to register sx127x\n");
      goto errout;
    }

errout:
  return ret;
}
