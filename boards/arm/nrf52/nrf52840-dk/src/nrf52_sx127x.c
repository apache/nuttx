/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_sx127x.c
 * This logic is specific for the RFM98 modules (433MHz)
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"
#include "nrf52_spi.h"

#include "nrf52840-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SX127X on SPI0 bus */

#define SX127X_SPI 0

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
  .pa_force      = true
};

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

  nrf52_gpiosetevent(GPIO_SX127X_DIO0, true, false, false, isr, arg);
  return OK;
}

/****************************************************************************
 * Name: sx127x_chip_reset
 ****************************************************************************/

static void sx127x_chip_reset(void)
{
  wlinfo("SX127X RESET\n");

  /* Configure reset as output */

  nrf52_gpio_config(GPIO_SX127X_RESET | GPIO_OUTPUT | GPIO_VALUE_ZERO);

  /* Set pin to zero */

  nrf52_gpio_write(GPIO_SX127X_RESET, false);

  /* Wait 1 ms */

  nxsig_usleep(1000);

  /* Configure reset as input */

  nrf52_gpio_config(GPIO_SX127X_RESET | GPIO_INPUT | GPIO_FLOAT);

  /* Wait 10 ms */

  nxsig_usleep(10000);
}

/****************************************************************************
 * Name: sx127x_opmode_change
 ****************************************************************************/

static int sx127x_opmode_change(int opmode)
{
  /* Do nothing */

  return OK;
}

/****************************************************************************
 * Name: sx127x_freq_select
 ****************************************************************************/

static int sx127x_freq_select(uint32_t freq)
{
  int ret = OK;

  /* NOTE: this depends on your module version */

  if (freq > SX127X_HFBAND_THR)
    {
      ret = -EINVAL;
      wlerr("HF band not supported\n");
    }

  return ret;
}

/****************************************************************************
 * Name: sx127x_pa_select
 ****************************************************************************/

static int sx127x_pa_select(bool enable)
{
  int ret = OK;

  /* Only PA_BOOST output connected to antenna */

  if (enable == false)
    {
      ret = -EINVAL;
      wlerr("Module supports only PA_BOOST pin,"
            " so PA_SELECT must be enabled!\n");
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nrf52_lpwaninitialize(void)
{
  FAR struct spi_dev_s *spidev;
  int ret = OK;

  wlinfo("Register the sx127x module\n");

  /* Setup DIO0 */

  nrf52_gpio_config(GPIO_SX127X_DIO0);

  /* Init SPI bus */

  spidev = nrf52_spibus_initialize(SX127X_SPI);
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
