/****************************************************************************
 * boards/arm/stm32f0l0g0/b-l072z-lrwan1/src/stm32_boot.c
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
#include <arch/board/board.h>
#include <nuttx/wireless/lpwan/sx127x.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_spi.h"

#include "b-l072z-lrwan1.h"

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

  (void)stm32_gpiosetevent(GPIO_SX127X_DIO0, true, false, false, isr, arg);
  return OK;
}

/****************************************************************************
 * Name: sx127x_chip_reset
 ****************************************************************************/

static void sx127x_chip_reset(void)
{
  wlinfo("SX127X RESET\n");

  /* Configure reset as output */

  stm32_configgpio(GPIO_SX127X_RESET | GPIO_OUTPUT | GPIO_SPEED_HIGH |
                   GPIO_OUTPUT_CLEAR);

  /* Set pin to zero */

  stm32_gpiowrite(GPIO_SX127X_RESET, false);

  /* Wait 1 ms */

  usleep(1000);

  /* Configure reset as input */

  stm32_configgpio(GPIO_SX127X_RESET | GPIO_INPUT | GPIO_FLOAT);

  /* Wait 10 ms */

  usleep(10000);
}

/****************************************************************************
 * Name: sx127x_opmode_change
 ****************************************************************************/

static int sx127x_opmode_change(int opmode)
{
  int ret = OK;

  /* Configure antena switch outputs in SLEEP mode */

  if (opmode == SX127X_OPMODE_SLEEP)
    {
      stm32_gpiowrite(GPIO_SX127X_CRF1, false);
      stm32_gpiowrite(GPIO_SX127X_CRF2, false);
      stm32_gpiowrite(GPIO_SX127X_CRF3, false);

      stm32_configgpio(GPIO_SX127X_CRF1 | GPIO_ANALOG);
      stm32_configgpio(GPIO_SX127X_CRF2 | GPIO_ANALOG);
      stm32_configgpio(GPIO_SX127X_CRF3 | GPIO_ANALOG);

      goto errout;
    }

  /* Configure antena switch outputs */

  stm32_configgpio(GPIO_SX127X_CRF1 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR);
  stm32_configgpio(GPIO_SX127X_CRF2 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR);
  stm32_configgpio(GPIO_SX127X_CRF3 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR);

  stm32_gpiowrite(GPIO_SX127X_CRF1, false);
  stm32_gpiowrite(GPIO_SX127X_CRF2, false);
  stm32_gpiowrite(GPIO_SX127X_CRF3, false);

  switch (opmode)
    {
      case SX127X_OPMODE_STANDBY:
      case SX127X_OPMODE_FSRX:
      case SX127X_OPMODE_FSTX:
        {
          break;
        }

      case SX127X_OPMODE_TX:
        {
          /* Set TX RFO or TX BOOST */

          if (g_high_power_output == true)
            {
              wlinfo("SET CRF3\n");
              stm32_gpiowrite(GPIO_SX127X_CRF3, true);
            }
          else
            {
              wlinfo("SET CRF2\n");
              stm32_gpiowrite(GPIO_SX127X_CRF2, true);
            }

          break;
        }

      case SX127X_OPMODE_RX:
      case SX127X_OPMODE_RXSINGLE:
      case SX127X_OPMODE_CAD:
        {
          /* Set antena RX */

          wlinfo("SET CRF1\n");
          stm32_gpiowrite(GPIO_SX127X_CRF1, true);

          break;
        }

      default:
        {
          wlerr("ERROR: invalid mode %d\n", opmode);
          ret = -EINVAL;
          break;
        }
    }

errout:
  return ret;
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
