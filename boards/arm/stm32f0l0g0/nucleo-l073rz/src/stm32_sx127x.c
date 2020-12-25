/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-l073rz/src/stm32_sx127x.c
 * This logic is specific for the RFM98 modules (433MHz)
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

#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <nuttx/wireless/lpwan/sx127x.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_spi.h"

#include "nucleo-l073rz.h"

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

  stm32_configgpio(GPIO_SX127X_RESET | GPIO_OUTPUT | GPIO_SPEED_HIGH |
                   GPIO_OUTPUT_CLEAR);

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
      wlerr("Module supports only PA_BOOST pin, "
            "so PA_SELECT must be enabled!\n");
    }

  return ret;
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
