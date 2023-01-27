/****************************************************************************
 * boards/xtensa/esp32/lilygo_tbeam_lora_gps/src/esp32_sx127x.c
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

#include "esp32_gpio.h"
#include "esp32_spi.h"
#include "hardware/esp32_gpio_sigmap.h"

#include "lilygo_tbeam_lora_gps.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SX127X on SPI2 bus */

#define SX127X_SPI 2

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sx127x_chip_reset(void);
static int sx127x_opmode_change(int opmode);
static int sx127x_freq_select(uint32_t freq);
static int sx127x_pa_select(bool enable);
static int sx127x_irq0_attach(xcpt_t isr, void *arg);

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

static int sx127x_irq0_attach(xcpt_t isr, void *arg)
{
  int irq = ESP32_PIN2IRQ(GPIO_SX127X_DIO0);
  int ret;

  /* Make sure the interrupt is disabled */

  esp32_gpioirqdisable(irq);

  wlinfo("Attach DIO0 IRQ\n");

  /* Attach to IRQ on pin connected to DIO0 */

  ret = irq_attach(irq, isr, arg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }

  /* IRQ on rising edge */

  esp32_gpioirqenable(irq, RISING);

  return OK;
}

/****************************************************************************
 * Name: sx127x_chip_reset
 ****************************************************************************/

static void sx127x_chip_reset(void)
{
  wlinfo("SX127X RESET\n");

  /* Configure reset as output */

  esp32_gpio_matrix_out(GPIO_SX127X_RESET, SIG_GPIO_OUT_IDX, 0, 0);
  esp32_configgpio(GPIO_SX127X_RESET, OUTPUT_FUNCTION_3 | INPUT_FUNCTION_3);

  /* Set pin to zero */

  esp32_gpiowrite(GPIO_SX127X_RESET, 0);

  /* Wait 1 ms */

  nxsig_usleep(1000);

  /* Set pin to high */

  esp32_gpiowrite(GPIO_SX127X_RESET, 1);

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

int esp32_lpwaninitialize(void)
{
  struct spi_dev_s *spidev;
  int ret = OK;

  wlinfo("Register the sx127x module\n");

  /* Setup DIO0 */

  esp32_configgpio(GPIO_SX127X_DIO0, INPUT_FUNCTION_3 | PULLDOWN);

  /* Init SPI bus */

  spidev = esp32_spibus_initialize(SX127X_SPI);
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
