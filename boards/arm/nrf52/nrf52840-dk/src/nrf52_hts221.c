/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_hts221.c
 *
 *   Copyright (C) 2020 Greg Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include "nrf52_i2c.h"
#include "nrf52840-dk.h"
#include <nuttx/sensors/hts221.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF52_I2C0_MASTER
#  error "HTS221 driver requires CONFIG_NRF52_I2C0_MASTER to be enabled"
#endif

/* HTS221 I2C address */

#define HTS221HUM_ADDR (0xbe >> 1) /* 7-bit */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf52_hts221_irq_attach(FAR struct hts221_config_s *state,
                                   xcpt_t isr, FAR void *arg);
static void nrf52_hts221_irq_enable(FAR const struct hts221_config_s *state,
                                    bool enable);
static void nrf52_hts221_irq_clear(FAR const struct hts221_config_s *state);
static int nrf52_hts221_set_power(FAR const struct hts221_config_s *state,
                                  bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static hts221_config_t g_hts221_config =
{
  .irq_attach = nrf52_hts221_irq_attach,
  .irq_enable = nrf52_hts221_irq_enable,
  .irq_clear  = nrf52_hts221_irq_clear,
  .set_power  = nrf52_hts221_set_power
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_hts221_irq_attach
 ****************************************************************************/

static int nrf52_hts221_irq_attach(FAR struct hts221_config_s *state,
                                   xcpt_t isr, FAR void *arg)
{
  sinfo("Attach HTS221 IRQ\n");

  /* TODO: IRQ on rising edge */

  /* nrf52_gpiosetevent(GPIO_HTS221_IRQ, true, false, false, isr, arg); */

  return OK;
}

/****************************************************************************
 * Name: nrf52_hts221_irq_enable
 ****************************************************************************/

static void nrf52_hts221_irq_enable(FAR const struct hts221_config_s *state,
                                   bool enable)
{
  return;
}

/****************************************************************************
 * Name: nrf52_hts221_irq_clear
 ****************************************************************************/

static void nrf52_hts221_irq_clear(FAR const struct hts221_config_s *state)
{
  return;
}

/****************************************************************************
 * Name: nrf52_hts221_set_power
 ****************************************************************************/

static int nrf52_hts221_set_power(FAR const struct hts221_config_s *state,
                                  bool on)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_hts221_initialize
 *
 * Description:
 *   Initialize I2C-based HTS221.
 *
 ****************************************************************************/

int nrf52_hts221_initialize(char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("Initializing HTS221!\n");

#ifdef CONFIG_NRF52_I2C0_MASTER
  i2c = nrf52_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sninfo("INFO: Initializing HTS221 hum-temp sensor over I2C%d\n", ret);

  ret = hts221_register(devpath, i2c, HTS221HUM_ADDR, &g_hts221_config);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize HTS221 hum-temp driver %s\n",
            devpath);
      return -ENODEV;
    }

  sninfo("INFO: HTS221 sensor has been initialized successfully\n");
#endif

  return ret;
}
