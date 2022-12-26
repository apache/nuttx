/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_hts221.c
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

static int nrf52_hts221_irq_attach(struct hts221_config_s *state,
                                   xcpt_t isr, void *arg);
static void nrf52_hts221_irq_enable(const struct hts221_config_s *state,
                                    bool enable);
static void nrf52_hts221_irq_clear(const struct hts221_config_s *state);
static int nrf52_hts221_set_power(const struct hts221_config_s *state,
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

static int nrf52_hts221_irq_attach(struct hts221_config_s *state,
                                   xcpt_t isr, void *arg)
{
  sinfo("Attach HTS221 IRQ\n");

  /* TODO: IRQ on rising edge */

  /* nrf52_gpiosetevent(GPIO_HTS221_IRQ, true, false, false, isr, arg); */

  return OK;
}

/****************************************************************************
 * Name: nrf52_hts221_irq_enable
 ****************************************************************************/

static void nrf52_hts221_irq_enable(const struct hts221_config_s *state,
                                    bool enable)
{
}

/****************************************************************************
 * Name: nrf52_hts221_irq_clear
 ****************************************************************************/

static void nrf52_hts221_irq_clear(const struct hts221_config_s *state)
{
}

/****************************************************************************
 * Name: nrf52_hts221_set_power
 ****************************************************************************/

static int nrf52_hts221_set_power(const struct hts221_config_s *state,
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
  struct i2c_master_s *i2c;
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
