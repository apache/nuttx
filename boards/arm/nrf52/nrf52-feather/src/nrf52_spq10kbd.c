/****************************************************************************
 * boards/arm/nrf52/nrf52-feather/src/nrf52_q10kbd.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/spq10kbd.h>
#include <nuttx/board.h>

#include "nrf52_i2c.h"
#include "nrf52-feather.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf_spq10kbd_intrstate_s
{
  xcpt_t isr;
  void *cbarg;
  bool enabled;
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static int  nrf52_spq10kbd_attach(FAR const struct spq10kbd_config_s *config,
              xcpt_t isr, FAR void *arg);
static void nrf52_spq10kbd_enable(FAR const struct spq10kbd_config_s *config,
              bool enable);
static void nrf52_spq10kbd_clear(FAR const struct spq10kbd_config_s *config);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct spq10kbd_config_s g_q10kbd_config =
{
  .address =   0x1f,
  .frequency = 400000,
  .attach =    nrf52_spq10kbd_attach,
  .enable =    nrf52_spq10kbd_enable,
  .clear =     nrf52_spq10kbd_clear,
};

struct nrf_spq10kbd_intrstate_s nrf_spq10kbd_intrstate =
{
  .isr = NULL,
  .enabled = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int  nrf52_spq10kbd_attach(FAR const struct spq10kbd_config_s *config,
              xcpt_t isr, FAR void *arg)
{
  nrf_spq10kbd_intrstate.isr = isr;
  nrf_spq10kbd_intrstate.cbarg = arg;
  return OK;
}

static void nrf52_spq10kbd_enable(FAR const struct spq10kbd_config_s *config,
              bool enable)
{
  nrf_spq10kbd_intrstate.enabled = enable;
}

static void nrf52_spq10kbd_clear(FAR const struct spq10kbd_config_s *config)
{
  /* Nothing to do here */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kb_int
 *
 * Description:
 *   This function is called a key press is detected on the keyboard
 *
 * Input Parameters:
 *   pin - Interrupt GPIO pin (not used)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kb_int(int pin)
{
  if (nrf_spq10kbd_intrstate.enabled)
    {
      /* We are faking this to look like an isr callback, so irq is 0 and
       * context is NULL. The driver does not care about these.
       */

      nrf_spq10kbd_intrstate.isr(0, NULL, nrf_spq10kbd_intrstate.cbarg);
    }
}

/****************************************************************************
 * Name: q10kbd_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   Q10 BlackBerry Keyboard
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int q10kbd_setup(int bus)
{
  FAR struct i2c_master_s *i2c;

  iinfo("Initializing\n");
  i2c = nrf52_i2cbus_initialize(bus);

  if (i2c != NULL)
    {
      /* Register the Q10 KBD driver instance */

      spq10kbd_register(i2c, &g_q10kbd_config, 'a', "/dev/djoy0");
    }

  return OK;
}
