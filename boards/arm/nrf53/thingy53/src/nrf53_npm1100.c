/****************************************************************************
 * boards/arm/nrf53/thingy53/src/nrf53_npm1100.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <debug.h>

#include <nuttx/power/battery_charger.h>
#include <nuttx/power/npm1100.h>

#include "nrf53_gpio.h"
#include "thingy53.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The nPM1100 CHG and ERR status pins are open-drain, active-low. Configure
 * them as inputs with a pull-up and report the raw pin level; the nPM1100
 * driver applies the active-low semantics.
 */

static bool nrf53_npm1100_read_chg(void)
{
  return nrf53_gpio_read(GPIO_NPM1100_CHG);
}

static bool nrf53_npm1100_read_err(void)
{
  return nrf53_gpio_read(GPIO_NPM1100_ERR);
}

static const struct npm1100_config_s g_npm1100_config =
{
  .read_chg = nrf53_npm1100_read_chg,
  .read_err = nrf53_npm1100_read_err,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_npm1100_init
 *
 * Description:
 *   Register the nPM1100 PMIC charge-status driver as /dev/charge0.
 *
 ****************************************************************************/

int nrf53_npm1100_init(void)
{
  FAR struct battery_charger_dev_s *charger;
  int ret;

  nrf53_gpio_config(GPIO_NPM1100_CHG | GPIO_PULLUP);
  nrf53_gpio_config(GPIO_NPM1100_ERR | GPIO_PULLUP);

  charger = npm1100_initialize(&g_npm1100_config);
  if (charger == NULL)
    {
      return -ENODEV;
    }

  ret = battery_charger_register("/dev/charge0", charger);
  if (ret < 0)
    {
      baterr("ERROR: battery_charger_register failed: %d\n", ret);
    }

  return ret;
}
