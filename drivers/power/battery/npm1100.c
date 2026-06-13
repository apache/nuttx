/****************************************************************************
 * drivers/power/battery/npm1100.c
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
 * The nPM1100 is a pin-strapped charger PMIC with no register interface.
 * It exposes only charge (CHG) and error (ERR) status on two open-drain,
 * active-low GPIOs; charge current is set by the ISET resistor in hardware.
 * This driver maps those two pins onto the standard battery_charger
 * lower-half (state / health / online). Charge-current and voltage controls
 * are not software-controllable and report -ENOSYS.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/npm1100.h>

#if defined(CONFIG_BATTERY_CHARGER) && defined(CONFIG_NPM1100)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct npm1100_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev;

  /* nPM1100 board configuration (status GPIO read callbacks) */

  FAR const struct npm1100_config_s *config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int npm1100_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status);
static int npm1100_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health);
static int npm1100_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status);
static int npm1100_voltage(FAR struct battery_charger_dev_s *dev, int value);
static int npm1100_current(FAR struct battery_charger_dev_s *dev, int value);
static int npm1100_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value);
static int npm1100_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_npm1100ops =
{
  npm1100_state,
  npm1100_health,
  npm1100_online,
  npm1100_voltage,
  npm1100_current,
  npm1100_input_current,
  npm1100_operate
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: npm1100_state
 ****************************************************************************/

static int npm1100_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status)
{
  FAR struct npm1100_dev_s *priv = (FAR struct npm1100_dev_s *)dev;

  /* CHG and ERR are active-low: a low pin level means the signal is
   * asserted. The nPM1100 cannot distinguish "charge complete" from "no
   * input" on these pins, so a de-asserted CHG with no error is reported as
   * idle.
   */

  if (!priv->config->read_err())
    {
      *status = BATTERY_FAULT;
    }
  else if (!priv->config->read_chg())
    {
      *status = BATTERY_CHARGING;
    }
  else
    {
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: npm1100_health
 ****************************************************************************/

static int npm1100_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health)
{
  FAR struct npm1100_dev_s *priv = (FAR struct npm1100_dev_s *)dev;

  if (!priv->config->read_err())
    {
      *health = BATTERY_HEALTH_UNSPEC_FAIL;
    }
  else
    {
      *health = BATTERY_HEALTH_GOOD;
    }

  return OK;
}

/****************************************************************************
 * Name: npm1100_online
 ****************************************************************************/

static int npm1100_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status)
{
  *status = true;
  return OK;
}

/****************************************************************************
 * Name: npm1100_voltage
 *
 * Description:
 *   Charge voltage is fixed by the nPM1100 hardware; not controllable.
 *
 ****************************************************************************/

static int npm1100_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: npm1100_current
 *
 * Description:
 *   Charge current is set by the ISET resistor; not software controllable.
 *
 ****************************************************************************/

static int npm1100_current(FAR struct battery_charger_dev_s *dev, int value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: npm1100_input_current
 ****************************************************************************/

static int npm1100_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: npm1100_operate
 ****************************************************************************/

static int npm1100_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: npm1100_initialize
 ****************************************************************************/

FAR struct battery_charger_dev_s *
  npm1100_initialize(FAR const struct npm1100_config_s *config)
{
  FAR struct npm1100_dev_s *priv;

  DEBUGASSERT(config != NULL && config->read_chg != NULL &&
              config->read_err != NULL);

  priv = (FAR struct npm1100_dev_s *)
    kmm_zalloc(sizeof(struct npm1100_dev_s));

  if (priv)
    {
      priv->dev.ops = &g_npm1100ops;
      priv->config  = config;
    }

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_CHARGER && CONFIG_NPM1100 */
