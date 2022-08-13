/****************************************************************************
 * drivers/power/battery/mcp73871.c
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

/* Lower half driver for Microchip MCP73871 battery charger
 *
 * The MCP73871 is a standalone battery charger, so this driver is not
 * supposed to do much things, it will only report faults and battery status.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/mcp73871.h>

/* This driver requires:
 *
 * CONFIG_BATTERY_CHARGER - Upper half battery driver support
 */

#if defined(CONFIG_BATTERY_CHARGER)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

struct mcp73871_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev; /* Battery charger device */

  /* MCP73871 configuration helpers */

  FAR struct mcp73871_config_s *config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Battery driver lower half methods */

static int mcp73871_state(FAR struct battery_charger_dev_s *dev,
             FAR int *status);
static int mcp73871_health(FAR struct battery_charger_dev_s *dev,
             FAR int *health);
static int mcp73871_online(FAR struct battery_charger_dev_s *dev,
             FAR bool *status);
static int mcp73871_voltage(FAR struct battery_charger_dev_s *dev,
             int value);
static int mcp73871_current(FAR struct battery_charger_dev_s *dev,
             int value);
static int mcp73871_input_current(FAR struct battery_charger_dev_s *dev,
             int value);
static int mcp73871_operate(FAR struct battery_charger_dev_s *dev,
             intptr_t param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_mcp73871ops =
{
  mcp73871_state,
  mcp73871_health,
  mcp73871_online,
  mcp73871_voltage,
  mcp73871_current,
  mcp73871_input_current,
  mcp73871_operate
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp73871_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int mcp73871_state(FAR struct battery_charger_dev_s *dev,
                          FAR int *status)
{
  FAR struct mcp73871_dev_s *priv = (FAR struct mcp73871_dev_s *)dev;
  uint8_t value = 0;

  /* Create a value with bits 2 1 0 = STAT1 | STAT2 | PG */

  value = (priv->config->read_stat1() << 2) |
          (priv->config->read_stat2() << 1) |
          (priv->config->read_pg()    << 0);

  switch (value)
  {
  case MCP73871_FAULT:
    *status = BATTERY_FAULT;
    break;

  case MCP73871_CHARGING:
    *status = BATTERY_CHARGING;
    break;

  case MCP73871_BAT_LOW:
    *status = BATTERY_DISCHARGING;
    break;

  case MCP73871_CHG_COMPLETE:
    *status = BATTERY_FULL;
    break;

  case MCP73871_NO_BATTERY:
    *status = BATTERY_FAULT;
    break;

  case MCP73871_NO_INPUT_PWR:
    *status = BATTERY_FAULT;
    break;

  default:
    *status = BATTERY_HEALTH_UNKNOWN;
    break;
  }

  return OK;
}

/****************************************************************************
 * Name: mcp73871_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int mcp73871_health(FAR struct battery_charger_dev_s *dev,
                           FAR int *health)
{
  FAR struct mcp73871_dev_s *priv = (FAR struct mcp73871_dev_s *)dev;
  uint8_t value = 0;

  /* Create a value with bits 2 1 0 = STAT1 | STAT2 | PG */

  value = (priv->config->read_stat1() << 2) |
          (priv->config->read_stat2() << 1) |
          (priv->config->read_pg()    << 0);

  switch (value)
  {
  case MCP73871_FAULT:
    *health = BATTERY_HEALTH_SAFE_TMR_EXP;
    break;

  case MCP73871_CHARGING:
  case MCP73871_CHG_COMPLETE:
    *health = BATTERY_HEALTH_GOOD;
    break;

  case MCP73871_NO_BATTERY:
    *health = BATTERY_HEALTH_DISCONNECTED;
    break;

  case MCP73871_BAT_LOW:
  case MCP73871_NO_INPUT_PWR:
  default:
    *health = BATTERY_HEALTH_UNKNOWN;
    break;
  }

  return OK;
}

/****************************************************************************
 * Name: mcp73871_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int mcp73871_online(FAR struct battery_charger_dev_s *dev,
                           FAR bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

/****************************************************************************
 * Name: mcp73871_powersupply
 *
 * Description:
 *   Set the Power Supply Current Limit.
 *
 ****************************************************************************/

static inline int mcp73871_powersupply(FAR struct mcp73871_dev_s *priv,
                                       int current)
{
  /* This chip doesn't support setting current limit by software */

  return OK;
}

/****************************************************************************
 * Name: mcp73871_setvolt
 *
 * Description:
 *   Set the voltage level to charge the battery. Voltage value in mV.
 *
 ****************************************************************************/

static inline int mcp73871_setvolt(FAR struct mcp73871_dev_s *priv,
                                   int volts)
{
  /* This chip doesn't support setting voltage by software */

  return OK;
}

/****************************************************************************
 * Name: mcp73871_setcurr
 *
 * Description:
 *   Set the current to charge the battery. Current value in mA.
 *
 ****************************************************************************/

static inline int mcp73871_setcurr(FAR struct mcp73871_dev_s *priv,
                                   int current)
{
  /* This chip doesn't support setting current by software */

  return OK;
}

/****************************************************************************
 * Name: mcp73871_voltage
 *
 * Description:
 *   Set battery charger voltage
 *
 ****************************************************************************/

static int mcp73871_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  /* This chip doesn't support setting voltage by software */

  return OK;
}

/****************************************************************************
 * Name: mcp73871_current
 *
 * Description:
 *   Set the battery charger current rate for charging
 *
 ****************************************************************************/

static int mcp73871_current(FAR struct battery_charger_dev_s *dev, int value)
{
  /* This chip doesn't support setting current rate by software */

  return OK;
}

/****************************************************************************
 * Name: mcp73871_input_current
 *
 * Description:
 *   Set the power-supply input current limit
 *
 ****************************************************************************/

static int mcp73871_input_current(FAR struct battery_charger_dev_s *dev,
                                  int value)
{
  /* This chip doesn't support setting input current limit by software */

  return OK;
}

/****************************************************************************
 * Name: mcp73871_operate
 *
 * Description:
 *   Do miscellaneous battery ioctl()
 *
 ****************************************************************************/

static int mcp73871_operate(FAR struct battery_charger_dev_s *dev,
                            uintptr_t param)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp73871_initialize
 *
 * Description:
 *   Initialize the BQ2425x battery driver and return an instance of the
 *   lower_half interface that may be used with battery_charger_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_CHARGER - Upper half battery driver support
 *
 * Input Parameters:
 *   config   - The config structure with function pointers to read/write.
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL
 *   pointer is returned on a failure to initialize the BQ2425x lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
           mcp73871_initialize(FAR struct mcp73871_config_s *config)
{
  FAR struct mcp73871_dev_s *priv;

  /* Allocate the MCP73871 device structure */

  priv = (FAR struct mcp73871_dev_s *)
    kmm_zalloc(sizeof(struct mcp73871_dev_s));

  if (priv)
    {
      /* Initialize the MCP73871 device structure */

      priv->dev.ops = &g_mcp73871ops;
      priv->config  = config;

      /* Enable the battery charge */

      priv->config->set_chg_ce(true);
    }

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_CHARGER */
