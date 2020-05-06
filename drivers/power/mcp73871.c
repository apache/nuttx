/****************************************************************************
 * drivers/power/mcp73871.c
 * Lower half driver for Microchip MCP73871 battery charger
 *
 *   Copyright (C) 2018 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* The MCP73871 is a standalone battery charger, so this driver is not
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

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_MCP73871
#  define baterr  _err
#  define batreg  _err
#else
#  define baterr  _none
#  define batreg  _none
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct mcp73871_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_charger_operations_s *ops; /* Battery operations */

  sem_t batsem;                /* Enforce mutually exclusive access */

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

      nxsem_init(&priv->batsem, 0, 1);
      priv->ops    = &g_mcp73871ops;
      priv->config = config;

      /* Enable the battery charge */

      priv->config->set_chg_ce(true);
    }

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_CHARGER */
