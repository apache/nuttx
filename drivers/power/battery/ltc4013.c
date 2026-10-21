/****************************************************************************
 * drivers/power/battery/ltc4013.c
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

/* Lower half driver for the Analog Devices LTC4013 battery charger
 * controller.  The chip is controlled through pins that the board
 * provides and the charge voltage is set by the feedback network of the
 * board, see include/nuttx/power/ltc4013.h.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/ltc4013.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Highest Vfb of Table 1 of the datasheet, in mV */

#define LTC4013_VFB_MAX_MV  2400

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A charge algorithm of Table 1 of the datasheet */

struct ltc4013_algo_s
{
  enum ltc4013_level_e mode1;
  enum ltc4013_level_e mode2;
  bool                 staged;  /* 3 or 4 stages, STAT1 on means float */
  uint16_t             vfb_mv;  /* Vfb of the VFLOAT column, mV */
};

struct ltc4013_dev_s
{
  /* The common part of the battery driver visible to the upper half,
   * it must be the first field.
   */

  struct battery_charger_dev_s dev;

  struct ltc4013_config_s config;   /* Board configuration */
  int                     mv;       /* Voltage requested, 0 if none */
  bool                    enabled;  /* The charger is enabled */
  enum ltc4013_algo_e     algo;     /* Selected algorithm */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ltc4013_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status);
static int ltc4013_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health);
static int ltc4013_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status);
static int ltc4013_voltage(FAR struct battery_charger_dev_s *dev,
                           int value);
static int ltc4013_current(FAR struct battery_charger_dev_s *dev,
                           int value);
static int ltc4013_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value);
static int ltc4013_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param);
static int ltc4013_chipid(FAR struct battery_charger_dev_s *dev,
                          FAR unsigned int *value);
static int ltc4013_get_voltage(FAR struct battery_charger_dev_s *dev,
                               FAR int *value);
static int ltc4013_voltage_info(FAR struct battery_charger_dev_s *dev,
                                FAR int *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_ltc4013ops =
{
  ltc4013_state,
  ltc4013_health,
  ltc4013_online,
  ltc4013_voltage,
  ltc4013_current,
  ltc4013_input_current,
  ltc4013_operate,
  ltc4013_chipid,
  ltc4013_get_voltage,
  ltc4013_voltage_info,
  NULL
};

/* Table 1 of the datasheet, in the order of enum ltc4013_algo_e */

static const struct ltc4013_algo_s g_algos[LTC4013_ALGO_NALGOS] =
{
  { LTC4013_LEVEL_HIGH,  LTC4013_LEVEL_FLOAT, false, 2367 },
  { LTC4013_LEVEL_FLOAT, LTC4013_LEVEL_FLOAT, false, LTC4013_VFB_MAX_MV },
  { LTC4013_LEVEL_LOW,   LTC4013_LEVEL_LOW,   false, 2267 },
  { LTC4013_LEVEL_LOW,   LTC4013_LEVEL_FLOAT, false, 2333 },
  { LTC4013_LEVEL_HIGH,  LTC4013_LEVEL_LOW,   true,  2267 },
  { LTC4013_LEVEL_FLOAT, LTC4013_LEVEL_LOW,   true,  2200 },
  { LTC4013_LEVEL_LOW,   LTC4013_LEVEL_HIGH,  true,  2267 },
  { LTC4013_LEVEL_HIGH,  LTC4013_LEVEL_HIGH,  true,  2267 },
  { LTC4013_LEVEL_FLOAT, LTC4013_LEVEL_HIGH,  true,  2200 }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltc4013_apply_mode
 *
 * Description:
 *   Set the MODE pins for an algorithm.
 *
 ****************************************************************************/

static int ltc4013_apply_mode(FAR struct ltc4013_dev_s *priv,
                              enum ltc4013_algo_e algo)
{
  return priv->config.mode(priv->config.arg, g_algos[algo].mode1,
                           g_algos[algo].mode2);
}

/****************************************************************************
 * Name: ltc4013_state
 *
 * Description:
 *   Return the current battery state from Table 2 of the datasheet.
 *
 ****************************************************************************/

static int ltc4013_state(FAR struct battery_charger_dev_s *dev,
                         FAR int *status)
{
  FAR struct ltc4013_dev_s *priv = (FAR struct ltc4013_dev_s *)dev;
  bool stat0;
  bool stat1;
  int ret;

  ret = priv->config.stat(priv->config.arg, &stat0, &stat1);
  if (ret < 0)
    {
      return ret;
    }

  if (stat0 && stat1)
    {
      /* Constant voltage for the Li-Ion and 2-stage algorithms, float
       * charging for the 3 and 4-stage ones.
       */

      *status = g_algos[priv->algo].staged ? BATTERY_FULL :
                                             BATTERY_CHARGING;
    }
  else if (stat0)
    {
      *status = BATTERY_CHARGING;
    }
  else if (stat1)
    {
      *status = BATTERY_FAULT;
    }
  else
    {
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: ltc4013_health
 *
 * Description:
 *   Return the current battery health.  The chip signals a low battery
 *   or a thermal shutdown with the same pattern, so it is not possible to
 *   tell them apart.
 *
 ****************************************************************************/

static int ltc4013_health(FAR struct battery_charger_dev_s *dev,
                          FAR int *health)
{
  int status;
  int ret;

  ret = ltc4013_state(dev, &status);
  if (ret < 0)
    {
      return ret;
    }

  *health = status == BATTERY_FAULT ? BATTERY_HEALTH_UNSPEC_FAIL :
                                      BATTERY_HEALTH_GOOD;
  return OK;
}

/****************************************************************************
 * Name: ltc4013_online
 *
 * Description:
 *   The chip cannot tell if a battery is connected: report whether the
 *   charger is enabled.
 *
 ****************************************************************************/

static int ltc4013_online(FAR struct battery_charger_dev_s *dev,
                          FAR bool *status)
{
  FAR struct ltc4013_dev_s *priv = (FAR struct ltc4013_dev_s *)dev;

  *status = priv->enabled;
  return OK;
}

/****************************************************************************
 * Name: ltc4013_voltage
 *
 * Description:
 *   Set the charge voltage, in mV, of the selected algorithm.  The board
 *   programs its feedback network.
 *
 ****************************************************************************/

static int ltc4013_voltage(FAR struct battery_charger_dev_s *dev,
                           int value)
{
  FAR struct ltc4013_dev_s *priv = (FAR struct ltc4013_dev_s *)dev;
  FAR const struct ltc4013_config_s *cfg = &priv->config;
  int ret;

  if (cfg->voltage == NULL)
    {
      return -ENOSYS;
    }

  if (value <= 0 || (cfg->v_min > 0 && value < cfg->v_min) ||
      (cfg->v_max > 0 && value > cfg->v_max))
    {
      return -EINVAL;
    }

  ret = cfg->voltage(cfg->arg, value, g_algos[priv->algo].vfb_mv);
  if (ret < 0)
    {
      return ret;
    }

  priv->mv = value;
  return OK;
}

/****************************************************************************
 * Name: ltc4013_current
 *
 * Description:
 *   The charge current is set by the sense resistor.
 *
 ****************************************************************************/

static int ltc4013_current(FAR struct battery_charger_dev_s *dev,
                           int value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ltc4013_input_current
 ****************************************************************************/

static int ltc4013_input_current(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ltc4013_enable
 *
 * Description:
 *   Enable or disable the charger.  When the voltage is set by software,
 *   enabling is refused until a voltage has been programmed.
 *
 ****************************************************************************/

static int ltc4013_enable(FAR struct ltc4013_dev_s *priv, bool enable)
{
  int ret;

  if (enable && priv->config.voltage != NULL && priv->mv == 0)
    {
      return -EINVAL;
    }

  ret = priv->config.enable(priv->config.arg, enable);
  if (ret < 0)
    {
      return ret;
    }

  priv->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: ltc4013_set_algo
 *
 * Description:
 *   Select an algorithm.  If a voltage has been requested the board
 *   programs it again with the new regulation voltage of the FB pin, so the
 *   voltage stays the same.
 *
 ****************************************************************************/

static int ltc4013_set_algo(FAR struct ltc4013_dev_s *priv,
                            enum ltc4013_algo_e algo)
{
  FAR const struct ltc4013_config_s *cfg = &priv->config;
  int ret;

  if ((unsigned int)algo >= LTC4013_ALGO_NALGOS)
    {
      return -EINVAL;
    }

  if (priv->enabled)
    {
      return -EBUSY;
    }

  if (priv->mv > 0)
    {
      ret = cfg->voltage(cfg->arg, priv->mv, g_algos[algo].vfb_mv);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = ltc4013_apply_mode(priv, algo);
  if (ret < 0)
    {
      if (priv->mv > 0)
        {
          cfg->voltage(cfg->arg, priv->mv, g_algos[priv->algo].vfb_mv);
        }

      return ret;
    }

  priv->algo = algo;
  return OK;
}

/****************************************************************************
 * Name: ltc4013_operate
 ****************************************************************************/

static int ltc4013_operate(FAR struct battery_charger_dev_s *dev,
                           uintptr_t param)
{
  FAR struct ltc4013_dev_s *priv = (FAR struct ltc4013_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
    (FAR struct batio_operate_msg_s *)param;

  switch (msg->operate_type)
    {
      case BATIO_OPRTN_CHARGE:
        return ltc4013_enable(priv, msg->u8[0] != 0);

      case LTC4013_OPRTN_ALGO:
        return ltc4013_set_algo(priv, (enum ltc4013_algo_e)msg->u8[0]);

      default:
        return -ENOSYS;
    }
}

/****************************************************************************
 * Name: ltc4013_chipid
 *
 * Description:
 *   The chip has no digital interface to read an id from.
 *
 ****************************************************************************/

static int ltc4013_chipid(FAR struct battery_charger_dev_s *dev,
                          FAR unsigned int *value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ltc4013_get_voltage
 *
 * Description:
 *   Return the charge voltage, in mV: the one last requested when the
 *   board sets it, or the one that a fixed divider programs with the
 *   selected algorithm.  This is the setpoint of the algorithm, not a
 *   measurement of the battery.
 *
 ****************************************************************************/

static int ltc4013_get_voltage(FAR struct battery_charger_dev_s *dev,
                               FAR int *value)
{
  FAR struct ltc4013_dev_s *priv = (FAR struct ltc4013_dev_s *)dev;
  FAR const struct ltc4013_config_s *cfg = &priv->config;

  if (cfg->voltage != NULL)
    {
      if (priv->mv == 0)
        {
          return -ENODATA;
        }

      *value = priv->mv;
      return OK;
    }

  if (cfg->r_bottom == 0)
    {
      return -ENODATA;
    }

  *value = ((cfg->r_top + cfg->r_bottom) * g_algos[priv->algo].vfb_mv +
            cfg->r_bottom / 2) / cfg->r_bottom;
  return OK;
}

/****************************************************************************
 * Name: ltc4013_voltage_info
 ****************************************************************************/

static int ltc4013_voltage_info(FAR struct battery_charger_dev_s *dev,
                                FAR int *value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltc4013_register
 ****************************************************************************/

int ltc4013_register(FAR const char *devpath,
                     FAR const struct ltc4013_config_s *config)
{
  FAR struct ltc4013_dev_s *priv;
  int ret;

  if (devpath == NULL || config == NULL || config->enable == NULL ||
      config->mode == NULL || config->stat == NULL ||
      (unsigned int)config->algo >= LTC4013_ALGO_NALGOS)
    {
      return -EINVAL;
    }

  /* The report of the fixed divider is computed in 32 bits: the divider
   * must stay below 1.7 Mohm.
   */

  if (config->voltage == NULL && config->r_bottom > 0 &&
      (uint64_t)config->r_top + config->r_bottom >
      UINT32_MAX / LTC4013_VFB_MAX_MV)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct ltc4013_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->dev.ops = &g_ltc4013ops;
  priv->config  = *config;
  priv->algo    = config->algo;

  /* Start with the charger disabled and the mode pins set */

  ret = config->enable(config->arg, false);
  if (ret < 0)
    {
      goto errout_free;
    }

  ret = ltc4013_apply_mode(priv, priv->algo);
  if (ret < 0)
    {
      goto errout_free;
    }

  ret = battery_charger_register(devpath, &priv->dev);
  if (ret < 0)
    {
      goto errout_free;
    }

  return OK;

errout_free:
  kmm_free(priv);
  return ret;
}
