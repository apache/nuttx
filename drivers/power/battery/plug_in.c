/****************************************************************************
 * drivers/power/battery/plug_in.c
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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <nuttx/config.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

struct plug_in_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev;   /* Battery charger device */

  /* Data fields specific to the lower half plug_in driver follow */

  FAR struct plug_in_lower_s *lower;
  FAR struct ioexpander_dev_s *io_dev;         /* Ioexpander device */
  struct work_s work;
  struct work_s detect_work;                /* charger detect work */
  bool charging;                            /* Mark charge_manager is not running */
  int batt_state_flag;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int plug_in_state(FAR struct battery_charger_dev_s *dev,
                          FAR int *status);
static int plug_in_health(FAR struct battery_charger_dev_s *dev,
                           FAR int *health);
static int plug_in_online(FAR struct battery_charger_dev_s *dev,
                           FAR bool *status);
static int plug_in_voltage(FAR struct battery_charger_dev_s *dev,
                            int value);
static int plug_in_current(FAR struct battery_charger_dev_s *dev,
                            int value);
static int plug_in_input_current(FAR struct battery_charger_dev_s *dev,
                                  int value);
static int plug_in_operate(FAR struct battery_charger_dev_s *dev,
                            FAR uintptr_t param);
static int plug_in_chipid(FAR struct battery_charger_dev_s *dev,
                           FAR unsigned int *value);
static int plug_in_get_voltage(FAR struct battery_charger_dev_s *dev,
                                FAR int *value);
static int plug_in_voltage_info(FAR struct battery_charger_dev_s *dev,
                                FAR int *value);
static int plug_in_get_protocol(FAR struct battery_charger_dev_s *dev,
                                FAR int *value);

/* Charger rx interrupt functions */

static int plug_in_det_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg);
static void plug_in_det_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_plug_in_ops =
{
  plug_in_state,
  plug_in_health,
  plug_in_online,
  plug_in_voltage,
  plug_in_current,
  plug_in_input_current,
  plug_in_operate,
  plug_in_chipid,
  plug_in_get_voltage,
  plug_in_voltage_info,
  plug_in_get_protocol,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void plug_in_det_worker(FAR void *arg)
{
  FAR struct plug_in_dev_s *priv = arg;
  int charging;
  int ret;
  int det_pin_int_config;

  ret = plug_in_state(&priv->dev, &charging);
  if (ret < 0)
    {
      baterr("Failed to read pin (plug_in_det): %d\n", ret);
    }

  priv->charging = charging ? true : false;
  battery_charger_changed(&priv->dev, BATTERY_STATE_CHANGED);

  det_pin_int_config = charging ? IOEXPANDER_VAL_RISING :
                                 IOEXPANDER_VAL_FALLING;

  ret = IOEXP_SETOPTION(priv->io_dev, priv->lower->detect_pin,
              IOEXPANDER_OPTION_INTCFG, (void *)det_pin_int_config);
  if (ret < 0)
    {
      baterr("Failed to set option: %d\n", ret);
      IOEP_DETACH(priv->io_dev, plug_in_det_interrupt_handler);
    }
}

static int plug_in_state(FAR struct battery_charger_dev_s *dev,
                          FAR int *status)
{
  FAR struct plug_in_dev_s *priv = (FAR struct plug_in_dev_s *)dev;
  bool plug_in_det;
  int  ret;

  /* Check PLUG_IN_DET */

  ret = IOEXP_READPIN(priv->io_dev, priv->lower->detect_pin,
                     &plug_in_det);
  if (ret < 0)
    {
      baterr("Failed to read pin (plug_in_det): %d\n", ret);
      return ret;
    }

  *status = !plug_in_det;

  return OK;
}

static int plug_in_health(FAR struct battery_charger_dev_s *dev,
                           FAR int *health)
{
  return OK;
}

static int plug_in_online(FAR struct battery_charger_dev_s *dev,
                           FAR bool *status)
{
  return OK;
}

static int plug_in_voltage(FAR struct battery_charger_dev_s *dev,
                            FAR int value)
{
  return OK;
}

static int plug_in_current(FAR struct battery_charger_dev_s *dev,
                            FAR int value)
{
  return OK;
}

static int plug_in_input_current(FAR struct battery_charger_dev_s *dev,
                                  FAR int value)
{
  return OK;
}

static int plug_in_operate(FAR struct battery_charger_dev_s *dev,
                            FAR uintptr_t param)
{
  return OK;
}

static int plug_in_chipid(FAR struct battery_charger_dev_s *dev,
                           FAR unsigned int *value)
{
  return OK;
}

static int plug_in_get_voltage(FAR struct battery_charger_dev_s *dev,
                                FAR int *value)
{
  return OK;
}

static int plug_in_voltage_info(FAR struct battery_charger_dev_s *dev,
                                FAR int *value)
{
  return OK;
}

static int plug_in_get_protocol(FAR struct battery_charger_dev_s *dev,
                                FAR int *value)
{
  return OK;
}

static int plug_in_det_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                        ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct plug_in_dev_s *priv = arg;

  work_queue(LPWORK, &priv->work, plug_in_det_worker, priv, 0);

  return OK;
}

static int plug_in_init_interrupt(FAR struct plug_in_dev_s *priv)
{
  int ret;
  FAR void *ioepattach;
  bool plug_in_det = 0;
  int det_pin_int_config = 0;

  ret = IOEXP_SETDIRECTION(priv->io_dev, priv->lower->detect_pin,
                           IOEXPANDER_DIRECTION_IN);
  if (ret < 0)
    {
      baterr("Failed to set direction: %d\n", ret);
      return ret;
    }

  ioepattach = IOEP_ATTACH(priv->io_dev, priv->lower->detect_pin,
                          plug_in_det_interrupt_handler, priv);
  if (ioepattach == NULL)
    {
      baterr("Failed to attach plug_in_interrupt_handler");
    }

  ret = IOEXP_READPIN(priv->io_dev, priv->lower->detect_pin,
                     &plug_in_det);
  if (ret < 0)
    {
      baterr("Failed to read pin (plug_in_det): %d\n", ret);
      ret = -EIO;
    }

  priv->charging = plug_in_det ? false : true;

  det_pin_int_config = plug_in_det ? IOEXPANDER_VAL_FALLING :
                                     IOEXPANDER_VAL_RISING;
  ret = IOEXP_SETOPTION(priv->io_dev, priv->lower->detect_pin,
                        IOEXPANDER_OPTION_INTCFG,
                        (void *)det_pin_int_config);
  if (ret < 0)
    {
      baterr("Failed to set option: %d\n", ret);
      IOEP_DETACH(priv->io_dev, plug_in_det_interrupt_handler);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct battery_charger_dev_s *
  plug_in_initialize(FAR struct plug_in_lower_s *lower,
                     FAR struct ioexpander_dev_s *io_dev)
{
  FAR struct plug_in_dev_s *priv;
  int ret;

  /* Initialize the plug_in device structure */

  priv = kmm_zalloc(sizeof(struct plug_in_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->dev.ops   = &g_plug_in_ops;
  priv->lower     = lower;
  priv->io_dev    = io_dev;
  priv->charging  = false;

  ret = plug_in_init_interrupt(priv);
  if (ret < 0)
    {
      baterr("Failed to init_interrupt: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  /* force notify state */

  work_queue(LPWORK, &priv->work, plug_in_det_worker, priv, 100);

  return (FAR struct battery_charger_dev_s *)priv;
}
