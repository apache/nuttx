/****************************************************************************
 * drivers/power/battery/goldfish_battery.c
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

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/power/battery_gauge.h>
#include <nuttx/power/battery_ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GOLDFISH_BATTERY_READ(data, addr) \
    (*(FAR volatile uint32_t *)(data->reg_base + addr))
#define GOLDFISH_BATTERY_WRITE(data, addr, x) \
    (*(FAR volatile uint32_t *)(data->reg_base + addr) = (x))
#define GOLDFISH_GAUGE "/dev/charge/goldfish_battery"

/****************************************************************************
 * Private type
 ****************************************************************************/

enum
{
  /* Status register */

  BATTERY_INT_STATUS = 0x00,

  /* Set this to enable IRQ */

  BATTERY_INT_ENABLE = 0x04,
  BATTERY_AC_ONLINE = 0x08,
  BATTERY_STATUS = 0x0c,
  BATTERY_HEALTH = 0x10,
  BATTERY_PRESENT = 0x14,
  BATTERY_CAPACITY = 0x18,
  BATTERY_VOLTAGE = 0x1c,
  BATTERY_TEMP = 0x20,
  BATTERY_CHARGE_COUNTER = 0x24,
  BATTERY_VOLTAGE_MAX = 0x28,
  BATTERY_CURRENT_MAX = 0x2c,
  BATTERY_CURRENT_NOW = 0x30,
  BATTERY_CURRENT_AVG = 0x34,
  BATTERY_CHARGE_FULL_UAH = 0x38,
  BATTERY_CYCLE_COUNT = 0x40,
  BATTERY_STATUS_CHANGED = 1u << 0,
  AC_STATUS_CHANGED = 1u << 1,
  BATTERY_INT_MASK = BATTERY_STATUS_CHANGED | AC_STATUS_CHANGED,
};

enum
{
  POWER_SUPPLY_STATUS_UNKNOWN,
  POWER_SUPPLY_STATUS_CHARGING,
  POWER_SUPPLY_STATUS_DISCHARGING,
  POWER_SUPPLY_STATUS_NOT_CHARGING,
  POWER_SUPPLY_STATUS_FULL,
};

struct goldfish_battery_data_s
{
  FAR void *reg_base;
  int irq;
  struct battery_gauge_dev_s battery;
  struct work_s work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int goldfish_charger_online(FAR struct battery_gauge_dev_s *dev,
                                   FAR bool *status);
static int goldfish_battery_state(FAR struct battery_gauge_dev_s *dev,
                                  FAR int *status);
static int goldfish_battery_voltage(FAR struct battery_gauge_dev_s *dev,
                                    FAR b16_t *value);
static int  goldfish_battery_capacity(FAR struct battery_gauge_dev_s *dev,
                                      FAR b16_t *value);
static int goldfish_battery_current(FAR struct battery_gauge_dev_s *dev,
                                    FAR b16_t *value);
static int goldfish_battery_temp(FAR struct battery_gauge_dev_s *dev,
                                 FAR b8_t *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_gauge_operations_s g_goldfish_gauge_ops =
{
  goldfish_battery_state,
  goldfish_charger_online,
  goldfish_battery_voltage,
  goldfish_battery_capacity,
  goldfish_battery_current,
  goldfish_battery_temp,
  NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int goldfish_charger_online(FAR struct battery_gauge_dev_s *dev,
                                   FAR bool *status)
{
  FAR struct goldfish_battery_data_s *priv =
      container_of(dev, struct goldfish_battery_data_s, battery);

  *status = GOLDFISH_BATTERY_READ(priv, BATTERY_AC_ONLINE) & 0x1;
  return OK;
}

static int goldfish_battery_state(FAR struct battery_gauge_dev_s *dev,
                                  FAR int *status)
{
  FAR struct goldfish_battery_data_s *priv =
      container_of(dev, struct goldfish_battery_data_s, battery);
  uint32_t regval;

  regval = GOLDFISH_BATTERY_READ(priv, BATTERY_STATUS);

  switch (regval)
  {
    case POWER_SUPPLY_STATUS_UNKNOWN:
      *status = BATTERY_UNKNOWN;
      break;
    case POWER_SUPPLY_STATUS_CHARGING:
      *status = BATTERY_CHARGING;
      break;
    case POWER_SUPPLY_STATUS_DISCHARGING:
      *status = BATTERY_DISCHARGING;
      break;
    case POWER_SUPPLY_STATUS_NOT_CHARGING:
      *status = BATTERY_IDLE;
      break;
    case POWER_SUPPLY_STATUS_FULL:
      *status = BATTERY_FULL;
      break;
    default:
      *status = BATTERY_UNKNOWN;
      break;
  }

  return OK;
}

static int goldfish_battery_voltage(FAR struct battery_gauge_dev_s *dev,
                                    FAR b16_t *value)
{
  FAR struct goldfish_battery_data_s *data =
      container_of(dev, struct goldfish_battery_data_s, battery);
  uint32_t regval;
  float vol;

  /* BATTERY_VOLTAGE units is µV */

  regval = GOLDFISH_BATTERY_READ(data, BATTERY_VOLTAGE);

  /* convert to unit V and fill b16_t */

  vol = regval / 1000000.0f;
  *value = ftob16(vol);
  return OK;
}

static int  goldfish_battery_capacity(FAR struct battery_gauge_dev_s *dev,
                                      FAR b16_t *value)
{
  FAR struct goldfish_battery_data_s *data =
      container_of(dev, struct goldfish_battery_data_s, battery);
  uint32_t regval;

  /* BATTERY_CAPACITY units is percentage */

  regval = GOLDFISH_BATTERY_READ(data, BATTERY_CAPACITY);
  *value = uitoub16(regval);
  return OK;
}

static int goldfish_battery_current(FAR struct battery_gauge_dev_s *dev,
                                    FAR b16_t *value)
{
  FAR struct goldfish_battery_data_s *data =
      container_of(dev, struct goldfish_battery_data_s, battery);
  uint32_t regval;
  float current;

  /* BATTERY_CURRENT_NOW units is µA */

  regval = GOLDFISH_BATTERY_READ(data, BATTERY_CURRENT_NOW);

  /* convert to unit mA and fill b16_t */

  current = regval / 1000.0f;
  *value = ftob16(current);
  return OK;
}

static int goldfish_battery_temp(FAR struct battery_gauge_dev_s *dev,
                                 FAR b8_t *value)
{
  FAR struct goldfish_battery_data_s *data =
      container_of(dev, struct goldfish_battery_data_s, battery);
  int32_t regval;
  float temp;

  /* BATTERY_TEMP units is 0.1 celsuis */

  regval = GOLDFISH_BATTERY_READ(data, BATTERY_TEMP);

  /* convert to unit celsuis and fill b16_t */

  temp = regval / 10.0f;
  *value = ftob8(temp);
  return OK;
}

static void goldfish_battery_work(FAR void *arg)
{
  FAR struct goldfish_battery_data_s *data = arg;
  uint32_t mask = BATTERY_STATE_CHANGED | BATTERY_VOLTAGE_CHANGED |
                  BATTERY_CURRENT_CHANGED | BATTERY_CAPACITY_CHANGED |
                  BATTERY_TEMPERATURE_CHANGED | BATTERY_ONLINE_CHANGED;
  int ret;

  ret = battery_gauge_changed(&data->battery, mask);
  if (ret < 0)
    {
      baterr("goldfish battery changed failed %d\n", ret);
    }

  return;
}

static int goldfish_battery_interrupt(int irq, FAR void *context, void *arg)
{
  FAR struct goldfish_battery_data_s *data = arg;
  uint32_t status;
  int ret;

  /* read status flags, which will clear the interrupt */

  status = GOLDFISH_BATTERY_READ(data, BATTERY_INT_STATUS);
  status &= BATTERY_INT_MASK;
  if (status)
    {
      ret = work_queue(HPWORK, &data->work,
                       goldfish_battery_work, data, 0);
      if (ret < 0)
        {
          baterr("battery interrupt %"PRIu32
                 "work_battery ret:%d\n", status, ret);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int goldfish_battery_register(FAR void *regs, int irq)
{
  int ret;
  FAR struct goldfish_battery_data_s *data;

  if (NULL == regs || irq < 0)
    {
      baterr(" regs:%p irq:%d\n", regs, irq);
      return -EINVAL;
    }

  data = kmm_zalloc(sizeof(struct goldfish_battery_data_s));
  if (NULL == data)
    {
      baterr(" no enough memory\n");
      return -ENOMEM;
    }

  data->irq = irq;
  data->reg_base = regs;
  ret = irq_attach(data->irq, goldfish_battery_interrupt, data);
  if (ret < 0)
    {
      baterr(" attach irq %d failed\n", irq);
      goto fail;
    }

  data->battery.ops = &g_goldfish_gauge_ops;
  ret = battery_gauge_register(GOLDFISH_GAUGE, &data->battery);
  if (ret < 0)
    {
      baterr("battery_gauge_register %s failed", GOLDFISH_GAUGE);
      irq_detach(data->irq);
      goto fail;
    }

  up_enable_irq(data->irq);
  GOLDFISH_BATTERY_WRITE(data, BATTERY_INT_ENABLE, BATTERY_INT_MASK);

  batinfo("goldfish_battery_register over");
  return 0;
fail:
  kmm_free(data);
  return ret;
}

