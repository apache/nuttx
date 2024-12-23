/****************************************************************************
 * drivers/power/battery/battery_fakegauge.c
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
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/power/battery_gauge.h>
#include <nuttx/power/battery_ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BATTERY_FAKE_VOLTAGE_MAX  (4200)
#define BATTERY_FAKE_VOLTAGE_MIN  (4000)
#define BATTERY_FAKE_CURRENT_MAX  (500)
#define BATTERY_FAKE_CURRENT_MIN  (100)
#define BATTERY_FAKE_CAPACITY_MAX (100)
#define BATTERY_FAKE_CAPACITY_MIN (60)    // we dont want too lower
#define BATTERY_FAKE_TEMP_MAX     (420)
#define BATTERY_FAKE_TEMP_MIN     (220)
#define BATTERY_FAKE_WORK_DELAY   MSEC2TICK(5000)
#define BATTERY_FAKE_GAUGE        "/dev/charge/fakegauge"

/****************************************************************************
 * Private type
 ****************************************************************************/

struct battery_fake_data_s
{
  uint32_t present;
  uint32_t voltage;
  uint32_t capacity;
  uint32_t current;
  uint32_t temperature;
  uint32_t status;
};

struct battery_fake_gauge_s
{
  struct battery_gauge_dev_s battery;
  struct battery_fake_data_s data;
  struct work_s poll_work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int battery_fake_present(FAR struct battery_gauge_dev_s *dev,
                                FAR bool *status);
static int battery_fake_state(FAR struct battery_gauge_dev_s *dev,
                              FAR int *status);
static int battery_fake_voltage(FAR struct battery_gauge_dev_s *dev,
                                FAR int *value);
static int battery_fake_capacity(FAR struct battery_gauge_dev_s *dev,
                                 FAR int *value);
static int battery_fake_current(FAR struct battery_gauge_dev_s *dev,
                                FAR int *value);
static int battery_fake_temperature(FAR struct battery_gauge_dev_s *dev,
                                    FAR int *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_gauge_operations_s g_battery_fake_ops =
{
  battery_fake_state,
  battery_fake_present,
  battery_fake_voltage,
  battery_fake_capacity,
  battery_fake_current,
  battery_fake_temperature,
  NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int battery_fake_present(FAR struct battery_gauge_dev_s *dev,
                                FAR bool *status)
{
  /* fake gauge is always keep present */

  *status = true;

  return OK;
}

static int battery_fake_state(FAR struct battery_gauge_dev_s *dev,
                              FAR int *status)
{
  FAR struct battery_fake_gauge_s *priv =
    container_of(dev, struct battery_fake_gauge_s, battery);

  *status = priv->data.status;
  return OK;
}

static int battery_fake_voltage(FAR struct battery_gauge_dev_s *dev,
                                FAR int *value)
{
  FAR struct battery_fake_gauge_s *priv =
    container_of(dev, struct battery_fake_gauge_s, battery);

  *value = priv->data.voltage;
  return OK;
}

static int  battery_fake_capacity(FAR struct battery_gauge_dev_s *dev,
                                  FAR int *value)
{
  FAR struct battery_fake_gauge_s *priv =
    container_of(dev, struct battery_fake_gauge_s, battery);

  *value = priv->data.capacity;
  return OK;
}

static int battery_fake_current(FAR struct battery_gauge_dev_s *dev,
                                FAR int *value)
{
  FAR struct battery_fake_gauge_s *priv =
    container_of(dev, struct battery_fake_gauge_s, battery);

  *value = priv->data.current;
  return OK;
}

static int battery_fake_temperature(FAR struct battery_gauge_dev_s *dev,
                                    FAR int *value)
{
  FAR struct battery_fake_gauge_s *priv =
    container_of(dev, struct battery_fake_gauge_s, battery);

  *value = priv->data.temperature;
  return OK;
}

static void battery_fake_data_init(FAR struct battery_fake_data_s *data)
{
  data->status = BATTERY_CHARGING;
  data->capacity = BATTERY_FAKE_CAPACITY_MIN;
  data->temperature = BATTERY_FAKE_TEMP_MIN;
  data->voltage = BATTERY_FAKE_VOLTAGE_MIN;
  data->current = BATTERY_FAKE_CURRENT_MIN;
  data->present = true;
}

static int battery_fake_data_charged(FAR struct battery_fake_gauge_s *priv)
{
  FAR struct battery_fake_data_s *data = &priv->data;
  int capacity;
  int voltage;
  int current;
  int temp;
  int ret;

  /* capacity */

  capacity = BATTERY_FAKE_CAPACITY_MIN +
    rand() % (BATTERY_FAKE_CAPACITY_MAX - BATTERY_FAKE_CAPACITY_MIN + 1);
  data->capacity = capacity;

  /* voltage */

  voltage = BATTERY_FAKE_VOLTAGE_MIN +
    rand() % (BATTERY_FAKE_VOLTAGE_MAX - BATTERY_FAKE_VOLTAGE_MIN + 1);
  data->voltage = voltage;

  /* current */

  current = BATTERY_FAKE_CURRENT_MIN +
    rand() % (BATTERY_FAKE_CURRENT_MAX - BATTERY_FAKE_CURRENT_MIN + 1);
  data->current = current;

  /* temperature */

  temp = BATTERY_FAKE_TEMP_MIN +
    rand() % (BATTERY_FAKE_TEMP_MAX - BATTERY_FAKE_TEMP_MIN + 1);
  data->temperature = temp;

  if (data->capacity > 10 && data->capacity < 95)
    {
      data->status = BATTERY_CHARGING;
    }
  else if (data->capacity >= 100)
    {
      data->status = BATTERY_DISCHARGING;
      data->current = 0;
    }
  else if (data->capacity >= 95 && data->capacity < 100)
    {
      data->status = BATTERY_FULL;
    }
  else
    {
      data->status = BATTERY_UNKNOWN;
    }

  ret = battery_gauge_changed(&priv->battery, BATTERY_STATE_CHANGED |
                              BATTERY_VOLTAGE_CHANGED |
                              BATTERY_CURRENT_CHANGED |
                              BATTERY_CAPACITY_CHANGED |
                              BATTERY_TEMPERATURE_CHANGED);
  if (ret < 0)
    {
      baterr("battery fake gauge changed failed %d\n", ret);
    }

  return ret;
}

static void battery_fake_work(FAR void *arg)
{
  FAR struct battery_fake_gauge_s *priv =
    (FAR struct battery_fake_gauge_s *)arg;

  battery_fake_data_charged(priv);

  work_queue(LPWORK, &priv->poll_work, battery_fake_work, priv,
             BATTERY_FAKE_WORK_DELAY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int battery_fake_gauge_register()
{
  int ret;
  FAR struct battery_fake_gauge_s *priv;

  priv = kmm_zalloc(sizeof(struct battery_fake_gauge_s));
  if (NULL == priv)
    {
      baterr(" no enough memory\n");
      return -ENOMEM;
    }

  battery_fake_data_init(&priv->data);

  priv->battery.ops = &g_battery_fake_ops;
  ret = battery_gauge_register(BATTERY_FAKE_GAUGE, &priv->battery);
  if (ret < 0)
    {
      baterr("battery_gauge_register %s failed\n", BATTERY_FAKE_GAUGE);
      kmm_free(priv);
      return ret;
    }

  work_queue(LPWORK, &priv->poll_work, battery_fake_work, priv,
             BATTERY_FAKE_WORK_DELAY);

  batinfo("battery fake gauge register successfully\n");
  return 0;
}
