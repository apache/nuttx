/****************************************************************************
 * drivers/thermal/thermal_dummy.c
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
#ifdef CONFIG_THERMAL_DUMMY_DEVFREQ
#include <nuttx/devfreq.h>
#endif
#include <nuttx/thermal.h>

#include <nuttx/debug.h>
#include <sys/param.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DUMMY_TEMP_RANGE_LOW  45
#define DUMMY_TEMP_RANGE_HIGH 90

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dummy_zone_device_s
{
  int temperature;
  bool raising;
  bool temp_jump;
};

struct dummy_cooling_device_s
{
  unsigned int cur_state;
  unsigned int max_state;
};

#ifdef CONFIG_THERMAL_DUMMY_DEVFREQ
struct dummy_devfreq_driver_s
{
  struct devfreq_driver_s driver;
  FAR const uint32_t *table;
  size_t table_len;
  uint32_t current;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Zone Device */

static int dummy_zdev_get_temp (FAR struct thermal_zone_device_s *zdev,
                                FAR int *temp);
static int dummy_zdev_set_trips(FAR struct thermal_zone_device_s *zdev,
                                int low, int high);

/* Cooling Device */

static int
dummy_cdev_get_max_state(FAR struct thermal_cooling_device_s *cdev,
                         FAR unsigned int *state);
static int
dummy_cdev_get_state    (FAR struct thermal_cooling_device_s *cdev,
                         FAR unsigned int *state);
static int
dummy_cdev_set_state    (FAR struct thermal_cooling_device_s *cdev,
                         unsigned int state);

/* devfreq */

#ifdef CONFIG_THERMAL_DUMMY_DEVFREQ
static FAR const uint32_t *
dummy_devfreq_get_table(FAR struct devfreq_s *devfreq);
static int dummy_devfreq_target_index(FAR struct devfreq_s *devfreq,
                                      size_t index);
static uint32_t dummy_devfreq_get_frequency(FAR struct devfreq_s *devfreq);
static int dummy_devfreq_suspend(FAR struct devfreq_s *devfreq);
static int dummy_devfreq_resume (FAR struct devfreq_s *devfreq);
#endif /* CONFIG_THERMAL_DUMMY_DEVFREQ */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Bind */

static const struct thermal_zone_trip_s g_dummy_trips[] =
{
  {.name = "cpu_crit",   .temp = 90, .hyst = 5, .type = THERMAL_CRITICAL},
  {.name = "cpu_alert1", .temp = 70, .hyst = 5, .type = THERMAL_HOT},
  {.name = "cpu_alert0", .temp = 60, .hyst = 5, .type = THERMAL_PASSIVE},
};

static const struct thermal_zone_map_s g_dummy_maps[] =
{
#ifdef CONFIG_THERMAL_DUMMY_DEVFREQ
  {
    .trip_name = "cpu_alert1",
    .cdev_name = CONFIG_THERMAL_CDEV_DEVFREQ_NAME,
    .low    = 3,
    .high   = THERMAL_NO_LIMIT,
    .weight = 20
  },
#endif
  {
    .trip_name = "cpu_alert1",
    .cdev_name = "fan0",
    .low    = THERMAL_NO_LIMIT,
    .high   = THERMAL_NO_LIMIT,
    .weight = 20
  },
#ifdef CONFIG_THERMAL_DUMMY_DEVFREQ
  {
    .trip_name = "cpu_alert0",
    .cdev_name = CONFIG_THERMAL_CDEV_DEVFREQ_NAME,
    .low    = THERMAL_NO_LIMIT,
    .high   = 2,
    .weight = 20
  },
#endif
  {
    .trip_name = "cpu_alert0",
    .cdev_name = "passive_dev",
    .low    = THERMAL_NO_LIMIT,
    .high   = THERMAL_NO_LIMIT,
    .weight = 20
  },
};

static const struct thermal_zone_params_s g_dummy_params =
{
  .gov_name = "step_wise",
  .passive_delay = CONFIG_THERMAL_DUMMY_POLLING_DELAY * 2,
  .polling_delay = CONFIG_THERMAL_DUMMY_POLLING_DELAY,
  .trips = g_dummy_trips,
  .num_trips = nitems(g_dummy_trips),
  .maps = g_dummy_maps,
  .num_maps = nitems(g_dummy_maps),
};

/* Zone device */

static const struct thermal_zone_device_ops_s g_dummy_zone_ops =
{
  .get_temp  = dummy_zdev_get_temp,
  .set_trips = dummy_zdev_set_trips,
};

static struct dummy_zone_device_s g_dummy_zone =
{
  .temperature = 45,
  .raising = true,
  .temp_jump = true,
};

static const struct thermal_cooling_device_ops_s g_dummy_cooling_ops =
{
  .set_state     = dummy_cdev_set_state,
  .get_state     = dummy_cdev_get_state,
  .get_max_state = dummy_cdev_get_max_state,
};

/* Cooling Device - fan0 */

static struct dummy_cooling_device_s g_dummy_fan0_data =
{
  .cur_state = 0,
  .max_state = 16,
};

/* Cooling Device - devfreq */

#ifdef CONFIG_THERMAL_DUMMY_DEVFREQ
static const uint32_t g_dummy_devfreq_table[] =
{
  100,
  300,
  500,
  700,
  900,
  DEVFREQ_ENTRY_END,
};
static struct dummy_devfreq_driver_s g_dummy_devfreq_driver =
{
  .driver =
    {
      /* A ceiling from the cooling device must win over any floor, which
       * is what a device defending a thermal budget wants.
       */

      .conflict_policy = DEVFREQ_CONFLICT_PREFER_LOW,
      .get_table       = dummy_devfreq_get_table,
      .target_index    = dummy_devfreq_target_index,
      .get_frequency   = dummy_devfreq_get_frequency,
      .suspend         = dummy_devfreq_suspend,
      .resume          = dummy_devfreq_resume,
    },
  .table = g_dummy_devfreq_table,

  /* Frequencies only.  target_index is never called with the terminator's
   * own index, so it is not counted here.
   */

  .table_len = nitems(g_dummy_devfreq_table) - 1,
};
#else
static struct dummy_cooling_device_s g_dummy_passive =
{
  .cur_state = 0,
  .max_state = 1,
};
#endif /* CONFIG_THERMAL_DUMMY_DEVFREQ */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Dummy Cooling Device Operations */

static int dummy_cdev_set_state(FAR struct thermal_cooling_device_s *cdev,
                                unsigned int state)
{
  FAR struct dummy_cooling_device_s *c = cdev->devdata;

  c->cur_state = state;
  return OK;
}

static int dummy_cdev_get_state(FAR struct thermal_cooling_device_s *cdev,
                                FAR unsigned int *state)
{
  FAR struct dummy_cooling_device_s *c = cdev->devdata;

  *state = c->cur_state;
  return OK;
}

static int
dummy_cdev_get_max_state(FAR struct thermal_cooling_device_s *cdev,
                         FAR unsigned int *state)
{
  FAR struct dummy_cooling_device_s *c = cdev->devdata;

  *state = c->max_state;
  return OK;
}

/* Sensor */

static int dummy_zdev_get_temp(FAR struct thermal_zone_device_s *zdev,
                               FAR int *temp)
{
  FAR struct dummy_zone_device_s *s = zdev->devdata;

  if (s->temperature >= DUMMY_TEMP_RANGE_HIGH)
    {
      s->raising = false;
    }
  else if (s->temperature <= DUMMY_TEMP_RANGE_LOW)
    {
      s->raising = true;
    }

  if (s->raising)
    {
      s->temperature++;
    }
  else
    {
      s->temperature--;
    }

  *temp = s->temperature + (s->temp_jump ? 2 : -2);
  s->temp_jump = !s->temp_jump;
  return OK;
}

static int dummy_zdev_set_trips(FAR struct thermal_zone_device_s *zdev,
                                int low, int high)
{
  return OK;
}

#ifdef CONFIG_THERMAL_DUMMY_DEVFREQ
static FAR const uint32_t *dummy_devfreq_get_table(
                                            FAR struct devfreq_s *devfreq)
{
  FAR struct dummy_devfreq_driver_s *driver =
                        (FAR struct dummy_devfreq_driver_s *)devfreq->driver;

  return driver->table;
}

static int dummy_devfreq_target_index(FAR struct devfreq_s *devfreq,
                                      size_t index)
{
  FAR struct dummy_devfreq_driver_s *driver =
                        (FAR struct dummy_devfreq_driver_s *)devfreq->driver;

  DEBUGASSERT(index < driver->table_len);

  driver->current = driver->table[index];
  return 0;
}

static uint32_t dummy_devfreq_get_frequency(FAR struct devfreq_s *devfreq)
{
  FAR struct dummy_devfreq_driver_s *driver =
                        (FAR struct dummy_devfreq_driver_s *)devfreq->driver;

  return driver->current;
}

static int dummy_devfreq_suspend(FAR struct devfreq_s *devfreq)
{
  return 0;
}

static int dummy_devfreq_resume(FAR struct devfreq_s *devfreq)
{
  return 0;
}
#endif /* CONFIG_THERMAL_DUMMY_DEVFREQ */

int thermal_dummy_init(void)
{
  FAR struct thermal_cooling_device_s *cdev;
  FAR struct thermal_zone_device_s *zdev;
  int ret = OK;

  /* Driver - devfreq.  The thermal core registers the cooling device over
   * it once this returns, so it has to exist by then.
   */

#ifdef CONFIG_THERMAL_DUMMY_DEVFREQ
  if (devfreq_register(CONFIG_THERMAL_CDEV_DEVFREQ_NAME,
                       devfreq_performance(),
                       &g_dummy_devfreq_driver.driver, NULL) == NULL)
    {
      therr("Dummy devfreq driver init failed!\n");
      return -ENOTSUP;
    }
#else
  cdev = thermal_cooling_device_register("passive_dev", &g_dummy_passive,
                                         &g_dummy_cooling_ops);
  if (cdev == NULL)
    {
      therr("Register cooling device passive_dev failed!\n");
      return -ENOTSUP;
    }
#endif /* CONFIG_THERMAL_DUMMY_DEVFREQ */

  /* Cooling Device */

  cdev = thermal_cooling_device_register("fan0", &g_dummy_fan0_data,
                                         &g_dummy_cooling_ops);
  if (cdev == NULL)
    {
      therr("Register cooling device fan0 failed!\n");
      return -ENOTSUP;
    }

  /* Zone Device */

  zdev = thermal_zone_device_register("cpu-thermal", &g_dummy_zone,
                                      &g_dummy_zone_ops, &g_dummy_params);
  if (zdev == NULL)
    {
      therr("Register zone device failed!\n");
      return -ENOTSUP;
    }

  return ret;
}
