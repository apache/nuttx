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
#ifdef CONFIG_THERMAL_DUMMY_CPUFREQ
#include <nuttx/cpufreq.h>
#endif
#include <nuttx/thermal.h>

#include <debug.h>
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

#ifdef CONFIG_THERMAL_DUMMY_CPUFREQ
struct dummy_cpufreq_driver_s
{
  struct cpufreq_driver driver;
  const struct cpufreq_frequency_table *table;
  size_t table_len;
  struct cpufreq_frequency_table current;
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

/* CPU Freq */

#ifdef CONFIG_THERMAL_DUMMY_CPUFREQ
FAR static const struct cpufreq_frequency_table *
dummy_cpufreq_get_table(FAR struct cpufreq_policy *driver);
static int dummy_cpufreq_target_index(FAR struct cpufreq_policy *driver,
                                      unsigned int index);
static int dummy_cpufreq_get_frequency(FAR struct cpufreq_policy *driver);
static int dummy_cpufreq_suspend(FAR struct cpufreq_policy *driver);
static int dummy_cpufreq_resume (FAR struct cpufreq_policy *driver);
#endif /* CONFIG_THERMAL_DUMMY_CPUFREQ */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Bind */

static const struct thermal_zone_trip_s g_dummy_trips[] =
{
  {.name = "cpu_crit",   .temp = 90, .hyst = 5, .type = THERMAL_CRITICAL},
  {.name = "cpu_alert1", .temp = 70, .hyst = 5, .type = THERMAL_HOT},
  {.name = "cpu_alert0", .temp = 60, .hyst = 5, .type = THERMAL_NORMAL},
};

static const struct thermal_zone_map_s g_dummy_maps[] =
{
  {
    .trip_name = "cpu_alert1",
    .cdev_name = "cpufreq",
    .low    = 3,
    .high   = THERMAL_NO_LIMIT,
    .weight = 20
  },
  {
    .trip_name = "cpu_alert1",
    .cdev_name = "fan0",
    .low    = THERMAL_NO_LIMIT,
    .high   = THERMAL_NO_LIMIT,
    .weight = 20
  },
  {
    .trip_name = "cpu_alert0",
    .cdev_name = "cpufreq",
    .low    = THERMAL_NO_LIMIT,
    .high   = 2,
    .weight = 20
  },
};

static const struct thermal_zone_params_s g_dummy_params =
{
  .gov_name = "step_wise",
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

/* Cooling Device - fan0 */

static struct dummy_cooling_device_s g_dummy_fan0_data =
{
  .cur_state = 0,
  .max_state = 16,
};

static const struct thermal_cooling_device_ops_s g_dummy_fan0_ops =
{
  .set_state     = dummy_cdev_set_state,
  .get_state     = dummy_cdev_get_state,
  .get_max_state = dummy_cdev_get_max_state,
};

/* Cooling Device - cpufreq */

#ifdef CONFIG_THERMAL_DUMMY_CPUFREQ
static const struct cpufreq_frequency_table g_dummy_cpufreq_table[] =
{
  {100},
  {300},
  {500},
  {700},
  {900},
  {CPUFREQ_TABLE_END},
};
static struct dummy_cpufreq_driver_s g_dummy_cpufreq_driver =
{
  .driver =
    {
      dummy_cpufreq_get_table,
      dummy_cpufreq_target_index,
      dummy_cpufreq_get_frequency,
      dummy_cpufreq_suspend,
      dummy_cpufreq_resume,
    },
  .table = g_dummy_cpufreq_table,
  .table_len = nitems(g_dummy_cpufreq_table),
};
#endif /* CONFIG_THERMAL_DUMMY_CPUFREQ */

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

#ifdef CONFIG_THERMAL_DUMMY_CPUFREQ
static FAR const struct cpufreq_frequency_table *dummy_cpufreq_get_table(
                                           FAR struct cpufreq_policy *policy)
{
  FAR struct dummy_cpufreq_driver_s *driver =
                         (FAR struct dummy_cpufreq_driver_s *)policy->driver;

  return driver->table;
}

static int dummy_cpufreq_target_index(FAR struct cpufreq_policy *policy,
                                      unsigned int index)
{
  FAR struct dummy_cpufreq_driver_s *driver =
                         (FAR struct dummy_cpufreq_driver_s *)policy->driver;

  DEBUGASSERT(index < driver->table_len);

  driver->current.frequency = driver->table[index].frequency;
  return 0;
}

static int dummy_cpufreq_get_frequency(FAR struct cpufreq_policy *policy)
{
  FAR struct dummy_cpufreq_driver_s *driver =
                         (FAR struct dummy_cpufreq_driver_s *)policy->driver;

  return driver->current.frequency;
}

static int dummy_cpufreq_suspend(FAR struct cpufreq_policy *driver)
{
  return 0;
}

static int dummy_cpufreq_resume(FAR struct cpufreq_policy *driver)
{
  return 0;
}
#endif /* CONFIG_THERMAL_DUMMY_CPUFREQ */

int thermal_dummy_init(void)
{
  FAR struct thermal_cooling_device_s *cdev;
  FAR struct thermal_zone_device_s *zdev;
  int ret = OK;

  /* Driver - CPUFreq */

#ifdef CONFIG_THERMAL_DUMMY_CPUFREQ
  ret = cpufreq_init(&g_dummy_cpufreq_driver.driver);
  if (ret < 0)
    {
      therr("Dummy cpufreq driver init failed!\n");
      return ret;
    }
#endif /* CONFIG_THERMAL_DUMMY_CPUFREQ */

  /* Cooling Device */

  cdev = thermal_cooling_device_register("fan0", &g_dummy_fan0_data,
                                         &g_dummy_fan0_ops);
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
      therr("Register zone deivce failed!\n");
      return -ENOTSUP;
    }

  return ret;
}
