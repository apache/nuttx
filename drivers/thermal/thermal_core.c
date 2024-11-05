/****************************************************************************
 * drivers/thermal/thermal_core.c
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

#include <nuttx/kmalloc.h>
#ifdef CONFIG_PM
#include <nuttx/power/pm.h>
#endif

#include <debug.h>
#include <stdio.h>
#include <sys/boardctl.h>

#include "sched/sched.h"
#include "thermal_core.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  zone_bind_cooling  (FAR struct thermal_zone_device_s *zdev,
                                int trip,
                                FAR struct thermal_cooling_device_s *cdev,
                                unsigned int upper, unsigned int lower,
                                unsigned int weight);
static void zone_unbind_cooling(FAR struct thermal_zone_device_s *zdev,
                                int trip,
                                FAR struct thermal_cooling_device_s *cdev);

static FAR struct thermal_governor_s *
find_governor_by_name          (FAR const char *name);
static int  zone_set_governor  (FAR struct thermal_zone_device_s *zdev,
                                FAR struct thermal_governor_s *gov);

static void device_bind        (FAR struct thermal_zone_device_s *zdev,
                                FAR struct thermal_cooling_device_s *cdev);
static void device_unbind      (FAR struct thermal_zone_device_s *zdev,
                                FAR struct thermal_cooling_device_s *cdev);

#ifdef CONFIG_PM
static void thermal_pm_notify(FAR struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node
g_cooling_dev_list = LIST_INITIAL_VALUE(g_cooling_dev_list);

static struct list_node
g_governor_list = LIST_INITIAL_VALUE(g_governor_list);

static struct list_node
g_zone_dev_list = LIST_INITIAL_VALUE(g_zone_dev_list);

static mutex_t g_thermal_lock = NXMUTEX_INITIALIZER;

static FAR struct thermal_governor_s *g_def_governor = NULL;

#ifdef CONFIG_PM
struct pm_callback_s g_thermal_pm_cb =
{
  .notify  = thermal_pm_notify,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int zone_set_governor(FAR struct thermal_zone_device_s *zdev,
                             FAR struct thermal_governor_s *gov)
{
  int ret = OK;

  /* The caller must use `g_thermal_lock` to protect zones and governors */

  if (zdev->governor && zdev->governor->unbind_from_tz)
    {
      zdev->governor->unbind_from_tz(zdev);
    }

  if (gov && gov->bind_to_tz)
    {
      ret = gov->bind_to_tz(zdev);
      if (ret < 0)
        {
          if (zdev->governor->bind_to_tz(zdev) < 0)
            {
              zdev->governor = NULL;
            }

          return ret;
        }
    }

  zdev->governor = gov;
  return ret;
}

static int zone_bind_cooling(FAR struct thermal_zone_device_s *zdev,
                             int trip,
                             FAR struct thermal_cooling_device_s *cdev,
                             unsigned int upper, unsigned int lower,
                             unsigned int weight)
{
  FAR struct thermal_instance_s *ins;
  FAR struct thermal_instance_s *pos;
  unsigned int max_state;
  int ret;

  ret = cdev->ops->get_max_state(cdev, &max_state);
  if (ret < 0)
    {
      therr("Get max state failed!\n");
      return ret;
    }

  list_for_every_entry(&zdev->instance_list, pos,
                       struct thermal_instance_s, zdev_node)
    {
      if (zdev == pos->zdev && cdev == pos->cdev && trip == pos->trip)
        {
          thwarn("Instance %s %s %d already exist!",
                 zdev->name, cdev->name, trip);
          return -EEXIST;
        }
    }

  ins = kmm_malloc(sizeof(struct thermal_instance_s));
  if (!ins)
    {
      return -ENOMEM;
    }

  ins->zdev = zdev;
  ins->cdev = cdev;
  ins->trip = trip;
  ins->target = THERMAL_NO_TARGET;

  if (lower == THERMAL_NO_LIMIT)
    {
      ins->lower = 0;
    }
  else
    {
      ins->lower = lower;
    }

  if (upper == THERMAL_NO_LIMIT)
    {
      ins->upper = max_state;
    }
  else
    {
      ins->upper = upper;
    }

  ins->weight = weight;

  thinfo("Adding instance zdev:%-4s cdev:%-4s h:%2u l:%2u t:%d\n",
         zdev->name, cdev->name, ins->upper, ins->lower, ins->trip);

  list_add_tail(&zdev->instance_list, &ins->zdev_node);
  list_add_tail(&cdev->instance_list, &ins->cdev_node);
  return OK;
}

static void zone_unbind_cooling(FAR struct thermal_zone_device_s *zdev,
                                int trip,
                                FAR struct thermal_cooling_device_s *cdev)
{
  FAR struct thermal_instance_s *ins;

  list_for_every_entry(&zdev->instance_list, ins,
                       struct thermal_instance_s, zdev_node)
    {
      if (zdev == ins->zdev && cdev == ins->cdev && trip == ins->trip)
        {
          list_delete(&ins->zdev_node);
          list_delete(&ins->cdev_node);

          kmm_free(ins);
          break;
        }
    }
}

static void device_bind(FAR struct thermal_zone_device_s *zdev,
                        FAR struct thermal_cooling_device_s *cdev)
{
  FAR const struct thermal_zone_map_s *map;
  FAR const struct thermal_zone_trip_s *trip;
  int ret;
  int i;
  int j;

  for (i = 0; i < zdev->params->num_maps; i++)
    {
      map = &zdev->params->maps[i];

      if (strcmp(map->cdev_name, cdev->name))
        {
          continue;
        }

      for (j = 0; j < zdev->params->num_trips; j++)
        {
          trip = &zdev->params->trips[j];
          if (strcmp(map->trip_name, trip->name))
            {
              continue;
            }

          ret = zone_bind_cooling(zdev, j, cdev, map->high, map->low,
                                  map->weight);
          if (ret < 0)
            {
              therr("Failed to bind %s and %s, trip %d\n",
                    zdev->name, cdev->name, j);
            }
        }
    }
}

static void device_unbind(FAR struct thermal_zone_device_s *zdev,
                          FAR struct thermal_cooling_device_s *cdev)
{
  int i;

  for (i = 0; i < zdev->params->num_trips; i++)
    {
      zone_unbind_cooling(zdev, i, cdev);
    }
}

static FAR struct thermal_governor_s *
find_governor_by_name(FAR const char *name)
{
  FAR struct thermal_governor_s *gov;

  if (!name)
    {
      return NULL;
    }

  /* The caller must use `g_thermal_lock` to protect governors */

  list_for_every_entry(&g_governor_list, gov, struct thermal_governor_s,
                       node)
    {
      if (!strcmp(gov->name, name))
        {
          return gov;
        }
    }

  return NULL;
}

#ifdef CONFIG_PM
static void thermal_pm_notify(FAR struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate)
{
  FAR struct thermal_zone_device_s *zdev;

  switch (pmstate)
    {
      case PM_SLEEP:
        {
          list_for_every_entry(&g_zone_dev_list, zdev,
                               struct thermal_zone_device_s, node)
            {
              work_cancel(LPWORK, &zdev->monitor);
            }
        }
        break;
      case PM_RESTORE:
      case PM_NORMAL:
      case PM_IDLE:
      case PM_STANDBY:
        {
          list_for_every_entry(&g_zone_dev_list, zdev,
                               struct thermal_zone_device_s, node)
            {
              if (zdev->enabled && work_available(&zdev->monitor))
                {
                  work_queue(LPWORK, &zdev->monitor,
                             (worker_t)thermal_zone_device_update, zdev,
                             zdev->params->polling_delay);
                }
            }
        }
        break;
      default:
        break;
    }

  return;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int thermal_zone_enable(FAR struct thermal_zone_device_s *zdev,
                        bool enabled)
{
  nxmutex_lock(&g_thermal_lock);

  if (enabled == zdev->enabled)
    {
      nxmutex_unlock(&g_thermal_lock);
      return OK;
    }

  zdev->enabled = enabled;
  work_cancel_sync(LPWORK, &zdev->monitor);

  nxmutex_unlock(&g_thermal_lock);

  thermal_zone_device_update(zdev);
  return OK;
}

int thermal_zone_get_trend(FAR struct thermal_zone_device_s *zdev)
{
  enum thermal_trend_e trend;

  if (zdev->ops->get_trend)
    {
      if (!zdev->ops->get_trend(zdev, &trend))
        {
          return trend;
        }
    }

  if (zdev->last_temperature == THERMAL_INVALID_TEMP ||
      zdev->temperature      == THERMAL_INVALID_TEMP)
    {
      trend = THERMAL_TREND_STABLE;
    }
  else if (zdev->last_temperature > zdev->temperature)
    {
      trend = THERMAL_TREND_DROPPING;
    }
  else if (zdev->last_temperature < zdev->temperature)
    {
      trend = THERMAL_TREND_RAISING;
    }
  else
    {
      trend = THERMAL_TREND_STABLE;
    }

  return trend;
}

int thermal_zone_get_trip_temp(FAR struct thermal_zone_device_s *zdev,
                               int trip,
                               FAR int *temp)
{
  if (!temp || trip >= zdev->params->num_trips)
    {
      return -EINVAL;
    }

  *temp = zdev->params->trips[trip].temp;
  return OK;
}

int thermal_zone_get_trip_type(FAR struct thermal_zone_device_s *zdev,
                               int trip,
                               FAR enum thermal_trip_type_e *type)
{
  if (!type || trip >= zdev->params->num_trips)
    {
      return -EINVAL;
    }

  *type = zdev->params->trips[trip].type;
  return OK;
}

int thermal_zone_get_trip_hyst(FAR struct thermal_zone_device_s *zdev,
                               int trip,
                               FAR int *hyst)
{
  if (!hyst || trip >= zdev->params->num_trips)
    {
      return -EINVAL;
    }

  *hyst = zdev->params->trips[trip].hyst;
  return OK;
}

/****************************************************************************
 * Name: thermal_register_governor
 *
 * Description:
 *   Register governor
 *
 * Input Parameters:
 *   gov - the struct thermal_governor_s addr
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 ****************************************************************************/

int thermal_register_governor(FAR struct thermal_governor_s *gov)
{
  FAR struct thermal_governor_s *pos;

  if (!gov || !gov->throttle)
    {
      therr("Invalid governor!\n");
      return -EINVAL;
    }

  nxmutex_lock(&g_thermal_lock);

  list_for_every_entry(&g_governor_list, pos,
                       struct thermal_governor_s, node)
    {
      if (!strcmp(gov->name, pos->name))
        {
          thwarn("Governor (%s) already exists!", gov->name);
          nxmutex_unlock(&g_thermal_lock);
          return -EEXIST;
        }
    }

  list_add_tail(&g_governor_list, &gov->node);

  thinfo("Register governor %s\n", gov->name);

  /* Default governor */

  if (!strcmp(gov->name, CONFIG_THERMAL_DEFAULT_GOVERNOR))
    {
      g_def_governor = gov;
      thinfo("Default governor %s registered!\n", g_def_governor->name);
    }

  nxmutex_unlock(&g_thermal_lock);
  return OK;
}

/****************************************************************************
 * Name: thermal_unregister_governor
 *
 * Description:
 *   Unregister governor
 *
 * Input Parameters:
 *   gov - the struct thermal_governor_s addr
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void thermal_unregister_governor(FAR struct thermal_governor_s *gov)
{
  FAR struct thermal_zone_device_s *zdev;

  if (!gov)
    {
      return;
    }

  nxmutex_lock(&g_thermal_lock);

  list_for_every_entry(&g_zone_dev_list, zdev,
                       struct thermal_zone_device_s, node)
    {
      if (zdev->governor == gov)
        {
          zone_set_governor(zdev, NULL);
        }
    }

  list_delete(&gov->node);
  nxmutex_unlock(&g_thermal_lock);
}

/****************************************************************************
 * Name: thermal_cooling_device_register
 *
 * Description:
 *   Register thermal cooling device.
 *
 * Input Parameters:
 *   name    - Name of cooling device
 *   devdata - Device driver data
 *   ops     - Operations
 *
 * Returned Value:
 *   Addr of created cooling device entry.
 ****************************************************************************/

FAR struct thermal_cooling_device_s *
thermal_cooling_device_register(FAR const char *name, void *devdata,
                          FAR const struct thermal_cooling_device_ops_s *ops)
{
  FAR struct thermal_zone_device_s *zdev;
  FAR struct thermal_cooling_device_s *cdev;

  nxmutex_lock(&g_thermal_lock);

  list_for_every_entry(&g_cooling_dev_list, cdev,
                       struct thermal_cooling_device_s, node)
    {
      if (!strcmp(cdev->name, name))
        {
          thwarn("Cooling device (%s) already exists!", name);
          nxmutex_unlock(&g_thermal_lock);
          return NULL;
        }
    }

  cdev = kmm_zalloc(sizeof(*cdev));
  if (!cdev)
    {
      therr("Cannot allocate memory for cooling device registering!\n");
      nxmutex_unlock(&g_thermal_lock);
      return NULL;
    }

  strlcpy(cdev->name, name, THERMAL_NAME_LEN);
  cdev->ops     = ops;
  cdev->devdata = devdata;

  list_initialize(&cdev->instance_list);
  list_add_tail(&g_cooling_dev_list, &cdev->node);

  list_for_every_entry(&g_zone_dev_list, zdev,
                       struct thermal_zone_device_s, node)
    {
      device_bind(zdev, cdev);
    }

  thinfo("Registered cooling device %s\n", cdev->name);
  nxmutex_unlock(&g_thermal_lock);

  return cdev;
}

/****************************************************************************
 * Name: thermal_cooling_device_unregister
 *
 * Description:
 *   Unregister thermal cooling device.
 *
 * Input Parameters:
 *   cdev - Cooling Device
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void
thermal_cooling_device_unregister(FAR struct thermal_cooling_device_s *cdev)
{
  FAR struct thermal_zone_device_s *zdev;

  /* Unbind */

  nxmutex_lock(&g_thermal_lock);

  list_delete(&cdev->node);

  list_for_every_entry(&g_zone_dev_list, zdev,
                       struct thermal_zone_device_s, node)
    {
      device_unbind(zdev, cdev);
    }

  kmm_free(cdev);
  nxmutex_unlock(&g_thermal_lock);
}

/****************************************************************************
 * Name: thermal_cooling_device_update
 *
 * Description:
 *   Update thermal cooling device.
 *
 * Input Parameters:
 *   cdev - Cooling Device.
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void thermal_cooling_device_update(FAR struct thermal_cooling_device_s *cdev)
{
  FAR struct thermal_instance_s *instance;
  unsigned int target  = THERMAL_NO_TARGET;
  unsigned int current = THERMAL_NO_TARGET;
  int ret;

  ret = cdev->ops->get_state(cdev, &current);
  if (ret < 0)
    {
      thwarn("Thermal get cooling state failed!\n");
      return;
    }

  list_for_every_entry(&cdev->instance_list, instance,
                       struct thermal_instance_s, cdev_node)
    {
      if ((instance->target != THERMAL_NO_TARGET) &&
          (instance->target > target ||
           target == THERMAL_NO_TARGET))
        {
          target = instance->target;
        }
    }

  if (target != THERMAL_NO_TARGET && target != current)
    {
      ret = cdev->ops->set_state(cdev, target);
      if (ret < 0)
        {
          thwarn("Thermal set cooling state of %s failed!\n", cdev->name);
        }
    }
}

/****************************************************************************
 * Name: thermal_zone_device_register
 *
 * Description:
 *   Register thermal zone device.
 *
 * Input Parameters:
 *   name    - Name of zone.
 *   devdata - Device driver data.
 *   ops     - Operations of zone deivce.
 *   params  - Parameter of zone device.
 *
 * Returned Value:
 *   Addr of created zone device entry.
 ****************************************************************************/

struct thermal_zone_device_s *
thermal_zone_device_register(FAR const char *name,
                             FAR void *devdata,
                             FAR const struct thermal_zone_device_ops_s *ops,
                             FAR const struct thermal_zone_params_s *params)
{
  FAR struct thermal_cooling_device_s *cdev;
  FAR struct thermal_governor_s *gov;
  FAR struct thermal_zone_device_s *pos;
  FAR struct thermal_zone_device_s *zdev;
  int i;

  if (!ops || !ops->get_temp)
    {
      therr("Invalid zone operations!\n");
      return NULL;
    }

  for (i = 0; i < params->num_trips; i++)
    {
      if (params->trips[i].type >= THERMAL_TRIP_TYPE_MAX)
        {
          therr("Invalid trip type (%d)!\n", params->trips[i].type);
          return NULL;
        }
    }

  nxmutex_lock(&g_thermal_lock);

  list_for_every_entry(&g_zone_dev_list, pos,
                       struct thermal_zone_device_s, node)
    {
      if (!strcmp(name, pos->name))
        {
          thwarn("Zone device (%s) already exists!", name);
          nxmutex_unlock(&g_thermal_lock);
          return NULL;
        }
    }

  zdev = kmm_zalloc(sizeof(struct thermal_zone_device_s));
  if (!zdev)
    {
      nxmutex_unlock(&g_thermal_lock);
      return NULL;
    }

  zdev->ops     = ops;
  zdev->devdata = devdata;
  zdev->enabled = true;

  strlcpy(zdev->name, name, THERMAL_NAME_LEN);

  zdev->params      = params;
  zdev->temperature = THERMAL_INVALID_TEMP;

  list_initialize(&zdev->instance_list);

  /* Set governor */

  gov = find_governor_by_name(zdev->params->gov_name);
  zone_set_governor(zdev, gov ? gov : g_def_governor);

  thinfo("Set governor of zone %s to %s.\n", zdev->name,
         zdev->governor ? zdev->governor->name : "");

  list_add_tail(&g_zone_dev_list, &zdev->node);

  list_for_every_entry(&g_cooling_dev_list, cdev,
                       struct thermal_cooling_device_s, node)
    {
      device_bind(zdev, cdev);
    }

#ifdef CONFIG_THERMAL_PROCFS
  thermal_zone_procfs_register(zdev);
#endif

  nxmutex_unlock(&g_thermal_lock);

  thinfo("Registered zone device %s\n", zdev->name);
  thermal_zone_device_update(zdev);
  return zdev;
}

/****************************************************************************
 * Name: thermal_zone_device_unregister
 *
 * Description:
 *   Unregister thermal zone device.
 *
 * Input Parameters:
 *   zdev - Zone Device
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void thermal_zone_device_unregister(FAR struct thermal_zone_device_s *zdev)
{
  FAR struct thermal_cooling_device_s *cdev;

  /* Disable Zone */

  thermal_zone_enable(zdev, false);

  /* Unbind */

  nxmutex_lock(&g_thermal_lock);

  list_for_every_entry(&g_cooling_dev_list, cdev,
                       struct thermal_cooling_device_s, node)
    {
      device_unbind(zdev, cdev);
    }

  list_delete(&zdev->node);

#ifdef CONFIG_THERMAL_PROCFS
  thermal_zone_procfs_unregister(zdev);
#endif

  zone_set_governor(zdev, NULL);
  kmm_free(zdev);
  nxmutex_unlock(&g_thermal_lock);
}

/****************************************************************************
 * Name: thermal_zone_device_update
 *
 * Description:
 *   Update thermal zone device.
 *   Get temperature from sensor and throttle if necessary.
 *
 * Input Parameters:
 *   zdev - Zone Device
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void thermal_zone_device_update(FAR struct thermal_zone_device_s *zdev)
{
  int trip_high = INT_MAX;
  int trip_low  = INT_MIN;
  int trip;
  int temp;
  int ret;

  nxmutex_lock(&g_thermal_lock);

  /* Update termerature */

  if (!zdev->enabled)
    {
      goto unlock;
    }

  ret = zdev->ops->get_temp(zdev, &temp);
  if (ret < 0)
    {
      therr("Failed to get temperature from zone %s \n", zdev->name);
      goto unlock;
    }

  zdev->last_temperature = zdev->temperature;
  zdev->temperature = temp;

  for (trip = 0; trip < zdev->params->num_trips; trip++)
    {
      enum thermal_trip_type_e type;
      int temp_low;
      int hyst;

      thermal_zone_get_trip_temp(zdev, trip, &temp);
      thermal_zone_get_trip_hyst(zdev, trip, &hyst);
      thermal_zone_get_trip_type(zdev, trip, &type);

      if (zdev->temperature < temp && trip_high > temp)
        {
          trip_high = temp;
        }

      temp_low = temp - hyst;

      if (zdev->temperature > temp_low && trip_low < temp_low)
        {
          trip_low  = temp_low;
        }

      /* Critical */

      if (type == THERMAL_CRITICAL && zdev->temperature >= temp)
        {
          therr("Thermal critical (%d), resetting...\n", zdev->temperature);
#ifdef CONFIG_BOARDCTL_RESET
          boardctl(BOARDIOC_RESET, BOARDIOC_SOFTRESETCAUSE_THERMAL);
#endif
        }
      else if (zdev->governor)
        {
          zdev->governor->throttle(zdev, trip);
        }
      else if(g_def_governor)
        {
          g_def_governor->throttle(zdev, trip);
        }
      else
        {
          therr("No valid governor!\n");
        }
    }

  if (zdev->ops->set_trips)
    {
      ret = zdev->ops->set_trips(zdev, trip_low, trip_high);
      if (ret < 0)
        {
          thwarn("Set trip points (l:%d, h:%d) for %s failed\n",
                 trip_low, trip_high, zdev->name);
        }
    }

  work_queue(LPWORK, &zdev->monitor, (worker_t)thermal_zone_device_update,
             zdev, zdev->params->polling_delay);

unlock:
  nxmutex_unlock(&g_thermal_lock);
}

/****************************************************************************
 * Name: thermal_init
 *
 * Description:
 *   Init thermal framework
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 ****************************************************************************/

int thermal_init(void)
{
  int ret = OK;

#ifdef CONFIG_THERMAL_GOVERNOR_STEP_WISE
  ret = thermal_register_step_wise_governor();
  if (ret < 0)
    {
      therr("Register step wise governor failed!\n");
      return ret;
    }
#endif

#ifdef CONFIG_THERMAL_DUMMY
  ret = thermal_dummy_init();
  if (ret < 0)
    {
      therr("Dummy driver init failed!\n");
      return ret;
    }
#endif

#ifdef CONFIG_THERMAL_CDEV_CPUFREQ
  if (NULL == thermal_cpufreq_cooling_register())
    {
      return -ENOTSUP;
    }
#endif

#ifdef CONFIG_PM
  ret = pm_register(&g_thermal_pm_cb);
  if (ret < 0)
    {
      therr("Register suspend notifier failed!\n");
      return ret;
    }
#endif

  return ret;
}
