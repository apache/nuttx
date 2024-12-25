/****************************************************************************
 * drivers/thermal/thermal_step_wise.c
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

#include <debug.h>

#include "thermal_core.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static unsigned int get_target_state(FAR struct thermal_instance_s *instance,
                                     enum thermal_trend_e trend,
                                     bool throttle);
static int step_wise_throttle(FAR struct thermal_zone_device_s *zdev,
                              int trip);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct thermal_governor_s g_step_wise_governor =
{
  .name = "step_wise",
  .throttle = step_wise_throttle,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline unsigned int
validate_state(FAR struct thermal_instance_s *instance, bool throttle,
               unsigned int state, int value)
{
  if ((state == THERMAL_TARGET_MIN && value == -1) ||
      (state == THERMAL_TARGET_MAX && value == 1))
    {
      value = 0;
    }

  if (state == THERMAL_NO_TARGET)
    {
      state = 0;
    }

  state += value;

  if (state > instance->upper)
    {
      state = instance->upper;
    }
  else if(state < instance->lower && throttle)
    {
      state = instance->lower;
    }

  return state;
}

static unsigned int get_target_state(FAR struct thermal_instance_s *instance,
                                     enum thermal_trend_e trend,
                                     bool throttle)
{
  FAR struct thermal_cooling_device_s *cdev = instance->cdev;
  unsigned int cur_state = instance->target;

  if (!cdev->ops || !cdev->ops->get_state)
    {
      return THERMAL_NO_TARGET;
    }

  if (cur_state == THERMAL_NO_TARGET)
    {
      if (throttle)
        {
          return validate_state(instance, throttle, cur_state, 1);
        }

      return THERMAL_NO_TARGET;
    }

  /* Update Cooling State */

  switch (trend)
    {
      case THERMAL_TREND_RAISING:
        if (throttle)
          {
            return validate_state(instance, throttle, cur_state, 1);
          }
        break;

      case THERMAL_TREND_DROPPING:
        if (!throttle)
          {
            return validate_state(instance, throttle, cur_state, -1);
          }
        break;

      case THERMAL_TREND_STABLE:
        if (throttle)
          {
            enum thermal_trip_type_e type;
            int ret;

            ret = thermal_zone_get_trip_type(instance->zdev, instance->trip,
                                             &type);
            if (ret >= 0 && type == THERMAL_HOT)
              {
                return validate_state(instance, throttle, cur_state, 1);
              }
          }
        break;

      default:
        break;
    }

  return THERMAL_NO_TARGET;
}

/* step_wise */

static int step_wise_throttle(FAR struct thermal_zone_device_s *zdev,
                              int trip)
{
  FAR struct thermal_instance_s *instance;
  enum thermal_trend_e trend;
  unsigned int next_state;
  bool throttle = false;
  int hyst_temp;
  int trip_temp;

  thermal_zone_get_trip_hyst(zdev, trip, &hyst_temp);
  thermal_zone_get_trip_temp(zdev, trip, &trip_temp);

  trend = thermal_zone_get_trend(zdev);

  if (zdev->temperature > trip_temp)
    {
      throttle = true;
    }
  else if (zdev->temperature > trip_temp - hyst_temp)
    {
      return OK;
    }

  list_for_every_entry(&zdev->instance_list, instance,
                       struct thermal_instance_s, zdev_node)
    {
      if (instance->trip != trip)
        {
          continue;
        }

      next_state = get_target_state(instance, trend, throttle);

      if (next_state == THERMAL_NO_TARGET || next_state == instance->target)
        {
          continue;
        }

      instance->target = next_state;
      thermal_cooling_device_update(instance->cdev);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int thermal_register_step_wise_governor(void)
{
  return thermal_register_governor(&g_step_wise_governor);
}
