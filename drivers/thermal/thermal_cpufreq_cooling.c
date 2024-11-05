/****************************************************************************
 * drivers/thermal/thermal_cpufreq_cooling.c
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

#include <nuttx/cpufreq.h>
#include <nuttx/kmalloc.h>

#include "thermal_core.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cpufreq_cooling_device_s
{
  FAR const struct cpufreq_frequency_table *table;
  FAR struct cpufreq_policy *policy;
  FAR struct cpufreq_qos *qos;
  unsigned int cur_state;
  unsigned int max_state;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cpufreq_get_max_state(FAR struct thermal_cooling_device_s *cdev,
                                 FAR unsigned int *state);
static int cpufreq_get_state    (FAR struct thermal_cooling_device_s *cdev,
                                 FAR unsigned int *state);
static int cpufreq_set_state    (FAR struct thermal_cooling_device_s *cdev,
                                 unsigned int state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct thermal_cooling_device_ops_s g_cpufreq_cdev_ops =
{
  .set_state     = cpufreq_set_state,
  .get_state     = cpufreq_get_state,
  .get_max_state = cpufreq_get_max_state,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cpufreq_get_max_state(FAR struct thermal_cooling_device_s *cdev,
                                 FAR unsigned int *state)
{
  struct cpufreq_cooling_device_s *cpufreq_cdev = cdev->devdata;

  *state = cpufreq_cdev->max_state;
  return OK;
}

static int cpufreq_get_state(FAR struct thermal_cooling_device_s *cdev,
                             FAR unsigned int *state)
{
  struct cpufreq_cooling_device_s *cpufreq_cdev = cdev->devdata;

  *state = cpufreq_cdev->cur_state;
  return OK;
}

static int cpufreq_set_state(FAR struct thermal_cooling_device_s *cdev,
                             unsigned int state)
{
  struct cpufreq_cooling_device_s *cpufreq_cdev = cdev->devdata;
  unsigned int index = cpufreq_cdev->max_state - state;
  int ret;

  thinfo("CPU Freq cooling %u %u \n",
                                   cpufreq_cdev->table[index].frequency,
                                   cpufreq_cdev->table[index + 1].frequency);

  if (cpufreq_cdev->qos == NULL)
    {
      cpufreq_cdev->qos = cpufreq_qos_add_request(
                                   cpufreq_cdev->policy,
                                   cpufreq_cdev->table[index].frequency,
                                   cpufreq_cdev->table[index + 1].frequency);
      if (!cpufreq_cdev->qos)
        {
          therr("Add qos request failed!");
          return -EINVAL;
        }
    }
  else
    {
      ret = cpufreq_qos_update_request(
                                   cpufreq_cdev->qos,
                                   cpufreq_cdev->table[index].frequency,
                                   cpufreq_cdev->table[index + 1].frequency);
      if (ret < 0)
        {
          therr("Update qos request failed!");
          return ret;
        }
    }

  cpufreq_cdev->cur_state = state;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thermal_cpufreq_cooling_register
 *
 * Description:
 *   Register cpufreq cooling device
 *
 * Input Parameters:
 *   policy - cpufreq policy
 *
 * Returned Value:
 *   Addr of created cooling device entry
 ****************************************************************************/

FAR struct thermal_cooling_device_s *thermal_cpufreq_cooling_register(void)
{
  FAR struct cpufreq_cooling_device_s *cpufreq_cdev;
  FAR const struct cpufreq_frequency_table *table;
  FAR struct thermal_cooling_device_s *cdev;
  FAR struct cpufreq_driver **driver;
  FAR struct cpufreq_policy *policy;
  unsigned int count;

  policy = cpufreq_policy_get();
  if (policy == NULL)
    {
      therr("Get cpufreq policy failed!\n");
      return NULL;
    }

  driver = (FAR struct cpufreq_driver **)policy;

  table = (*driver)->get_table(policy);
  if (table == NULL)
    {
      therr("Get cpufreq table failed!\n");
      return NULL;
    }

  for (count = 0; table[count].frequency != CPUFREQ_TABLE_END; count++)
    {
    }

  if (count < 2)
    {
      therr("Invalid cpufreq table!\n");
      return NULL;
    }

  cpufreq_cdev = kmm_zalloc(sizeof(*cpufreq_cdev));
  if (cpufreq_cdev == NULL)
    {
      therr("No memory for cpufreq cooling device registering!\n");
      return NULL;
    }

  cpufreq_cdev->table = table;
  cpufreq_cdev->policy = policy;
  cpufreq_cdev->max_state = count - 2;
  thinfo("max level of cpufreq is %d \n", cpufreq_cdev->max_state);

  cdev = thermal_cooling_device_register("cpufreq", cpufreq_cdev,
                                         &g_cpufreq_cdev_ops);
  if (cdev == NULL)
    {
      kmm_free(cpufreq_cdev);
    }

  return cdev;
}

/****************************************************************************
 * Name: thermal_cpufreq_cooling_unregister
 *
 * Description:
 *   Unregister cpufreq cooling device
 *
 * Input Parameters:
 *   cdev - Addr of cpufre cooling devcie entry
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void
thermal_cpufreq_cooling_unregister(FAR struct thermal_cooling_device_s *cdev)
{
  struct cpufreq_cooling_device_s *cpufreq_cdev;
  int ret;

  cpufreq_cdev = cdev->devdata;

  if (cpufreq_cdev->qos)
    {
      ret = cpufreq_qos_remove_request(cpufreq_cdev->qos);
      if (ret < 0)
        {
          thinfo("ret=%d\n", ret);
          therr("Remove cpufreq qos failed!\n");
        }
    }

  thermal_cooling_device_unregister(cdev);
  kmm_free(cpufreq_cdev);
}
