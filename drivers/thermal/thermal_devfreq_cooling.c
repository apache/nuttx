/****************************************************************************
 * drivers/thermal/thermal_devfreq_cooling.c
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

#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/devfreq.h>
#include <nuttx/kmalloc.h>

#include "thermal_core.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct devfreq_cooling_device_s
{
  FAR const uint32_t *table;
  FAR struct devfreq_s *devfreq;
  FAR struct qos_request_s *qos;
  unsigned int cur_state;
  unsigned int max_state;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int devfreq_get_max_state(FAR struct thermal_cooling_device_s *cdev,
                                 FAR unsigned int *state);
static int devfreq_get_state    (FAR struct thermal_cooling_device_s *cdev,
                                 FAR unsigned int *state);
static int devfreq_set_state    (FAR struct thermal_cooling_device_s *cdev,
                                 unsigned int state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct thermal_cooling_device_ops_s g_devfreq_cdev_ops =
{
  .set_state     = devfreq_set_state,
  .get_state     = devfreq_get_state,
  .get_max_state = devfreq_get_max_state,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devfreq_cooling_freq
 *
 * Description:
 *   The nth usable frequency of a devfreq table, counting up from the
 *   lowest.  DEVFREQ_ENTRY_INVALID marks a frequency the device cannot be
 *   held at, so those entries are not counted: n selects among the
 *   frequencies a cooling state can actually install, not table positions.
 *
 * Input Parameters:
 *   table - devfreq frequency table, terminated by DEVFREQ_ENTRY_END
 *   n     - which usable entry to return, zero being the lowest
 *
 * Returned Value:
 *   The frequency in kHz, or DEVFREQ_ENTRY_INVALID if the table holds fewer
 *   than n + 1 usable entries.
 *
 ****************************************************************************/

static uint32_t devfreq_cooling_freq(FAR const uint32_t *table,
                                     unsigned int n)
{
  unsigned int i;

  for (i = 0; table[i] != DEVFREQ_ENTRY_END; i++)
    {
      if (table[i] == DEVFREQ_ENTRY_INVALID)
        {
          continue;
        }

      if (n == 0)
        {
          return table[i];
        }

      n--;
    }

  return DEVFREQ_ENTRY_INVALID;
}

static int devfreq_get_max_state(FAR struct thermal_cooling_device_s *cdev,
                                 FAR unsigned int *state)
{
  FAR struct devfreq_cooling_device_s *devfreq_cdev = cdev->devdata;

  *state = devfreq_cdev->max_state;
  return OK;
}

static int devfreq_get_state(FAR struct thermal_cooling_device_s *cdev,
                             FAR unsigned int *state)
{
  FAR struct devfreq_cooling_device_s *devfreq_cdev = cdev->devdata;

  *state = devfreq_cdev->cur_state;
  return OK;
}

static int devfreq_set_state(FAR struct thermal_cooling_device_s *cdev,
                             unsigned int state)
{
  FAR struct devfreq_cooling_device_s *devfreq_cdev = cdev->devdata;
  uint32_t ceiling;
  int ret;

  if (state > devfreq_cdev->max_state)
    {
      return -EINVAL;
    }

  /* The cooling state counts upwards as the device is asked to do less,
   * and the table climbs the other way, so the two are read from opposite
   * ends: state zero leaves the top entry available, and the highest state
   * holds the device at the bottom one.
   */

  ceiling = devfreq_cooling_freq(devfreq_cdev->table,
                                 devfreq_cdev->max_state - state);
  if (ceiling == DEVFREQ_ENTRY_INVALID)
    {
      therr("No frequency for cooling state %u!\n", state);
      return -EINVAL;
    }

  thinfo("devfreq cooling state %u, ceiling %" PRIu32 " kHz\n",
         state, ceiling);

  /* A ceiling and no floor.  Where another request wants more than this
   * allows, a driver carrying DEVFREQ_CONFLICT_PREFER_LOW resolves it in
   * favour of the ceiling, which is what protects the device.
   */

  if (devfreq_cdev->qos == NULL)
    {
      devfreq_cdev->qos = devfreq_qos_add_request(devfreq_cdev->devfreq,
                                                  0, ceiling);
      if (devfreq_cdev->qos == NULL)
        {
          therr("Add qos request failed!\n");
          return -EINVAL;
        }
    }
  else
    {
      ret = devfreq_qos_update_request(devfreq_cdev->devfreq,
                                       devfreq_cdev->qos, 0, ceiling);
      if (ret < 0)
        {
          therr("Update qos request failed!\n");
          return ret;
        }
    }

  devfreq_cdev->cur_state = state;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thermal_devfreq_cooling_register
 *
 * Description:
 *   Register a cooling device over a devfreq device, which the thermal
 *   framework then throttles by capping its frequency.
 *
 *   The devfreq device must already be registered, since it is found by
 *   name.  The cooling device may be registered before or after the zones
 *   that use it; the core binds them either way.
 *
 * Input Parameters:
 *   devfreq_name - name the devfreq device was registered under
 *   cdev_name    - name for the cooling device, matched against the
 *                  cdev_name of a zone's cooling map
 *
 * Returned Value:
 *   Addr of created cooling device entry
 ****************************************************************************/

FAR struct thermal_cooling_device_s *
thermal_devfreq_cooling_register(FAR const char *devfreq_name,
                                 FAR const char *cdev_name)
{
  FAR struct devfreq_cooling_device_s *devfreq_cdev;
  FAR struct thermal_cooling_device_s *cdev;
  FAR struct devfreq_s *devfreq;
  FAR const uint32_t *table;
  unsigned int count;
  unsigned int i;

  devfreq = devfreq_find_by_name(devfreq_name);
  if (devfreq == NULL)
    {
      therr("No devfreq device named %s!\n", devfreq_name);
      return NULL;
    }

  table = devfreq->freq_table;
  if (table == NULL)
    {
      therr("Get devfreq table failed!\n");
      return NULL;
    }

  /* Count what the device can be held at, not what the table holds: an
   * entry of DEVFREQ_ENTRY_INVALID is a hole the driver has punched and
   * cannot be installed as a ceiling, so it earns no cooling state.
   */

  for (count = 0, i = 0; table[i] != DEVFREQ_ENTRY_END; i++)
    {
      if (table[i] != DEVFREQ_ENTRY_INVALID)
        {
          count++;
        }
    }

  if (count < 2)
    {
      therr("Invalid devfreq table!\n");
      return NULL;
    }

  devfreq_cdev = kmm_zalloc(sizeof(*devfreq_cdev));
  if (devfreq_cdev == NULL)
    {
      therr("No memory for devfreq cooling device registering!\n");
      return NULL;
    }

  devfreq_cdev->table     = table;
  devfreq_cdev->devfreq   = devfreq;
  devfreq_cdev->max_state = count - 1;
  thinfo("max level of %s is %u\n", devfreq_name, devfreq_cdev->max_state);

  cdev = thermal_cooling_device_register(cdev_name, devfreq_cdev,
                                         &g_devfreq_cdev_ops);
  if (cdev == NULL)
    {
      kmm_free(devfreq_cdev);
    }

  return cdev;
}

/****************************************************************************
 * Name: thermal_devfreq_cooling_unregister
 *
 * Description:
 *   Unregister devfreq cooling device
 *
 * Input Parameters:
 *   cdev - Addr of devfreq cooling device entry
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void
thermal_devfreq_cooling_unregister(FAR struct thermal_cooling_device_s *cdev)
{
  FAR struct devfreq_cooling_device_s *devfreq_cdev;
  int ret;

  devfreq_cdev = cdev->devdata;

  if (devfreq_cdev->qos)
    {
      ret = devfreq_qos_remove_request(devfreq_cdev->devfreq,
                                       devfreq_cdev->qos);
      if (ret < 0)
        {
          therr("Remove devfreq qos failed: %d!\n", ret);
        }
    }

  thermal_cooling_device_unregister(cdev);
  kmm_free(devfreq_cdev);
}
