/****************************************************************************
 * drivers/thermal/thermal_core.h
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

#ifndef __DRIVERS_THERMAL_THERMAL_CORE_H
#define __DRIVERS_THERMAL_THERMAL_CORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

 #include <nuttx/thermal.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct thermal_instance_s
{
  FAR struct thermal_cooling_device_s *cdev;
  FAR struct thermal_zone_device_s    *zdev;

  int  trip;

  /* Cooling State */

  unsigned int target;   /* Expected Cooling State */
  unsigned int upper;    /* The Maximum cooling state for this trip point */
  unsigned int lower;    /* Minimum cooling state */
  unsigned int weight;

  struct list_node cdev_node;
  struct list_node zdev_node;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Cooling Device */

void
thermal_cooling_device_update (FAR struct thermal_cooling_device_s *cdev);
#ifdef CONFIG_THERMAL_CDEV_CPUFREQ
FAR struct thermal_cooling_device_s *thermal_cpufreq_cooling_register(void);
void thermal_cpufreq_cooling_unregister(
                                  FAR struct thermal_cooling_device_s *cdev);
#endif /* CONFIG_THERMAL_CDEV_CPUFREQ */

/* Zone Device */

int thermal_zone_enable       (FAR struct thermal_zone_device_s *zdev,
                               bool enabled);
int thermal_zone_get_trend    (FAR struct thermal_zone_device_s *zdev);
int thermal_zone_get_trip_temp(FAR struct thermal_zone_device_s *zdev,
                               int trip,
                               FAR int *temp);
int thermal_zone_get_trip_type(FAR struct thermal_zone_device_s *zdev,
                               int trip,
                               FAR enum thermal_trip_type_e *type);
int thermal_zone_get_trip_hyst(FAR struct thermal_zone_device_s *zdev,
                               int trip,
                               FAR int *hyst);

/* Governor */

int thermal_register_step_wise_governor(void);

/* ProcFS */

#ifdef CONFIG_THERMAL_PROCFS
int thermal_zone_procfs_register(FAR struct thermal_zone_device_s *zdev);
void thermal_zone_procfs_unregister(FAR struct thermal_zone_device_s *zdev);
#endif

/* Dummy Driver */

#ifdef CONFIG_THERMAL_DUMMY
int thermal_dummy_init(void);
#endif

#endif /* __DRIVERS_THERMAL_THERMAL_CORE_H */
