/****************************************************************************
 * include/nuttx/thermal.h
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

#ifndef __INCLUDE_NUTTX_THERMAL_H
#define __INCLUDE_NUTTX_THERMAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Generic */

#define THERMAL_NAME_LEN 32

/* Trips */

#define THERMAL_NO_LIMIT 0

/* Temperature */

#define THERMAL_INVALID_TEMP INT_MIN

/* Cooling State */

#define THERMAL_TARGET_MIN   0
#define THERMAL_TARGET_MAX   (UINT_MAX - 1)
#define THERMAL_NO_TARGET    UINT_MAX

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct thermal_cooling_device_ops_s;
struct thermal_zone_device_ops_s;
struct thermal_zone_device_s;
struct thermal_zone_params_s;

enum thermal_trend_e
{
  THERMAL_TREND_RAISING,
  THERMAL_TREND_DROPPING,
  THERMAL_TREND_STABLE,
};

enum thermal_trip_type_e
{
  THERMAL_NORMAL,
  THERMAL_HOT,
  THERMAL_CRITICAL,
  THERMAL_TRIP_TYPE_MAX,
};

struct thermal_governor_s
{
  struct list_node node;

  FAR const char *name;

  CODE int  (*bind_to_tz)    (FAR struct thermal_zone_device_s *zdev);
  CODE int  (*throttle)      (FAR struct thermal_zone_device_s *zdev,
                              int trip);
  CODE void (*unbind_from_tz)(FAR struct thermal_zone_device_s *zdev);
};

struct thermal_cooling_device_s
{
  struct list_node node;

  char name[THERMAL_NAME_LEN];
  FAR void *devdata;
  FAR const struct thermal_cooling_device_ops_s *ops;

  struct list_node instance_list;
};

struct thermal_zone_device_s
{
  struct list_node node;

  char name[THERMAL_NAME_LEN];
  bool enabled;

  FAR void *devdata;

  int last_temperature;
  int temperature;

  FAR const struct thermal_zone_device_ops_s *ops;
  FAR struct thermal_governor_s *governor;

  FAR const struct thermal_zone_params_s *params;
  struct work_s monitor;

  struct list_node instance_list;
};

struct thermal_cooling_device_ops_s
{
  CODE int (*set_state)    (FAR struct thermal_cooling_device_s *cdev,
                            unsigned int state);
  CODE int (*get_state)    (FAR struct thermal_cooling_device_s *cdev,
                            FAR unsigned int *state);
  CODE int (*get_max_state)(FAR struct thermal_cooling_device_s *cdev,
                            FAR unsigned int *state);
};

struct thermal_zone_device_ops_s
{
  CODE int  (*get_temp)    (FAR struct thermal_zone_device_s *zdev,
                            FAR int *temp);
  CODE int  (*get_trend)   (FAR struct thermal_zone_device_s *zdev,
                            FAR enum thermal_trend_e *trend);
  CODE int  (*set_trips)   (FAR struct thermal_zone_device_s *zdev,
                            int low, int high);
};

struct thermal_zone_map_s
{
  FAR const char *trip_name;

  FAR const char *cdev_name;
  unsigned int low;
  unsigned int high;

  unsigned int weight;
};

struct thermal_zone_trip_s
{
  FAR const char *name;
  unsigned int temp;
  unsigned int hyst;
  enum thermal_trip_type_e type;
};

struct thermal_zone_params_s
{
  FAR const char *gov_name;
  int polling_delay;
  FAR const struct thermal_zone_trip_s *trips;
  int num_trips;
  FAR const struct thermal_zone_map_s *maps;
  int num_maps;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Governor */

int  thermal_register_governor  (FAR struct thermal_governor_s *gov);
void thermal_unregister_governor(FAR struct thermal_governor_s *gov);

/* Cooling Device */

FAR struct thermal_cooling_device_s *
thermal_cooling_device_register(FAR const char *name, void *devdata,
                         FAR const struct thermal_cooling_device_ops_s *ops);
void
thermal_cooling_device_unregister(
                               FAR struct thermal_cooling_device_s *cdev);

/* Zone Device */

FAR struct thermal_zone_device_s *
thermal_zone_device_register(FAR const char *name, FAR void *devdata,
                             FAR const struct thermal_zone_device_ops_s *ops,
                             FAR const struct thermal_zone_params_s *params);
void
thermal_zone_device_unregister(
                             FAR struct thermal_zone_device_s *zdev);
void
thermal_zone_device_update  (FAR struct thermal_zone_device_s *zdev);

/* Thermal Framework initialization */

int thermal_init(void);

#endif /* __INCLUDE_NUTTX_THERMAL_H */
