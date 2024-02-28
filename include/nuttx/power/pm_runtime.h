/****************************************************************************
 * include/nuttx/power/pm_runtime.h
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

#ifndef __INCLUDE_NUTTX_POWER_PM_RUNTIME_H
#define __INCLUDE_NUTTX_POWER_PM_RUNTIME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>

#ifdef CONFIG_PM_RUNTIME

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  RPM_ACTIVE = 0,
  RPM_SUSPENDED,
  RPM_SUSPENDING,
} rpm_state_e;

struct pm_runtime_ops_s;
struct pm_runtime_s
{
  rmutex_t lock;
  unsigned int use_count;
  rpm_state_e state;
  unsigned int suspend_delay;
  struct work_s suspend_work;
  FAR const struct pm_runtime_ops_s *ops;
};

struct pm_runtime_ops_s
{
  CODE int (*runtime_suspend)(FAR struct pm_runtime_s *rpm);
  CODE int (*runtime_resume)(FAR struct pm_runtime_s *rpm);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void pm_runtime_init(FAR struct pm_runtime_s *rpm, rpm_state_e state,
                     FAR struct pm_runtime_ops_s *rops);
int pm_runtime_get(FAR struct pm_runtime_s *rpm);
int pm_runtime_put(FAR struct pm_runtime_s *rpm);
int pm_runtime_put_autosuspend(FAR struct pm_runtime_s *rpm);
void pm_runtime_set_autosuspend_delay(FAR struct pm_runtime_s *rpm,
                                      unsigned int delay);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PM_RUNTIME */
#endif /* __INCLUDE_NUTTX_POWER_PM_RUNTIME_H */
