/****************************************************************************
 * drivers/power/greedy_governor.h
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

#ifndef __DRIVERS_POWER_GREEDY_GOVERNER_H
#define __DRIVERS_POWER_GREEDY_GOVERNER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/input/ioctl.h>
#include <nuttx/power/pm.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pm_greedy_governor_register
 *
 * Description:
 *   Return the greedy governor instance.
 *
 * Returned Value:
 *   A pointer to the governor struct. Otherwise NULL is returned on error.
 *
 ****************************************************************************/

FAR const struct pm_governor_s *pm_greedy_governor_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_POWER_GREEDY_GOVERNER_H */
