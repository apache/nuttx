/****************************************************************************
 * include/nuttx/power/consumer.h
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

#ifndef __INCLUDE_NUTTX_POWER_CONSUMER_H
#define __INCLUDE_NUTTX_POWER_CONSUMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <semaphore.h>
#include <nuttx/power/regulator.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

FAR struct regulator_s *regulator_get(const char *id);
void regulator_put(FAR struct regulator_s *regulator);
int regulator_is_enabled(FAR struct regulator_s *regulator);
int regulator_enable(FAR struct regulator_s *regulator);
int regulator_enable_delay(FAR struct regulator_s *regulator, int ms);
int regulator_disable(FAR struct regulator_s *regulator);
int regulator_disable_deferred(FAR struct regulator_s *regulator, int ms);
int regulator_set_voltage(FAR struct regulator_s *regulator, int min_uv,
                          int max_uv);
int regulator_get_voltage(FAR struct regulator_s *regulator);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_POWER_CONSUMER_H */
