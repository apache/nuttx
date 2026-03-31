/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_pwm.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership.  The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HT32F491X3_PWM_H
#define __ARCH_ARM_SRC_HT32F491X3_HT32F491X3_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/timers/pwm.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ht32f491x3_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper-half PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer instance.
 *
 * Returned Value:
 *   On success, a pointer to the HT32 lower-half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *ht32f491x3_pwminitialize(int timer);

#endif /* __ARCH_ARM_SRC_HT32F491X3_HT32F491X3_PWM_H */
