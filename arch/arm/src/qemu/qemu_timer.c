/****************************************************************************
 * arch/arm/src/qemu/qemu_timer.c
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

#include <nuttx/timers/arch_alarm.h>

#include "arm_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CYCLES_PER_SEC  62500000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_initialize(void)
{
  up_alarm_set_lowerhalf(arm_timer_initialize(CYCLES_PER_SEC));
}
