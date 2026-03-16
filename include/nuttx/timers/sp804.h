/****************************************************************************
 * include/nuttx/timers/sp804.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_SP804_H
#define __INCLUDE_NUTTX_TIMERS_SP804_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/oneshot.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: sp804_initialize
 *
 * Description:
 *   Initialize the SP804 dual timer and return a oneshot lowerhalf
 *   driver instance that can be bound to the OS alarm infrastructure
 *   via up_alarm_set_lowerhalf().
 *
 * Input Parameters:
 *   base - Physical base address of the SP804 timer block.
 *   irq  - IRQ number associated with the timer interrupt.
 *   freq - Input clock frequency in Hz.
 *
 * Returned Value:
 *   On success, a non-NULL oneshot lowerhalf pointer is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *
sp804_initialize(uintptr_t base, int irq, uint32_t freq);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_SP804_H */
