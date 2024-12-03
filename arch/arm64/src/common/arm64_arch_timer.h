/****************************************************************************
 * arch/arm64/src/common/arm64_arch_timer.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_ARCH_TIMER_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_ARCH_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/oneshot.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_oneshot_initialize
 *
 * Description:
 *   This function initialize generic timer hardware module
 *   and return an instance of a "lower half" timer interface.
 *
 * Returned Value:
 *   On success, a non-NULL oneshot_lowerhalf_s is returned to the caller.
 *   In the event of any failure, a NULL value is returned.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *arm64_oneshot_initialize(void);

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_ARCH_TIMER_H */
