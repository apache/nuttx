/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_atomic.c
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

#include <nuttx/hwspinlock/hwspinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPH_SMP  14

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct hwspinlock_ops_s g_cxd56_hwspinlock_ops;

struct hwspinlock_dev_s g_atomic_hwspinlock =
{
  .id       = SPH_SMP,
  .priority = 0,
  .ops      = &g_cxd56_hwspinlock_ops
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
