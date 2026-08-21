/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_hwspinlock.c
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

#include "arm_internal.h"
#include "hardware/cxd56_sph.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool cxd56_hwspinlock_trylock(struct hwspinlock_dev_s *dev);
static void cxd56_hwspinlock_unlock(struct hwspinlock_dev_s *dev);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct hwspinlock_ops_s g_cxd56_hwspinlock_ops =
{
  .trylock = cxd56_hwspinlock_trylock,
  .unlock  = cxd56_hwspinlock_unlock
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool cxd56_hwspinlock_trylock(struct hwspinlock_dev_s *dev)
{
  uint32_t sphlocked = ((up_cpu_index() + 2) << 16) | 0x1;

  putreg32(REQ_LOCK, CXD56_SPH_REQ(dev->id));

  return getreg32(CXD56_SPH_STS(dev->id)) == sphlocked;
}

static void cxd56_hwspinlock_unlock(struct hwspinlock_dev_s *dev)
{
  putreg32(REQ_UNLOCK, CXD56_SPH_REQ(dev->id));
}
