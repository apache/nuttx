/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_testset.c
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

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "hardware/cxd56_sph.h"
#include "cxd56_sph.h"

#include "arm_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPH_SMP 14  /* Use hardware semaphore #14 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_testset
 *
 * Description:
 *   Perform and atomic test and set operation on the provided spinlock.
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   lock - The address of spinlock object.
 *
 * Returned Value:
 *   The spinlock is always locked upon return.  The value of previous value
 *   of the spinlock variable is returned, either SP_LOCKED if the spinlock
 *   as previously locked (meaning that the test-and-set operation failed to
 *   obtain the lock) or SP_UNLOCKED if the spinlock was previously unlocked
 *   (meaning that we successfully obtained the lock)
 *
 ****************************************************************************/

spinlock_t up_testset(volatile FAR spinlock_t *lock)
{
  spinlock_t ret;
  uint32_t sphlocked = ((up_cpu_index() + 2) << 16) | 0x1;

  /* Lock hardware semaphore */

  do
    {
      putreg32(REQ_LOCK, CXD56_SPH_REQ(SPH_SMP));
    }
  while (getreg32(CXD56_SPH_STS(SPH_SMP)) != sphlocked);

  ret = *lock;

  if (ret == SP_UNLOCKED)
    {
      *lock = SP_LOCKED;
      SP_DMB();
    }

  /* Unlock hardware semaphore */

  putreg32(REQ_UNLOCK, CXD56_SPH_REQ(SPH_SMP));

  return ret;
}
