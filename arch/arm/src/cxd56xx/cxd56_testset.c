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
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPH_SMP 14  /* Use hardware semaphore #14 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_testset2
 ****************************************************************************/

spinlock_t up_testset2(volatile spinlock_t *lock)
{
  register uintptr_t ret asm("r0") = (uintptr_t)(lock);

  asm volatile (
    "mov r1, #1 \n"
    "1: \n"
    "ldrexb r2, [%0] \n"
    "cmp r2, r1 \n"
    "beq 2f \n"
    "strexb r2, r1, [%0] \n"
    "cmp r2, r1 \n"
    "beq 1b \n"
    "dmb \n"
    "mov %0, #0 \n"
    "bx lr \n"
    "2: \n"
    "strexb r2, r1, [%0] \n" /* dummy strex to release */
    "mov %0, #1 \n"
    : "+r" (ret)
    :
    : "r1", "r2");

  return ret;
}

/****************************************************************************
 * Name: up_testset
 *
 * Description:
 *   Perform an atomic test and set operation on the provided spinlock.
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object.
 *
 * Returned Value:
 *   The spinlock is always locked upon return.  The previous value of the
 *   spinlock variable is returned, either SP_LOCKED if the spinlock was
 *   previously locked (meaning that the test-and-set operation failed to
 *   obtain the lock) or SP_UNLOCKED if the spinlock was previously unlocked
 *   (meaning that we successfully obtained the lock).
 *
 ****************************************************************************/

spinlock_t up_testset(volatile spinlock_t *lock)
{
#ifdef CONFIG_CXD56_TESTSET_WITH_HWSEM
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
#else
  spinlock_t ret = up_testset2(lock);
#endif

  return ret;
}
