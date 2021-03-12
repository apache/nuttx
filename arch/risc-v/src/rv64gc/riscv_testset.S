/****************************************************************************
 * arch/risc-v/src/rv64gc/riscv_testset.S
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
#include <arch/spinlock.h>

  .file "arm_testset.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .globl up_testset

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

  .text

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_testset
 *
 * Description:
 *   Perform an atomic test and set operation on the provided spinlock.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   lock - The address of spinlock object (a0).
 *
 * Returned Value:
 *   The spinlock is always locked upon return.  The value of previous value
 *   of the spinlock variable is returned, either SP_LOCKED if the spinlock
 *   as previously locked (meaning that the test-and-set operation failed to
 *   obtain the lock) or SP_UNLOCKED if the spinlock was previously unlocked
 *   (meaning that we successfully obtained the lock)
 *
 * Modifies: a1, a2
 *
 ****************************************************************************/

  .globl up_testset
  .type  up_testset, %function

up_testset:

  li     a1, SP_LOCKED

  /* Test if the spinlock is locked or not */

retry:
  lr.d   a2, (a0)       /* Test if spinlock is locked or not */
  beq    a2, a1, locked /* Already locked? Go to locked: */

  /* Not locked ... attempt to lock it */

  sc.d   a2, a1, (a0)   /* Attempt to set the locked state (a1) to (a0) */
  bnez   a2, retry      /* a2 will not be zero, if sc.b failed, try again */

  /* Lock acquired -- return SP_UNLOCKED */

  fence                 /* Required before accessing protected resource */
  li     a0, SP_UNLOCKED
  jr     ra

  /* Lock not acquired -- return SP_LOCKED */

locked:
  li     a0, SP_LOCKED
  jr     ra
  .size	up_testset, . - up_testset
  .end
