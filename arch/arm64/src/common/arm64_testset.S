/****************************************************************************
 * arch/arm64/src/common/arm64_testset.S
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

#include "arch/syscall.h"
#include "arm64_macro.inc"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm64_testset.S"

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

GTEXT(up_testset)
SECTION_FUNC(text, up_testset)

up_testset:

    mov      x1, #SP_LOCKED

    /* Test if the spinlock is locked or not */

1:
    ldaxr    x2, [x0]    /* Test if spinlock is locked or not */
    cmp      x2, x1        /* Already locked? */
    beq      2f            /* If already locked, return SP_LOCKED */

    /* Not locked ... attempt to lock it */

    stxr     w2, x1, [x0]    /* Attempt to set the locked state */
    cbnz     w2, 1b            /* w2 will be 1 is stxr failed */

    /* Lock acquired -- return SP_UNLOCKED */

    mov      x0, #SP_UNLOCKED

    ret

    /* Lock not acquired -- return SP_LOCKED */

2:
    mov      x0, #SP_LOCKED
    ret
