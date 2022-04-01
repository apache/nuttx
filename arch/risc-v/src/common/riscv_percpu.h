/****************************************************************************
 * arch/risc-v/src/common/riscv_percpu.h
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

#ifndef __ARCH_RISC_V_SRC_COMMON_RISCV_PERCPU_H
#define __ARCH_RISC_V_SRC_COMMON_RISCV_PERCPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <nuttx/arch.h>
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RISCV_PERCPU_HARTID_OFFSET   (0 * INT_REG_SIZE)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Per CPU save area. Access to this structure can be gained via the
 * supervisor scratch (sscratch) register. Prior to this, every CPU that
 * wishes to access this information must call riscv_percpu_add_hart() which
 * will set up sscratch to point to the CPUs own area
 */

struct riscv_percpu_s
{
  uintptr_t hartid;  /* Hart ID */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_percpu_init
 *
 * Description:
 *   Initialize the per CPU structures, should only be done on the boot
 *   hart.
 *
 ****************************************************************************/

void riscv_percpu_init(void);

/****************************************************************************
 * Name: riscv_percpu_add_hart
 *
 * Description:
 *   Get add a hart to the per CPU area
 *
 * Input Parameters:
 *   hartid - Hart number
 *
 ****************************************************************************/

void riscv_percpu_add_hart(uintptr_t hartid);

/****************************************************************************
 * Name: riscv_percpu_get_hartid
 *
 * Description:
 *   Get harts own hartid by reading it from the per CPU area. This is safe
 *   to use from lower privilege modes than M-mode.
 *
 * Returned Value:
 *   Hart id
 *
 ****************************************************************************/

uintptr_t riscv_percpu_get_hartid(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISC_V_SRC_COMMON_RISCV_PERCPU_H */
