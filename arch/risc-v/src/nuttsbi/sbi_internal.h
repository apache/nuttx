/****************************************************************************
 * arch/risc-v/src/nuttsbi/sbi_internal.h
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

#ifndef __ARCH_RISC_V_SRC_NUTTSBI_SBI_INTERNAL_H
#define __ARCH_RISC_V_SRC_NUTTSBI_SBI_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Amount of harts, based on chip capability, not utilization */

#define MMODE_HART_CNT      (CONFIG_NUTTSBI_HART_CNT)

/* The machine mode interrupts should not be too complex */

#define MMODE_IRQSTACK      (1024)

/* Timer interrupt is the only one we handle, others are discarded */

#define MTIMER_IRQ          (7)

/* Timer memory mapped registers */

#define MTIMER_TIME_BASE    (CONFIG_NUTTSBI_MTIME_BASE)
#define MTIMER_CMP_BASE     (CONFIG_NUTTSBI_MTIMECMP_BASE)

/* For stack alignment */

#define STACK_ALIGNMENT     16
#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* Temporary stack placement and size */

#define TEMP_STACK_BASE     (_ebss)
#define TEMP_STACK          (1024)
#define TEMP_STACK_SIZE     (STACK_ALIGN_DOWN(TEMP_STACK))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: riscv_mscratch_assign
 *
 * Description:
 *   Assign the mscratch register for hartid. Sets the M-mode interrupt stack
 *   which is a must because M-mode deals with flat addressing and cannot
 *   share the user stack for exception handling.
 *
 * Input Parameters:
 *   hartid - Hartid.
 *
 ****************************************************************************/

void sbi_mscratch_assign(uintptr_t hartid);

/****************************************************************************
 * Name: sbi_start
 *
 * Description:
 *   Sets up entry to board specific start routine in S-mode. Mandatory
 *   trampoline function when the native SBI is used. Called from M-mode.
 *
 ****************************************************************************/

void sbi_start(void) noreturn_function;

/****************************************************************************
 * Name: sbi_init_mtimer
 *
 * Description:
 *   Set up access to mtimer for SBI. This is an extremely light weight way
 *   of provisioning the mtimer memory mapped registers to SBI.
 *
 * Input Parameters:
 *   mtime    - Pointer to machine time memory mapped register.
 *   mtimecmp - Pointer to the base of the mtimecmp memory mapped registers,
 *              implementation assumes there is 1 per hart and that they
 *              follow each other.
 *
 ****************************************************************************/

void sbi_init_mtimer(uintptr_t mtime, uintptr_t mtimecmp);

/****************************************************************************
 * Name: sbi_get_mtime
 *
 * Description:
 *   Read value of mtime
 *
 * Returned value:
 *   Full 64-bit system time
 *
 ****************************************************************************/

uint64_t sbi_get_mtime(void);

/****************************************************************************
 * Name: sbi_set_mtimecmp
 *
 * Description:
 *   Set value of mtimecmp.
 *
 * Input Parameters:
 *   value - Value for mtimecmp.
 *
 ****************************************************************************/

void sbi_set_mtimecmp(uint64_t value);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISC_V_SRC_NUTTSBI_SBI_INTERNAL_H */
