/****************************************************************************
 * arch/x86_64/src/intel64/intel64_cpu.h
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

#ifndef __ARCH_X86_64_SRC_INTEL64_INTEL64_CPU_H
#define __ARCH_X86_64_SRC_INTEL64_INTEL64_CPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>

#include <arch/arch.h>

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_cpu_tss_init
 *
 * Description:
 *  Initialize the TSS
 *
 ****************************************************************************/

void x86_64_cpu_tss_init(int cpu);

/****************************************************************************
 * Name: x86_64_cpu_tss_now_get
 *
 * Description:
 *   Get CPU TSS data associated with the current CPU
 *
 ****************************************************************************/

struct tss_s *x86_64_cpu_tss_now_get(void);

/****************************************************************************
 * Name: x86_64_cpu_init
 *
 * Description:
 *   Initialize CPU data.
 *
 ****************************************************************************/

void x86_64_cpu_init(void);

/****************************************************************************
 * Name: x86_64_lopaic_to_cpu
 *
 * Description:
 *   Get CPU index for a given Local APIC ID
 *
 ****************************************************************************/

uint8_t x86_64_loapic_to_cpu(uint8_t loapic);

/****************************************************************************
 * Name: x86_64_cpu_to_loapic
 *
 * Description:
 *   Get Local APIC ID for a given CPU index
 *
 ****************************************************************************/

uint8_t x86_64_cpu_to_loapic(uint8_t cpu);

/****************************************************************************
 * Name: x86_64_cpu_ready_set
 *
 * Description:
 *   Set CPU ready flag
 *
 ****************************************************************************/

void x86_64_cpu_ready_set(uint8_t cpu);

/****************************************************************************
 * Name: x86_64_cpu_ready_get
 *
 * Description:
 *   Get CPU ready flag
 *
 ****************************************************************************/

bool x86_64_cpu_ready_get(uint8_t cpu);

/****************************************************************************
 * Name: x86_64_cpu_count_get
 *
 * Description:
 *   Get CPU counter
 *
 ****************************************************************************/

uint8_t x86_64_cpu_count_get(void);

/****************************************************************************
 * Name: x86_64_cpu_priv_set
 *
 * Description:
 *   Save CPU private data
 *
 ****************************************************************************/

void x86_64_cpu_priv_set(uint8_t cpu);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_SRC_INTEL64_INTEL64_CPU_H */
