/****************************************************************************
 * arch/risc-v/src/common/riscv_ipi.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_IPI_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_IPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "riscv_internal.h"
#include "riscv_sbi.h"
#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

static inline void riscv_ipi_send(int cpu)
{
#if defined(CONFIG_ARCH_USE_S_MODE)
  riscv_sbi_send_ipi(0x1, riscv_cpuid_to_hartid(cpu));
#elif defined(RISCV_IPI)
  putreg32(1, (uintptr_t)RISCV_IPI + (4 * riscv_cpuid_to_hartid(cpu)));
#else
#  error "No IPI support for this SoC"
#endif
}

static inline void riscv_ipi_clear(int cpu)
{
#if defined(CONFIG_ARCH_USE_S_MODE)
  CLEAR_CSR(CSR_IP, IP_SIP);
#elif defined(RISCV_IPI)
  putreg32(0, (uintptr_t)RISCV_IPI + (4 * riscv_cpuid_to_hartid(cpu)));
#else
#  error "No IPI support for this SoC"
#endif
}

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_IPI_H */
