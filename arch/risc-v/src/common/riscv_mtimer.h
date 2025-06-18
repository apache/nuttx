/****************************************************************************
 * arch/risc-v/src/common/riscv_mtimer.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_MTIMER_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_MTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/timers/oneshot.h>
#include <arch/barriers.h>

#include "riscv_internal.h"
#include "riscv_sbi.h"

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_USE_S_MODE
static inline uint64_t riscv_mtimer_get(uintreg_t mtime_addr)
{
#  if CONFIG_ARCH_RV_MMIO_BITS == 64
  /* mtime_addr is -1, means this SoC:
   * 1. does NOT support 64bit/DWORD write for the mtimer compare value regs,
   * 2. has NO memory mapped regs which hold the value of mtimer counter,
   *    it could be read from the CSR "time".
   */

  return -1 == mtime_addr ? READ_CSR(CSR_TIME) : getreg64(mtime_addr);
#  else
  uint32_t hi;
  uint32_t lo;

  do
    {
      hi = getreg32(mtime_addr + 4);
      lo = getreg32(mtime_addr);
    }
  while (getreg32(mtime_addr + 4) != hi);

  return ((uint64_t)hi << 32) | lo;
#  endif
}

static inline
void riscv_mtimer_set(uintreg_t mtime_addr,
                      uintreg_t mtimecmp_addr, uint64_t value)
{
#  if CONFIG_ARCH_RV_MMIO_BITS == 64
  if (-1 != mtime_addr)
    {
      putreg64(value, mtimecmp_addr);
    }
  else
#  endif
    {
      putreg32(UINT32_MAX, mtimecmp_addr + 4);
      putreg32(value, mtimecmp_addr);
      putreg32(value >> 32, mtimecmp_addr + 4);
    }

  UP_DSB();
}

#else

#  ifdef CONFIG_ARCH_RV_EXT_SSTC
static inline void riscv_write_stime(uint64_t value)
{
#    ifdef CONFIG_ARCH_RV64
  WRITE_CSR(CSR_STIMECMP, value);
#    else
  WRITE_CSR(CSR_STIMECMP, (uint32_t)value);
  WRITE_CSR(CSR_STIMECMPH, (uint32_t)(value >> 32));
#    endif /* CONFIG_ARCH_RV64 */
}
#  endif /* CONFIG_ARCH_RV_EXT_SSTC */

static inline uint64_t riscv_mtimer_get(uintreg_t mtime_addr)
{
  UNUSED(mtime_addr);
  return riscv_sbi_get_time();
}

static inline
void riscv_mtimer_set(uintreg_t mtime_addr,
                      uintreg_t mtimecmp_addr, uint64_t value)
{
  UNUSED(mtimecmp_addr);
  UNUSED(mtime_addr);
#  ifndef CONFIG_ARCH_RV_EXT_SSTC
  riscv_sbi_set_timer(value);
#  else
  riscv_write_stime(value);
#  endif /* CONFIG_ARCH_RV_EXT_SSTC */
}
#endif /* CONFIG_ARCH_USE_S_MODE */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct oneshot_lowerhalf_s *
riscv_mtimer_initialize(uintreg_t mtime, uintreg_t mtimecmp,
                        int irq, uint64_t freq);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_MTIMER_H */
