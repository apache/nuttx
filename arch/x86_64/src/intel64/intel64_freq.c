/****************************************************************************
 * arch/x86_64/src/intel64/intel64_freq.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

unsigned long g_x86_64_timer_freq;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint64_t x86_64_timer_tsc_freq_vmware(void)
{
  uint32_t eax_tsc;
  uint32_t ebx_apic;
  uint32_t ecx;
  uint32_t edx;

  /* CPUID Leaf 0x40000010, Timing Information.
   * Timing information leaf first defined by VMware.
   * It is also adopted by many hypervisors such as ACRN Hypervisor.
   * The leaf returns the TSC frequency and APIC frequency.
   * EAX - TSC frequency in kHz.
   * EBX - APIC frequency in kHz.
   */

  x86_64_cpuid(X86_64_CPUID_TSC_VMWARE, 0x0,
               &eax_tsc, &ebx_apic, &ecx, &edx);

  /* Suppress the warning. */

  UNUSED(ecx);
  UNUSED(edx);
  UNUSED(ebx_apic);

  return 1000ul * eax_tsc;
}

static inline uint64_t x86_64_timer_tsc_freq_15h(void)
{
  uint32_t crystal_freq;
  uint32_t numerator;
  uint32_t denominator;
  uint32_t edx;

  /* CPUID Leaf 0x15h, TSC frequency properties.
   * The leaf returns the TSC frequency properties.
   * EAX - Denominator.
   * EBX - Numerator.
   * ECX - Crystal Frequency.
   */

  x86_64_cpuid(X86_64_CPUID_TSC, 0x0,
               &denominator, &numerator, &crystal_freq, &edx);

  /* Suppress the warning. */

  UNUSED(edx);

  if (numerator == 0 || denominator == 0 || crystal_freq == 0)
    {
      return 0;
    }

  return crystal_freq / denominator * numerator;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_timer_initialize
 *
 * Description:
 *   Initializes all platform-specific timer facilities.  This function is
 *   called early in the initialization sequence by up_initialize().
 *   On return, the current up-time should be available from
 *   up_timer_gettime() and the interval timer is ready for use (but not
 *   actively timing.
 *
 *   Provided by platform-specific code and called from the architecture-
 *   specific logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in the initialization sequence before any special
 *   concurrency protections are required.
 *
 ****************************************************************************/

void x86_64_timer_calibrate_freq(void)
{
#ifdef CONFIG_ARCH_INTEL64_TSC_DEADLINE
  g_x86_64_timer_freq = CONFIG_ARCH_INTEL64_CORE_FREQ_KHZ * 1000ul;

  if (CONFIG_ARCH_INTEL64_CORE_FREQ_KHZ == 0)
    {
#  ifndef CONFIG_ARCH_INTEL64_TSC_FREQ_VMWARE
      g_x86_64_timer_freq = x86_64_timer_tsc_freq_15h();
#  else
      g_x86_64_timer_freq = x86_64_timer_tsc_freq_vmware();
#  endif
    }
#elif defined(CONFIG_ARCH_INTEL64_TSC)
  g_x86_64_timer_freq = CONFIG_ARCH_INTEL64_APIC_FREQ_KHZ * 1000ul;
#endif

  if (g_x86_64_timer_freq == 0)
    {
      /* The TSC frequency is not available */

      PANIC();
    }
}
