/****************************************************************************
 * boards/x86_64/intel64/qemu-intel64/src/qemu_freq.c
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern unsigned long x86_64_timer_freq;

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
#ifdef CONFIG_ARCH_INTEL64_HAVE_TSC_DEADLINE

  unsigned long crystal_freq;
  unsigned long numerator;
  unsigned long denominator;

  asm volatile("cpuid"
      : "=c" (crystal_freq), "=b" (numerator), "=a" (denominator)
      : "a" (X86_64_CPUID_TSC)
      : "rdx", "memory");

  if (numerator == 0 || denominator == 0 || crystal_freq == 0)
    {
      x86_64_timer_freq = CONFIG_ARCH_INTEL64_CORE_FREQ_KHZ * 1000L;
    }
  else
    {
      x86_64_timer_freq = crystal_freq / denominator * numerator;
    }

#else
  x86_64_timer_freq = CONFIG_ARCH_INTEL64_APIC_FREQ_KHZ * 1000L;
#endif
}
