/****************************************************************************
 * boards/x86_64/intel64/qemu/src/qemu_freq.c
 *
 *   Copyright (C) 2020 Chung-Fan Yang. All rights reserved.
 *
 *   Author: Chung-Fan Yang <sonic.tw.tp@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

extern unsigned long x86_64_timer_freq;

/****************************************************************************
 * Name: x86_64_timer_initialize
 *
 * Description:
 *   Initializes all platform-specific timer facilities.  This function is
 *   called early in the initialization sequence by up_intialize().
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
  unsigned long numerator, denominator;

  asm volatile("cpuid" : "=c" (crystal_freq), "=b" (numerator), "=a" (denominator)
      : "a" (X86_64_CPUID_TSC)
      : "rdx", "memory");

  if(numerator == 0 || denominator == 0 || crystal_freq == 0)
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
