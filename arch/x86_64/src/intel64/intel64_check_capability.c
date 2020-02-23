/****************************************************************************
 *  arch/x86_64/src/intel64/intel64_capability.c
 *
 *   Copyright (C) 2020 Chung-Fan Yang.
 *   All rights reserved.
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

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_check_capability
 *
 * Description:
 *   Called from up_lowsetup to check various CPU capabilities, matching the
 *   RTOS config
 *
 ****************************************************************************/

void x86_64_check_and_enable_capability(void)
{
  unsigned long ecx;

  bool tsc_deadline;
  bool sse3_xsave;
  bool rdrand;
  bool pcid;

  asm volatile("cpuid" : "=c" (ecx) : "a" (1)
      : "rbx", "rdx", "memory");

    /* Check timer availability */
#ifdef CONFIG_ARCH_INTEL64_HAVE_TSC_DEADLINE
  tsc_deadline = !!(ecx & (1 << 24));

  if(!tsc_deadline)
    goto err;
#endif

#ifdef CONFIG_ARCH_INTEL64_HAVE_SSE3
  sse3_xsave = !!(ecx & (1 << 0));
  sse3_xsave &= !!(ecx & (1 << 26));

  if(!sse3_xsave)
    goto err;

  __enable_sse3();
#endif

#ifdef CONFIG_ARCH_INTEL64_HAVE_RDRAND
  rdrand = !!(ecx & (1 << 30));

  if(!rdrand)
    goto err;
#endif

#ifdef CONFIG_ARCH_INTEL64_HAVE_PCID
  pcid = !!(ecx & (1 << 17));

  if(!pcid)
    goto err;

  __enable_pcid();
#endif

  return;

err:
    asm volatile ("cli");
    asm volatile ("hlt");
    goto err;
}

