/****************************************************************************
 * arch/x86_64/src/intel64/intel64_check_capability.c
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

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "x86_64_internal.h"

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
  uint32_t ebx;
  uint32_t ecx;
  uint32_t require = 0;

  /* Check SSE3 instructions availability */

#ifdef CONFIG_ARCH_X86_64_SSE3
  require |= X86_64_CPUID_01_SSE3;
#endif

  /* Check Suplement SSE3 instructions availability */

#ifdef CONFIG_ARCH_X86_64_SSEE3
  require |= X86_64_CPUID_01_SSEE3;
#endif

  /* Check Fused multiply-add (FMA) instructions availability */

#ifdef CONFIG_ARCH_X86_64_FMA
  require |= X86_64_CPUID_01_FMA;
#endif

  /* Check process context identifiers availability */

#ifdef CONFIG_ARCH_INTEL64_HAVE_PCID
  require |= X86_64_CPUID_01_PCID;
#endif

  /* Check SSE4.1 instructions availability */

#ifdef CONFIG_ARCH_X86_64_SSE41
  require |= X86_64_CPUID_01_SSE41;
#endif

  /* Check SSE4.2 instructions availability */

#ifdef CONFIG_ARCH_X86_64_SSE42
  require |= X86_64_CPUID_01_SSE42;
#endif

  /* Check x2APIC availability */

  require |= X86_64_CPUID_01_X2APIC;

  /* Check timer availability */

#ifdef CONFIG_ARCH_INTEL64_TSC_DEADLINE
  require |= X86_64_CPUID_01_TSCDEA;
#endif

  /* Check XSAVE/XRSTOR availability */

#ifdef CONFIG_ARCH_X86_64_HAVE_XSAVE
  require |= X86_64_CPUID_01_XSAVE;
#endif

  /* Check AVX instructions availability */

#ifdef CONFIG_ARCH_X86_64_AVX
  require |= X86_64_CPUID_01_AVX;
#endif

  /* Check RDRAND feature availability */

#ifdef CONFIG_ARCH_INTEL64_HAVE_RDRAND
  require |= X86_64_CPUID_01_RDRAND;
#endif

  __asm__ volatile("cpuid" : "=c" (ecx) : "a" (X86_64_CPUID_CAP)
                   : "rdx", "memory");

  /* Check features availability from ECX */

  if ((ecx & require) != require)
    {
      goto err;
    }

  /* Extended features */

  require = 0;

  /* Check AVX512 Foundation instructions availability */

#ifdef CONFIG_ARCH_X86_64_AVX512
  require |= X86_64_CPUID_07_AVX512F;
#endif

  /* Check CLWB instruction availability */

#ifdef CONFIG_ARCH_INTEL64_HAVE_CLWB
  require |= X86_64_CPUID_07_CLWB;
#endif

  __asm__ volatile("cpuid" : "=b" (ebx) : "a" (X86_64_CPUID_EXTCAP), "c" (0)
                   : "rdx", "memory");

  /* Check features availability */

  if ((ebx & require) != require)
    {
      goto err;
    }

#if defined(CONFIG_ARCH_HAVE_SSE) || defined(CONFIG_ARCH_X86_64_AVX) || \
    defined(CONFIG_ARCH_X86_64_AVX512)
  __enable_sse_avx();
#endif

#ifdef CONFIG_ARCH_X86_64_HAVE_XSAVE
  /* Check XSAVE state area size for the current XCR0 state */

  __asm__ volatile("cpuid" : "=b" (ebx)
                   : "a" (X86_64_CPUID_XSAVE), "c" (0)
                   : "rdx", "memory");

  if (XCPTCONTEXT_XMM_AREA_SIZE < ebx)
    {
      goto err;
    }
#endif

#ifdef CONFIG_ARCH_INTEL64_HAVE_PCID
  __enable_pcid();
#endif

  /* Enable I- and D-Caches */

  up_enable_icache();
  up_enable_dcache();

  return;

err:
  __asm__ volatile ("cli");
  __asm__ volatile ("hlt");

  goto err;
}

