/****************************************************************************
 * arch/x86_64/src/intel64/intel64_check_capability.c
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
  unsigned long ecx;
  unsigned long require;

  require = X86_64_CPUID_01_X2APIC;

  /* Check timer availability */

#ifdef CONFIG_ARCH_INTEL64_HAVE_TSC_DEADLINE
  require |= X86_64_CPUID_01_TSCDEA;
#endif

#ifdef CONFIG_ARCH_INTEL64_HAVE_XSAVE
  require |= X86_64_CPUID_01_XSAVE;
#endif

#ifdef CONFIG_ARCH_INTEL64_HAVE_RDRAND
  require |= X86_64_CPUID_01_RDRAND;
#endif

#ifdef CONFIG_ARCH_INTEL64_HAVE_PCID
  require |= X86_64_CPUID_01_PCID;
#endif

  asm volatile("cpuid" : "=c" (ecx) : "a" (X86_64_CPUID_CAP)
      : "rbx", "rdx", "memory");

  /* Check x2APIC availability */

  if ((ecx & require) != require)
    {
      goto err;
    }

#ifdef CONFIG_ARCH_INTEL64_HAVE_XSAVE
  __enable_sse_avx();
#endif

#ifdef CONFIG_ARCH_INTEL64_HAVE_PCID
  __enable_pcid();
#endif

  return;

err:
  asm volatile ("cli");
  asm volatile ("hlt");
  goto err;
}

