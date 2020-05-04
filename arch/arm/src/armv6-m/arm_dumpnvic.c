/****************************************************************************
 * arch/arm/src/armv6-m/arm_dumpnvic.c
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_arch.h"

#include "nvic.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  arm_dumpnvic
 *
 * Description:
 *   Dump all NVIC and SYSCON registers along with a user message.
 *
 ****************************************************************************/

void arm_dumpnvic(FAR const char *msg)
{
#ifdef CONFIG_DEBUG_INFO
  irqstate_t flags;
  int i;

  /* The following requires exclusive access to the NVIC/SYSCON registers */

  flags = enter_critical_section();

  _info("NVIC: %s\n", msg);
  _info("   ISER: %08x  ICER: %08x  ISPR: %08x  ICPR: %08x\n",
       getreg32(ARMV6M_NVIC_ISER), getreg32(ARMV6M_NVIC_ICER),
       getreg32(ARMV6M_NVIC_ISPR), getreg32(ARMV6M_NVIC_ICPR));

  for (i = 0 ; i < 8; i += 4)
    {
      _info("   IPR%d: %08x  IPR%d: %08x  IPR%d: %08x  IPR%d: %08x\n",
           i,     getreg32(ARMV6M_NVIC_IPR(i)),
           i + 1, getreg32(ARMV6M_NVIC_IPR(i + 1)),
           i + 2, getreg32(ARMV6M_NVIC_IPR(i + 2)),
           i + 3, getreg32(ARMV6M_NVIC_IPR(i + 3)));
    }

  _info("SYSCON:\n");
  _info("  CPUID: %08x  ICSR: %08x AIRCR: %08x   SCR: %08x\n",
       getreg32(ARMV6M_SYSCON_CPUID), getreg32(ARMV6M_SYSCON_ICSR),
       getreg32(ARMV6M_SYSCON_AIRCR), getreg32(ARMV6M_SYSCON_SCR));
  _info("    CCR: %08x SHPR2: %08x SHPR3: %08x\n",
       getreg32(ARMV6M_SYSCON_CCR),   getreg32(ARMV6M_SYSCON_SHPR2),
       getreg32(ARMV6M_SYSCON_SHPR3));

  leave_critical_section(flags);
#endif
}

#endif /* CONFIG_DEBUG_FEATURES */
