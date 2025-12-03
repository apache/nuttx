/****************************************************************************
 * arch/tricore/src/common/tricore_initialize.c
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

#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/board.h>
#include <nuttx/init.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uintptr_t *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
static inline void up_color_intstack(void)
{
  uint32_t *ptr = (uint32_t *)g_intstackalloc;
  ssize_t size;

  for (size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
       size > 0;
       size -= sizeof(uint32_t))
    {
      *ptr++ = INTSTACK_COLOR;
    }
}
#else
#  define up_color_intstack()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_initialize(void)
{
  up_color_intstack();

  aurix_earlyserialinit();
  aurix_serialinit();
}

#define IFX_CPUn_REG(n)             (0xf8800000 + 0x40000 * (n))

#define IFX_CPU_HALT                BIT(0)
#define IFX_CPU_BOOTCON(n)          (IFX_CPUn_REG(n) + 0x1fe60)
#define IFX_CPU_PC(n)               (IFX_CPUn_REG(n) + 0x1fe08)


#define IFX_CPU_PROTSPRSE(n)        (IFX_CPUn_REG(n) + 0xe008)
#define IFX_CPU_ACCENSPRCFG_WRA(n)  (IFX_CPUn_REG(n) + 0xe020)
#define IFX_CPU_ACCENSPRCFG_WRB(n)  (IFX_CPUn_REG(n) + 0xe024)

#define IFX_CPU_PROTDLMUSE(n)       (IFX_CPUn_REG(n) + 0xe048)
#define IFX_CPU_ACCENDLMUCFG_WRA(n) (IFX_CPUn_REG(n) + 0xe060)
#define IFX_CPU_ACCENDLMUCFG_WRB(n) (IFX_CPUn_REG(n) + 0xe064)

#define IFX_CPU_PROTSFRSE(n)        (IFX_CPUn_REG(n) + 0xe088)
#define IFX_CPU_ACCENSFRCFG_WRA(n)  (IFX_CPUn_REG(n) + 0xe0a0)

#define IFX_CPU_PROTSTMSE(n)        (IFX_CPUn_REG(n) + 0xe0d8)
#define IFX_CPU_ACCENSTMCFG_WRA(n)  (IFX_CPUn_REG(n) + 0xe0e0)

#define IFX_CPU_PROT_RANGE(c)       ((c) << 8)


extern const unsigned int _flash_banks[];

static void tricore_enable_sfr_access(int n)
{
  putreg32(0, IFX_CPU_PROTSFRSE(n));
  putreg32(0xffffffffu, IFX_CPU_ACCENSFRCFG_WRA(n));

  /* Allow the whole 0-7 range */

  putreg32(IFX_CPU_PROT_RANGE(0x7), IFX_CPU_PROTSTMSE(n));
  putreg32(0xffffffffu, IFX_CPU_ACCENSTMCFG_WRA(n));

  /* Allow the whole 0-15 range */

  putreg32(IFX_CPU_PROT_RANGE(0xf), IFX_CPU_PROTDLMUSE(n));
  putreg32(0xffffffffu, IFX_CPU_ACCENDLMUCFG_WRA(n));
  putreg32(0xffffffffu, IFX_CPU_ACCENDLMUCFG_WRB(n));

  /* Allow the whole 0-15 range */

  putreg32(IFX_CPU_PROT_RANGE(0xf), IFX_CPU_PROTSPRSE(n));
  putreg32(0xffffffffu, IFX_CPU_ACCENSPRCFG_WRA(n));
  putreg32(0xffffffffu, IFX_CPU_ACCENSPRCFG_WRB(n));
}

void tricore_start(int current_cpu)
{
  /* Start the other cores from CPU 0 */

  if (current_cpu == 0)
    {
      int i;

      for (i = 1; i < CONFIG_ARCH_TRICORE_CPU_COUNT; i++)
        {
          uintptr_t hreg = IFX_CPU_BOOTCON(i);

          tricore_enable_sfr_access(i);

          if (getreg32(hreg))
            {
              putreg32(_flash_banks[i], IFX_CPU_PC(i));
              modreg32(0, IFX_CPU_HALT, hreg);
            }
        }
    }

  nx_start();
}
