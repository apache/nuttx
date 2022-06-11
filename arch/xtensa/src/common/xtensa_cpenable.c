/****************************************************************************
 * arch/xtensa/src/common/xtensa_cpenable.c
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

#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include <arch/xtensa/xtensa_coproc.h>
#include <arch/chip/core-isa.h>

#include "xtensa.h"

#if XCHAL_CP_NUM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_coproc_enable
 *
 * Description:
 *   Enable a set of co-processors.
 *
 * Input Parameters:
 *   cpset   - A bit set of co-processors to be enabled.  Matches bit layout
 *             of the CPENABLE register.  Bit 0-XCHAL_CP_NUM:  0 = no change
 *             1 = enable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xtensa_coproc_enable(int cpset)
{
  irqstate_t flags;
  uint32_t cpenable;

  /* These operations must be atomic */

  flags = enter_critical_section();

  if (cpset != 0)
    {
      /* Enable the co-processors */

      cpenable = xtensa_get_cpenable();
      cpenable |= cpset;
      xtensa_set_cpenable(cpenable);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: xtensa_coproc_disable
 *
 * Description:
 *   Enable a set of co-processors.
 *
 * Input Parameters:
 *   cpset   - A bit set of co-processors to be enabled.  Matches bit layout
 *             of the CPENABLE register.  Bit 0-XCHAL_CP_NUM:  0 = no change
 *             1 = disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xtensa_coproc_disable(int cpset)
{
  irqstate_t flags;
  uint32_t cpenable;

  /* These operations must be atomic */

  flags = enter_critical_section();

  if (cpset != 0)
    {
      /* Disable the co-processors */

      cpenable = xtensa_get_cpenable();
      cpenable &= ~cpset;
      xtensa_set_cpenable(cpenable);
    }

  leave_critical_section(flags);
}

#endif /* XCHAL_CP_NUM */
