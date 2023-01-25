/****************************************************************************
 * arch/arm/src/imxrt/imxrt_daisy.c
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
#include "chip.h"
#include "arm_internal.h"
#include "hardware/imxrt_daisy.h"
#include "imxrt_iomuxc.h"

/* Include chip-specific daisy input selection */

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "imxrt102x_daisy.c"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "imxrt105x_daisy.c"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "imxrt106x_daisy.c"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT117x)
#  include "imxrt117x_daisy.c"
#else
#  error Unrecognized i.MX RT architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_daisy_select
 ****************************************************************************/
#if !defined(IMXRT_DAISY_SELECT_PROVIDED)
void imxrt_daisy_select(unsigned int index, unsigned int alt)
{
  uintptr_t address;
  const struct imxrt_daisy_t *daisy = &g_daisy_select[index];

  index = daisy->alts[alt].index;
  if (index != DAISY_INDEX_INVALID)
    {
      alt = daisy->alts[alt].sel;
      address = IMXRT_IOMUXC_BASE + IMXRT_INPUT_INDEX2OFFSET(index);
      putreg32(alt, address);
    }
}
#endif
