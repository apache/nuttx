/****************************************************************************
 * arch/xtensa/src/common/xtensa_fpucmp.c
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
#include <string.h>
#include <strings.h>

#include <nuttx/irq.h>

#include "xtensa.h"

#ifdef CONFIG_ARCH_FPU

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef XCHAL_CP_ID_FPU
static uint32_t g_coproc_sa_offsets[] =
{
  XTENSA_CP0_SA, XTENSA_CP1_SA, XTENSA_CP2_SA, XTENSA_CP3_SA,
  XTENSA_CP4_SA, XTENSA_CP5_SA, XTENSA_CP6_SA, XTENSA_CP7_SA
};

static uint32_t g_coproc_sa_sizes[] =
{
  XCHAL_CP0_SA_SIZE, XCHAL_CP1_SA_SIZE, XCHAL_CP2_SA_SIZE, XCHAL_CP3_SA_SIZE,
  XCHAL_CP4_SA_SIZE, XCHAL_CP5_SA_SIZE, XCHAL_CP6_SA_SIZE, XCHAL_CP7_SA_SIZE
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fpucmp
 *
 * Description:
 *   Compare FPU areas from thread context.
 *   This comparison will skip disabled coprocessors.
 *
 * Input Parameters:
 *   saveregs1 - Pointer to the saved FPU registers.
 *   saveregs2 - Pointer to the saved FPU registers.
 *
 * Returned Value:
 *   True if FPU areas compare equal, False otherwise.
 *
 ****************************************************************************/

bool up_fpucmp(const void *saveregs1, const void *saveregs2)
{
#ifdef XCHAL_CP_ID_FPU
  const uint32_t *regs1 = saveregs1;
  const uint32_t *regs2 = saveregs2;
  uint32_t cpenable = xtensa_get_cpenable();
  int ndx = 0;
  bool ret = true;
  int i;

  while (ret && (i = ffs(cpenable)))
    {
      uint32_t reg_offset;

      ndx += i;
      if ((ndx - 1) == XCHAL_CP_ID_FPU)
        {
          reg_offset = g_coproc_sa_offsets[ndx - 1] / 4;
          ret = memcmp(&regs1[COMMON_CTX_REGS + reg_offset],
                       &regs2[COMMON_CTX_REGS + reg_offset],
                       g_coproc_sa_sizes[ndx - 1]) == 0;
          break;
        }

      cpenable >>= i;
    }

  return ret;
#else
  return true;
#endif
}
#endif /* CONFIG_ARCH_FPU */
