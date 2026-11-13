/****************************************************************************
 * arch/tricore/src/common/tricore_start.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_START_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_START_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/irq.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
/* Entry of the copy table of the TASKING locator: the action (0 end,
 * 1 copy, 2 clear) in the low half word, the destination, the source and
 * the size in bytes
 */

struct tricore_copytable_s
{
  uint32_t action;
  uint8_t *dst;
  const uint8_t *src;
  uint32_t size;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Provided by the linker script */

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
extern const struct tricore_copytable_s _lc_ub_table[];
#else
extern const uintptr_t __clear_table[];
extern const uintptr_t __copy_table[];
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_csa_init
 *
 * Description:
 *   Link the CSAs from start to end into the free context list, with LCX
 *   at the third last.  It runs before any function call is possible, so
 *   it must not call.
 *
 ****************************************************************************/

static inline_function void tricore_csa_init(uintptr_t *start,
                                             uintptr_t *end)
{
  uint32_t num = ((uintptr_t)end - (uintptr_t)start) / TC_CONTEXT_SIZE;
  uintptr_t *prev = NULL;
  uintptr_t *csa = start;
  uint32_t link;
  uint32_t k;

  for (k = 0; k < num; k++)
    {
      link = tricore_addr2csa(csa);
      if (k == 0)
        {
          tricore_mtcr(TRICORE_CPU_FCX, link);
        }
      else
        {
          *prev = link;
        }

      if (k == num - 3)
        {
          tricore_mtcr(TRICORE_CPU_LCX, link);
        }

      prev = csa;
      csa += TC_CONTEXT_REGS;
    }

  *prev = 0;
  UP_DSB();
}

/****************************************************************************
 * Name: tricore_copy
 *
 * Description:
 *   Copy len bytes from src to dst, or clear them if src is NULL.
 *
 ****************************************************************************/

static inline void tricore_copy(uint8_t *dst, const uint8_t *src,
                                uint32_t len)
{
  while (len >= 8)
    {
      *(uint64_t *)dst = src != NULL ? *(const uint64_t *)src : 0;
      dst += 8;
      src = src != NULL ? src + 8 : NULL;
      len -= 8;
    }

  while (len-- > 0)
    {
      *dst++ = src != NULL ? *src++ : 0;
    }
}

/****************************************************************************
 * Name: tricore_data_init
 *
 * Description:
 *   Clear .bss and copy .data from the tables of the linker script, the
 *   clear and copy tables on GCC and the copy table of the locator on
 *   TASKING.
 *
 ****************************************************************************/

static inline void tricore_data_init(void)
{
#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
  const struct tricore_copytable_s *entry;

  for (entry = _lc_ub_table; (entry->action & 0xffff) != 0; entry++)
    {
      tricore_copy(entry->dst,
                   (entry->action & 0xffff) == 1 ? entry->src : NULL,
                   entry->size);
    }
#else
  const uintptr_t *table;

  for (table = __clear_table; table[1] != 0xffffffff; table += 2)
    {
      tricore_copy((uint8_t *)table[0], NULL, table[1]);
    }

  for (table = __copy_table; table[2] != 0xffffffff; table += 3)
    {
      tricore_copy((uint8_t *)table[1], (const uint8_t *)table[0],
                   table[2]);
    }
#endif
}

#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_START_H */
