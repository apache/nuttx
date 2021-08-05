/****************************************************************************
 * arch/xtensa/src/common/xtensa_counter.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_XTENSA_COUNTER_H
#define __ARCH_XTENSA_SRC_COMMON_XTENSA_COUNTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "xtensa_timer.h"

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_getcount
 *
 * Description:
 *   Get the current value of the cycle count register.
 *
 ****************************************************************************/

static inline uint32_t xtensa_getcount(void)
{
  uint32_t count;

  __asm__ __volatile__
  (
    "rsr %0, CCOUNT"  : "=r"(count)
  );

  return count;
}

/****************************************************************************
 * Name: xtensa_setcount
 *
 * Description:
 *   Set the value of the cycle count register.
 *
 ****************************************************************************/

static inline void xtensa_setcount(uint32_t ticks)
{
  __asm__ __volatile__
  (
    "wsr    %0, ccount\n"
    :
    : "a"(ticks)
    : "memory"
  );
}

/****************************************************************************
 * Name: xtensa_getcompare
 *
 * Description:
 *   Get the old value of the compare register.
 *
 ****************************************************************************/

static inline uint32_t xtensa_getcompare(void)
{
  uint32_t compare;

  __asm__ __volatile__
  (
    "rsr %0, %1"  : "=r"(compare) : "i"(XT_CCOMPARE)
  );

  return compare;
}

/****************************************************************************
 * Name: xtensa_getcompare
 *
 * Description:
 *   Set the value of the compare register.
 *
 ****************************************************************************/

static inline void xtensa_setcompare(uint32_t compare)
{
  __asm__ __volatile__
  (
    "wsr %0, %1" : : "r"(compare), "i"(XT_CCOMPARE)
  );
}

#endif /* __ARCH_XTENSA_SRC_COMMON_XTENSA_COUNTER_H */
