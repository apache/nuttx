/****************************************************************************
 * arch/risc-v/src/common/supervisor/riscv_sbi.c
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

#include <stdint.h>

#include "riscv_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_RV64
static inline uint64_t rdtime(void)
{
  uint32_t hi;
  uint32_t lo;

  do
    {
      hi = READ_CSR(timeh);
      lo = READ_CSR(time);
    }
  while (hi != READ_CSR(timeh));

  return (((uint64_t) hi) << 32) | lo;
}
#else
#define rdtime() READ_CSR(time)
#endif /* CONFIG_ARCH_RV64 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_sbi_set_timer
 *
 * Description:
 *   Set new compare match value for timer
 *
 * Input Parameters:
 *   stime_value - Value to set
 *
 ****************************************************************************/

void riscv_sbi_set_timer(uint64_t stime_value)
{
#if 0
#error "Missing functionality..."
#endif
}

/****************************************************************************
 * Name: riscv_sbi_get_time
 *
 * Description:
 *   Get value of mtime
 *
 * Return:
 *   Value of mtime
 *
 ****************************************************************************/

uint64_t riscv_sbi_get_time(void)
{
  return rdtime();
}
