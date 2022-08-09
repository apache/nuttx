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
 * Pre-processor Definitions
 ****************************************************************************/

/* SBI Extension IDs */

#define SBI_EXT_TIME            0x54494D45

/* SBI function IDs for TIME extension */

#define SBI_EXT_TIME_SET_TIMER  0x0

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

static inline uintptr_t sbi_ecall(unsigned int extid, unsigned int fid,
                                  uintptr_t parm0, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  register long r0 asm("a0") = (long)(parm0);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);
  register long r4 asm("a4") = (long)(parm4);
  register long r5 asm("a5") = (long)(parm5);
  register long r6 asm("a6") = (long)(fid);
  register long r7 asm("a7") = (long)(extid);

  asm volatile
    (
     "ecall"
     : "+r"(r0), "+r"(r1)
     : "r"(r2), "r"(r3), "r"(r4), "r"(r5), "r"(r6), "r"(r7)
     : "memory"
     );

  return r1;
}

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
#ifdef CONFIG_ARCH_RV64
  sbi_ecall(SBI_EXT_TIME, SBI_EXT_TIME_SET_TIMER, stime_value, 0, 0, 0, 0,
            0);
#else
  sbi_ecall(SBI_EXT_TIME, SBI_EXT_TIME_SET_TIMER, stime_value,
            stime_value >> 32, 0, 0, 0, 0);
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
#ifdef CONFIG_ARCH_RV64
  return READ_CSR(time);
#else
  uint32_t hi;
  uint32_t lo;

  do
    {
      hi = READ_CSR(timeh);
      lo = READ_CSR(time);
    }
  while (hi != READ_CSR(timeh));

  return (((uint64_t) hi) << 32) | lo;
#endif
}
