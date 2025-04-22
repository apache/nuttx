/****************************************************************************
 * libs/libc/machine/xtensa/arch_mcount.c
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

#include <sys/gmon.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _mcount
 *
 * Description:
 *   This is the Xtensa mcount function.  It is called by the profiling logic
 *   to record the call.
 *
 * Input Parameters:
 *   frompc - The address of the calling instruction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

noinstrument_function
void _mcount(uintptr_t from_pc)
{
#ifndef __XTENSA_CALL0_ABI__
  uint32_t high_bit_mask = (0x03 << 30);
  uint32_t low_bit_mask = ~high_bit_mask;
  uint32_t pc_high_bits = ((uint32_t)_mcount) & high_bit_mask;
#endif
  uintptr_t self_pc;

  __asm__ __volatile__
  (
    "mov %0, a0\n"
    : "=r" (self_pc)
  );

#ifndef __XTENSA_CALL0_ABI__
  self_pc = pc_high_bits | (self_pc & low_bit_mask);
  from_pc = pc_high_bits | (from_pc & low_bit_mask);
#endif

  mcount_internal(from_pc, self_pc);
}
