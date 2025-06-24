/****************************************************************************
 * arch/xtensa/src/common/xtensa_cpuint.c
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

#include <assert.h>
#include <sys/types.h>

#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_specregs.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_enable_cpuint
 *
 * Description:
 *   Enables an interrupts. Does not simply set INTENABLE directly,
 *   but operates on a shadow copy of the CPU INTENABLE register then
 *   writes that value to the hardware INTENABLE register.  Can be called
 *   from interrupt handlers.
 *
 *   NOTE: It is possible only to enable interrupts on the current CPU
 *   because there is an INTENABLE register implemented in each CPU.
 *
 ****************************************************************************/

void xtensa_enable_cpuint(uint32_t *shadow, uint32_t intnum)
{
  DEBUGASSERT(intnum < XCHAL_NUM_INTERRUPTS);

  if (intnum < 32)
    {
      shadow[0] |= (1 << intnum);
      __asm__ __volatile__
      (
        "wsr %0, INTENABLE\n"
        "rsync\n"
        :
        : "r"(shadow[0])
        :
      );
    }
#if XCHAL_NUM_INTERRUPTS > 32
  else if (intnum < 64)
    {
      shadow[1] |= (1 << (intnum - 32));
      __asm__ __volatile__
      (
        "wsr %0, INTENABLE1\n"
        "rsync\n"
        :
        : "r"(shadow[1])
        :
      );
    }
#endif
#if XCHAL_NUM_INTERRUPTS > 64
  else if (intnum < 96)
    {
      shadow[2] |= (1 << (intnum - 64));
      __asm__ __volatile__
      (
        "wsr %0, INTENABLE2\n"
        "rsync\n"
        :
        : "r"(shadow[2])
        :
      );
    }
#endif
#if XCHAL_NUM_INTERRUPTS > 96
  else if (intnum < 128)
    {
      shadow[3] |= (1 << (intnum - 96));
      __asm__ __volatile__
      (
        "wsr %0, INTENABLE3\n"
        "rsync\n"
        :
        : "r"(shadow[3])
        :
      );
    }
#endif
}

/****************************************************************************
 * Name: xtensa_disable_cpuint
 *
 * Description:
 *   Disables an interrupts. Does not simply clear INTENABLE directly,
 *   but operates on a shadow copy of the CPU INTENABLE register then
 *   writes that value to the hardware INTENABLE register.  Can be called
 *   from interrupt handlers.
 *
 *   NOTE: It is possible only to disable interrupts on the current CPU
 *   because there is an INTENABLE register implemented in each CPU.
 *
 ****************************************************************************/

void xtensa_disable_cpuint(uint32_t *shadow, uint32_t intnum)
{
  DEBUGASSERT(intnum < XCHAL_NUM_INTERRUPTS);

  if (intnum < 32)
    {
      shadow[0] &= ~(1 << intnum);
      __asm__ __volatile__
      (
        "wsr %0, INTENABLE\n"
        "rsync\n"
        :
        : "r"(shadow[0])
        :
      );
    }
#if XCHAL_NUM_INTERRUPTS > 32
  else if (intnum < 64)
    {
      shadow[1] &= ~(1 << (intnum - 32));
      __asm__ __volatile__
      (
        "wsr %0, INTENABLE1\n"
        "rsync\n"
        :
        : "r"(shadow[1])
        :
      );
    }
#endif
#if XCHAL_NUM_INTERRUPTS > 64
  else if (intnum < 96)
    {
      shadow[2] &= ~(1 << (intnum - 64));
      __asm__ __volatile__
      (
        "wsr %0, INTENABLE2\n"
        "rsync\n"
        :
        : "r"(shadow[2])
        :
      );
    }
#endif
#if XCHAL_NUM_INTERRUPTS > 96
  else if (intnum < 128)
    {
      shadow[3] &= ~(1 << (intnum - 96));
      __asm__ __volatile__
      (
        "wsr %0, INTENABLE3\n"
        "rsync\n"
        :
        : "r"(shadow[3])
        :
      );
    }
#endif
}
