/****************************************************************************
 * arch/risc-v/src/nuttsbi/sbi_mcall.h
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

#ifndef __ARCH_RISC_V_SRC_NUTTSBI_SBI_MCALL_H
#define __ARCH_RISC_V_SRC_NUTTSBI_SBI_MCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum mcall_num_e
{
  MCALL_INVALID,
  MCALL_GET_TIMER,
  MCALL_SET_TIMER,
  MCALL_LAST
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mcall0
 *
 * Description:
 *   Generate an environment call to machine mode
 *
 * Input Parameters:
 *   nbr - Environment call number
 *
 ****************************************************************************/

static inline uintptr_t mcall0(unsigned int nbr)
{
  register long r0 asm("a0") = (long)(nbr);

  asm volatile
    (
     "ecall"
     :: "r"(r0)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

/****************************************************************************
 * Name: mcall1
 *
 * Description:
 *   Generate an environment call to machine mode
 *
 * Input Parameters:
 *   nbr - Environment call number
 *   parm1 - Argument for ecall
 *
 ****************************************************************************/

static inline uintptr_t mcall1(unsigned int nbr, uintptr_t parm1)
{
  register long r0 asm("a0") = (long)(nbr);
  register long r1 asm("a1") = (long)(parm1);

  asm volatile
    (
     "ecall"
     :: "r"(r0), "r"(r1)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

/****************************************************************************
 * Name: sbi_mcall_get_time
 *
 * Description:
 *   Read the current system time (mtime)
 *
 * Returned Value:
 *   time - Value of mtime
 *
 ****************************************************************************/

#define sbi_mcall_get_time() \
  mcall0(MCALL_GET_TIMER)

/****************************************************************************
 * Name: sbi_mcall_set_timer
 *
 * Description:
 *   Set new compare match value for timer. Time is in absolute time, so
 *   user must either obtain system time and calculate the offset, or keep
 *   the old compare match value in memory
 *
 * Input Parameters:
 *   stime_value - Absolute time for next compare match event
 *
 ****************************************************************************/

#define sbi_mcall_set_timer(stime_value) \
  (void)mcall1(MCALL_SET_TIMER, stime_value)

#endif /* __ARCH_RISC_V_SRC_NUTTSBI_SBI_MCALL_H */
