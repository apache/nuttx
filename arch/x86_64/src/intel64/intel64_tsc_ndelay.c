/****************************************************************************
 * arch/x86_64/src/intel64/intel64_tsc_ndelay.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

extern unsigned long g_x86_64_timer_freq;

/* REP NOP (PAUSE) is a good thing to insert into busy-wait loops. */

static inline void rep_nop(void)
{
  __asm__ __volatile__("rep;nop": : :"memory");
}

/* TSC based delay: */

static inline void delay_tsc(uint64_t cycles)
{
  uint64_t bclock;
  irqstate_t irq;
  uint64_t now;

  irq = up_irq_save();
  bclock = rdtscp();

  for (; ; )
    {
      now = rdtscp();
      if (now - bclock >= cycles)
        {
          break;
        }

      rep_nop();
    }

  up_irq_restore(irq);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_nsdelay
 *
 * Description:
 *   Delay inline for the requested number of nanoseconds.
 *   *** NOT multi-tasking friendly ***
 *
 *
 ****************************************************************************/

void up_ndelay(unsigned long nanoseconds)
{
  uint64_t cycles;

  cycles = (nanoseconds * g_x86_64_timer_freq + NSEC_PER_SEC - 1)
           / NSEC_PER_SEC;

  delay_tsc(cycles);
}
