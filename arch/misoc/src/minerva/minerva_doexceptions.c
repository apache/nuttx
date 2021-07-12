/****************************************************************************
 * arch/misoc/src/minerva/minerva_doexceptions.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "group/group.h"
#include "minerva.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *minerva_doexception(uint32_t * regs)
{
  uint32_t mcause;
  uint32_t pending;
  uint32_t *ret;

  board_autoled_on(LED_INIRQ);

  mcause = regs[REG_CSR_MCAUSE_NDX];

  if (mcause & 0x80000000)
    {
      pending = irq_pending();
      ret = minerva_decodeirq(pending, regs);
    }
  else
    {
      mcause = mcause & 0x7fffffff;
      if (mcause == 11)
        {
          regs[REG_CSR_MEPC_NDX] += 4;
          ret = minerva_doirq(MINERVA_IRQ_SWINT, regs);
        }
      else
        {
          while (1);
        }
    }

  board_autoled_off(LED_INIRQ);
  return ret;
}
