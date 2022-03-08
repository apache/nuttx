/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_intercpu_interrupt.c
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
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"
#include "hardware/esp32s3_system.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_fromcpu_interrupt
 *
 * Description:
 *   Common logic called to handle the from CPU0/1 interrupts.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32s3_fromcpu_interrupt(int fromcpu)
{
  uintptr_t regaddr;

  DEBUGASSERT((unsigned)fromcpu < CONFIG_SMP_NCPUS);
  DEBUGASSERT(fromcpu != up_cpu_index());

  /* Clear the interrupt from the other CPU */

  regaddr = (fromcpu == 0) ? SYSTEM_CPU_INTR_FROM_CPU_0_REG :
                             SYSTEM_CPU_INTR_FROM_CPU_1_REG;
  putreg32(0, regaddr);

  /* Call pause handler */

  xtensa_pause_handler();

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_fromcpu[0,1]_interrupt
 *
 * Description:
 *   Called to handle the from CPU0/1 interrupts.
 *
 ****************************************************************************/

int IRAM_ATTR esp32s3_fromcpu0_interrupt(int irq, void *context, void *arg)
{
  return esp32s3_fromcpu_interrupt(0);
}

int IRAM_ATTR esp32s3_fromcpu1_interrupt(int irq, void *context, void *arg)
{
  return esp32s3_fromcpu_interrupt(1);
}

/****************************************************************************
 * Name: xtensa_intercpu_interrupt
 *
 * Description:
 *   Called to trigger a CPU interrupt
 *
 ****************************************************************************/

int IRAM_ATTR xtensa_intercpu_interrupt(int tocpu, int intcode)
{
  int fromcpu;

  DEBUGASSERT((unsigned)tocpu < CONFIG_SMP_NCPUS &&
              (unsigned)intcode <= UINT8_MAX);

  fromcpu = up_cpu_index();
  DEBUGASSERT(fromcpu != tocpu);

  /* Generate an Inter-Processor Interrupt */

  if (fromcpu == 0)
    {
      putreg32(SYSTEM_CPU_INTR_FROM_CPU_0, SYSTEM_CPU_INTR_FROM_CPU_0_REG);
    }
  else
    {
      putreg32(SYSTEM_CPU_INTR_FROM_CPU_1, SYSTEM_CPU_INTR_FROM_CPU_1_REG);
    }

  return OK;
}

#endif /* CONFIG_SMP */
