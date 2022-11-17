/****************************************************************************
 * arch/sparc/src/sparc_v8/sparc_v8_irq.c
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

#include <arch/irq.h>
#include <arch/types.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Save the current interrupt state and disable interrupts.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Interrupt state prior to disabling interrupts.
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  /* register irqstate_t status; */

  register irqstate_t ret;

  /* Get psr status */

  /* sparc_get_psr( status ); */

  /* Save the status */

  ret     = sparc_disable_interrupts();

  /* BM3803_REG.int_mask = 0; */

  return ret;                 /* Return status before interrupts disabled */
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the previous up_irq_enable state (i.e., the one previously
 *   returned by up_irq_save())
 *
 * Input Parameters:
 *   state - The interrupt state to be restored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t irqstate)
{
  sparc_enable_interrupts(irqstate);
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_irq_enable(void)
{
  uint32_t            psr;

  psr = sparc_disable_interrupts();
  sparc_enable_interrupts(psr);
}

/****************************************************************************
 * Name: up_irq_disable
 *
 * Description:
 *   Disable interrupts
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   the entire PSR contents
 *
 ****************************************************************************/

uint32_t up_irq_disable(void)
{
  uint32_t            psr;

  psr = sparc_disable_interrupts();

  return psr;
}
