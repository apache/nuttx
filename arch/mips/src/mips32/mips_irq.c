/****************************************************************************
 * arch/mips/src/mips32/mips_irq.c
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
#include <arch/mips32/cp0.h>

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
  register irqstate_t status;
  register irqstate_t ret;

  status  = cp0_getstatus();       /* Get CP0 status */
  ret     = status;                /* Save the status */
  status &= ~CP0_STATUS_INT_MASK;  /* Clear all interrupt mask bits */
  status |= CP0_STATUS_INT_SW0;    /* Enable only the SW0 interrupt */
  cp0_putstatus(status);           /* Disable the rest of interrupts */
  return ret;                      /* Return saved status */
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
  register irqstate_t status;

  status    = cp0_getstatus();       /* Get CP0 status */
  status   &= ~CP0_STATUS_INT_MASK;  /* Clear all interrupt mask bits */
  irqstate &= CP0_STATUS_INT_MASK;   /* Retain interrupt mask bits only */
  status   |= irqstate;              /* Set new interrupt mask bits */
  cp0_putstatus(status);             /* Restore interrupt state */
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
  register irqstate_t status;

  /* Set the status register.  It will be the same as the current status
   * register with some changes:
   *
   * 1. Clear the BEV bit (This bit should already be clear)
   * 2. Clear the UM bit so that the task executes in kernel mode
   *   (This bit should already be clear)
   * 3. Clear all the Interrupt mask bits.
   * 4. Make sure the IE is set
   */

  status  = cp0_getstatus();
  status &= ~(CP0_STATUS_BEV | CP0_STATUS_UM);
  status &= ~CP0_STATUS_INT_MASK;
  status |=  CP0_STATUS_IE;
  cp0_putstatus(status);
}
