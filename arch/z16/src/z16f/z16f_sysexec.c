/****************************************************************************
 * arch/z16/src/z16f/z16f_sysexec.c
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

#include <nuttx/arch.h>

#include "chip.h"
#include "z16_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  z16f_sysexec
 *
 * Description:
 *   Handle a Z16F system exception
 *
 ****************************************************************************/

void z16f_sysexec(FAR chipreg_t *regs)
{
  uint16_t excp;

  /* Save that register reference so that it can be used for built-in
   * diagnostics.
   */

  g_current_regs = regs;

  /* The cause of the system exception is indicated in the SYSEXCPH&L
   * registers
   */

  excp = getreg16(Z16F_SYSEXCP);
  if ((excp & Z16F_SYSEXCP_SPOVF) != 0)
    {
      _err("ERROR: SP OVERFLOW\n");
    }

  if ((excp & Z16F_SYSEXCP_PCOVF) != 0)
    {
      _err("ERROR: PC OVERFLOW\n");
    }

  if ((excp & Z16F_SYSEXCP_DIV0) != 0)
    {
      _err("ERROR: Divide by zero\n");
    }

  if ((excp & Z16F_SYSEXCP_DIVOVF) != 0)
    {
      _err("ERROR: Divide overflow\n");
    }

  if ((excp & Z16F_SYSEXCP_ILL) != 0)
    {
      _err("ERROR: Illegal instruction\n");
    }

  if ((excp & Z16F_SYSEXCP_WDTOSC) != 0)
    {
      _err("ERROR: WDT oscillator failure\n");
    }

  if ((excp & Z16F_SYSEXCP_PRIOSC) != 0)
    {
      _err("ERROR: Primary Oscillator Failure\n");
    }

  if ((excp & Z16F_SYSEXCP_WDT) != 0)
    {
      _err("ERROR: Watchdog timeout\n");
      z16f_reset();
    }

  PANIC();
}
