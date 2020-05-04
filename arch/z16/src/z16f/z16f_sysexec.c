/****************************************************************************
 * arch/z16/src/1`z16f/z16f_sysexec.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
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
