/****************************************************************************
 * arch/sim/src/up_head.c
 *
 *   Copyright (C) 2007-2009, 2011-2013, 2016 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/power/pm.h>

#include "up_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static jmp_buf g_simabort;
static int g_exitcode = EXIT_SUCCESS;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   This is the main entry point into the simulation.
 *
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
#ifdef CONFIG_SMP
  /* In the SMP case, configure the main thread as CPU 0 */

  sim_cpu0_initialize();
#endif

#ifdef CONFIG_PM
  /* Power management should be initialized early in the (simulated) boot
   * sequence.
   */

  pm_initialize();
#endif

  /* Then start NuttX */

  if (setjmp(g_simabort) == 0)
    {
#ifdef CONFIG_SMP
      /* Start the CPU0 emulation.  This should not return. */

      sim_cpu0_start();
#else
      /* Start the Nuttx emulation.  This should not return. */

      os_start();
#endif
    }

  /* Restore the original terminal mode and return the exit code */

  simuart_terminate();
  return g_exitcode;
}

/****************************************************************************
 * Name: up_assert
 *
 * Description:
 *   Called to terminate the simulation abnormally in the event of a failed
 *   assertion.
 *
 ****************************************************************************/

void up_assert(const uint8_t *filename, int line)
{
  /* Show the location of the failed assertion */

#ifdef CONFIG_SMP
  fprintf(stderr, "CPU%d: Assertion failed at file:%s line: %d\n",
          up_cpu_index(), filename, line);
#else
  fprintf(stderr, "Assertion failed at file:%s line: %d\n",
          filename, line);
#endif

  /* Allow for any board/configuration specific crash information */

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), this_task(), filename, line);
#endif

  /* Exit the simulation */

  g_exitcode = EXIT_FAILURE;
  longjmp(g_simabort, 1);
}

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Power off the board.  This function may or may not be supported by a
 *   particular board architecture.
 *
 * Input Parameters:
 *   status - Status information provided with the power off event.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_POWEROFF
int board_power_off(int status)
{
  /* Save the return code and exit the simulation */

  g_exitcode = status;
  longjmp(g_simabort, 1);
}
#endif
