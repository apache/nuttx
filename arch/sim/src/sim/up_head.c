/****************************************************************************
 * arch/sim/src/sim/up_head.c
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

#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include <syslog.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/syslog/syslog_rpmsg.h>

#include "up_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

int g_argc;
char **g_argv;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_RPMSG
static char g_logbuffer[4096];
#endif

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

__attribute__ ((visibility("default")))
int main(int argc, char **argv, char **envp)
{
  g_argc = argc;
  g_argv = argv;

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(g_logbuffer, sizeof(g_logbuffer));
#endif

  /* Start NuttX */

#ifdef CONFIG_SMP
  /* Start the CPU0 emulation.  This should not return. */

  sim_cpu0_start();
#endif
  /* Start the NuttX emulation.  This should not return. */

  nx_start();

  return EXIT_FAILURE;
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
  /* Abort simulator */

  host_abort(status);

  /* Does not really return */

  return 0;
}
#endif
