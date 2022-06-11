/****************************************************************************
 * arch/sim/src/sim/up_assert.c
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
#include <sched/sched.h>
#include <debug.h>
#include <stdlib.h>
#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_BOARD_CRASHDUMP
#  include <nuttx/board.h>
#endif

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 *
 * Description:
 *   Called to terminate the simulation abnormally in the event of a failed
 *   assertion.
 *
 ****************************************************************************/

void up_assert(const char *filename, int lineno)
{
  struct tcb_s *rtcb = running_task();

  /* Flush any buffered SYSLOG data (prior to the assertion) */

  syslog_flush();

  /* Show the location of the failed assertion */

#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed CPU%d at file:%s line: %d task: %s\n",
         up_cpu_index(), filename, lineno, rtcb->name);
#else
  _alert("Assertion failed CPU%d at file:%s line: %d\n",
         up_cpu_index(), filename, lineno);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed at file:%s line: %d task: %s\n",
         filename, lineno, rtcb->name);
#else
  _alert("Assertion failed at file:%s line: %d\n",
         filename, lineno);
#endif
#endif

  /* Show back trace */

#ifdef CONFIG_SCHED_BACKTRACE
  sched_dumpstack(rtcb->pid);
#endif

  /* Flush any buffered SYSLOG data (from the above) */

  syslog_flush();

  /* Allow for any board/configuration specific crash information */

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), rtcb, filename, lineno);
#endif

  /* Flush any buffered SYSLOG data */

  syslog_flush();

  if (CURRENT_REGS || rtcb->flink == NULL)
    {
      /* Exit the simulation */

      host_abort(EXIT_FAILURE);
    }
}
