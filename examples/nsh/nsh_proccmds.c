/****************************************************************************
 * examples/nsh/nsh_proccmds.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*exec_t)(void);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_statenames[] =
{
  "INVALID ",
  "PENDING ",
  "READY   ", 
  "RUNNING ", 
  "INACTIVE", 
  "WAITSEM ", 
#ifndef CONFIG_DISABLE_MQUEUE
  "WAITSIG ", 
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  "MQNEMPTY", 
  "MQNFULL "
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ps_task
 ****************************************************************************/

static void ps_task(FAR _TCB *tcb, FAR void *arg)
{
  struct nsh_vtbl_s *vtbl = (struct nsh_vtbl_s*)arg;
  int i;

  /* Show task status */

  nsh_output(vtbl, "%5d %3d %4s %7s%c%c %8s ",
             tcb->pid, tcb->sched_priority,
             tcb->flags & TCB_FLAG_ROUND_ROBIN ? "RR  " : "FIFO",
             tcb->flags & TCB_FLAG_PTHREAD ? "PTHREAD" : "TASK   ",
             tcb->flags & TCB_FLAG_NONCANCELABLE ? 'N' : ' ',
             tcb->flags & TCB_FLAG_CANCEL_PENDING ? 'P' : ' ',
             g_statenames[tcb->task_state]);

  /* Show task name and arguments */

  nsh_output(vtbl, "%s(", tcb->argv[0]);

  /* Special case 1st argument (no comma) */

  if (tcb->argv[1])
    {
     nsh_output(vtbl, "%p", tcb->argv[1]);
    }

  /* Then any additional arguments */

#if CONFIG_MAX_TASK_ARGS > 2
  for (i = 2; i <= CONFIG_MAX_TASK_ARGS && tcb->argv[i]; i++)
    {
      nsh_output(vtbl, ", %p", tcb->argv[i]);
     }
#endif
  nsh_output(vtbl, ")\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_exec
 ****************************************************************************/

int cmd_exec(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *endptr;
  long addr;

  addr = strtol(argv[1], &endptr, 0);
  if (!addr || endptr == argv[1] || *endptr != '\0')
    {
       nsh_output(vtbl, g_fmtarginvalid, argv[0]);
       return ERROR;
    }

  nsh_output(vtbl, "Calling %p\n", (exec_t)addr);
  ((exec_t)addr)();
  return OK;
}

/****************************************************************************
 * Name: cmd_ps
 ****************************************************************************/

int cmd_ps(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_output(vtbl, "PID   PRI SCHD TYPE   NP STATE    NAME\n");
  sched_foreach(ps_task, vtbl);
  return OK;
}

/****************************************************************************
 * Name: cmd_sleep
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
int cmd_sleep(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *endptr;
  long secs;

  secs = strtol(argv[1], &endptr, 0);
  if (!secs || endptr == argv[1] || *endptr != '\0')
    {
       nsh_output(vtbl, g_fmtarginvalid, argv[0]);
       return ERROR;
    }
  sleep(secs);
  return OK;
}
#endif

/****************************************************************************
 * Name: cmd_usleep
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
int cmd_usleep(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *endptr;
  long usecs;

  usecs = strtol(argv[1], &endptr, 0);
  if (!usecs || endptr == argv[1] || *endptr != '\0')
    {
       nsh_output(vtbl, g_fmtarginvalid, argv[0]);
       return ERROR;
    }
  usleep(usecs);
  return OK;
}
#endif
