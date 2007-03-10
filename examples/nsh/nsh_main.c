/************************************************************
 * nsh_main.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>

/************************************************************
 * Definitions
 ************************************************************/

#define CONFIG_NSH_LINE_SIZE 80

/************************************************************
 * Private Types
 ************************************************************/

typedef void (*cmd_t)(const char *cmd, const char *arg);
typedef void (*exec_t)(void);

struct cmdmap_s
{
  const char *cmd;
  cmd_t       handler;
  const char *usage;
};

/************************************************************
 * Private Function Prototypes
 ************************************************************/

static void cmd_echo(const char *cmd, const char *arg);
static void cmd_exec(const char *cmd, const char *arg);
static void cmd_help(const char *cmd, const char *arg);
static void cmd_ls(const char *cmd, const char *arg);
static void cmd_ps(const char *cmd, const char *arg);

/************************************************************
 * Private Data
 ************************************************************/

static char line[CONFIG_NSH_LINE_SIZE];
static const char delim[] = " \t\n";

static const struct cmdmap_s g_cmdmap[] =
{
  { "echo", cmd_echo, "<string>" },
  { "exec", cmd_exec, "<hex-address>" },
  { "help", cmd_help, NULL },
  { "ls",   cmd_ls,   "<path>" },
  { "ps",   cmd_ps,   NULL },
  { NULL,   NULL,     NULL }
};

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

static const char g_fmtargrequired[] = "nsh: %s: argument required\n";
static const char g_fmtarginvalid[]  = "nsh: %s: argument invalid\n";
static const char g_fmtcmdnotfound[] = "nsh: %s: command not found\n";
static const char g_fmtcmdnotimpl[]  = "nsh: %s: command not implemented\n";

/************************************************************
 * Private Functions
 ************************************************************/

static char *trim_arg(char *arg)
{
  if (arg)
    {
      int len;

      /* Skip any leading white space */

      while (strchr(" \t", *arg) != NULL) arg++;  

      /* Skip any leading white space */

      len = strlen(arg);
      if (len > 0)
        {
          char *ptr;

          for (ptr = &arg[strlen(arg)-1];
               ptr >= arg &&
               strchr(" \t\n", *ptr) != NULL;
               ptr--)
            {
              *ptr = '\0';
            }
        }
    }
  return arg;
}

/************************************************************
 * Name: cmd_echo
 ************************************************************/

static void cmd_echo(const char *cmd, const char *arg)
{
  /* Echo the rest of the line */

  puts(arg);
}

/************************************************************
 * Name: cmd_exec
 ************************************************************/

static void cmd_exec(const char *cmd, const char *arg)
{
  char *endptr;
  long addr;

  if (!arg)
    {
       printf(g_fmtargrequired, "exec");
       return;
    }

  addr = strtol(arg, &endptr, 0);
  if (!addr || endptr == arg || *endptr != '\0')
    {
       printf(g_fmtarginvalid, "exec");
       return;
    }

  printf("Calling %p\n", (exec_t)addr);
  ((exec_t)addr)();
}

/************************************************************
 * Name: cmd_help
 ************************************************************/

static void cmd_help(const char *cmd, const char *arg)
{
  const struct cmdmap_s *ptr;

  printf("NSH commands:\n");
  for (ptr = g_cmdmap; ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          printf("  %s %s\n", ptr->cmd, ptr->usage);
        }
      else
        {
          printf("  %s\n", ptr->cmd);
        }
    }
}

/************************************************************
 * Name: cmd_ls
 ************************************************************/

static void cmd_ls(const char *cmd, const char *arg)
{
  printf(g_fmtcmdnotimpl, cmd);
}

/************************************************************
 * Name: ps_task
 ************************************************************/

static void ps_task(FAR _TCB *tcb, FAR void *arg)
{
  boolean needcomma = FALSE;
  int i;
  printf("%5d %3d %4s %7s%c%c %8s ",
         tcb->pid, tcb->sched_priority,
         tcb->flags & TCB_FLAG_ROUND_ROBIN ? "RR  " : "FIFO",
         tcb->flags & TCB_FLAG_PTHREAD ? "PTHREAD" : "TASK   ",
         tcb->flags & TCB_FLAG_NONCANCELABLE ? 'N' : ' ',
         tcb->flags & TCB_FLAG_CANCEL_PENDING ? 'P' : ' ',
         g_statenames[tcb->task_state]);

  printf("%s(", tcb->argv[0]);
  for (i = 1; i < NUM_TASK_ARGS+1 && tcb->argv[i]; i++)
    {
      if (needcomma)
        {
          printf(", %p", tcb->argv[i]);
        }
      else
        {
          printf("%p", tcb->argv[i]);
        }
     }
  printf(")\n");
}

/************************************************************
 * Name: cmd_ps
 ************************************************************/

static void cmd_ps(const char *cmd, const char *arg)
{
  printf("PID   PRI SCHD TYPE   NP STATE    NAME\n");
  sched_foreach(ps_task, NULL);
}

/************************************************************
 * Name: cmd_unrecognized
 ************************************************************/

static void cmd_unrecognized(const char *cmd, const char *arg)
{
  printf(g_fmtcmdnotfound, cmd);
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name: user_initialize
 ************************************************************/

void user_initialize(void)
{
  /* stub */
}

/************************************************************
 * Name: user_start
 ************************************************************/

int user_start(int parm1, int parm2, int parm3, int parm4)
{
  printf("NuttShell (NSH)\n");
  fflush(stdout);

  for (;;)
    {
      const struct cmdmap_s *cmd;
      cmd_t handler = cmd_unrecognized;
      char *saveptr;
      char *token;

      /* Get the next line of input */

      fgets(line, CONFIG_NSH_LINE_SIZE, stdin);

      /* Parse out the command at the beginning of the line */

      token = strtok_r(line, " \t\n", &saveptr);
      if (token)
        {
          /* See if the command is one that we understand */

          for (cmd = g_cmdmap; cmd->cmd; cmd++)
            {
              if (strcmp(cmd->cmd, token) == 0)
                {
                  handler = cmd->handler;
                  break;
                }
            }
          handler(token, trim_arg(saveptr));
        }
      fflush(stdout);
    }
}
