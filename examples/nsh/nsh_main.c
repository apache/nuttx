/****************************************************************************
 * nsh_main.c
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
#include <string.h>
#include <sched.h>
#include <errno.h>

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cmdmap_s
{
  const char *cmd;        /* Name of the command */
  cmd_t       handler;    /* Function that handles the command */
  ubyte       minargs;    /* Minimum number of arguments (including command) */
  ubyte       maxargs;    /* Maximum number of arguments (including command) */
  const char *usage;      /* Usage instructions for 'help' command */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cmd_help(FAR void *handle, int argc, char **argv);
static void cmd_unrecognized(FAR void *handle, int argc, char **argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char delim[] = " \t\n";

static const struct cmdmap_s g_cmdmap[] =
{
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "cat",      cmd_cat,      2, 2, "<path>" },
  { "cp",       cmd_cp,       3, 3, "<source-path> <dest-path>" },
#endif
#ifndef CONFIG_DISABLE_ENVIRON
  { "echo",     cmd_echo,     0, NSH_MAX_ARGUMENTS, "[<string|$name> [<string|$name>...]]" },
#else
  { "echo",     cmd_echo,     0, NSH_MAX_ARGUMENTS, "[<string> [<string>...]]" },
#endif
  { "exec",     cmd_exec,     2, 3, "<hex-address>" },
  { "exit",     cmd_exit,     1, 1, NULL },
  { "help",     cmd_help,     1, 1, NULL },
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
  { "ifconfig", cmd_ifconfig, 1, 1, NULL },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "ls",       cmd_ls,       2, 5, "[-lRs] <dir-path>" },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "mkdir",    cmd_mkdir,    2, 2, "<path>" },
#ifdef CONFIG_FS_FAT
  { "mkfatfs",  cmd_mkfatfs,  2, 2, "<path>" },
#endif
  { "mkfifo",   cmd_mkfifo,   2, 2, "<path>" },
#ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
  { "mount",    cmd_mount,    4, 5, "-t <fstype> <block-device> <dir-path>" },
#endif
#endif
  { "ps",       cmd_ps,       1, 1, NULL },
#ifndef CONFIG_DISABLE_ENVIRON
  { "set",      cmd_set,      3, 3, "<name> <value>" },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "rm",       cmd_rm,       2, 2, "<file-path>" },
  { "rmdir",    cmd_rmdir,    2, 2, "<dir-path>" },
#endif
#if  CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "sh",       cmd_sh,       2, 2, "<script-path>" },
# endif  /* CONFIG_NFILE_STREAMS */
#ifndef CONFIG_DISABLE_SIGNALS
  { "sleep",    cmd_sleep,    2, 2, "<sec>" },
#endif /* CONFIG_DISABLE_SIGNALS */
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
# ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
  { "umount",   cmd_umount,   2, 2, "<dir-path>" },
# endif
#endif
#ifndef CONFIG_DISABLE_ENVIRON
  { "unset",    cmd_unset,    2, 2, "<name>" },
#endif
#ifndef CONFIG_DISABLE_SIGNALS
  { "usleep",   cmd_usleep,   2, 2, "<usec>" },
#endif /* CONFIG_DISABLE_SIGNALS */
  { NULL,     NULL,           1, 1, NULL }
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const char g_nshprompt[]         = "nsh> ";
const char g_fmtargrequired[]    = "nsh: %s: missing required argument(s)\n";
const char g_fmtarginvalid[]     = "nsh: %s: argument invalid\n";
const char g_fmtcmdnotfound[]    = "nsh: %s: command not found\n";
const char g_fmtcmdnotimpl[]     = "nsh: %s: command not implemented\n";
const char g_fmtnosuch[]         = "nsh: %s: no such %s: %s\n";
const char g_fmttoomanyargs[]    = "nsh: %s: too many arguments\n";
#ifdef CONFIG_EXAMPLES_NSH_STRERROR
const char g_fmtcmdfailed[]      = "nsh: %s: %s failed: %s\n";
#else
const char g_fmtcmdfailed[]      = "nsh: %s: %s failed: %d\n";
#endif
const char g_fmtcmdoutofmemory[] = "nsh: %s: out of memory\n";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_help
 ****************************************************************************/

static void cmd_help(FAR void *handle, int argc, char **argv)
{
  const struct cmdmap_s *ptr;

  nsh_output(handle, "NSH commands:\n");
  nsh_output(handle, "  nice [-d] <cmd> &\n");
  for (ptr = g_cmdmap; ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          nsh_output(handle, "  %s %s\n", ptr->cmd, ptr->usage);
        }
      else
        {
          nsh_output(handle, "  %s\n", ptr->cmd);
        }
    }
}

/****************************************************************************
 * Name: cmd_unrecognized
 ****************************************************************************/

static void cmd_unrecognized(FAR void *handle, int argc, char **argv)
{
  nsh_output(handle, g_fmtcmdnotfound, argv[0]);
}

/****************************************************************************
 * Name: nsh_execute
 ****************************************************************************/

static int nsh_execute(int argc, char *argv[])
{
   const struct cmdmap_s *cmdmap;
   const char            *cmd;
   void                  *handle;
   cmd_t                  handler = cmd_unrecognized;

   /* Parse all of the arguments following the command name.  The form
    * of argv is:
    *
    * argv[0]:    Task name "nsh_execute"
    * argv[1]:    This is string version of the handle needed to execute
    *             the command (under telnetd).  It is a string because
    *             binary values cannot be provided via char *argv[]
    * argv[2]:    The command name.  This is argv[0] when the arguments
    *             are, finally, received by the command handler
    * argv[3]:    The beginning of argument (up to NSH_MAX_ARGUMENTS)
    * argv[argc]: NULL terminating pointer
    */

   handle  = (void*)strtol(argv[1], NULL, 16);
   cmd     = argv[2];
   argc   -= 2;

   /* See if the command is one that we understand */

   for (cmdmap = g_cmdmap; cmdmap->cmd; cmdmap++)
     {
       if (strcmp(cmdmap->cmd, cmd) == 0)
         {
           /* Check if a valid number of arguments was provided.  We
            * do this simple, imperfect checking here so that it does
            * not have to be performed in each command.
            */

           if (argc < cmdmap->minargs)
             {
               /* Fewer than the minimum number were provided */

               nsh_output(handle, g_fmtargrequired, cmd);
               return ERROR;
             }
           else if (argc > cmdmap->maxargs)
             {
               /* More than the maximum number were provided */

               nsh_output(handle, g_fmttoomanyargs, cmd);
               return ERROR;
             }
           else
             {
               /* A valid number of arguments were provided (this does
                * not mean they are right).
                */

               handler = cmdmap->handler;
               break;
             }
         }
     }

   handler(handle, argc, &argv[2]);
   return OK;
}

/****************************************************************************
 * Name: nsh_setprio
 ****************************************************************************/

static inline void nsh_setprio(void)
{
  int max_priority = sched_get_priority_max(SCHED_RR);
  int min_priority = sched_get_priority_min(SCHED_RR);
  struct sched_param param;

  param.sched_priority = (max_priority + min_priority) >> 1;
  (void)sched_setscheduler(0, SCHED_RR, &param);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_initialize
 ****************************************************************************/

void user_initialize(void)
{
  /* stub */
}

/****************************************************************************
 * Name: user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  nsh_setprio();
  return nsh_main();
}

/****************************************************************************
 * Name: nsh_parse
 ****************************************************************************/

int nsh_parse(FAR void *handle, char *cmdline)
{
  FAR char *argv[NSH_MAX_ARGUMENTS+4];
  FAR char  strhandle[2*sizeof(FAR char*)+3];
  FAR char *saveptr;
  FAR char *cmd;
  boolean   bg;
  int       nice = 0;
  int       argc;
  int       ret;

  memset(argv, 0, (NSH_MAX_ARGUMENTS+4)*sizeof(char*));

  /* Parse out the command at the beginning of the line */

  cmd = strtok_r(cmdline, " \t\n", &saveptr);
  if (cmd)
    {
      /* Check if the command is preceded by "nice" */

      if (strcmp(cmd, "nice") == 0)
        {
          /* Nicenesses range from -20 (most favorable scheduling) to 19
           * (least  favorable).  Default is 10.
           */

          nice = 10;

          /* Get the cmd (or -d option of nice command) */

          cmd = strtok_r(NULL, " \t\n", &saveptr);
          if (cmd && strcmp(cmd, "-d") == 0)
            {
              FAR char *val = strtok_r(NULL, " \t\n", &saveptr);
              if (val)
                {
                  char *endptr;
                  nice = (int)strtol(val, &endptr, 0);
                  if (nice > 19 || nice < -20 || endptr == val || *endptr != '\0')
                    {
                      nsh_output(handle, g_fmtarginvalid, "nice");
                      return ERROR;
                    }
                  cmd = strtok_r(NULL, " \t\n", &saveptr);
                }
            }
        }
    }

  /* Check if any command was provided */

  if (cmd)
    {
      /* Parse all of the arguments following the command name.  The form
       * of argv is:
       *
       * argv[0]:    Not really used.  It is just a place hold for where the
       *             task name would be if the same command were executed
       *             in the "background"
       * argv[1]:    This is string version of the handle needed to execute
       *             the command (under telnetd).  It is a string because
       *             binary values cannot be provided via char *argv[]
       * argv[2]:    The command name.  This is argv[0] when the arguments
       *             are, finally, received by the command handler
       * argv[3]:    The beginning of argument (up to NSH_MAX_ARGUMENTS)
       * argv[argc]: NULL terminating pointer
       */

      sprintf(strhandle, "%p\n", handle);
      argv[0] = "nsh_execute";
      argv[1] = strhandle;
      argv[2] = cmd;
      for (argc = 3; argc < NSH_MAX_ARGUMENTS+4; argc++)
        {
          argv[argc] = strtok_r(NULL, " \t\n", &saveptr);
          if (!argv[argc])
            {
              break;
            }
        }

      /* Check if the command should run in background */

      bg = FALSE;
      if (strcmp(argv[argc-1], "&") == 0)
        {
          bg = TRUE;
          argv[argc-1] = NULL;
          argc--;
        }

      if (argc > NSH_MAX_ARGUMENTS+3)
        {
          nsh_output(handle, g_fmttoomanyargs, cmd);
        }

      if (bg)
        {
          struct sched_param param;
          int priority;

          /* Get the execution priority of this task */

          ret = sched_getparam(0, &param);
          if (ret != 0)
            {
              nsh_output(handle, g_fmtcmdfailed, cmd, "sched_getparm", NSH_ERRNO);
              return ERROR;
            }

          /* Determine the priority to execute the command */

          priority = param.sched_priority;
          if (nice != 0)
            {
              priority -= nice;
              if (nice < 0)
                {
                  int max_priority = sched_get_priority_max(SCHED_RR);
                  if (priority > max_priority)
                    {
                      priority = max_priority;
                    }
                }
              else
                {
                  int min_priority = sched_get_priority_min(SCHED_RR);
                  if (priority < min_priority)
                    {
                      priority = min_priority;
                    }
                }
            }

          /* Execute the command as a separate task at the appropriate priority */

#ifndef CONFIG_CUSTOM_STACK
          ret = task_create("nsh_execute", priority, CONFIG_EXAMPLES_NSH_STACKSIZE,
                            nsh_execute, &argv[1]);
#else
          ret = task_create("nsh_execute", priority, nsh_execute, &argv[1]);
#endif
          if (ret < 0)
            {
              nsh_output(handle, g_fmtcmdfailed, cmd, "task_create", NSH_ERRNO);
              return ERROR;
            }
#ifdef CONFIG_DEBUG
          nsh_output(handle, "%s [%d:%d:%d]\n", cmd, ret, priority, param.sched_priority);
#else
          nsh_output(handle, "%s [%d]\n", cmd, ret);
#endif
        }
      else
        {
          ret = nsh_execute(argc, argv);
        }

      /* Return success if the command succeeded (or at least, starting of the
       * command task succeeded).
       */

      if (ret == 0)
        {
          return OK;
        }
    }

  return ERROR;
}
