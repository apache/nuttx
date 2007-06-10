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
 * Included Files
 ************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#ifndef CONFIG_DISABLE_MOUNTPOINT
# include <sys/stat.h>
# include <sys/mount.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <sched.h>
#include <errno.h>

/************************************************************
 * Definitions
 ************************************************************/

#define CONFIG_NSH_LINE_SIZE 80
#undef  CONFIG_FULL_PATH
#define NSH_MAX_ARGUMENTS     6

#define errno                (*get_errno_ptr())

/************************************************************
 * Private Types
 ************************************************************/

typedef void (*cmd_t)(int argc, char **argv);
typedef void (*exec_t)(void);

struct cmdmap_s
{
  const char *cmd;        /* Name of the command */
  cmd_t       handler;    /* Function that handles the command */
  ubyte       minargs;    /* Minimum number of arguments (including command) */
  ubyte       maxargs;    /* Maximum number of arguments (including command) */
  const char *usage;      /* Usage instructions for 'help' command */
};

/************************************************************
 * Private Function Prototypes
 ************************************************************/

static void cmd_echo(int argc, char **argv);
static void cmd_exec(int argc, char **argv);
static void cmd_help(int argc, char **argv);
static void cmd_ls(int argc, char **argv);
#ifndef CONFIG_DISABLE_MOUNTPOINT
static void cmd_mkdir(int argc, char **argv);
static void cmd_mount(int argc, char **argv);
#endif
static void cmd_ps(int argc, char **argv);
#ifndef CONFIG_DISABLE_MOUNTPOINT
static void cmd_umount(int argc, char **argv);
#endif

/************************************************************
 * Private Data
 ************************************************************/

static char line[CONFIG_NSH_LINE_SIZE];
static const char delim[] = " \t\n";

static const struct cmdmap_s g_cmdmap[] =
{
  { "echo",   cmd_echo,   2, 2, "<string>" },
  { "exec",   cmd_exec,   2, 3, "<hex-address>" },
  { "help",   cmd_help,   1, 1, NULL },
  { "ls",     cmd_ls,     2, 2, "<path>" },
#ifndef CONFIG_DISABLE_MOUNTPOINT
  { "mkdir",  cmd_mkdir,  2, 2, "<path>" },
  { "mount",  cmd_mount,  4, 5, "-t <fstype> <device> <dir>" },
#endif
  { "ps",     cmd_ps,     1, 1, NULL },
#ifndef CONFIG_DISABLE_MOUNTPOINT
  { "umount", cmd_umount, 2, 2, "<mountpoint-dir>" },
#endif
  { NULL,     NULL,       1, 1, NULL }
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

static const char g_fmtargrequired[] = "nsh: %s: missing required argument(s)\n";
static const char g_fmtarginvalid[]  = "nsh: %s: argument invalid\n";
static const char g_fmtcmdnotfound[] = "nsh: %s: command not found\n";
static const char g_fmtcmdnotimpl[]  = "nsh: %s: command not implemented\n";
static const char g_fmtnosuch[]      = "nsh: %s: no such %s: %s\n";
static const char g_fmttoomanyargs[] = "nsh: %s: too many arguments\n";
static const char g_fmtcmdfailed[]   = "nsh: %s: %s failed: %s\n";

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Name: trim_dir
 ************************************************************/

#ifdef CONFIG_FULL_PATH
void trim_dir(char *arg)
{
 /* Skip any '/' characters white space */

 int len = strlen(arg) - 1;
 while (len > 0 && arg[len] == '/')
   {
      arg[len] = '\0';
      len--;
   }
}
#endif

/************************************************************
 * Name: cmd_echo
 ************************************************************/

static void cmd_echo(int argc, char **argv)
{
  /* Echo the rest of the line */

  puts(argv[1]);
}

/************************************************************
 * Name: cmd_exec
 ************************************************************/

static void cmd_exec(int argc, char **argv)
{
  char *endptr;
  long addr;

  addr = strtol(argv[1], &endptr, 0);
  if (!addr || endptr == argv[1] || *endptr != '\0')
    {
       printf(g_fmtarginvalid, argv[0]);
       return;
    }

  printf("Calling %p\n", (exec_t)addr);
  ((exec_t)addr)();
}

/************************************************************
 * Name: cmd_help
 ************************************************************/

static void cmd_help(int argc, char **argv)
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

static void cmd_ls(int argc, char **argv)
{
  DIR *dirp;

#ifdef CONFIG_FULL_PATH
  trim_dir(arg);
#endif
  dirp = opendir(argv[1]);

  if (!dirp)
    {
      printf(g_fmtnosuch, argv[0], "directory", argv[1]);
    }

  for (;;)
    {
      struct dirent *entryp = readdir(dirp);
      if (!entryp)
        {
          break;
        }

      if (DIRENT_ISDIRECTORY(entryp->d_type))
        {
#ifdef CONFIG_FULL_PATH
          printf("  %s/%s/\n", arg, entryp->d_name);
#else
          printf("  %s/\n", entryp->d_name);
#endif
        }
      else
        {
#ifdef CONFIG_FULL_PATH
          printf("  %s/%s\n", arg, entryp->d_name);
#else
          printf("  %s\n", entryp->d_name);
#endif
        }

    }
  closedir(dirp);
}

/************************************************************
 * Name: cmd_mount
 ************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static void cmd_mkdir(int argc, char **argv)
{
  int result = mkdir(argv[1], 0777);
  if ( result < 0)
    {
      printf(g_fmtcmdfailed, argv[0], "mkdir", strerror(errno));
    }
}
#endif

/************************************************************
 * Name: cmd_mount
 ************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static void cmd_mount(int argc, char **argv)
{
  char *filesystem = 0;
  int result;

  /* Get the mount options */

  int option;
  while ((option = getopt(argc, argv, ":t:")) != ERROR)
    {
      switch (option)
        {
          case 't':
            filesystem = optarg;
            break;

          case ':':
            printf(g_fmtargrequired, argv[0]);
            return;

          case '?':
          default:
            printf(g_fmtarginvalid, argv[0]);
            return;
        }
    }

  /* There are two required arguments after the options */

  if (optind + 2 >  argc)
    {
      printf(g_fmtargrequired, argv[0]);
      return;
    }

  /* Perform the mount */
  result = mount(argv[optind], argv[optind+1], filesystem, 0, NULL);
  if ( result < 0)
    {
      printf(g_fmtcmdfailed, argv[0], "mount", strerror(errno));
    }
}
#endif

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
  for (i = 1; i < CONFIG_MAX_TASK_ARGS+1 && tcb->argv[i]; i++)
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

static void cmd_ps(int argc, char **argv)
{
  printf("PID   PRI SCHD TYPE   NP STATE    NAME\n");
  sched_foreach(ps_task, NULL);
}

/************************************************************
 * Name: cmd_umount
 ************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static void cmd_umount(int argc, char **argv)
{
  /* Perform the umount */
  int result = umount(argv[1]);
  if ( result < 0)
    {
      printf(g_fmtcmdfailed, argv[0], "umount", strerror(errno));
    }
}
#endif

/************************************************************
 * Name: cmd_unrecognized
 ************************************************************/

static void cmd_unrecognized(int argc, char **argv)
{
  printf(g_fmtcmdnotfound, argv[0]);
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

int user_start(int argc, char *argv[])
{
  printf("NuttShell (NSH)\n");
  fflush(stdout);

  for (;;)
    {
      const struct cmdmap_s *cmdmap;
      char *saveptr;
      char *cmd;

      /* Get the next line of input */

      fgets(line, CONFIG_NSH_LINE_SIZE, stdin);

      /* Parse out the command at the beginning of the line */

      cmd = strtok_r(line, " \t\n", &saveptr);
      if (cmd)
        {
          cmd_t handler = cmd_unrecognized;

          /* Parse all of the arguments following the command name */

          char *cmd_argv[NSH_MAX_ARGUMENTS+1];
          int   cmd_argc;

          cmd_argv[0] = cmd;
          for (cmd_argc = 1; cmd_argc < NSH_MAX_ARGUMENTS+1; cmd_argc++)
            {
              cmd_argv[cmd_argc] = strtok_r( NULL, " \t\n", &saveptr);
              if ( !cmd_argv[cmd_argc] )
                {
                  break;
                }
            }

          if (cmd_argc > NSH_MAX_ARGUMENTS)
            {
              printf(g_fmttoomanyargs, cmd);
            }

          /* See if the command is one that we understand */

          for (cmdmap = g_cmdmap; cmdmap->cmd; cmdmap++)
            {
              if (strcmp(cmdmap->cmd, cmd) == 0)
                {
                  /* Check if a valid number of arguments was provided.  We
               * do this simple, imperfect checking here so that it does
               * not have to be performed in each command.
               */

                  if (cmd_argc < cmdmap->minargs)
                    {
                      /* Fewer than the minimum number were provided */

                      printf(g_fmtargrequired, cmd);
                      handler = NULL;
                      break;
                    }
                  else if (cmd_argc > cmdmap->maxargs)
                    {
                      /* More than the maximum number were provided */

                      printf(g_fmttoomanyargs, cmd);
                      handler = NULL;
                      break;
                    }
                  else
                    {
                      /* A valid number of arguments were provided (this does
                  * not mean they are right.
                  */

                      handler = cmdmap->handler;
                      break;
                    }
              }
            }

          /* If a error was detected above, handler will be nullified to
         * prevent reporting multiple errors.
         */

          if (handler)
            {
              handler(cmd_argc, cmd_argv);
            }
        }
      fflush(stdout);
    }
}
