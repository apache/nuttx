/****************************************************************************
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdio.h>
#include <string.h>

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define CONFIG_NSH_LINE_SIZE 80
#undef  CONFIG_FULL_PATH
#define NSH_MAX_ARGUMENTS     6

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

static void cmd_help(int argc, char **argv);
static void cmd_echo(int argc, char **argv);
static void cmd_unrecognized(int argc, char **argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char line[CONFIG_NSH_LINE_SIZE];
static const char delim[] = " \t\n";

static const struct cmdmap_s g_cmdmap[] =
{
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "cat",    cmd_cat,    2, 2, "<path>" },
  { "cp",     cmd_cp,     3, 3, "<source-path> <dest-path>" },
#endif
  { "echo",   cmd_echo,   2, 2, "<string>" },
  { "exec",   cmd_exec,   2, 3, "<hex-address>" },
  { "help",   cmd_help,   1, 1, NULL },
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "ls",     cmd_ls,     2, 5, "[-lRs] <dir-path>" },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "mkdir",  cmd_mkdir,  2, 2, "<path>" },
  { "mount",  cmd_mount,  4, 5, "-t <fstype> <block-device> <dir-path>" },
#endif
  { "ps",     cmd_ps,     1, 1, NULL },
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "rm",     cmd_rm,     2, 2, "<file-path>" },
  { "rmdir",  cmd_rmdir,  2, 2, "<dir-path>" },
  { "umount", cmd_umount, 2, 2, "<dir-path>" },
#endif
  { NULL,     NULL,       1, 1, NULL }
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const char g_fmtargrequired[]    = "nsh: %s: missing required argument(s)\n";
const char g_fmtarginvalid[]     = "nsh: %s: argument invalid\n";
const char g_fmtcmdnotfound[]    = "nsh: %s: command not found\n";
const char g_fmtcmdnotimpl[]     = "nsh: %s: command not implemented\n";
const char g_fmtnosuch[]         = "nsh: %s: no such %s: %s\n";
const char g_fmttoomanyargs[]    = "nsh: %s: too many arguments\n";
const char g_fmtcmdfailed[]      = "nsh: %s: %s failed: %s\n";
const char g_fmtcmdoutofmemory[] = "nsh: %s: out of memory\n";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_echo
 ****************************************************************************/

static void cmd_echo(int argc, char **argv)
{
  /* Echo the rest of the line */

  puts(argv[1]);
}

/****************************************************************************
 * Name: cmd_help
 ****************************************************************************/

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

/****************************************************************************
 * Name: cmd_unrecognized
 ****************************************************************************/

static void cmd_unrecognized(int argc, char **argv)
{
  printf(g_fmtcmdnotfound, argv[0]);
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
