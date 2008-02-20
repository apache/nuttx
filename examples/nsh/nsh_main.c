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
#include <string.h>

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define CONFIG_NSH_LINE_SIZE 80
#undef  CONFIG_FULL_PATH

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
  { "cat",      cmd_cat,    2, 2, "<path>" },
  { "cp",       cmd_cp,     3, 3, "<source-path> <dest-path>" },
#endif
#ifndef CONFIG_DISABLE_ENVIRON
  { "echo",     cmd_echo,   0, NSH_MAX_ARGUMENTS, "[<string|$name> [<string|$name>...]]" },
#else
  { "echo",     cmd_echo,   0, NSH_MAX_ARGUMENTS, "[<string> [<string>...]]" },
#endif
  { "exec",     cmd_exec,   2, 3, "<hex-address>" },
  { "exit",     cmd_exit,   1, 1, NULL },
  { "help",     cmd_help,   1, 1, NULL },
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
  { "ifconfig", cmd_ifconfig, 1, 1, NULL },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "ls",       cmd_ls,     2, 5, "[-lRs] <dir-path>" },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "mkdir",    cmd_mkdir,  2, 2, "<path>" },
#ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
  { "mount",    cmd_mount,  4, 5, "-t <fstype> <block-device> <dir-path>" },
#endif
#endif
  { "ps",       cmd_ps,     1, 1, NULL },
#ifndef CONFIG_DISABLE_ENVIRON
  { "set",      cmd_set,    3, 3, "<name> <value>" },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "rm",       cmd_rm,     2, 2, "<file-path>" },
  { "rmdir",    cmd_rmdir,  2, 2, "<dir-path>" },
# ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
  { "umount",   cmd_umount, 2, 2, "<dir-path>" },
#endif
#endif
#ifndef CONFIG_DISABLE_ENVIRON
  { "unset",  cmd_unset,  2, 2, "<name>" },
#endif
  { NULL,     NULL,       1, 1, NULL }
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
#ifdef CONFIG_NSH_STRERROR
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
  return nsh_main();
}

/****************************************************************************
 * Name: nsh_parse
 ****************************************************************************/

int nsh_parse(FAR void *handle, char *cmdline)
{
  const struct cmdmap_s *cmdmap;
  char *argv[NSH_MAX_ARGUMENTS+1];
  char *saveptr;
  char *cmd;
  int   argc;

  /* Parse out the command at the beginning of the line */

  cmd = strtok_r(cmdline, " \t\n", &saveptr);
  if (cmd)
    {
      cmd_t handler = cmd_unrecognized;

      /* Parse all of the arguments following the command name */


      argv[0] = cmd;
      for (argc = 1; argc < NSH_MAX_ARGUMENTS+1; argc++)
        {
          argv[argc] = strtok_r( NULL, " \t\n", &saveptr);
          if (!argv[argc])
            {
              break;
            }
        }

      if (argc > NSH_MAX_ARGUMENTS)
        {
          nsh_output(handle, g_fmttoomanyargs, cmd);
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

      handler(handle, argc, argv);
      return OK;
    }

  return ERROR;
}
