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
#include <sys/stat.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <fcntl.h>
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

static void cmd_help(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
static void cmd_exit(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
static void cmd_unrecognized(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_delim[]      = " \t\n";
static const char g_redirect1[]  = ">";
static const char g_redirect2[]  = ">>";

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

static void cmd_help(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  const struct cmdmap_s *ptr;

  nsh_output(vtbl, "NSH command form:\n");
  nsh_output(vtbl, "  [nice [-d <niceness>>]] <cmd> [[> <file>|>> <file>] &]\n");
  nsh_output(vtbl, "Where <cmd> is one of:\n");
  for (ptr = g_cmdmap; ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          nsh_output(vtbl, "  %s %s\n", ptr->cmd, ptr->usage);
        }
      else
        {
          nsh_output(vtbl, "  %s\n", ptr->cmd);
        }
    }
}

/****************************************************************************
 * Name: cmd_unrecognized
 ****************************************************************************/

static void cmd_unrecognized(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_output(vtbl, g_fmtcmdnotfound, argv[0]);
}

/****************************************************************************
 * Name: cmd_exit
 ****************************************************************************/

static void cmd_exit(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_exit(vtbl);
}

/****************************************************************************
 * Name: nsh_execute
 ****************************************************************************/

static int nsh_execute(int argc, char *argv[])
{
   const struct cmdmap_s *cmdmap;
   const char            *cmd;
   struct nsh_vtbl_s     *vtbl;
   cmd_t                  handler = cmd_unrecognized;

   /* Parse all of the arguments following the command name.  The form
    * of argv is:
    *
    * argv[0]:      Task name "nsh_execute"
    * argv[1]:      This is string version of the vtbl needed to execute
    *               the command (under telnetd).  It is a string because
    *               binary values cannot be provided via char *argv[]
    * argv[2]:      The command name.  This is argv[0] when the arguments
    *               are, finally, received by the command vtblr
    * argv[3]:      The beginning of argument (up to NSH_MAX_ARGUMENTS)
    * argv[argc]:   NULL terminating pointer
    *
    * Maximum size is NSH_MAX_ARGUMENTS+4
    */

   vtbl  = (struct nsh_vtbl_s*)strtol(argv[1], NULL, 16);
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

               nsh_output(vtbl, g_fmtargrequired, cmd);
               return ERROR;
             }
           else if (argc > cmdmap->maxargs)
             {
               /* More than the maximum number were provided */

               nsh_output(vtbl, g_fmttoomanyargs, cmd);
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

   handler(vtbl, argc, &argv[2]);
   nsh_release(vtbl);
   return OK;
}

/****************************************************************************
 * Name: nsh_argument
 ****************************************************************************/

char *nsh_argument(char **saveptr)
{
  char *pbegin = *saveptr;
  char *pend   = NULL;
  const char *term;
#ifndef CONFIG_DISABLE_ENVIRON
  boolean quoted = FALSE;
#endif

  /* Find the beginning of the next token */

  for (;
       *pbegin && strchr(g_delim, *pbegin) != NULL;
       pbegin++);

  /* If we are at the end of the string with nothing
   * but delimiters found, then return NULL.
   */

  if (!*pbegin)
    {
      return NULL;
    }

  /* Does the token begin with '>' */

  if (*pbegin == '>')
    {
      /* Yes.. does it begin with ">>"? */

      if (*(pbegin + 1) == '>')
        {
          *saveptr = pbegin + 2;
	  pbegin = g_redirect2;
        }
      else
        {
          *saveptr = pbegin + 1;
	  pbegin = g_redirect1;
        }
    }
  else
    {
      /* Does the token begin with '"'? */

      if (*pbegin == '"')
        {
          /* Yes.. then only another '"' can terminate the string */

          pbegin++;
          term = "\"";
#ifndef CONFIG_DISABLE_ENVIRON
          quoted = TRUE;
#endif
        }
      else
        {
          /* No, then any of the usual terminators will terminate the argument */

          term = g_delim;
        }

      /* Find the end of the string */

      for (pend = pbegin + 1;
           *pend && strchr(term, *pend) == NULL;
           pend++);

      /* pend either points to the end of the string or to
       * the first delimiter after the string.
       */

      if (*pend)
        {
          /* Turn the delimiter into a null terminator */

          *pend++ = '\0';
        }

      /* Save the pointer where we left off */

      *saveptr = pend;
    }

  /* Check for references to environment variables */

#ifndef CONFIG_DISABLE_ENVIRON
  if (pbegin[0] == '$' && !quoted)
    {
      /* Yes.. return the value of the environment variable with this name */

      char *value = getenv(pbegin+1);
      if (value)
        {
          return value;
        }
      else
        {
          return "";
        }
    }
#endif

  /* Return the beginning of the token. */

  return pbegin;
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
  int mid_priority;
  int ret;

  /* Set the priority of this task to something in the middle so that 'nice'
   * can both raise and lower the priority.
   */

  mid_priority = (sched_get_priority_max(SCHED_RR) + sched_get_priority_min(SCHED_RR)) >> 1;
    {
      struct sched_param param;

      param.sched_priority = mid_priority;
      (void)sched_setscheduler(0, SCHED_RR, &param);
    }

  /* If both the console and telnet are selected as front-ends, then run
   * the telnet front end on another thread.
   */

#if defined(CONFIG_EXAMPLES_NSH_CONSOLE) && defined(CONFIG_EXAMPLES_NSH_TELNET)
# ifndef CONFIG_CUSTOM_STACK
  ret = task_create("nsh_telnetmain", mid_priority, CONFIG_EXAMPLES_NSH_STACKSIZE,
                    nsh_telnetmain, NULL);
# else
  ret = task_create("nsh_telnetmain", mid_priority, nsh_telnetmain, NULL);
# endif
  if (ret < 0)
   {
     fprintf(stderr, g_fmtcmdfailed, "user_start", "task_create", NSH_ERRNO);
   }

  /* If only the telnet front-end is selected, run it on this thread */

#elif defined(CONFIG_EXAMPLES_NSH_TELNET)
  return nsh_telnetmain(0, NULL);
#endif

/* If the serial console front end is selected, then run it on this thread */

#ifdef CONFIG_EXAMPLES_NSH_CONSOLE
  return nsh_consolemain(0, NULL);
#endif
}

/****************************************************************************
 * Name: nsh_parse
 ****************************************************************************/

int nsh_parse(FAR struct nsh_vtbl_s *vtbl, char *cmdline)
{
  FAR char *argv[NSH_MAX_ARGUMENTS+7];
  FAR char  strvtbl[2*sizeof(FAR char*)+3];
  FAR char *saveptr;
  FAR char *cmd;
  FAR char *redirfile;
  boolean   bg = FALSE;
  boolean   redirect = FALSE;
  int       fd = -1;
  int       nice = 0;
  int       oflags;
  int       argc;
  int       ret;

  memset(argv, 0, (NSH_MAX_ARGUMENTS+4)*sizeof(char*));

  /* Parse out the command at the beginning of the line */

  saveptr = cmdline;
  cmd = nsh_argument(&saveptr);
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

          cmd = nsh_argument(&saveptr);
          if (cmd && strcmp(cmd, "-d") == 0)
            {
              FAR char *val = nsh_argument(&saveptr);
              if (val)
                {
                  char *endptr;
                  nice = (int)strtol(val, &endptr, 0);
                  if (nice > 19 || nice < -20 || endptr == val || *endptr != '\0')
                    {
                      nsh_output(vtbl, g_fmtarginvalid, "nice");
                      return ERROR;
                    }
                  cmd = nsh_argument(&saveptr);
                }
            }
        }
    }

  /* Check if any command was provided */

  if (!cmd)
    {
      return OK; /* Newline only is not an error */
    }

  /* Parse all of the arguments following the command name.  The form
   * of argv is:
   *
   * argv[0]:      Not really used.  It is just a place hold for where the
   *               task name would be if the same command were executed
   *               in the "background"
   * argv[1]:      This is string version of the vtbl needed to execute
   *               the command (under telnetd).  It is a string because
   *               binary values cannot be provided via char *argv[].  NOTE
   *               that this value is filled in later.
   * argv[2]:      The command name.  This is argv[0] when the arguments
   *               are, finally, received by the command vtblr
   * argv[3]:      The beginning of argument (up to NSH_MAX_ARGUMENTS)
   * argv[argc-3]: Possibly '>' or '>>'
   * argv[argc-2]: Possibly <file>
   * argv[argc-1]: Possibly '&'
   * argv[argc]:   NULL terminating pointer
   *
   * Maximum size is NSH_MAX_ARGUMENTS+7
   */

  argv[0] = "nsh_execute";
  argv[2] = cmd;
  for (argc = 3; argc < NSH_MAX_ARGUMENTS+6; argc++)
    {
      argv[argc] = nsh_argument(&saveptr);
      if (!argv[argc])
        {
          break;
        }
    }
  argv[argc] = NULL;

  /* Check if the command should run in background */

  if (argc > 3 && strcmp(argv[argc-1], "&") == 0)
    {
      bg = TRUE;
      argv[argc-1] = NULL;
      argc--;
    }

  /* Check if the output was re-directed using > or >> */

  if (argc > 5)
    {
      /* Check for redirection to a new file */

      if (strcmp(argv[argc-2], g_redirect1) == 0)
        {
          redirect  = TRUE;
          oflags    = O_WRONLY|O_CREAT|O_TRUNC;
          redirfile = argv[argc-1];
          argc     -= 2;
        }

      /* Check for redirection by appending to an existing file */

      else if (strcmp(argv[argc-2], g_redirect2) == 0)
        {
          redirect  = TRUE;
          oflags    = O_WRONLY|O_CREAT|O_APPEND;
          redirfile = argv[argc-1];
          argc     -= 2;
        }
    }

  /* Redirected output? */

  if (redirect)
    {
      /* Open the redirection file.  This file will eventually
       * be closed by a call to either nsh_release (if the command
       * is executed in the background) or by nsh_undirect if the
       * command is executed in the foreground.
       */

      fd = open(redirfile, oflags, 0666);
      if (fd < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "open", NSH_ERRNO);
          return ERROR;
        }
    }

  /* Check if the maximum number of arguments was exceeded */

  if (argc > NSH_MAX_ARGUMENTS+3)
    {
      nsh_output(vtbl, g_fmttoomanyargs, cmd);
    }

  /* Handle the case where the command is executed in background */

  if (bg)
    {
      struct sched_param param;
      int priority;
      struct nsh_vtbl_s *bkgvtbl;

      /* Get a cloned copy of the vtbl with reference count=1.
       * after the command has been processed, the nsh_release() call
       * at the end of nsh_execute() will destroy the clone.
       */

      bkgvtbl = nsh_clone(vtbl);

      /* Place a string copy of the cloned vtbl in the argument list */

      sprintf(strvtbl, "%p\n", bkgvtbl);
      argv[1] = strvtbl;

      /* Handle redirection of output via a file descriptor */

      if (redirect)
        {
          (void)nsh_redirect(bkgvtbl, fd, NULL);
        }

      /* Get the execution priority of this task */

      ret = sched_getparam(0, &param);
      if (ret != 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "sched_getparm", NSH_ERRNO);
          goto errout;
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
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "task_create", NSH_ERRNO);
          goto errout;
        }
      nsh_output(vtbl, "%s [%d:%d:%d]\n", cmd, ret, priority, param.sched_priority);

      /* If the output was redirected, then file descriptor should
       * be closed.  The created task has its one, independent copy of
       * the file descriptor
       */

      if (redirect)
        {
          (void)close(fd);
        }
    }
  else
    {
      ubyte save[SAVE_SIZE];

      /* Increment the reference count on the vtbl.  This reference count will
       * be decremented at the end of nsh_execute() and exists only for compatibility
       * with the background command logic.
       */

      nsh_addref(vtbl);

      /* Place a string copy of the original vtbl in the argument list */

      sprintf(strvtbl, "%p\n", vtbl);
      argv[1] = strvtbl;

      /* Handle redirection of output via a file descriptor */

      if (redirect)
        {
          nsh_redirect(vtbl, fd, save);
        }

      /* Then execute the command in "foreground" -- i.e., while the user waits
       * for the next prompt.
       */

      ret = nsh_execute(argc, argv);

      /* Restore the original output.  Undirect will close the redirection
       * file descriptor.
       */

      if (redirect)
        {
          nsh_undirect(vtbl, save);
        }

      if (ret < 0)
        {
          return ERROR;
        }
    }

  /* Return success if the command succeeded (or at least, starting of the
   * command task succeeded).
   */

  return OK;

errout:
  if (redirect)
    {
      close(fd);
    }
  return ERROR;
}
