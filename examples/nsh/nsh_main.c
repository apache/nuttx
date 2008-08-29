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

static int cmd_help(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
static int cmd_exit(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
static int cmd_unrecognized(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_delim[]      = " \t\n";
static const char g_redirect1[]  = ">";
static const char g_redirect2[]  = ">>";
static const char g_exitstatus[] = "$?";
static const char g_success[]    = "0";
static const char g_failure[]    = "1";

static const struct cmdmap_s g_cmdmap[] =
{
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "cat",      cmd_cat,      2, NSH_MAX_ARGUMENTS, "<path> [<path> [<path> ...]]" },
#ifndef CONFIG_DISABLE_ENVIRON
  { "cd",       cmd_cd,       1, 2, "[<dir-path>|-|~|..]" },
#endif
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
  { "ls",       cmd_ls,       1, 5, "[-lRs] <dir-path>" },
#endif
  { "mb",       cmd_mb,       2, 3, "<hex-address>[=<hex-value>][ <hex-byte-count>]" },
  { "mem",      cmd_mem,      1, 1, NULL },
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "mkdir",    cmd_mkdir,    2, 2, "<path>" },
#ifdef CONFIG_FS_FAT
  { "mkfatfs",  cmd_mkfatfs,  2, 2, "<path>" },
#endif
  { "mkfifo",   cmd_mkfifo,   2, 2, "<path>" },
#endif
  { "mh",       cmd_mh,       2, 3, "<hex-address>[=<hex-value>][ <hex-byte-count>]" },
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
#ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
  { "mount",    cmd_mount,    4, 5, "-t <fstype> <block-device> <dir-path>" },
#endif
#endif
  { "mw",       cmd_mw,       2, 3, "<hex-address>[=<hex-value>][ <hex-byte-count>]" },
  { "ps",       cmd_ps,       1, 1, NULL },
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
  { "pwd",      cmd_pwd,      1, 1, NULL },
#endif
#ifndef CONFIG_DISABLE_ENVIRON
  { "set",      cmd_set,      3, 3, "<name> <value>" },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "rm",       cmd_rm,       2, 2, "<file-path>" },
  { "rmdir",    cmd_rmdir,    2, 2, "<dir-path>" },
#endif
#if  CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "sh",       cmd_sh,       2, 2, "<script-path>" },
#endif  /* CONFIG_NFILE_DESCRIPTORS && CONFIG_NFILE_STREAMS */
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

const char g_nshgreeting[]       = "NuttShell (NSH)\n";
const char g_nshprompt[]         = "nsh> ";
const char g_fmtargrequired[]    = "nsh: %s: missing required argument(s)\n";
const char g_fmtarginvalid[]     = "nsh: %s: argument invalid\n";
const char g_fmtargrange[]       = "nsh: %s: value out of range\n";
const char g_fmtcmdnotfound[]    = "nsh: %s: command not found\n";
const char g_fmtcmdnotimpl[]     = "nsh: %s: command not implemented\n";
const char g_fmtnosuch[]         = "nsh: %s: no such %s: %s\n";
const char g_fmttoomanyargs[]    = "nsh: %s: too many arguments\n";
const char g_fmtdeepnesting[]    = "nsh: %s: nesting too deep\n";
const char g_fmtcontext[]        = "nsh: %s: not valid in this context\n";
#ifdef CONFIG_EXAMPLES_NSH_STRERROR
const char g_fmtcmdfailed[]      = "nsh: %s: %s failed: %s\n";
#else
const char g_fmtcmdfailed[]      = "nsh: %s: %s failed: %d\n";
#endif
const char g_fmtcmdoutofmemory[] = "nsh: %s: out of memory\n";
const char g_fmtinternalerror[]  = "nsh: %s: Internal error\n";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_help
 ****************************************************************************/

static int cmd_help(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  const struct cmdmap_s *ptr;

  nsh_output(vtbl, "NSH command forms:\n");
  nsh_output(vtbl, "  [nice [-d <niceness>>]] <cmd> [[> <file>|>> <file>] &]\n");
  nsh_output(vtbl, "OR\n");
  nsh_output(vtbl, "  if <cmd>\n");
  nsh_output(vtbl, "  then\n");
  nsh_output(vtbl, "    [sequence of <cmd>]\n");
  nsh_output(vtbl, "  else\n");
  nsh_output(vtbl, "    [sequence of <cmd>]\n");
  nsh_output(vtbl, "  fi\n");
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
  return OK;
}

/****************************************************************************
 * Name: cmd_unrecognized
 ****************************************************************************/

static int cmd_unrecognized(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_output(vtbl, g_fmtcmdnotfound, argv[0]);
  return ERROR;
}

/****************************************************************************
 * Name: cmd_exit
 ****************************************************************************/

static int cmd_exit(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_exit(vtbl);
  return OK;
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
   int                    ret;

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

   ret = handler(vtbl, argc, &argv[2]);
   nsh_release(vtbl);
   return ret;
}

/****************************************************************************
 * Name: nsh_argument
 ****************************************************************************/

char *nsh_argument(FAR struct nsh_vtbl_s *vtbl, char **saveptr)
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

  /* Does the token begin with '>' -- redirection of output? */

  if (*pbegin == '>')
    {
      /* Yes.. does it begin with ">>"? */

      if (*(pbegin + 1) == '>')
        {
          *saveptr = pbegin + 2;
          pbegin   = (char*)g_redirect2;
        }
      else
        {
          *saveptr = pbegin + 1;
          pbegin   = (char*)g_redirect1;
        }
    }

  /* Does the token begin with '#' -- comment */

  else if (*pbegin == '#')
    {
      /* Return NULL meaning that we are at the end of the line */

      *saveptr = pbegin;
      pbegin   = NULL;
    }
  else
    {
      /* Otherwise, we are going to have to parse to find the end of
       * the token.  Does the token begin with '"'?
       */

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

      /* Check for references to environment variables */

      if (pbegin[0] == '$' && !quoted)
        {
          /* Check for built-in variables */

          if (strcmp(pbegin, g_exitstatus) == 0)
            {
              if (vtbl->np.np_fail)
                {
                  return (char*)g_failure;
                }
              else
                {
                  return (char*)g_success;
                }
            }

          /* Not a built-in? Return the value of the environment variable with this name */
#ifndef CONFIG_DISABLE_ENVIRON
          else
            {
              char *value = getenv(pbegin+1);
              if (value)
                {
                  return value;
                }
              else
                {
                  return (char*)"";
                }
            }
#endif
        }
    }

  /* Return the beginning of the token. */

  return pbegin;
}

/****************************************************************************
 * Name: nsh_cmdenabled
 ****************************************************************************/

static inline boolean nsh_cmdenabled(FAR struct nsh_vtbl_s *vtbl)
{
  struct nsh_parser_s *np = &vtbl->np;
  boolean ret = !np->np_st[np->np_ndx].ns_disabled;
  if (ret)
    {
      switch (np->np_st[np->np_ndx].ns_state)
        {
          case NSH_PARSER_NORMAL :
          case NSH_PARSER_IF:
          default:
            break;

          case NSH_PARSER_THEN:
            ret = !np->np_st[np->np_ndx].ns_ifcond;
            break;

          case NSH_PARSER_ELSE:
            ret = np->np_st[np->np_ndx].ns_ifcond;
            break;
        }
    }
  return ret;
}

/****************************************************************************
 * Name: nsh_ifthenelse
 ****************************************************************************/

static inline int nsh_ifthenelse(FAR struct nsh_vtbl_s *vtbl, FAR char **ppcmd, FAR char **saveptr)
{
  struct nsh_parser_s *np = &vtbl->np;
  FAR char *cmd = *ppcmd;
  boolean disabled;

  if (cmd)
    {
      /* Check if the command is preceeded by "if" */

      if (strcmp(cmd, "if") == 0)
        {
          /* Get the cmd following the if */

          *ppcmd = nsh_argument(vtbl, saveptr);
          if (!*ppcmd)
            {
              nsh_output(vtbl, g_fmtarginvalid, "if");
              goto errout;
            }

          /* Verify that "if" is valid in this context */

          if (np->np_st[np->np_ndx].ns_state != NSH_PARSER_NORMAL &&
              np->np_st[np->np_ndx].ns_state != NSH_PARSER_THEN &&
              np->np_st[np->np_ndx].ns_state != NSH_PARSER_ELSE)
            {
              nsh_output(vtbl, g_fmtcontext, "if");
              goto errout;
            }

          /* Check if we have exceeded the maximum depth of nesting */

          if (np->np_ndx >= CONFIG_EXAMPLES_NSH_NESTDEPTH-1)
            {
              nsh_output(vtbl, g_fmtdeepnesting, "if");
              goto errout;            
            }

          /* "Push" the old state and set the new state */

          disabled                          = !nsh_cmdenabled(vtbl);
          np->np_ndx++;
          np->np_st[np->np_ndx].ns_state    = NSH_PARSER_IF;
          np->np_st[np->np_ndx].ns_disabled = disabled;
          np->np_st[np->np_ndx].ns_ifcond   = FALSE;
        }
      else if (strcmp(cmd, "then") == 0)
        {
          /* Get the cmd following the then -- there shouldn't be one */

          *ppcmd = nsh_argument(vtbl, saveptr);
          if (*ppcmd)
            {
              nsh_output(vtbl, g_fmtarginvalid, "then");
              goto errout;
            }

          /* Verify that "then" is valid in this context */

          if (np->np_st[np->np_ndx].ns_state != NSH_PARSER_IF)
            {
              nsh_output(vtbl, g_fmtcontext, "then");
              goto errout;
            }
          np->np_st[np->np_ndx].ns_state = NSH_PARSER_THEN;
        }
      else if (strcmp(cmd, "else") == 0)
        {
          /* Get the cmd following the else -- there shouldn't be one */

          *ppcmd = nsh_argument(vtbl, saveptr);
          if (*ppcmd)
            {
              nsh_output(vtbl, g_fmtarginvalid, "else");
              goto errout;
            }

          /* Verify that "then" is valid in this context */

          if (np->np_st[np->np_ndx].ns_state != NSH_PARSER_THEN)
            {
              nsh_output(vtbl, g_fmtcontext, "else");
              goto errout;
            }
          np->np_st[np->np_ndx].ns_state = NSH_PARSER_ELSE;
        }
      else if (strcmp(cmd, "fi") == 0)
        {
          /* Get the cmd following the fi -- there should be one */

          *ppcmd = nsh_argument(vtbl, saveptr);
          if (*ppcmd)
            {
              nsh_output(vtbl, g_fmtarginvalid, "fi");
              goto errout;
            }

          /* Verify that "fi" is valid in this context */

          if (np->np_st[np->np_ndx].ns_state != NSH_PARSER_THEN &&
              np->np_st[np->np_ndx].ns_state != NSH_PARSER_ELSE)
            {
              nsh_output(vtbl, g_fmtcontext, "fi");
              goto errout;
            }

          if (np->np_ndx < 1) /* Shouldn't happen */
            {
              nsh_output(vtbl, g_fmtinternalerror, "if");
              goto errout;            
            }

          /* "Pop" the previous state */

          np->np_ndx--;
        }
      else if (np->np_st[np->np_ndx].ns_state == NSH_PARSER_IF)
        {
          nsh_output(vtbl, g_fmtcontext, cmd);
          goto errout;
        }
    }
  return OK;

errout:
  np->np_ndx               = 0;
  np->np_st[0].ns_state    = NSH_PARSER_NORMAL;
  np->np_st[0].ns_disabled = FALSE;
  np->np_st[0].ns_ifcond   = FALSE;
  return ERROR;
}

/****************************************************************************
 * Name: nsh_saveresult
 ****************************************************************************/

static inline int nsh_saveresult(FAR struct nsh_vtbl_s *vtbl, boolean result)
{
  struct nsh_parser_s *np = &vtbl->np;

  if (np->np_st[np->np_ndx].ns_state == NSH_PARSER_IF)
    {
      np->np_fail = FALSE;
      np->np_st[np->np_ndx].ns_ifcond = result;
      return OK;
    }
  else
    {
      np->np_fail = result;
      return result ? ERROR : OK;
    }
}

/****************************************************************************
 * Name: nsh_nice
 ****************************************************************************/

static inline int nsh_nice(FAR struct nsh_vtbl_s *vtbl, FAR char **ppcmd, FAR char **saveptr)
{
  FAR char *cmd = *ppcmd;
 
  vtbl->np.np_nice = 0;
  if (cmd)
    {
      /* Check if the command is preceded by "nice" */

      if (strcmp(cmd, "nice") == 0)
        {
          /* Nicenesses range from -20 (most favorable scheduling) to 19
           * (least  favorable).  Default is 10.
           */

          vtbl->np.np_nice = 10;

          /* Get the cmd (or -d option of nice command) */

          cmd = nsh_argument(vtbl, saveptr);
          if (cmd && strcmp(cmd, "-d") == 0)
            {
              FAR char *val = nsh_argument(vtbl, saveptr);
              if (val)
                {
                  char *endptr;
                  vtbl->np.np_nice = (int)strtol(val, &endptr, 0);
                  if (vtbl->np.np_nice > 19 || vtbl->np.np_nice < -20 ||
                      endptr == val || *endptr != '\0')
                    {
                      nsh_output(vtbl, g_fmtarginvalid, "nice");
                      return ERROR;
                    }
                  cmd = nsh_argument(vtbl, saveptr);
                }
            }

          /* Return the real command name */

          *ppcmd = cmd;
        }
    }
  return OK;
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
  FAR char *redirfile = NULL;
  int       fd = -1;
  int       oflags = 0;
  int       argc;
  int       ret;

  /* Initialize parser state */

  memset(argv, 0, (NSH_MAX_ARGUMENTS+4)*sizeof(char*));
  vtbl->np.np_bg       = FALSE;
  vtbl->np.np_redirect = FALSE;

  /* Parse out the command at the beginning of the line */

  saveptr = cmdline;
  cmd = nsh_argument(vtbl, &saveptr);

  /* Handler if-then-else-fi */

  if (nsh_ifthenelse(vtbl, &cmd, &saveptr) != 0)
    {
      goto errout;
    }

  /* Handle nice */

  if (nsh_nice(vtbl, &cmd, &saveptr) != 0)
    {
      goto errout;
    }

  /* Check if any command was provided -OR- if command processing is
   * currently disabled.
   */

  if (!cmd || !nsh_cmdenabled(vtbl))
    {
      /* An empty line is not an error and an unprocessed command cannot
       * generate an error, but neither should they change the last
       * command status.
       */

      return OK;
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
      argv[argc] = nsh_argument(vtbl, &saveptr);
      if (!argv[argc])
        {
          break;
        }
    }
  argv[argc] = NULL;

  /* Check if the command should run in background */

  if (argc > 3 && strcmp(argv[argc-1], "&") == 0)
    {
      vtbl->np.np_bg = TRUE;
      argv[argc-1] = NULL;
      argc--;
    }

  /* Check if the output was re-directed using > or >> */

  if (argc > 5)
    {
      /* Check for redirection to a new file */

      if (strcmp(argv[argc-2], g_redirect1) == 0)
        {
          vtbl->np.np_redirect = TRUE;
          oflags               = O_WRONLY|O_CREAT|O_TRUNC;
          redirfile            = nsh_getfullpath(vtbl, argv[argc-1]);
          argc                -= 2;
        }

      /* Check for redirection by appending to an existing file */

      else if (strcmp(argv[argc-2], g_redirect2) == 0)
        {
          vtbl->np.np_redirect = TRUE;
          oflags               = O_WRONLY|O_CREAT|O_APPEND;
          redirfile            = nsh_getfullpath(vtbl, argv[argc-1]);
          argc                -= 2;
        }
    }

  /* Redirected output? */

  if (vtbl->np.np_redirect)
    {
      /* Open the redirection file.  This file will eventually
       * be closed by a call to either nsh_release (if the command
       * is executed in the background) or by nsh_undirect if the
       * command is executed in the foreground.
       */

      fd = open(redirfile, oflags, 0666);
      nsh_freefullpath(redirfile);
      redirfile = NULL;

      if (fd < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "open", NSH_ERRNO);
          goto errout;
        }
    }

  /* Check if the maximum number of arguments was exceeded */

  if (argc > NSH_MAX_ARGUMENTS+3)
    {
      nsh_output(vtbl, g_fmttoomanyargs, cmd);
    }

  /* Handle the case where the command is executed in background */

  if (vtbl->np.np_bg)
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

      if (vtbl->np.np_redirect)
        {
          (void)nsh_redirect(bkgvtbl, fd, NULL);
        }

      /* Get the execution priority of this task */

      ret = sched_getparam(0, &param);
      if (ret != 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "sched_getparm", NSH_ERRNO);
          goto errout_with_redirect;
        }

      /* Determine the priority to execute the command */

      priority = param.sched_priority;
      if (vtbl->np.np_nice != 0)
        {
          priority -= vtbl->np.np_nice;
          if (vtbl->np.np_nice < 0)
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
          goto errout_with_redirect;
        }
      nsh_output(vtbl, "%s [%d:%d:%d]\n", cmd, ret, priority, param.sched_priority);

      /* If the output was redirected, then file descriptor should
       * be closed.  The created task has its one, independent copy of
       * the file descriptor
       */

      if (vtbl->np.np_redirect)
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

      if (vtbl->np.np_redirect)
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

      if (vtbl->np.np_redirect)
        {
          nsh_undirect(vtbl, save);
        }

      if (ret < 0)
        {
          goto errout;
        }
    }

  /* Return success if the command succeeded (or at least, starting of the
   * command task succeeded).
   */

  return nsh_saveresult(vtbl, FALSE);

errout_with_redirect:
  if (vtbl->np.np_redirect)
    {
      close(fd);
    }
errout:
  return nsh_saveresult(vtbl, TRUE);
}
