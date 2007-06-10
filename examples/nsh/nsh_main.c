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

#if CONFIG_NFILE_DESCRIPTORS > 0
# include <sys/stat.h>
# include <fcntl.h>
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
# include <sys/mount.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <limits.h>
#include <string.h>
#include <sched.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define CONFIG_NSH_LINE_SIZE 80
#undef  CONFIG_FULL_PATH
#define NSH_MAX_ARGUMENTS     6

#define LSFLAGS_SIZE          1
#define LSFLAGS_LONG          2
#define LSFLAGS_RECURSIVE     4

#define errno                (*get_errno_ptr())

/****************************************************************************
 * Private Types
 ****************************************************************************/

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

typedef int (*direntry_handler_t)(const char *, struct dirent *, void *);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static void cmd_cat(int argc, char **argv);
#endif
static void cmd_echo(int argc, char **argv);
static void cmd_exec(int argc, char **argv);
static void cmd_help(int argc, char **argv);
#if CONFIG_NFILE_DESCRIPTORS > 0
static void cmd_ls(int argc, char **argv);
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
static void cmd_mkdir(int argc, char **argv);
static void cmd_mount(int argc, char **argv);
#endif
static void cmd_ps(int argc, char **argv);
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
static void cmd_umount(int argc, char **argv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char line[CONFIG_NSH_LINE_SIZE];
static const char delim[] = " \t\n";

static const struct cmdmap_s g_cmdmap[] =
{
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "cat",    cmd_cat,    2, 2, "<path>" },
#endif
  { "echo",   cmd_echo,   2, 2, "<string>" },
  { "exec",   cmd_exec,   2, 3, "<hex-address>" },
  { "help",   cmd_help,   1, 1, NULL },
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "ls",     cmd_ls,     2, 5, "[-lRs] <path>" },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
  { "mkdir",  cmd_mkdir,  2, 2, "<path>" },
  { "mount",  cmd_mount,  4, 5, "-t <fstype> <device> <dir>" },
#endif
  { "ps",     cmd_ps,     1, 1, NULL },
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trim_dir
 ****************************************************************************/

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

/****************************************************************************
 * Name: getdirpath
 ****************************************************************************/

static char *getdirpath(const char *path, const char *file)
{
  char buffer[PATH_MAX+1];
  if (strcmp(path, "/") == 0)
    {
      sprintf(buffer, "/%s", file);
    }
  else
    {
      sprintf(buffer, "%s/%s", path, file);
    }

  buffer[PATH_MAX] = '\0';
  return strdup(buffer);
}

/****************************************************************************
 * Name: foreach_direntry
 ****************************************************************************/

static int foreach_direntry(const char *cmd, const char *dirpath,
                            direntry_handler_t handler, void *pvarg)
{
  DIR *dirp;
  int ret = OK;

  /* Trim trailing '/' from directory names */

#ifdef CONFIG_FULL_PATH
  trim_dir(arg);
#endif

  /* Open the directory */

  dirp = opendir(dirpath);

  if (!dirp)
    {
      /* Failed to open the directory */

      printf(g_fmtnosuch, cmd, "directory", dirpath);
      return ERROR;
    }

  /* Read each directory entry */

  for (;;)
    {
      struct dirent *entryp = readdir(dirp);
      if (!entryp)
        {
          /* Finished with this directory */

          break;
        }

      /* Call the handler with this directory entry */

      if (handler(dirpath, entryp, pvarg) <  0)
        {
          /* The handler reported a problem */

          ret = ERROR;
          break;
        }
    }

  closedir(dirp);
  return ret;
}

/****************************************************************************
 * Name: cmd_cat
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static void cmd_cat(int argc, char **argv)
{
  char buffer[1024];

  /* Open the file for reading */

  int fd = open(argv[1], O_RDONLY);
  if (fd < 0)
    {
      printf(g_fmtcmdfailed, argv[0], "open", strerror(errno));
      return;
    }

  /* And just dump it byte for byte into stdout */

  for (;;)
    {
      int nbytesread = read(fd, buffer, 1024);

      /* Check for read errors */

      if (nbytesread < 0)
        {
          printf(g_fmtcmdfailed, argv[0], "read", strerror(errno));
          break;
        }

      /* Check for data successfully read */

      else if (nbytesread > 0)
        {
          int nbyteswritten = 0;
          char *ptr = buffer;

          while (nbyteswritten < nbytesread)
            {
              int n = write(1, buffer, nbytesread);
              if (n < 0)
                {
                  printf(g_fmtcmdfailed, argv[0], "write", strerror(errno));
                  break;
                }
              nbyteswritten += n;
            }
        }

      /* Otherwise, it is the end of file */

      else
        {
          break;
        }
    }

  (void)close(fd);
}
#endif

/****************************************************************************
 * Name: cmd_echo
 ****************************************************************************/

static void cmd_echo(int argc, char **argv)
{
  /* Echo the rest of the line */

  puts(argv[1]);
}

/****************************************************************************
 * Name: cmd_exec
 ****************************************************************************/

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
 * Name: ls_handler
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int ls_handler(const char *dirpath, struct dirent *entryp, void *pvarg)
{
  unsigned int lsflags = (unsigned int)pvarg;
  int ret;

  /* Check if any options will require that we stat the file */

  if ((lsflags & (LSFLAGS_SIZE|LSFLAGS_LONG)) != 0)
    {
      struct stat buf;
      char *fullpath = getdirpath(dirpath, entryp->d_name);

      /* Yes, stat the file */

      ret = stat(fullpath, &buf);
      free(fullpath);
      if (ret != 0)
        {
          printf(g_fmtcmdfailed, "ls", "stat", strerror(errno));
          return OK;
        }

      if ((lsflags & LSFLAGS_LONG) != 0)
        {
          char details[] = "----------";
          if (S_ISDIR(buf.st_mode))
            {
              details[0]='d';
            }
          else if (S_ISCHR(buf.st_mode))
            {
              details[0]='c';
            }
          else if (S_ISBLK(buf.st_mode))
            {
              details[0]='b';
            }

          if ((buf.st_mode & S_IRUSR) != 0)
            {
              details[1]='r';
            }

          if ((buf.st_mode & S_IWUSR) != 0)
            {
              details[2]='w';
            }

          if ((buf.st_mode & S_IXUSR) != 0)
            {
              details[3]='x';
            }

          if ((buf.st_mode & S_IRGRP) != 0)
            {
              details[4]='r';
            }

          if ((buf.st_mode & S_IWGRP) != 0)
            {
              details[5]='w';
            }

          if ((buf.st_mode & S_IXGRP) != 0)
            {
              details[6]='x';
            }

          if ((buf.st_mode & S_IROTH) != 0)
            {
              details[7]='r';
            }

          if ((buf.st_mode & S_IWOTH) != 0)
            {
              details[8]='w';
            }

          if ((buf.st_mode & S_IXOTH) != 0)
            {
              details[9]='x';
            }

          printf(" %s", details);
        }

      if ((lsflags & LSFLAGS_SIZE) != 0)
        {
          printf("%8d", buf.st_size);
        }
    }

  /* then provide the filename that is common to normal and verbose output */

#ifdef CONFIG_FULL_PATH
  printf(" %s/%s", arg, entryp->d_name);
#else
  printf(" %s", entryp->d_name);
#endif

  if (DIRENT_ISDIRECTORY(entryp->d_type))
    {
      printf("/\n");
    }
  else
    {
      putchar('\n');
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: ls_recursive
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int ls_recursive(const char *dirpath, struct dirent *entryp, void *pvarg)
{
  /* Is this entry a directory? */

  if (DIRENT_ISDIRECTORY(entryp->d_type))
    {
      /* Yes.. */

      char *newpath;
      newpath = getdirpath(dirpath, entryp->d_name);

      /* List the directory contents */

      printf("%s:\n", newpath);
      foreach_direntry("ls", newpath, ls_handler, pvarg);

      /* Then recurse to list each directory within the directory */

      foreach_direntry("ls", newpath, ls_recursive, pvarg);
      free(newpath);
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: cmd_ls
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static void cmd_ls(int argc, char **argv)
{
  unsigned int lsflags = 0;
  int ret;

  /* Get the ls options */

  int option;
  while ((option = getopt(argc, argv, "lRs")) != ERROR)
    {
      switch (option)
        {
          case 'l':
            lsflags |= (LSFLAGS_SIZE|LSFLAGS_LONG);
            break;

          case 'R':
            lsflags |= LSFLAGS_RECURSIVE;
            break;

          case 's':
            lsflags |= LSFLAGS_SIZE;
            break;

          case '?':
          default:
            printf(g_fmtarginvalid, argv[0]);
            return;
        }
    }

  /* There are one required arguments after the options */

  if (optind + 1 <  argc)
    {
      printf(g_fmttoomanyargs, argv[0]);
      return;
    }
  else if (optind + 1 >  argc)
    {
      printf(g_fmtargrequired, argv[0]);
      return;
    }

  /* List the directory contents */

  printf("%s:\n", argv[optind]);
  ret = foreach_direntry("ls", argv[optind], ls_handler, (void*)lsflags);
  if (ret == OK && (lsflags & LSFLAGS_RECURSIVE) != 0)
    {
      /* Then recurse to list each directory within the directory */

      ret = foreach_direntry("ls", argv[optind], ls_recursive, (void*)lsflags);
    }
}
#endif

/****************************************************************************
 * Name: cmd_mount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
static void cmd_mkdir(int argc, char **argv)
{
  int result = mkdir(argv[1], 0777);
  if ( result < 0)
    {
      printf(g_fmtcmdfailed, argv[0], "mkdir", strerror(errno));
    }
}
#endif

/****************************************************************************
 * Name: cmd_mount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
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

  if (optind + 2 <  argc)
    {
      printf(g_fmttoomanyargs, argv[0]);
      return;
    }
  else if (optind + 2 >  argc)
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

/****************************************************************************
 * Name: ps_task
 ****************************************************************************/

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

/****************************************************************************
 * Name: cmd_ps
 ****************************************************************************/

static void cmd_ps(int argc, char **argv)
{
  printf("PID   PRI SCHD TYPE   NP STATE    NAME\n");
  sched_foreach(ps_task, NULL);
}

/****************************************************************************
 * Name: cmd_umount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
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
