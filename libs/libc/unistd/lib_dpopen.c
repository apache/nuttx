/****************************************************************************
 * libs/libc/unistd/lib_dpopen.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/ioctl.h>
#include <sys/wait.h>
#include <errno.h>
#include <fcntl.h>
#include <paths.h>
#include <spawn.h>
#include <string.h>
#include <unistd.h>

#if defined(CONFIG_NET_LOCAL) && defined(CONFIG_NET_LOCAL_STREAM)
#  include <sys/socket.h>
#endif

#include "libc.h"

/* The kernel side of libc does not run commands */

#ifndef __KERNEL__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The shell that runs the command.  With CONFIG_LIBC_ENVPATH, PATH gives
 * the location of "sh".
 */

#ifdef CONFIG_LIBC_ENVPATH
#  define DPOPEN_SHELL "sh"
#else
#  define DPOPEN_SHELL _PATH_BSHELL
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dpopen_direct
 *
 * Description:
 *   Start a command without a shell, for a system that has none: split the
 *   command at blanks and start the program in the first word with
 *   posix_spawnp().  Shell syntax (pipes, redirection, quotes, variables)
 *   is not supported.
 *
 ****************************************************************************/

static int dpopen_direct(FAR pid_t *pid, FAR const char *command,
                         FAR const posix_spawn_file_actions_t *actions)
{
  FAR char **argv;
  FAR char *saveptr;
  FAR char *words;
  size_t len = strlen(command);
  size_t nargs = len / 2 + 2;   /* A word needs a character and a blank */
  size_t argc = 0;
  int errcode;

  argv = lib_malloc(nargs * sizeof(FAR char *) + len + 1);
  if (argv == NULL)
    {
      return ENOMEM;
    }

  words = (FAR char *)&argv[nargs];
  memcpy(words, command, len + 1);

  argv[0] = strtok_r(words, " \t\n", &saveptr);
  while (argv[argc] != NULL)
    {
      argv[++argc] = strtok_r(NULL, " \t\n", &saveptr);
    }

  /* posix_spawnp() copies the arguments, so they can be freed after it */

  errcode = argc > 0 ? posix_spawnp(pid, argv[0], actions, NULL, argv,
                                    NULL) : EINVAL;
  lib_free(argv);
  return errcode;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dpopen
 *
 * Description:
 *   Run the command "sh -c command" in a new process and return a file
 *   descriptor connected to it.  If there is no shell, the command is
 *   split at blanks and its first word is started directly, without shell
 *   syntax.
 *
 *   This is the descriptor counterpart of popen(), as dprintf() is the
 *   counterpart of fprintf().
 *
 * Input Parameters:
 *   command - The command string for the shell.
 *   oflag   - O_RDONLY to read the standard output of the command,
 *             O_WRONLY to write its standard input, or O_RDWR for both
 *             (needs local stream sockets).  O_CLOEXEC sets the
 *             close-on-exec flag of the returned descriptor.
 *   pid     - The location that receives the process ID of the shell.
 *
 * Returned Value:
 *   A file descriptor on success.  On failure, -1 is returned and errno is
 *   set.
 *
 ****************************************************************************/

int dpopen(FAR const char *command, int oflag, FAR pid_t *pid)
{
  posix_spawn_file_actions_t file_actions;
  FAR char *argv[4];
  int fd[2];
  int childfd;
  int parentfd;
  int errcode;

  if (command == NULL || pid == NULL)
    {
      set_errno(EINVAL);
      return -1;
    }

  /* Create a pipe or a socket pair.  fd[0] is the read end of the pipe
   * and fd[1] is the write end.
   */

  if ((oflag & O_ACCMODE) == O_RDWR)
    {
#if defined(CONFIG_NET_LOCAL) && defined(CONFIG_NET_LOCAL_STREAM)
      if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, fd) < 0)
        {
          return -1;
        }

      childfd  = fd[0];
      parentfd = fd[1];
#else
      set_errno(ENOTSUP);
      return -1;
#endif
    }
  else if ((oflag & O_ACCMODE) != O_RDONLY &&
           (oflag & O_ACCMODE) != O_WRONLY)
    {
      set_errno(EINVAL);
      return -1;
    }
  else if (pipe2(fd, O_CLOEXEC) < 0)
    {
      return -1;
    }
  else if ((oflag & O_ACCMODE) == O_RDONLY)
    {
      childfd  = fd[1];  /* The command writes to the pipe */
      parentfd = fd[0];  /* The caller reads from the pipe */
    }
  else
    {
      childfd  = fd[0];  /* The command reads from the pipe */
      parentfd = fd[1];  /* The caller writes to the pipe */
    }

  errcode = posix_spawn_file_actions_init(&file_actions);
  if (errcode != 0)
    {
      goto errout_with_pipe;
    }

  /* Connect the standard input or output of the command to the pipe */

  if ((oflag & O_ACCMODE) != O_WRONLY)
    {
      errcode = posix_spawn_file_actions_adddup2(&file_actions, childfd,
                                                 STDOUT_FILENO);
      if (errcode != 0)
        {
          goto errout_with_actions;
        }
    }

  if ((oflag & O_ACCMODE) != O_RDONLY)
    {
      errcode = posix_spawn_file_actions_adddup2(&file_actions, childfd,
                                                 STDIN_FILENO);
      if (errcode != 0)
        {
          goto errout_with_actions;
        }
    }

  argv[0] = "sh";
  argv[1] = "-c";
  argv[2] = (FAR char *)command;
  argv[3] = NULL;

  errcode = posix_spawn(pid, DPOPEN_SHELL, &file_actions, NULL, argv,
                        NULL);
  if (errcode == ENOENT)
    {
      /* There is no shell: start the command directly */

      errcode = dpopen_direct(pid, command, &file_actions);
    }

  posix_spawn_file_actions_destroy(&file_actions);
  if (errcode != 0)
    {
      goto errout_with_pipe;
    }

  /* The other ends of earlier pipes are close-on-exec, so a new command
   * does not keep them open.  Clear the flag only if the caller wants.
   */

  if ((oflag & O_CLOEXEC) == 0)
    {
      ioctl(parentfd, FIONCLEX, 0);
    }

  close(childfd);
  return parentfd;

errout_with_actions:
  posix_spawn_file_actions_destroy(&file_actions);

errout_with_pipe:
  close(fd[0]);
  close(fd[1]);
  set_errno(errcode);
  return -1;
}

/****************************************************************************
 * Name: dpclose
 *
 * Description:
 *   Close a file descriptor from dpopen() and wait for the command to
 *   terminate.
 *
 * Input Parameters:
 *   fd  - The file descriptor from dpopen().
 *   pid - The process ID from dpopen().
 *
 * Returned Value:
 *   The termination status of the command, as from waitpid().  On
 *   failure, -1 is returned and errno is set.
 *
 ****************************************************************************/

int dpclose(int fd, pid_t pid)
{
  int status;
  int ret;

  if (fd >= 0)
    {
      close(fd);
    }

  do
    {
      ret = waitpid(pid, &status, 0);
    }
  while (ret < 0 && get_errno() == EINTR);

  return ret < 0 ? -1 : status;
}

#endif /* __KERNEL__ */
